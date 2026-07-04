#include "readsb.h"
#include "net_gpsd.h"

void handle_radarcape_position(float lat, float lon, float alt) {
    if (Modes.netIngest || Modes.netReceiverId) {
        return;
    }

    if (!isfinite(lat) || lat < -90 || lat > 90 || !isfinite(lon) || lon < -180 || lon > 180 || !isfinite(alt)) {
        return;
    }

    if (!Modes.userLocationValid) {
        Modes.fUserLat = lat;
        Modes.fUserLon = lon;
        Modes.userLocationValid = 1;
        receiverPositionChanged(lat, lon, alt);
    }
}

/**
 * Convert 32bit binary angular measure to double degree.
 * See https://www.globalspec.com/reference/14722/160210/Chapter-7-5-3-Binary-Angular-Measure
 * @param data Data buffer start (MSB first)
 * @return Angular degree.
 */

static double bam32ToDouble(uint32_t bam) {
    return (double) ((int32_t) ntohl(bam) * 8.38190317153931E-08);
}

//
//=========================================================================
//
// This function decodes a GNS HULC protocol message


void decodeHulcMessage(char *p) {
    // silently ignore these messages if proper SDR isn't set
    if (SdrConfig.sdr_type != SDR_GNS)
        return;

    int alt = 0;
    double lat = 0.0;
    double lon = 0.0;
    char id = *p++; //Get message id
    unsigned char len = *p++; // Get message length
    hulc_status_msg_t hsm;

    if (id == 0x01 && len == 0x18) {
        // HULC Status message
        for (int j = 0; j < len; j++) {
            hsm.buf[j] = *p++;
            // unescape
            if (*p == 0x1A) {
                p++;
            }
        }
        /*
        // Antenna serial
        Modes.receiver.antenna_serial = ntohl(hsm.status.serial);
        // Antenna status flags
        Modes.receiver.antenna_flags = ntohs(hsm.status.flags);
        // Reserved for internal use
        Modes.receiver.antenna_reserved = ntohs(hsm.status.reserved);
        // Antenna Unix epoch (not used)
        // Antenna GPS satellites used for fix
        Modes.receiver.antenna_gps_sats = hsm.status.satellites;
        // Antenna GPS HDOP*10, thus 12 is HDOP 1.2
        Modes.receiver.antenna_gps_hdop = hsm.status.hdop;
        */

        // Antenna GPS latitude
        lat = bam32ToDouble(hsm.status.latitude);
        // Antenna GPS longitude
        lon = bam32ToDouble(hsm.status.longitude);
        // Antenna GPS altitude
        alt = ntohs(hsm.status.altitude);
        uint32_t antenna_flags = ntohs(hsm.status.flags);
        // Use only valid GPS position
        if ((antenna_flags & 0xE000) == 0xE000) {
            if (!isfinite(lat) || lat < -90 || lat > 90 || !isfinite(lon) || lon < -180 || lon > 180) {
                return;
            }
            // only use when no fixed location is defined
            if (!Modes.userLocationValid) {
                Modes.fUserLat = lat;
                Modes.fUserLon = lon;
                Modes.userLocationValid = 1;
                receiverPositionChanged(lat, lon, alt);
            }
        }
    } else if (id == 0x01 && len > 0x18) {
        // Future use planed.
    } else if (id == 0x24 && len == 0x10) {
        // Response to command #00
        fprintf(stderr, "Firmware: v%0u.%0u.%0u\n", *(p + 5), *(p + 6), *(p + 7));
    }
}

// recompute global Mode A/C setting

int handle_gpsd(struct client *c, char *p, int remote, int64_t now, struct messageBuffer *mb) {
    MODES_NOTUSED(c);
    MODES_NOTUSED(remote);
    MODES_NOTUSED(now);
    MODES_NOTUSED(mb);

    if (Modes.debug_gps) {
        fprintTime(stderr, now);
        fprintf(stderr, " gpsdebug: received from GPSD: \'%s\'\n", p);
    }

    struct char_buffer msg;
    msg.alloc = strlen(p) + 128;
    msg.buffer = cmalloc(msg.alloc);
    sprintf(msg.buffer, "%s\n", p);
    msg.len = strlen(msg.buffer);

    // remove spaces in place
    char *d = p;
    char *s = p;
    do {
        while (*s == ' ') {
            s++;
        }
        *d = *s++;
    } while (*d++);

    // filter all messages but TPV type
    if (!strstr(p, "\"class\":\"TPV\"")) {
        if (Modes.debug_gps) {
            fprintf(stderr, "gpsdebug: class is not \"TPV\" : ignoring message.\n");
        }
        goto exit;
    }
    // filter all messages which don't have lat / lon
    char *latp = strstr(p, "\"lat\":");
    char *lonp = strstr(p, "\"lon\":");
    char *altp = strstr(p, "\"alt\":");
    if (!latp || !lonp) {
        if (Modes.debug_gps) {
            fprintf(stderr, "gpsdebug: lat / lon not present: ignoring message.\n");
        }
        goto exit;
    }
    latp += 6;
    lonp += 6;

    char *saveptr = NULL;
    strtok_r(latp, ",", &saveptr);
    saveptr = NULL;
    strtok_r(lonp, ",", &saveptr);

    double lat = strtod(latp, NULL);
    double lon = strtod(lonp, NULL);

    double alt_m = -2e6;
    if (altp) {
        altp += 6;
        saveptr = NULL;
        strtok_r(altp, ",", &saveptr);
        alt_m = strtod(altp, NULL);
    }

    if (Modes.debug_gps) {
        if (alt_m > -1e6) {
            fprintf(stderr, "gpsdebug: parsed lat,lon: %11.6f,%11.6f (alt: %.0f m)\n", lat, lon, alt_m);
        } else {
            fprintf(stderr, "gpsdebug: parsed lat,lon: %11.6f,%11.6f (no alt)\n", lat, lon);
        }
    }
    //fprintf(stderr, "%11.6f %11.6f\n", lat, lon);


    if (!isfinite(lat) || lat < -89.9 || lat > 89.9 || !isfinite(lon) || lon < -180 || lon > 180) {
        if (Modes.debug_gps) {
            fprintf(stderr, "gpsdebug: lat lon implausible, ignoring\n");
        }
        goto exit;
    }
    if (fabs(lat) < 0.1 && fabs(lon) < 0.1) {
        if (Modes.debug_gps) {
            fprintf(stderr, "gpsdebug: lat lon implausible, ignoring\n");
        }
        goto exit;
    }

    if (Modes.debug_gps) {
        fprintf(stderr, "gpsdebug: Updating position, writing receiver.json\n");
    }

    Modes.fUserLat = lat;
    Modes.fUserLon = lon;
    if (alt_m > -1e6) {
        Modes.fUserAlt = alt_m;
    }
    Modes.userLocationValid = 1;

    if (Modes.json_dir) {
        free(writeJsonToFile(Modes.json_dir, "receiver.json", generateReceiverJson()).buffer); // location changed
        if (msg.len) {
            writeJsonToFile(Modes.json_dir, "gpsd.json", msg);
        }
    }

exit:
    free(msg.buffer);
    return 0;
}


