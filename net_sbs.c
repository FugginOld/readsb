#include "readsb.h"
#include "net_sbs.h"

int decodeSbsLineMlat(struct client *c, char *line, int remote, int64_t now, struct messageBuffer *mb) {
    MODES_NOTUSED(remote);
    return decodeSbsLine(c, line, 64 + SOURCE_MLAT, now, mb);
}

int decodeSbsLinePrio(struct client *c, char *line, int remote, int64_t now, struct messageBuffer *mb) {
    MODES_NOTUSED(remote);
    return decodeSbsLine(c, line, 64 + SOURCE_PRIO, now, mb);
}

int decodeSbsLineJaero(struct client *c, char *line, int remote, int64_t now, struct messageBuffer *mb) {
    MODES_NOTUSED(remote);
    return decodeSbsLine(c, line, 64 + SOURCE_JAERO, now, mb);
}

int decodeSbsLine(struct client *c, char *line, int remote, int64_t now, struct messageBuffer *mb) {
    size_t line_len = strlen(line);
    size_t max_len = 200;

    if (Modes.receiver_focus && c->receiverId != Modes.receiver_focus)
        return 0;
    if (line_len < 2) // heartbeat
        return 0;
    if (line_len < 20 || line_len >= max_len)
        goto basestation_invalid;

    struct modesMessage *mm = netGetMM(mb);
    mm->client = c;

    char *p = line;
    char *t[23]; // leave 0 indexed entry empty, place 22 tokens into array

    MODES_NOTUSED(c);
    if (remote >= 64)
        mm->source = remote - 64;
    else
        mm->source = SOURCE_SBS;

    switch(mm->source) {
        case SOURCE_SBS:
            mm->addrtype = ADDR_OTHER;
            break;
        case SOURCE_MLAT:
            mm->addrtype = ADDR_MLAT;
            break;
        case SOURCE_JAERO:
            mm->addrtype = ADDR_JAERO;
            break;
        case SOURCE_PRIO:
            mm->addrtype = ADDR_OTHER;
            break;

        default:
            mm->addrtype = ADDR_OTHER;
    }

    // Mark messages received over the internet as remote so that we don't try to
    // pass them off as being received by this instance when forwarding them
    mm->remote = 1;
    mm->signalLevel = 0;
    mm->sbs_in = 1;

    char *endptr = NULL;
    int badValue = 0; // set this to 1 if a field couldn't be parsed

    // SBS fields:
    // MSG,3,1,1,icaoHex,1,messageDate,messageTime,currentDate,currentTime,callsign_8char,altitude_ft,groundspeed_kts,track,lat,lon,vert_rate_fpm,squawk,squawkChangeAlert,squawkEmergencyFlag,squawkIdentFlag,groundFlag_0airborne_-1ground\r\n

    // sample message from mlat-client basestation output
    //MSG,3,1,1,4AC8B3,1,2019/12/10,19:10:46.320,2019/12/10,19:10:47.789,,36017,,,51.1001,10.1915,,,,,,
    //
    for (int i = 1; i < 23; i++) {
        t[i] = strsep(&p, ",");
        if (!p && i < 22)
            goto basestation_invalid;
    }

    // check field 1
    if (!t[1] || strcmp(t[1], "MSG") != 0)
        goto basestation_invalid;

    if (!t[2] || strlen(t[2]) != 1)
        goto basestation_invalid;

    mm->sbsMsgType = atoi(t[2]);

    if (!t[5] || strlen(t[5]) < 6 || strlen(t[5]) > 7) // icao must be 6 characters
        goto basestation_invalid;

    char *icao = t[5];
    int non_icao = 0;
    if (icao[0] == '~') {
        icao++;
        non_icao = 1;
    }
    unsigned char *chars = (unsigned char *) &(mm->addr);
    for (int j = 0; j < 6; j += 2) {
        int high = hexDigitVal(icao[j]);
        int low = hexDigitVal(icao[j + 1]);

        if (high == -1 || low == -1)
            goto basestation_invalid;

        chars[2 - j / 2] = (high << 4) | low;
    }

    if (non_icao) {
        mm->addr |= MODES_NON_ICAO_ADDRESS;
    }

    // date t7: 2019/12/10
    // time t8: 19:10:46.320
    // separate milliseconds in the time field
    int milli = 0;
    char *msp = t[8]; // millisecond pointer
    // discard strsep result it's just t[8]
    // strsep is used to advance msp past the decimal separator
    strsep(&msp, ".");
    if (msp) {
        milli = strtol(msp, NULL, 10);
    }

    // for easy parsing override the null byte between t7 and t8 with a space
    *(t[8] - 1) = ' ';
    struct tm tm;
    memset(&tm, 0, sizeof(tm));
    char *parseRes = strptime(t[7], "%Y/%m/%d %H:%M:%S", &tm);
    if (parseRes) {
        mm->sysTimestamp = timegm(&tm) * 1000LL + milli;
        if (mm->sysTimestamp > now + 1 * SECONDS || mm->sysTimestamp < now - 20 * MINUTES) {
            // inhibit future and past timestamps and timegm errors (returns -1)
            mm->sysTimestamp = now;
        }
    } else {
        // record reception time as the time we read it.
        mm->sysTimestamp = now;
    }

    //fprintf(stderr, "%x type %s: ", mm->addr, t[2]);
    //fprintf(stderr, "%x: %d, %0.5f, %0.5f\n", mm->addr, mm->baro_alt, mm->decoded_lat, mm->decoded_lon);
    //field 11, callsign
    if (t[11] && strlen(t[11]) > 0) {
        strncpy(mm->callsign, t[11], 9);
        mm->callsign[8] = '\0';
        mm->callsign_valid = 1;
        for (unsigned i = 0; i < 8; ++i) {
            if (mm->callsign[i] == '\0')
                mm->callsign[i] = ' ';
            if (!(mm->callsign[i] >= 'A' && mm->callsign[i] <= 'Z') &&
                    !(mm->callsign[i] >= '0' && mm->callsign[i] <= '9') &&
                    mm->callsign[i] != ' ') {
                // Bad callsign, ignore it
                mm->callsign_valid = 0;
                break;
            }
        }
        //fprintf(stderr, "call: %s, ", mm->callsign);
    }
    // field 12, altitude
    if (t[12] && strlen(t[12]) > 0) {
        double tmp = strtod(t[12], &endptr);
        if (endptr != t[12] && isfinite(tmp)) {
            mm->baro_alt = tmp;
            mm->baro_alt_valid = 1;
            mm->baro_alt_unit = UNIT_FEET;
        } else {
            badValue = 1;
        }
        //fprintf(stderr, "alt: %d, ", mm->baro_alt);
    }
    // field 13, groundspeed
    if (t[13] && strlen(t[13]) > 0) {
        double tmp = strtod(t[13], &endptr);
        if (endptr != t[13] && isfinite(tmp)) {
            mm->gs_valid = 1;
            mm->gs.v0 = tmp;
        } else {
            badValue = 1;
        }
        //fprintf(stderr, "gs: %.1f, ", mm->gs.selected);
    }
    //field 14, heading
    if (t[14] && strlen(t[14]) > 0) {
        mm->heading = strtod(t[14], &endptr);
        if (endptr != t[14] && isfinite(mm->heading)) {
            mm->heading_valid = 1;
            mm->heading_type = HEADING_GROUND_TRACK;
        } else {
            badValue = 1;
        }
        //fprintf(stderr, "track: %.1f, ", mm->heading);
    }
    // field 15 and 16, position
    if (t[15] && strlen(t[15]) && t[16] && strlen(t[16])) {
        mm->decoded_lat = strtod(t[15], &endptr);
        char *endptr2 = NULL;
        mm->decoded_lon = strtod(t[16], &endptr2);
        if (
                endptr != t[15] && endptr2 != t[16]
                && isfinite(mm->decoded_lat) && isfinite(mm->decoded_lon)
                && mm->decoded_lat <= 90 && mm->decoded_lat >= -90
                && mm->decoded_lon >= -180 && mm->decoded_lon <= 180
           ) {
            mm->sbs_pos_valid = 1;
        } else {
            badValue = 1;
        }
        //fprintf(stderr, "pos: (%.2f, %.2f)\n", mm->decoded_lat, mm->decoded_lon);
    }
    // field 17 vertical rate, assume baro
    if (t[17] && strlen(t[17]) > 0) {
        double tmp = strtod(t[17], &endptr);
        if (endptr != t[17] && isfinite(tmp)) {
            mm->baro_rate = tmp;
            mm->baro_rate_valid = 1;
        } else {
            badValue = 1;
        }
        //fprintf(stderr, "vRate: %d, ", mm->baro_rate);
    }
    // field 18 squawk
    if (t[18] && strlen(t[18]) > 0) {
        long int tmp = strtol(t[18], &endptr, 10);
        if (endptr != t[18]) {
            mm->squawkDec = tmp;
            mm->squawkHex = squawkDec2Hex(mm->squawkDec);
            mm->squawk_valid = 1;
            //fprintf(stderr, "squawk: %04x %s, ", mm->squawkHex, t[18]);
        } else {
            badValue = 1;
        }
    }
    // field 19 (originally squawk change) used to indicate by some versions of mlat-server the number of receivers which contributed to the postiions
    if (t[19] && strlen(t[19]) > 0) {
        long int tmp = strtol(t[19], &endptr, 10);
        if (tmp > 0 && mm->source == SOURCE_MLAT) {
            mm->receiverCountMlat = tmp;
        } else if (!strcmp(t[19], "0")) {
            mm->alert_valid = 1;
            mm->alert = 0;
        } else if (!strcmp(t[19], "-1")) {
            mm->alert_valid = 1;
            mm->alert = 1;
        }
    }

    // field 20 (originally emergency status) used to indicate by some versions of mlat-server the estimated error in km
    if (t[20] && strlen(t[20]) > 0) {
        long tmp = strtol(t[20], &endptr, 10);
        if (tmp > 0 && mm->source == SOURCE_MLAT) {
            mm->mlatEPU = tmp;
            if (tmp > UINT16_MAX)
                mm->mlatEPU = UINT16_MAX;

            //fprintf(stderr, "mlatEPU: %d\n", mm->mlatEPU);
        } else if (!strcmp(t[21], "0")) {
            mm->squawk_emergency_valid = 1;
            mm->squawk_emergency = 0;
        } else if (!strcmp(t[21], "-1")) {
            mm->squawk_emergency_valid = 1;
            mm->squawk_emergency = 1;
        }
    }

    // Field 21 is the Squawk Ident flag
    if (t[21] && strlen(t[21]) > 0) {
        if (!strcmp(t[21], "1")) {
            mm->spi_valid = 1;
            mm->spi = 1;
        } else if (!strcmp(t[21], "0")) {
            mm->spi_valid = 1;
            mm->spi = 0;
        }
    }

    // field 22 ground status
    if (t[22] && strlen(t[22]) > 0) {
        if (!strncmp(t[22], "-1", 2)) {
            mm->airground = AG_GROUND;
        } else if (!strncmp(t[22], "0", 1)) {
            mm->airground = AG_AIRBORNE;
        }
        //fprintf(stderr, "onground, ");
    }


    // set nic / rc to 0 / unknown
    mm->decoded_nic = 0;
    mm->decoded_rc = RC_UNKNOWN;

    //fprintf(stderr, "\n");

    netUseMessage(mm);

    Modes.stats_current.remote_received_basestation_valid++;

    if (Modes.debug_garbage && badValue) {
        for (size_t i = 0; i < line_len; i++) {
            line[i] = (line[i] == '\0' ? ',' : line[i]);
        }
        fprintf(stderr, "SBS badValue: %.*s%s\n", (int) imin(200, line_len), line, line_len > 200 ? " [ ... ] " : "");
    }

    return 0;

basestation_invalid:

    if (Modes.debug_garbage) {
        for (size_t i = 0; i < line_len; i++) {
            line[i] = (line[i] == '\0' ? ',' : line[i]);
        }
        fprintf(stderr, "SBS invalid: %.*s%s\n", (int) imin(200, line_len), line, line_len > 200 ? " [ ... ] " : "");
    }
    Modes.stats_current.remote_received_basestation_invalid++;
    return 0;
}
//
//=========================================================================
//
// Write SBS output to TCP clients
//

void modesSendSBSOutput(struct modesMessage *mm, struct aircraft *a, struct net_writer *writer) {
    char *p;
    struct timespec now;
    struct tm stTime_receive, stTime_now;
    int msgType;

    p = prepareWrite(writer, Modes.sbs_extra_fields ? 260 : 200);
    if (!p)
        return;

    //
    // SBS BS style output checked against the following reference
    // http://www.homepages.mcb.net/bones/SBS/Article/Barebones42_Socket_Data.htm - seems comprehensive
    //
    // SBS fields:
    // MSG,3,1,1,icaoHex,1,messageDate,messageTime,currentDate,currentTime,callsign_8char,altitude_ft,groundspeed_kts,track,lat,lon,vert_rate_fpm,squawk,squawkChangeAlert,squawkEmergencyFlag,squawkIdentFlag,groundFlag_0airborne_-1ground\r\n

    if (mm->sbs_in) {
        msgType = mm->sbsMsgType;
    } else {
        // Decide on the basic SBS Message Type
        switch (mm->msgtype) {
            case 4:
            case 20:
                msgType = 5;
                break;
                break;

            case 5:
            case 21:
                msgType = 6;
                break;

            case 0:
            case 16:
                msgType = 7;
                break;

            case 11:
                msgType = 8;
                break;

            case 17:
            case 18:
                if (mm->metype >= 1 && mm->metype <= 4) {
                    msgType = 1;
                } else if (mm->metype >= 5 && mm->metype <= 8) {
                    msgType = 2;
                } else if (mm->metype >= 9 && mm->metype <= 18) {
                    msgType = 3;
                } else if (mm->metype == 19) {
                    msgType = 4;
                } else {
                    return;
                }
                break;

            default:
                return;
        }
    }

    // Fields 1 to 6 : SBS message type and ICAO address of the aircraft and some other stuff
    p += sprintf(p, "MSG,%d,1,1,%s%06X,1,", msgType, (a->addr & MODES_NON_ICAO_ADDRESS) ? "~" : "", a->addr & 0xFFFFFF);

    // Find current system time
    clock_gettime(CLOCK_REALTIME, &now);
    gmtime_r(&now.tv_sec, &stTime_now);

    // Find message reception time
    time_t received = (time_t) (mm->sysTimestamp / 1000);
    gmtime_r(&received, &stTime_receive);

    // Fields 7 & 8 are the message reception time and date
    p += sprintf(p, "%04d/%02d/%02d,", (stTime_receive.tm_year + 1900), (stTime_receive.tm_mon + 1), stTime_receive.tm_mday);
    p += sprintf(p, "%02d:%02d:%02d.%03u,", stTime_receive.tm_hour, stTime_receive.tm_min, stTime_receive.tm_sec, (unsigned) (mm->sysTimestamp % 1000));

    // Fields 9 & 10 are the current time and date
    p += sprintf(p, "%04d/%02d/%02d,", (stTime_now.tm_year + 1900), (stTime_now.tm_mon + 1), stTime_now.tm_mday);
    p += sprintf(p, "%02d:%02d:%02d.%03u", stTime_now.tm_hour, stTime_now.tm_min, stTime_now.tm_sec, (unsigned) (now.tv_nsec / 1000000U));

    // Field 11 is the callsign (if we have it)
    if (mm->callsign_valid) {
        p += sprintf(p, ",%s", mm->callsign);
    } else {
        p += sprintf(p, ",");
    }

    // Field 12 is the altitude (if we have it)
    if (Modes.use_gnss) {
        if (mm->geom_alt_valid) {
            p += sprintf(p, ",%dH", mm->geom_alt);
        } else if (mm->baro_alt_valid && trackDataValid(&a->geom_delta_valid)) {
            p += sprintf(p, ",%dH", mm->baro_alt + a->geom_delta);
        } else if (mm->baro_alt_valid) {
            p += sprintf(p, ",%d", mm->baro_alt);
        } else {
            p += sprintf(p, ",");
        }
    } else {
        if (mm->baro_alt_valid) {
            p += sprintf(p, ",%d", mm->baro_alt);
        } else if (mm->geom_alt_valid && trackDataValid(&a->geom_delta_valid)) {
            p += sprintf(p, ",%d", mm->geom_alt - a->geom_delta);
        } else {
            p += sprintf(p, ",");
        }
    }

    // Field 13 is the ground Speed (if we have it)
    if (mm->gs_valid) {
        p += sprintf(p, ",%.0f", mm->gs.selected);
    } else {
        p += sprintf(p, ",");
    }

    // Field 14 is the ground Heading (if we have it)
    if (mm->heading_valid && mm->heading_type == HEADING_GROUND_TRACK) {
        p += sprintf(p, ",%.0f", mm->heading);
    } else {
        p += sprintf(p, ",");
    }

    // Fields 15 and 16 are the Lat/Lon (if we have it)
    if (mm->cpr_decoded || mm->sbs_pos_valid) {
        p += sprintf(p, ",%1.6f,%1.6f", mm->decoded_lat, mm->decoded_lon);
    } else {
        p += sprintf(p, ",,");
    }

    // Field 17 is the VerticalRate (if we have it)
    if (Modes.use_gnss) {
        if (mm->geom_rate_valid) {
            p += sprintf(p, ",%dH", mm->geom_rate);
        } else if (mm->baro_rate_valid) {
            p += sprintf(p, ",%d", mm->baro_rate);
        } else {
            p += sprintf(p, ",");
        }
    } else {
        if (mm->baro_rate_valid) {
            p += sprintf(p, ",%d", mm->baro_rate);
        } else if (mm->geom_rate_valid) {
            p += sprintf(p, ",%d", mm->geom_rate);
        } else {
            p += sprintf(p, ",");
        }
    }

    // Field 18 is  the Squawk (if we have it)
    if (Modes.sbsOverrideSquawk != -1) {
        p += sprintf(p, ",%04d", Modes.sbsOverrideSquawk);
    } else if (mm->squawk_valid) {
        p += sprintf(p, ",%04d", mm->squawkDec);
    } else {
        p += sprintf(p, ",");
    }

    if (mm->receiverCountMlat) {
        p += sprintf(p, ",%d", mm->receiverCountMlat);
    } else if (mm->alert_valid) {
        // Field 19 is the Squawk Changing Alert flag (if we have it)
        if (mm->alert) {
            p += sprintf(p, ",-1");
        } else {
            p += sprintf(p, ",0");
        }
    } else {
        p += sprintf(p, ",");
    }

    if (mm->mlatEPU) {
        p += sprintf(p, ",%d", mm->mlatEPU);
    } else if (mm->squawk_emergency_valid) {
        // Field 20 is the Squawk Emergency flag (if we have it)
        if (mm->squawk_emergency) {
            p += sprintf(p, ",-1");
        } else {
            p += sprintf(p, ",0");
        }
    } else if (mm->squawk_valid) {
        // Field 20 is the Squawk Emergency flag (if we have it)
        if ((mm->squawkHex == 0x7500) || (mm->squawkHex == 0x7600) || (mm->squawkHex == 0x7700)) {
            p += sprintf(p, ",-1");
        } else {
            p += sprintf(p, ",0");
        }
    } else {
        p += sprintf(p, ",");
    }

    // Field 21 is the Squawk Ident flag (if we have it)
    if (mm->spi_valid) {
        if (mm->spi) {
            p += sprintf(p, ",-1");
        } else {
            p += sprintf(p, ",0");
        }
    } else {
        p += sprintf(p, ",");
    }

    // Field 22 is the OnTheGround flag (if we have it)
    switch (mm->airground) {
        case AG_GROUND:
            p += sprintf(p, ",-1");
            break;
        case AG_AIRBORNE:
            p += sprintf(p, ",0");
            break;
        default:
            p += sprintf(p, ",");
            break;
    }

    if (Modes.sbs_extra_fields) {
        // Field 23: RSSI in dBFS.
        // signalLevel is power (unsigned char / 255)^2, range [0, 1].
        // Guard: clamp to a minimum to avoid log10(0) = -inf even if signalLevel
        // is somehow a subnormal. Anything below 1e-10 is below receiver noise floor.
        if (mm->signalLevel > 0) {
            p += sprintf(p, ",%.1f", 10.0 * log10(fmax(mm->signalLevel, 1e-10)));
        } else {
            p += sprintf(p, ",");
        }

        // Field 24: aircraft category (A0-D7) from ADS-B identification message.
        // Encoded as ((0x0E - metype) << 4) | mesub; output as two hex chars.
        // Empty on message types that do not carry identification data.
        if (mm->category_valid) {
            p += sprintf(p, ",%02X", mm->category);
        } else {
            p += sprintf(p, ",");
        }
    }

    p += sprintf(p, "\r\n");

    completeWrite(writer, p);
}



