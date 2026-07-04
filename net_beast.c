#include "readsb.h"
#include "net_beast.h"
#include "net_uat.h"
#include "net_gpsd.h"

// Decode a little-endian IEEE754 float (binary32)
static float ieee754_binary32_le_to_float(uint8_t *data) {
    double sign = (data[3] & 0x80) ? -1.0 : 1.0;
    int16_t raw_exponent = ((data[3] & 0x7f) << 1) | ((data[2] & 0x80) >> 7);
    uint32_t raw_significand = ((data[2] & 0x7f) << 16) | (data[1] << 8) | data[0];

    if (raw_exponent == 0) {
        if (raw_significand == 0) {
            /* -0 is treated like +0 */
            return 0;
        } else {
            /* denormal */
            return ldexp(sign * raw_significand, -126 - 23);
        }
    }

    if (raw_exponent == 255) {
        if (raw_significand == 0) {
            /* +/-infinity */
            return sign < 0 ? -INFINITY : INFINITY;
        } else {
            /* NaN */
#ifdef NAN
            return NAN;
#else
            return 0.0f;
#endif
        }
    }

    /* normalized value */
    return ldexp(sign * ((1 << 23) | raw_significand), raw_exponent - 127 - 23);
}

static inline uint32_t readPingEscaped(char *p) {
    unsigned char *pu = (unsigned char *) p;
    uint32_t res = 0;
    res += *pu++ << 16;
    if (*pu == 0x1a)
        pu++;
    res += *pu++ << 8;
    if (*pu == 0x1a)
        pu++;
    res += *pu++;
    if (*pu == 0x1a)
        pu++;
    if (0 && Modes.debug_ping)
        fprintf(stderr, "readPing: %d\n", res);
    return res;
}

static inline uint32_t readPing(char *p) {
    unsigned char *pu = (unsigned char *) p;
    uint32_t res = 0;
    res += *pu++ << 16;
    res += *pu++ << 8;
    res += *pu++;
    if (0 && Modes.debug_ping)
        fprintf(stderr, "readPing: %d\n", res);
    return res;
}


static char *netTimestamp(char *p, int64_t timestamp) {
    unsigned char ch;
    /* timestamp, big-endian */
    *p++ = (ch = (timestamp >> 40));
    if (0x1A == ch) {
        *p++ = ch;
    }
    *p++ = (ch = (timestamp >> 32));
    if (0x1A == ch) {
        *p++ = ch;
    }
    *p++ = (ch = (timestamp >> 24));
    if (0x1A == ch) {
        *p++ = ch;
    }
    *p++ = (ch = (timestamp >> 16));
    if (0x1A == ch) {
        *p++ = ch;
    }
    *p++ = (ch = (timestamp >> 8));
    if (0x1A == ch) {
        *p++ = ch;
    }
    *p++ = (ch = (timestamp));
    if (0x1A == ch) {
        *p++ = ch;
    }
    return p;
}

//
//=========================================================================
//
// Write raw output in Beast Binary format with Timestamp to TCP clients
//

void modesSendBeastOutput(struct modesMessage *mm, struct net_writer *writer) {
    int msgLen = mm->msgbits / 8;
    // 0x1a 0xe3 receiverId(2*8) 0x1a msgType timestamp+signal(2*7) message(2*msgLen)
    char *p = prepareWrite(writer, (2 + 2 * 8 + 2 + 2 * 7) + 2 * msgLen);
    unsigned char ch;
    int j;
    int sig;
    unsigned char *msg = (Modes.net_verbatim ? mm->verbatim : mm->msg);

    if (!p)
        return;

    // receiverId, big-endian, in own message to make it backwards compatible
    // only send the receiverId when it changes
    if (Modes.netReceiverId && writer->lastReceiverId != mm->receiverId) {
        writer->lastReceiverId = mm->receiverId;
        *p++ = 0x1a;
        // other dump1090 / readsb versions or beast implementations should discard unknown message types
        *p++ = 0xe3; // good enough guess no one is using this.
        for (int i = 7; i >= 0; i--) {
            *p++ = (ch = ((mm->receiverId >> (8 * i)) & 0xFF));
            if (0x1A == ch) {
                *p++ = ch;
            }
        }
    }

    *p++ = 0x1a;
    if (msgLen == MODES_SHORT_MSG_BYTES) {
        *p++ = '2';
    } else if (msgLen == MODES_LONG_MSG_BYTES) {
        *p++ = '3';
    } else if (msgLen == MODEAC_MSG_BYTES) {
        *p++ = '1';
    } else {
        return;
    }

    /* timestamp, big-endian */
    p = netTimestamp(p, mm->timestamp);

    sig = nearbyint(sqrt(mm->signalLevel) * 255);
    if (mm->signalLevel > 0 && sig < 1)
        sig = 1;
    if (sig > 255)
        sig = 255;
    *p++ = ch = (char) sig;
    if (0x1A == ch) {
        *p++ = ch;
    }

    for (j = 0; j < msgLen; j++) {
        *p++ = (ch = msg[j]);
        if (0x1A == ch) {
            *p++ = ch;
        }
    }

    completeWrite(writer, p);
}


void modesDumpBeastData(struct modesMessage *mm) {
    if (!Modes.dump_fw) {
        return;
    }
    int msgLen = mm->msgbits / 8;
    // 0x1a 0xe3 receiverId(2*8) 0x1a msgType timestamp+signal(2*7) message(2*msgLen)
    char store[(2 + 2 * 8 + 2 + 2 * 7) + 2 * MODES_LONG_MSG_BYTES];
    char *p = store;
    unsigned char ch;
    int j;
    int sig;
    unsigned char *msg = (Modes.net_verbatim ? mm->verbatim : mm->msg);

    char *start = p;

    // receiverId, big-endian, in own message to make it backwards compatible
    // only send the receiverId when it changes
    if (Modes.netReceiverId && Modes.dump_lastReceiverId != mm->receiverId) {
        Modes.dump_lastReceiverId = mm->receiverId;
        *p++ = 0x1a;
        // other dump1090 / readsb versions or beast implementations should discard unknown message types
        *p++ = 0xe3; // good enough guess no one is using this.
        for (int i = 7; i >= 0; i--) {
            *p++ = (ch = ((mm->receiverId >> (8 * i)) & 0xFF));
            if (0x1A == ch) {
                *p++ = ch;
            }
        }
    }

    *p++ = 0x1a;
    if (msgLen == MODES_SHORT_MSG_BYTES) {
        *p++ = '2';
    } else if (msgLen == MODES_LONG_MSG_BYTES) {
        *p++ = '3';
    } else if (msgLen == MODEAC_MSG_BYTES) {
        *p++ = '1';
    } else {
        return;
    }

    /* timestamp, big-endian */
    if (Modes.dump_reduce && mm->timestamp && !(mm->timestamp >= MAGIC_MLAT_TIMESTAMP && mm->timestamp <= MAGIC_MLAT_TIMESTAMP + 10)) {
        // clobber timestamp for better compression
        p = netTimestamp(p, MAGIC_ANY_TIMESTAMP);
    } else {
        p = netTimestamp(p, mm->timestamp);
    }

    sig = nearbyint(sqrt(mm->signalLevel) * 255);
    if (mm->signalLevel > 0 && sig < 1)
        sig = 1;
    if (sig > 255)
        sig = 255;
    *p++ = ch = (char) sig;
    if (0x1A == ch) {
        *p++ = ch;
    }

    for (j = 0; j < msgLen; j++) {
        *p++ = (ch = msg[j]);
        if (0x1A == ch) {
            *p++ = ch;
        }
    }

    int64_t now = mm->sysTimestamp;
    if (now > Modes.dump_next_ts) {
        //fprintf(stderr, "%ld\n", (long) now);
        Modes.dump_next_ts = now + 1;
        const char dump_ts_prefix[] = { 0x1A, 0xe8 };
        zstdFwPutData(Modes.dump_fw, (uint8_t *) dump_ts_prefix, sizeof(dump_ts_prefix));
        zstdFwPutData(Modes.dump_fw, (uint8_t *) &now, sizeof(int64_t));
    }

    zstdFwPutData(Modes.dump_fw, (uint8_t *) start, p - start);
}


void sendBeastSettings(int fd, const char *settings) {
    int len;
    char *buf, *p;

    len = strlen(settings) * 3;
    buf = p = alloca(len);

    while (*settings) {
        *p++ = 0x1a;
        *p++ = '1';
        *p++ = *settings++;
    }

    anetWrite(fd, buf, len);
}


int handleBeastCommand(struct client *c, char *p, int remote, int64_t now, struct messageBuffer *mb) {
    MODES_NOTUSED(remote);
    MODES_NOTUSED(now);
    MODES_NOTUSED(mb);
    if (p[0] == 'P') {
        // got ping
        c->ping = readPingEscaped(p+1);
        c->pingReceived = now;
        c->pingEnabled = 1;
        if (0 && Modes.debug_ping)
            fprintf(stderr, "Got Ping: %d\n", c->ping);
    } else if (p[0] == '1') {
        switch (p[1]) {
            case 'j':
                c->modeac_requested = 0;
                break;
            case 'J':
                c->modeac_requested = 1;
                break;
        }
        autoset_modeac();
    } else if (p[0] == 'W') {
        switch (p[1]) {
            case 'S':
                dropHalfUntil(now, c, now + PING_REDUCE_DURATION);
                break;
        }
    }
    return 0;
}

//
//=========================================================================
//
// This function decodes a Beast binary format message
//
// The message is passed to the higher level layers, so it feeds
// the selected screen output, the network output and so forth.
//
// If the message looks invalid it is silently discarded.
//
// The function always returns 0 (success) to the caller as there is no
// case where we want broken messages here to close the client connection.
//
// to save a couple cycles we remove the escapes in the calling function and expect nonescaped messages here

int decodeBinMessage(struct client *c, char *p, int remote, int64_t now, struct messageBuffer *mb) {
    uint16_t msgLen = 0;
    int j;
    unsigned char ch;
    struct modesMessage *mm = netGetMM(mb);
    unsigned char *msg = mm->msg;

    mm->client = c;

    ch = *p++; /// Get the message type


    mm->receiverId = c->receiverId;
    if (unlikely(Modes.incrementId)) {
        mm->receiverId += now / (10 * MINUTES);
    }

    if (ch == '2') {
        msgLen = MODES_SHORT_MSG_BYTES;
    } else if (ch == '3') {
        msgLen = MODES_LONG_MSG_BYTES;
    } else if (ch == '1') {
        if (!Modes.mode_ac) {
            if (remote) {
                Modes.stats_current.remote_received_modeac++;
            } else {
                Modes.stats_current.demod_modeac++;
            }
            return 0;
        }
        msgLen = MODEAC_MSG_BYTES;
    } else if (ch == '5') {
        // Special case for Radarcape position messages.
        float lat, lon, alt;
        unsigned char msg[21];
        for (j = 0; j < 21; j++) { // and the data
            msg[j] = ch = *p++;
        }

        lat = ieee754_binary32_le_to_float(msg + 4);
        lon = ieee754_binary32_le_to_float(msg + 8);
        alt = ieee754_binary32_le_to_float(msg + 12);

        handle_radarcape_position(lat, lon, alt);
        return 0;
    } else if (ch == 'H') {
        decodeHulcMessage(p);
        return 0;
    } else if (ch == 'P') {
        // pong message
        // only accept pong message if not ingest or client ping "enabled"
        if (!Modes.netIngest || c->pingEnabled) {
            c->pong = readPing(p);
            return pongReceived(c, now);
        }
        return 0;
    } else {
        // unknown msg type
        return 0;
    }

    /* Beast messages are marked depending on their source. From internet they are marked
     * remote so that we don't try to pass them off as being received by this instance
     * when forwarding them.
     */
    mm->remote = remote;

    mm->timestamp = 0;
    // Grab the timestamp (big endian format)
    for (j = 0; j < 6; j++) {
        ch = *p++;
        mm->timestamp = mm->timestamp << 8 | (ch & 255);
    }

    // record reception time as the time we read it.
    mm->sysTimestamp = now;
    //fprintf(stderr, "epoch: %.6f\n", mm->sysTimestamp / 1000.0);


    ch = *p++; // Grab the signal level
    mm->signalLevel = ((unsigned char) ch / 255.0);
    mm->signalLevel = mm->signalLevel * mm->signalLevel;

    /* In case of Mode-S Beast use the signal level per message for statistics */
    if (c == Modes.serial_client) {
        Modes.stats_current.signal_power_sum += mm->signalLevel;
        Modes.stats_current.signal_power_count += 1;

        if (mm->signalLevel > Modes.stats_current.peak_signal_power)
            Modes.stats_current.peak_signal_power = mm->signalLevel;
        if (mm->signalLevel > 0.50119)
            Modes.stats_current.strong_signal_count++; // signal power above -3dBFS
    }

    for (j = 0; j < msgLen; j++) { // and the data
        msg[j] = ch = *p++;
    }

    int result = -10;
    if (msgLen == MODEAC_MSG_BYTES) { // ModeA or ModeC
        if (remote) {
            Modes.stats_current.remote_received_modeac++;
        } else {
            Modes.stats_current.demod_modeac++;
        }
        decodeModeAMessage(mm, ((msg[0] << 8) | msg[1]));
        result = 0;
    } else {
        if (remote) {
            Modes.stats_current.remote_received_modes++;
        } else {
            Modes.stats_current.demod_preambles++;
        }
        result = decodeModesMessage(mm);
        if (result < 0) {
            if (result == -1) {
                if (remote) {
                    Modes.stats_current.remote_rejected_unknown_icao++;
                } else {
                    Modes.stats_current.demod_rejected_unknown_icao++;
                }
            } else {
                if (remote) {
                    Modes.stats_current.remote_rejected_bad++;
                } else {
                    Modes.stats_current.demod_rejected_bad++;
                }
            }
        } else {
            if (remote) {
                Modes.stats_current.remote_accepted[mm->correctedbits]++;
            } else {
                Modes.stats_current.demod_accepted[mm->correctedbits]++;
            }
        }
    }
    if (c->pongReceived && c->pongReceived > now + 100) {
        // if messages are received with more than 100 ms delay after a pong, recalculate c->rtt
        pongReceived(c, now);
    }
    if (c->rtt > Modes.ping_reject && Modes.netIngest) {
        // don't discard CPRs, if we have better data speed_check generally will take care of delayed CPR messages
        // this way we get basic data even from high latency receivers
        // super high latency receivers are getting disconnected in pongReceived()
        if (!mm->cpr_valid) {
            Modes.stats_current.remote_rejected_delayed++;
            return 0; // discard
        }
    }
    if (c->unreasonable_messagerate) {
        mm->garbage = 1;
    }
    if ((Modes.garbage_ports || Modes.netReceiverId) && receiverCheckBad(mm->receiverId, now)) {
        mm->garbage = 1;
    }

    netUseMessage(mm);
    return 0;
}


//
//
//=========================================================================
//
// This function decodes a planefinder binary format message
//
// The message is passed to the higher level layers, so it feeds
// the selected screen output, the network output and so forth.
//
// If the message looks invalid it is silently discarded.
//
// The function always returns 0 (success) to the caller as there is no
// case where we want broken messages here to close the client connection.
//
// For packet ID 0x41, the format is:
// Byte     Value       Notes
// 0        <DLE>
// 1        ID          0x41
// 2        padding     always 0
// 3        byte        the lower 4 bits map to: 0 = mode AC, 1 = mode S short, 2 = mode S long. Bit 4 indicates CRC. 5-7 is undefined in the spec I received, although bit 5 is in use (it can be ignored).
// 4        byte        signal strength
// 5-8      long        epoch time
// 9-12     long        nanoseconds
// 13-27    byte        data, mode AC/S

int readBeastcommand(struct client *c, int64_t now, struct messageBuffer *mb) {
    char *p;

    while (c->som < c->eod && ((p = memchr(c->som, (char) 0x1a, c->eod - c->som)) != NULL)) { // The first byte of buffer 'should' be 0x1a
        char *eom; // one byte past end of message

        c->som = p; // consume garbage up to the 0x1a
        ++p; // skip 0x1a

        if (p >= c->eod) {
            // Incomplete message in buffer, retry later
            break;
        }

        if (*p == '1') {
            eom = p + 2;
        } else if (*p == 'W') { // W command
            eom = p + 2;
        } else if (*p == 'P') { // ping from the receiver
            eom = p + 4;
        } else {
            // Not a valid beast command, skip 0x1a and try again
            ++c->som;
            continue;
        }

        // we need to be careful of double escape characters in the message body
        for (p = c->som + 1; p < c->eod && p < eom; p++) {
            if (0x1A == *p) {
                p++;
                eom++;
            }
        }

        if (eom > c->eod) { // Incomplete message in buffer, retry later
            break;
        }

        char *start = c->som + 1;

        // advance to next message
        c->som = eom;

        // Pass message to handler.
        if (c->service->read_handler(c, start, c->remote, now, mb)) {
            modesCloseClient(c);
            return -1;
        }
    }
    return 0;
}


int readBeast(struct client *c, int64_t now, struct messageBuffer *mb) {
    // This is the Beast Binary scanning case.
    // If there is a complete message still in the buffer, there must be the separator 'sep'
    // in the buffer, note that we full-scan the buffer at every read for simplicity.

    char *p;

    //fprintf(stderr, "readBeast\n");

    if (0 && c->orphaned_bytes && now - c->orphaned_bytes_ts > 500) {
        fprintTimePrecise(stderr, now);
        fprintf(stderr, " orphaned bytes: %5d orphaned for: %8.3f\n", c->orphaned_bytes, (now - c->orphaned_bytes_ts) / 1000.0);
    }

    while (c->som < c->eod && ((p = memchr(c->som, (char) 0x1a, c->eod - c->som)) != NULL)) { // The first byte of buffer 'should' be 0x1a

        if (p > c->som) {
            garbageIncrement(c, p - c->som, __LINE__);
        }
        Modes.stats_current.remote_malformed_beast += p - c->som;

        //lastSom = p;
        c->som = p; // consume garbage up to the 0x1a
        ++p; // skip 0x1a

        if (p >= c->eod) {
            // Incomplete message in buffer, retry later
            break;
        }

        char *eom; // one byte past end of message
        unsigned char ch;

        if (!c->service) { fprintf(stderr, "c->service null ohThee9u\n"); }


        if (Modes.synthetic_now) {
            now = Modes.synthetic_now;
            Modes.syntethic_now_suppress_errors = 0;
        }

        ch = *p;
        if (ch == 0xe8) {
            // message with synthetic timestamp from --dump-beast file prepended
            p++;

            int64_t ts;
            if (p + sizeof(int64_t) > c->eod) {
                break;
            }

            memcpy(&ts, p, sizeof(int64_t));
            p += sizeof(int64_t);

            int64_t old_now = now;

            if (Modes.dump_accept_synthetic_now) {
                now = Modes.synthetic_now = ts;
            } else if (Modes.dump_ignore_synthetic_now) {
                // do nothing
            } else {
                static int64_t antiSpam;
                if (now > antiSpam) {
                    antiSpam = now + 30 * SECONDS;
                    char sample[256];
                    hexDumpString(c->som, c->eod - c->som, sample, sizeof(sample));
                    sample[sizeof(sample) - 1] = '\0';

                    fprintf(stderr, "%s: Synthetic timestamp detected without --devel=accept_synthetic"
                            " or --devel=ignore_synthetic specified, disconnecting client: %s port %s,"
                            " hexdump of data containing 0x1A 0xE8: %s\n",
                            c->service->descr, c->host, c->port, sample);
                }

                if (Modes.netIngest) {
                    modesCloseClient(c);
                    return -1;
                }
            }

            //fprintf(stderr, "%ld %ld\n", (long) now, (long) (c->eod - c->som));

            c->som = p; // set start of next message
            if (*p != 0x1A) {
                //fprintf(stderr, "..\n");
                c->som++;
                continue;
            }
            p++; // skip 0x1a
            if (p >= c->eod) {
                // Incomplete message in buffer, retry later
                break;
            }

            if (Modes.synthetic_now) {
                if (priorityTasksPending()) {
                    if (now - old_now > 5 * SECONDS) {
                        Modes.syntethic_now_suppress_errors = 1;
                    }
                    pthread_mutex_unlock(&Threads.decode.mutex);
                    priorityTasksRun();
                    pthread_mutex_lock(&Threads.decode.mutex);
                    Modes.syntethic_now_suppress_errors = 0;
                }
            }
        } else if (ch == 0xe3) {
            // message with receiverId prepended
            p++;
            uint64_t receiverId = 0;
            eom = p + 8;
            // we need to be careful of double escape characters in the receiverId
            for (int j = 0; j < 8 && p < c->eod && p < eom; j++) {
                ch = *p++;
                if (ch == 0x1A) {
                    ch = *p++;
                    eom++;
                    if (p < c->eod && ch != 0x1A) { // check that it's indeed a double escape
                                                 // might be start of message rather than double escape.
                        garbageIncrement(c, p - 1 - c->som, __LINE__);
                        Modes.stats_current.remote_malformed_beast += p - 1 - c->som;
                        c->som = p - 1;
                        goto beastWhileContinue;
                    }
                }
                // Grab the receiver id (big endian format)
                receiverId = receiverId << 8 | (ch & 255);
            }

            if (eom + 2 > c->eod)// Incomplete message in buffer, retry later
                break;

            if (!Modes.netIngest) {
                c->receiverId = receiverId;
            }

            c->som = p; // set start of next message
            if (*p != 0x1A) {
                continue;
            }
            p++; // skip 0x1a
            if (p >= c->eod) {
                // Incomplete message in buffer, retry later
                break;
            }
        }

        if (!c->service) { fprintf(stderr, "c->service null waevem0E\n"); }

        ch = *p;
        if (ch == '2') {
            eom = p + 1 + 6 + 1 + MODES_SHORT_MSG_BYTES;
        } else if (ch == '3') {
            eom = p + 1 + 6 + 1 + MODES_LONG_MSG_BYTES;
        } else if (ch == '1') {
            eom = p + 1 + 6 + 1 + MODEAC_MSG_BYTES;
            if (0) {
                char sample[256];
                char *sampleStart = c->som - 32;
                if (sampleStart < c->buf)
                    sampleStart = c->buf;
                *c->som = 'X';
                hexDumpString(sampleStart, c->eod - sampleStart, sample, sizeof(sample));
                *c->som = 0x1a;
                sample[sizeof(sample) - 1] = '\0';
                fprintf(stderr, "modeAC: som pos %d, sample %s, eom > c->eod %d\n", (int) (c->som - c->buf), sample, eom > c->eod);
            }
        } else if (ch == '5') {
            eom = p + MODES_LONG_MSG_BYTES + 8;
        } else if (ch == 0xeb) {
            // encapsulated UAT message, variable length but guaranteed not to have 0x1a because
            // it's only readable ascii chars
            eom = memchr(p, '\n', c->eod - p);
            if (!eom) {
                if (memchr(p, (char) 0x1A, c->eod - p)) {
                    // malformed message, skip
                    c->som++;
                    continue;
                } else {
                    // incomplete message, wait for rest to arrive
                    break;
                }
            }
            *eom = '\0';
            p++;
            decodeUatMessage(c, p, 1, now, mb);
            c->som = eom;
            continue;
        } else if (ch == 0xec) {
            // 0x1a | 0xec | s/l/u byte (short / long / uplink) | 6 byte MLAT timestamp | rssi byte |  payload
            p++;
            if (*p == 's' || *p == 'l' || *p == 'u') {

                if (0 && *p != 'u') {
                    char sample[512];
                    int len = (*p == 's') ? 30 : 48;
                    len += 3 + 6 + 1 + 2;

                    if (len > c->eod - c->som) {
                        len = c->eod - c->som;
                    }
                    fprintf(stderr, "len: %d\n", len);

                    hexDumpString(c->som, len, sample, sizeof(sample));
                    sample[sizeof(sample) - 1] = '\0';
                    fprintf(stderr, "Binary message: %s\n", sample);
                }

                int res = decodeEncapsulatedUAT(c, p, 1, now, mb);
                if (res == 0) {
                    // return of 0 means the message was incomplete, wait for rest to arrive
                    break;
                } else if (res == -1) {
                    // message was malformed
                    c->som++;
                    continue;
                } else {
                    // message good, advance by message length as returned by function
                    c->som = p + res;
                    continue;
                }
            } else {
                // malformed message, skip
                c->som++;
                continue;
            }
        } else if (ch == 0xe4) {
            p++;
            if (c->eod - p < 36) {
                if (!memchr(p, (char) 0x1A, c->eod - p)) {
                    // incomplete uuid
                    break;
                } else {
                    // malformed uuid
                    continue;
                }
            }
            // read UUID and continue with next message
            c->som = read_uuid(c, p, c->eod);
            continue;
        } else if (ch == 'P') {
            //unsigned char *pu = (unsigned char*) p;
            //fprintf(stderr, "%x %x %x %x %x\n", pu[0], pu[1], pu[2], pu[3], pu[4]);
            eom = p + 4;
        } else if (ch == 'W') {
            // read command
            p++;
            ch = *p;
            if (ch == 'O') {
                // O for high resolution timer, both P and p already used for previous iterations
                // explicitely enable ping for this client
                c->pingEnabled = 1;
                uint32_t newPing = now & ((1 << 24) - 1);
                if (Modes.debug_ping)
                    fprintf(stderr, "Initial Ping: %d\n", newPing);
                pingClient(c, newPing);
                if (!c->service) {
                    fprintf(stderr, "c->service null Ieseey5s\n");
                    return -1;
                }
                if (flushClient(c, now) < 0) {
                    return -1;
                }
                if (!c->service) {
                    fprintf(stderr, "c->service null EshaeC7n\n");
                    return -1;
                }
            }
            c->som += 2;
            continue;
        } else {
            // Not a valid beast message, skip 0x1a
            // Skip following byte as well:
            // either: 0x1a (likely not a start of message but rather escaped 0x1a)
            // or: any other char is skipped anyhow when looking for the next 0x1a
            c->som += 2;
            Modes.stats_current.remote_malformed_beast += 2;
            garbageIncrement(c, 2, __LINE__);
            continue;
        }

        if (!c->service) { fprintf(stderr, "c->service null quooJ1ea\n"); return -1; }

        if (eom > c->eod) // Incomplete message in buffer, retry later
            break;

        char noEscapeStorage[MODES_LONG_MSG_BYTES + 8 + 16]; // 16 extra for good measure
        char *noEscape = p;

        // we need to be careful of double escape characters in the message body
        if (memchr(p, (char) 0x1A, eom - p)) {
            char *t = noEscapeStorage;
            while (p < eom) {
                if (*p == (char) 0x1A) {
                    p++;
                    eom++;
                    if (eom > c->eod) { // Incomplete message in buffer, retry later
                        break;
                    }
                    if (*p != (char) 0x1A) { // check that it's indeed a double escape
                                             // might be start of message rather than double escape.
                                             //
                        garbageIncrement(c, p - 1 - c->som, __LINE__);
                        Modes.stats_current.remote_malformed_beast += p - 1 - c->som;
                        c->som = p - 1;

                        if (0) {
                            char sample[256];
                            char *sampleStart = c->som - 32;
                            if (sampleStart < c->buf)
                                sampleStart = c->buf;
                            *c->som = 'X';
                            hexDumpString(sampleStart, c->eod - sampleStart, sample, sizeof(sample));
                            *c->som = 0x1a;
                            sample[sizeof(sample) - 1] = '\0';
                            fprintf(stderr, "not a double Escape: som pos %d, sample %s, eom - som %d\n", (int) (c->som - c->buf), sample, (int) (eom - c->som));

                        }

                        goto beastWhileContinue;
                    }
                }
                *t++ = *p++;
            }
            noEscape = noEscapeStorage;
        }

        if (eom > c->eod) // Incomplete message in buffer, retry later
            break;

        if (!c->service) {
            fprintf(stderr, "c->service null hahGh1Sh\n");
            return -1;
        }

        // if we get some valid data, reduce the garbage counter.
        if (c->garbage > 128)
            c->garbage -= 128;

        // advance to next message
        c->som = eom;

        // Have a 0x1a followed by 1/2/3/4/5 - pass message to handler.
        int res = c->service->read_handler(c, noEscape, c->remote, now, mb);

        if (!c->service) {
            return -1;
        }
        if (res) {
            modesCloseClient(c);
            return -1;
        }


beastWhileContinue:
        ;
    }

    if (0 && c->eod - c->som > 256) {
        fprintf(stderr, "beastWhile >256 remaining: %d\n", (int) (c->eod - c->som));
    }
    if (c->eod - c->som > 600) {
        //fprintf(stderr, "beastWhile too much data remaining, garbage?!\n");
        garbageIncrement(c, c->eod - c->som, __LINE__);
        Modes.stats_current.remote_malformed_beast += c->eod - c->som;
        c->som = c->eod;
    }

    if (c->eod - c->som > 0) {
        //fprintf(stderr, " bytes still in buffer: %d\n", (int) (c->eod - c->som));
    }
    c->orphaned_bytes_ts = now;
    c->orphaned_bytes = c->eod - c->som;

    return 0;
}


