#include "readsb.h"
#include "net_uat.h"
#include "uat2esnt/uat2esnt.h"

static void replayUatMsg(char *msg, int msgLen) {
    char *p = prepareWrite(&Modes.uat_replay_out, msgLen + 1);
    if (!p) {
        return;
    }

    memcpy(p, msg, msgLen);
    p += msgLen;
    *p++ = '\n';
    completeWrite(&Modes.uat_replay_out, p);

    return;
}


int decodeEncapsulatedUAT(struct client *c, char *msg, int remote, int64_t now, struct messageBuffer *mb) {
    MODES_NOTUSED(remote);
    // 0x1a | 0xec | s/l/u byte (short / long / uplink) | 6 byte MLAT timestamp | rssi byte |  payload

    int debugIncomplete = 0;

    char buf[2048];
    char *out = buf;
    char *end = buf + sizeof(buf);
    char *p = msg;

    int bytes = 0;
    if (*p == 'u') {
        bytes = 552;
        out = safe_snprintf(out, end, "+");
        if (debugIncomplete) {
            fprintTimePrecise(stderr, mstime());
            fprintf(stderr, "uat uplink bytes in buffer: %d\n", (int) (c->eod - p));
        }
    } else if (*p == 's') {
        bytes = 30;
        out = safe_snprintf(out, end, "-");
    } else if (*p == 'l') {
        bytes = 48;
        out = safe_snprintf(out, end, "-");
    } else {
        return 2;
    }
    p++;

    if (c->eod - p < 6 + 1 + bytes) {
        // message is guaranteed to be incomplete
        if (debugIncomplete) {
            fprintf(stderr, "uat_incomplete %d < %d\n", (int) (c->eod - p), 1 + 6 + 1 + bytes);
        }
        return 0;
    }

    int64_t timestamp = 0;
    // Grab the timestamp (big endian format)
    for (int j = 0; j < 6; j++) {
        if (*p == 0x1a) {
            p++;
            if (*p != 0x1a) {
                // invalid message
                return -1;
            }
        }
        timestamp = timestamp << 8 | (((unsigned char) *p) & 255);
        p++;
    }


    double signalLevel = ((unsigned char) *p / 255.0);
    //fprintf(stderr, "level: %d %f\n", (unsigned char) *p, signalLevel);
    signalLevel = signalLevel * signalLevel;
    p++;

    for (int j = 0; j < bytes; j++) {
        if (p >= c->eod) {
            // incomplete message
            if (debugIncomplete) {
                fprintf(stderr, "uat_incomplete %d\n", __LINE__);
            }
            return 0;
        }
        if (*p == 0x1a) {
            p++;
            if (p >= c->eod) {
                // incomplete message
                if (debugIncomplete) {
                    fprintf(stderr, "uat_incomplete %d\n", __LINE__);
                }
                return 0;
            }
            if (*p != 0x1a) {
                // invalid message
                return -1;
            }
        }
        printHexDigit(out, *p);
        out += 2;
        p++;
    }

    out = safe_snprintf(out, end, ";");

    if (signalLevel > 1.125e-5) {
        out = safe_snprintf(out, end, "rssi=%.1f;", 10.0f * log10f(signalLevel));
    }

    //fprintf(stderr, "passing to decodeUatMessage: %s\n", buf);
    decodeUatMessage(c, buf, 1, now, mb);

    if (p - msg == 0 && debugIncomplete) {
        fprintf(stderr, "uat_incomplete %d\n", __LINE__);
    }
    return (p - msg);
}


int decodeUatMessage(struct client *c, char *msg, int remote, int64_t now, struct messageBuffer *mb) {
    MODES_NOTUSED(remote);

    int msgLen = strlen(msg);
    char *end = msg + msgLen;
    char output[8192];

    replayUatMsg(msg, msgLen);

    uat2esnt_convert_message(msg, end, output, output + sizeof(output));

    char *som = output;
    char *eod = som + strlen(som);
    char *p;

    //fprintf(stderr, "START:\n%sEND\n", som);

    while (((p = memchr(som, '\n', eod - som)) != NULL)) {
        *p = '\0';

        struct modesMessage *mm = netGetMM(mb);

        //fprintf(stderr, "AVR:%s\n", som);
        int success = decodeHexMessage(c, som, now, mm);

        if (success) {
            struct aircraft *a = aircraftGet(mm->addr);
            if (!a) { // If it's a currently unknown aircraft....
                a = aircraftCreate(mm->addr); // ., create a new record for it,
            }
            // ignore the first UAT message
            if (now > a->seen + 300 * SECONDS) {
                //fprintf(stderr, "IGNORING first UAT message from: %06x\n", a->addr);
                trackTouchSeen(a, now);
                return 0;
            }
            netUseMessage(mm);
            //displayModesMessage(mm);
        }
        som = p + 1;
    }
    return 0;
}


