#include "readsb.h"
#include "net_planefinder.h"

int decodePfMessage(struct client *c, char *p, int remote, int64_t now, struct messageBuffer *mb) {
    MODES_NOTUSED(remote);

// Planefinder uses bit stuffing, so if we see a DLE byte, we need the next byte
#define nextByte do { \
    if (p >= c->eod) { return 0; } \
    if (*p == DLE) { p++; } \
    if (p >= c->eod) { return 0; } \
    ch = *p++; \
} while (0)

    int msgLen = 0;
    int j;
    unsigned char ch;
    struct modesMessage *mm = netGetMM(mb);
    unsigned char *msg = mm->msg;

    mm->client = c;
    mm->remote = 1;

    // Skip the DLE in the beginning
    p++;

    // Packet ID / type
    nextByte; /// Get the message type
    // This shouldn't happen because we check it in the readPlanefinder() function
    if (ch != 0xc1) {
        return 0;
    }

    // Padding
    nextByte;


    // Packet type
    nextByte;
    if (ch & 0x10) {
        // CRC: ignore field
    }
    if ((ch & 0xF) == 0) {
        if (!Modes.mode_ac) {
            return 0;
        }
        msgLen = MODEAC_MSG_BYTES;
    } else if ((ch & 0xF) == 1) {
        msgLen = MODES_SHORT_MSG_BYTES;
    } else if ((ch & 0xF) == 2) {
        msgLen = MODES_LONG_MSG_BYTES;
    } else {
        if (Modes.debug_planefinder) {
            fprintf(stderr, "Unknown message type: %d\n", ch);
        }
        return 0;
    }

    // Signal strength
    nextByte;
    mm->signalLevel = ((unsigned char) ch / 255.0);
    mm->signalLevel = mm->signalLevel * mm->signalLevel; // square it to get power

    mm->timestamp = 0;
    int64_t seconds = 0;
    for (j = 0; j < 4; j++) {
        nextByte;
        seconds = seconds << 8 | (ch & 255);
    }

    int64_t nanoseconds = 0;
    for (j = 0; j < 4; j++) {
        nextByte;
        nanoseconds = nanoseconds << 8 | (ch & 255);
    }

    if (Modes.debug_planefinder) {
        fprintf(stderr, "sec: %12lld ns: %12lld\n", (long long) seconds, (long long) nanoseconds);
    }
    mm->timestamp = seconds * 1000000000LL + nanoseconds;

    // record reception time as the time we read it.
    mm->sysTimestamp = now;

    for (j = 0; j < msgLen; j++) { // and the data
        nextByte;
        msg[j] = ch;
    }

    int result = -10;
    if (msgLen == MODEAC_MSG_BYTES) { // ModeA or ModeC
        Modes.stats_current.remote_received_modeac++;
        decodeModeAMessage(mm, ((msg[0] << 8) | msg[1]));
        result = 0;
    } else {
        Modes.stats_current.remote_received_modes++;
        result = decodeModesMessage(mm);
        if (result < 0) {
            if (result == -1) {
                Modes.stats_current.remote_rejected_unknown_icao++;
            } else {
                Modes.stats_current.remote_rejected_bad++;
            }
        } else {
            Modes.stats_current.remote_accepted[mm->correctedbits]++;
        }
    }
    if (c->unreasonable_messagerate) {
        mm->garbage = 1;
    }
    if ((Modes.garbage_ports || Modes.netReceiverId) && receiverCheckBad(mm->receiverId, now)) {
        mm->garbage = 1;
    }
    if (Modes.debug_planefinder && (Modes.mode_ac || msgLen != MODEAC_MSG_BYTES)) {
        displayModesMessage(mm);
    }

    netUseMessage(mm);
    return 0;

#undef nextByte
}

// exception decoding subroutine, return 1 for success, 0 for failure

int readPlanefinder(struct client *c, int64_t now, struct messageBuffer *mb) {
    char *p;
    unsigned char pid;

    char *start;
    char *end;

    // Scan the entire buffer, see if we can find one or more messages.
    while (c->som < c->eod && ((p = memchr(c->som, DLE, c->eod - c->som)) != NULL)) {
        end = NULL;

        // Make sure we didn't jump to a DLE that's in the middle of a message. TBD if we need this
        if (p+1 < c->eod && *(p+1) != DLE && *(p+1) != ETX) {
            // Good to go!
        } else {
            c->som = p+1;
            continue;
        }

        // Now, check if we have the end of the message in the buffer
        start = p;
        p++; // Skip start DLE
        p++; // Skip packet ID

        while (p < c->eod) {
            if (*p  == DLE) {
                // Potential message end found; it's either a DLE, ETX sequence or a DLE, DLE (the first is an escape for the second)
                if (p+1 < c->eod && *(p+1) == ETX) {
                    // We found an actual end!
                    end = p+1;
                    break;
                }
            }
            p++;
        }

        if (p >= c->eod) {
            // We reached the end of the buffer and didn't find a message. We'll call this function again when there's more data available
            return 0;
        }

        // Next time we loop through this, start from the next message
        c->som = end+1;

        // We only process messages with ID 0xc1. Others are valid, but not relevant for us
        pid = *(start+1);
        if (pid != 0xc1) {
            continue;
        }

        // Pass message to handler.
        if (decodePfMessage(c, start, c->remote, now, mb)) {
            modesCloseClient(c);
            return -1;
        }
    }
    return 0;
}


