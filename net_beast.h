// Part of readsb, a Mode-S/ADSB/TIS message decoder.
//
// net_beast.h: Beast binary protocol adapter
//
// This file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// any later version.

#ifndef NET_BEAST_H
#define NET_BEAST_H

void modesSendBeastOutput(struct modesMessage *mm, struct net_writer *writer);
void modesDumpBeastData(struct modesMessage *mm);
void sendBeastSettings(int fd, const char *settings);
int handleBeastCommand(struct client *c, char *p, int remote, int64_t now, struct messageBuffer *mb);
int decodeBinMessage(struct client *c, char *p, int remote, int64_t now, struct messageBuffer *mb);
int readBeastcommand(struct client *c, int64_t now, struct messageBuffer *mb);
int readBeast(struct client *c, int64_t now, struct messageBuffer *mb);

#endif /* NET_BEAST_H */
