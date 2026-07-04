// Part of readsb, a Mode-S/ADSB/TIS message decoder.
//
// net_sbs.h: SBS/BaseStation text protocol adapter
//
// This file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// any later version.

#ifndef NET_SBS_H
#define NET_SBS_H

int decodeSbsLineMlat(struct client *c, char *line, int remote, int64_t now, struct messageBuffer *mb);
int decodeSbsLinePrio(struct client *c, char *line, int remote, int64_t now, struct messageBuffer *mb);
int decodeSbsLineJaero(struct client *c, char *line, int remote, int64_t now, struct messageBuffer *mb);
int decodeSbsLine(struct client *c, char *line, int remote, int64_t now, struct messageBuffer *mb);
void modesSendSBSOutput(struct modesMessage *mm, struct aircraft *a, struct net_writer *writer);

#endif /* NET_SBS_H */
