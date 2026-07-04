// Part of readsb, a Mode-S/ADSB/TIS message decoder.
//
// net_uat.h: UAT protocol adapter
//
// This file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// any later version.

#ifndef NET_UAT_H
#define NET_UAT_H

int decodeEncapsulatedUAT(struct client *c, char *msg, int remote, int64_t now, struct messageBuffer *mb);
int decodeUatMessage(struct client *c, char *msg, int remote, int64_t now, struct messageBuffer *mb);

#endif /* NET_UAT_H */
