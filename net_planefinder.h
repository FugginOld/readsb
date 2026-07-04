// Part of readsb, a Mode-S/ADSB/TIS message decoder.
//
// net_planefinder.h: Planefinder protocol adapter
//
// This file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// any later version.

#ifndef NET_PLANEFINDER_H
#define NET_PLANEFINDER_H

int decodePfMessage(struct client *c, char *p, int remote, int64_t now, struct messageBuffer *mb);
int readPlanefinder(struct client *c, int64_t now, struct messageBuffer *mb);

#endif /* NET_PLANEFINDER_H */
