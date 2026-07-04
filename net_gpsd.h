// Part of readsb, a Mode-S/ADSB/TIS message decoder.
//
// net_gpsd.h: GPSD / Radarcape HULC protocol adapter
//
// This file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// any later version.

#ifndef NET_GPSD_H
#define NET_GPSD_H

void decodeHulcMessage(char *p);
int handle_gpsd(struct client *c, char *p, int remote, int64_t now, struct messageBuffer *mb);
void handle_radarcape_position(float lat, float lon, float alt);

#endif /* NET_GPSD_H */
