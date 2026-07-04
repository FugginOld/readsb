// Part of readsb, a Mode-S/ADSB/TIS message decoder.
//
// sdr.h: generic SDR infrastructure (header)
//
// Copyright (c) 2019 Michael Wolf <michael@mictronics.de>
//
// This code is based on a detached fork of dump1090-fa.
//
// Copyright (c) 2016-2017 Oliver Jowett <oliver@mutability.co.uk>
// Copyright (c) 2017 FlightAware LLC
//
// This file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// any later version.
//
// This file is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SDR_H
#define SDR_H

typedef enum
{
    SDR_NONE = 0, SDR_IFILE, SDR_RTLSDR, SDR_BLADERF, SDR_MICROBLADERF, SDR_HACKRF, SDR_MODESBEAST, SDR_PLUTOSDR, SDR_SOAPYSDR, SDR_GNS
} sdr_type_t;

// SDR subsystem config + runtime state. Written by CLI parsing (readsb.c) and
// sdr.c's own init/open/close paths; read from anywhere that needs to know
// which backend is active or its current gain/frequency/filter settings.
struct sdrConfig {
    sdr_type_t sdr_type; // where are we getting data from?
    uint32_t sdr_buf_size;
    uint32_t sdr_buf_samples;
    char *dev_name;
    pthread_mutex_t sdrControlMutex;
    int8_t sdrInitialized;
    int8_t sdrOpenFailed;
    int8_t increaseGain;
    int8_t lowerGain;
    int8_t autoGain;
    int8_t gainQuiet;
    int8_t gainStartup;
    char *gainArg;
    int minGain;
    int gain;
    int dc_filter; // should we apply a DC filter?
    int freq;
    float estimated_ppm;
    int32_t devel_log_ppm;
    int8_t biastee;
};
extern struct sdrConfig SdrConfig;

// Common interface to different SDR inputs.

void sdrInitConfig ();
bool sdrHandleOption (int argc, char *argv);
bool sdrOpen ();
void sdrRun ();
bool sdrHasRun();
void sdrCancel ();
void sdrClose ();
void sdrSetGain (char *reason);

void lockReader();
void unlockReader();
void wakeDecode();

#endif
