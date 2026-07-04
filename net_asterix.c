#include "readsb.h"
#include "net_asterix.h"
#include "ais_charset.h"

#define FSPEC_MAX 24

static uint8_t char_to_ais(int ch)
{
    char *match;
    if (!ch)
        return 32;

    match = strchr(ais_charset, ch);
    if (match)
        return (uint8_t)(match - ais_charset);
    else
        return 32;
}


static void readFspec(uint8_t *fspec, char **p, char *end){
    memset(fspec, 0x0, FSPEC_MAX);

    if (*p >= end) {
        return;
    }

    fspec[0] = **p;
    (*p)++;

    for (int i = 1; i < FSPEC_MAX && *p < end && *(*p - 1) & 0x1; i++){
        fspec[i] = *(*p);
        (*p)++;
    }
}

//
// Read Asterix Time
//

static uint64_t readAsterixTime(char **p) {
    int rawtime = ((*(*p) & 0xff) << 16) + ((*(*p + 1) & 0xff) << 8) + (*(*p + 2) & 0xff);
    int mssm = (int)(rawtime / .128);
    long midnight = (long)(mstime() / 86400000) * 86400000;
    int diff = (int)(midnight + mssm - mstime());
    (*p) += 3;
    if (abs(diff) > 43200000){
    	return midnight - 86400000 + mssm;
    }
    return midnight + mssm;
}

//
// Read Asterix High Precision Time
//

static void readAsterixHighPrecisionTime(uint64_t *timeStamp, char **p) {
    uint8_t fsi = (**p & 0xc0) >> 6;
    double offset = ((**p & 0x3f) << 24) + ((*(*p + 1) & 0xff) << 16) + ((*(*p + 2) & 0xff) << 8) + (*(*p + 3) & 0xff);
    (*p) += 4;
    offset = offset * pow(2, -27);
    uint64_t wholesecond = (int)(*timeStamp / 1000) * 1000;
    switch(fsi)
    {
    	case 1:
	    wholesecond += 1;
	    break;
	case 2:
	    wholesecond -= 1;
	    break;
    }
    *timeStamp = wholesecond + offset;
}

//
//=========================================================================
//
// Read ASTERIX input from TCP clients
//

int decodeAsterixMessage(struct client *c, char *p, int remote, int64_t now, struct messageBuffer *mb) {
    //uint16_t msgLen = (*(p + 1) << 8) + *(p + 2);
    //int j;
    unsigned char category;
    struct modesMessage *mm = netGetMM(mb);
    mm->client = c;
    MODES_NOTUSED(c);
    if (remote >= 64)
        mm->source = remote - 64;
    else
        mm->source = SOURCE_INDIRECT;
    mm->remote = 1;
    mm->sbs_in = 1;
    mm->signalLevel = 0;
    category = *p; // Get the category
    p += 3;

    uint8_t fspec[FSPEC_MAX];
    readFspec(fspec, &p, c->eod);

    mm->receiverId = c->receiverId;
    if (unlikely(Modes.incrementId)) {
        mm->receiverId += now / (10 * MINUTES);
    }
    mm->sysTimestamp = -1;
    switch(category){
        case 21: // ADS-B Message
            if(!(fspec[1] & 0x10)){ // no address. this is useless to us
                return -1;
            }
            if (fspec[0] & 0x80){ // ID021/010 Data Source Identification
                p += 2;
            }
            uint8_t addrtype = 3;
            if (fspec[0] & 0x40){ // ID021/040 Target Report Descriptor
                uint8_t trd[FSPEC_MAX];
                readFspec(trd, &p, c->eod);
                addrtype = (trd[0] & 0xE0) >> 5;
                if (!(trd[0] & 0x18)){
                    mm->alt_q_bit = 1;
                }
                if (trd[1] & 0x40){
                    mm->airground = AG_GROUND;
                }
                else {
                    mm->airground = AG_AIRBORNE;
                }
            }
            if (fspec[0] & 0x20){ // I021/161 Track Number
                p += 2;
            }
            if (fspec[0] & 0x10){ // I021/015 Service Identification
                p += 1;
            }
            if (fspec[0] & 0x8){ // I021/071 Time of Applicability for Position 3
                mm->sysTimestamp = readAsterixTime(&p);
            }
            if (fspec[0] & 0x4){ // I021/130 Position in WGS-84 co-ordinates
                int lat = (*p & 0xff) << 16;
                lat += (*(p + 1) & 0xff) << 8;
                lat += (*(p + 2) & 0xff);
                p += 3;
                int lon = (*p & 0xff) << 16;
                lon += (*(p + 1) & 0xff) << 8;
                lon += (*(p + 2) & 0xff);
                p += 3;
                if (lat >= 0x800000){
                    lat -= 0x1000000;
                }
                if (lon >= 0x800000){
                    lon -= 0x1000000;
                }
                double latitude = lat * (180 / pow(2, 23));
                double longitude = lon * (180 / pow(2, 23));
                if (latitude <= 90 && latitude >= -90 && longitude >= -180 && longitude <= 180){
                    mm->sbs_pos_valid = true;
                    mm->decoded_lat = latitude;
                    mm->decoded_lon = longitude;
                }
            }
            if (fspec[0] & 0x2){ // I021/131 Position in WGS-84 co-ordinates, high res.
                int lat = (*p & 0xff) << 24;
                lat += (*(p + 1) & 0xff) << 16;
                lat += (*(p + 2) & 0xff) << 8;
                lat += (*(p + 3) & 0xff);
                p += 4;
                int lon = *p << 24;
                lon += (*(p + 1) & 0xff) << 16;
                lon += (*(p + 2) & 0xff) << 8;
                lon += (*(p + 3) & 0xff);
                p += 4;
                double latitude = lat * (180 / pow(2, 30));
                double longitude = lon * (180 / pow(2, 30));
                if (latitude <= 90 && latitude >= -90 && longitude >= -180 && longitude <= 180){
                    mm->sbs_pos_valid = true;
                    mm->decoded_lat = latitude;
                    mm->decoded_lon = longitude;
                }
            }
            if (fspec[1] & 0x80){ // I021/072 Time of Applicability for Velocity
                if (mm->sysTimestamp == -1){
                    mm->sysTimestamp = readAsterixTime(&p);
                }
                else {
                    p += 3;
                }
            }
            if (fspec[1] & 0x40){ // I021/150 Air Speed
                uint16_t raw_speed = (*p & 0x7f) << 8;
                raw_speed += *(p + 1) & 0xff;
                if (*p & 0x80){ //Mach
                    mm->mach = raw_speed * 0.001;
                    mm->mach_valid = true;
                }
                else{ // IAS
                    mm->ias = (raw_speed * pow(2, -14)) * 3600;
                    mm->ias_valid = true;
                }
                p += 2;
            }
            if (fspec[1] & 0x20){ // I021/151 True Airspeed
                uint16_t raw_speed = (*p & 0x7f) << 8;
                raw_speed += *(p + 1) & 0xff;
                if (!(*p & 0x80)){
                    mm->tas_valid = true;
                    mm->tas = raw_speed;
                }
                p += 2;
            }
            // I021/080 Target Address
            mm->addr = (((*p & 0xff) << 16) + ((*(p + 1) & 0xff) << 8) + (*(p + 2) & 0xff)) & 0xffffff;
            if (addrtype == 3){
                mm->addr |= MODES_NON_ICAO_ADDRESS;
            }
            p += 3;
            if (fspec[1] & 0x8){ // I021/073 Time of Message Reception of Position
                if (mm->cpr_decoded || mm->sbs_pos_valid){
                    uint64_t ts = readAsterixTime(&p);
                    if (fspec[1] & 0x4){ // I021/074 Time of Message Reception of Position=High Precision
                        readAsterixHighPrecisionTime(&ts, &p);
                    }
                    if (mm->sysTimestamp == -1){
                        mm->sysTimestamp = ts;
                    }
                }
                else if (fspec[1] & 0x4) {
                    p += 7;
                }
                else {
                    p += 3;
                }
            }
            if (fspec[1] & 0x2){ // I021/075 Time of Message Reception of Velocity
                if (mm->ias_valid || mm->mach_valid || mm->gs_valid){
                    uint64_t ts = readAsterixTime(&p);
                    if (fspec[2] & 0x80){ // I021/076 Time of Message Reception of Velocity=High Precision
                        readAsterixHighPrecisionTime(&ts, &p);
                    }
                    if (mm->sysTimestamp == -1){
                        mm->sysTimestamp = ts;
                    }
                }
                else if (fspec[2] & 0x80) {
                    p += 7;
                }
                else {
                    p += 3;
                }
            }
            if (fspec[2] & 0x40){ // I021/140 Geometric Height
                int16_t raw_alt = (((*p & 0xff) << 8) + (*(p + 1) & 0xff));
                double alt = raw_alt * 6.25;
                if (alt >= -1500 && alt <= 150000){
                    mm->geom_alt_valid = true;
                    mm->geom_alt_unit = UNIT_FEET;
                    mm->geom_alt = alt;
                }
                p += 2;
            }
            uint8_t qi[FSPEC_MAX];
            //uint8_t nucp_or_nic;
            uint8_t nucr_or_nacv;
            uint8_t nicbaro = 0;
            uint8_t sil;
            uint8_t nacp = 0;
            uint8_t sils;
            uint8_t sda = 0;
            uint8_t gva = 0;
            //uint8_t pic;
            if (fspec[2] & 0x20){ // I021/090 Quality Indicators
                readFspec(qi, &p, c->eod);
                //nucp_or_nic = (qi[0] & 0x1e) >> 1;
                nucr_or_nacv = (qi[0] & 0xe0) >> 5;
                mm->accuracy.nac_v_valid = true;
                mm->accuracy.nac_v = nucr_or_nacv;
                if (qi[0] & 0x1){
                    nicbaro = (qi[1] & 0x80) >> 7;
                    sil = (qi[1] & 0x60) >> 5;
                    mm->accuracy.sil = sil;
                    nacp = (qi[1] & 0x1e) >> 1;
                    if (qi[1] & 0x1){
                        sils = (qi[2] & 0x20) >> 5;
                        if (sils){
                            mm->accuracy.sil_type = SIL_PER_SAMPLE;
                        }
                        else{
                            mm->accuracy.sil_type = SIL_PER_HOUR;
                        }
                        sda = (qi[2] & 0x18) >> 3;
                        gva = (qi[2] & 0x6) >> 1;
                        //pic = (qi[3] & 0xf0) >> 4;
                    }
                    else{
                        mm->accuracy.sil_type = SIL_UNKNOWN;
                    }
                }
            }
            if (fspec[2] & 0x10){ // I021/210 MOPS Version
                mm->opstatus.valid = true;
                mm->opstatus.version = ((*p) & 0x38) >> 3;
                uint8_t ltt = (*p) & 0x7;
                p++;
                switch(mm->opstatus.version){
                    case 1:
                        mm->accuracy.nac_p_valid = true;
                        mm->accuracy.nac_p = nacp;
                        mm->accuracy.nic_baro_valid = true;
                        mm->accuracy.nic_baro = nicbaro;
                        mm->accuracy.sil_type = SIL_UNKNOWN;
                        break;
                    case 2:
                        mm->accuracy.nac_p_valid = true;
                        mm->accuracy.nac_p = nacp;
                        mm->accuracy.gva_valid = true;
                        mm->accuracy.gva = gva;
                        mm->accuracy.sda_valid = true;
                        mm->accuracy.sda = sda;
                        mm->accuracy.nic_baro_valid = true;
                        mm->accuracy.nic_baro = nicbaro;
                        break;
                }
                switch (ltt) {
                    case 0:
                        if (!addrtype){
                            mm->addrtype = ADDR_TISB_ICAO;
                        }
                        else{
                            mm->addrtype = ADDR_TISB_OTHER;
                        }
                        break;
                    case 1:
                        if (!addrtype){
                            mm->addrtype = ADDR_ADSR_ICAO;
                        }
                        else{
                            mm->addrtype = ADDR_ADSR_OTHER;
                        }
                        break;
                    case 2:
                        if (!addrtype){
                            mm->addrtype = ADDR_ADSB_ICAO;
                        }
                        else{
                            mm->addrtype = ADDR_ADSB_OTHER;
                        }
                        break;
                    default:
                        mm->addrtype = ADDR_UNKNOWN;
                        break;
                }
            }
            if (fspec[2] & 0x8){ // I021/070 Mode 3/A Code
                mm->squawkHex = (((*p & 0xe) << 11) + ((*p & 0x1) << 10) + ((*(p + 1) & 0xC0) << 2) + ((*(p + 1) & 0x38) << 1) + ((*(p + 1) & 0x7))) ;
                mm->squawkDec = squawkHex2Dec(mm->squawkHex);
                mm->squawk_valid = true;
                p += 2;
            }
            if (fspec[2] & 0x4){ // I021/230 Roll Angle
                int16_t roll = ((*p & 0xff) << 8) + (*(p + 1) & 0xff);
                mm->roll = roll * 0.01;
                mm->roll_valid = true;
                p += 2;
            }
            if (fspec[2] & 0x2){ // I021/145 Flight Level
                int16_t alt = ((*p & 0xff) << 8) + (*(p + 1) & 0xff);
                mm->baro_alt_valid = true;
                mm->baro_alt = alt * 25;
                mm->baro_alt_unit = UNIT_FEET;
                p += 2;
            }
            if (fspec[3] & 0x80){ // I021/152 Magnetic Heading
                mm->heading_valid = true;
                mm->heading_type = HEADING_MAGNETIC;
                uint16_t heading = ((*p & 0xff) << 8) + (*(p + 1) & 0xff);
                mm->heading = heading * (360 / pow(2, 16));
                p += 2;
            }
            if (fspec[3] & 0x40){ // I021/200 Target Status
                mm->spi_valid = true;
                mm->alert_valid = true;
                mm->emergency_valid = true;
                mm->nav.modes_valid = true;
                mm->nav.modes |= (*p & 0b01000000) >> 4;
                mm->emergency = (*p & 0b00011100) >> 2;
                mm->alert = (*p & 0b11);
                mm->spi = (*p & 0b11) == 3;
                p++;
            }
            if (fspec[3] & 0x20){ // ID021/155 Barometric Vertical Rate
                if (*p & 0x80){ //range exceeded
                    p += 2;
                }
                else{
                    int16_t vr = ((*p & 0x7f) << 9) + ((*(p + 1) & 0xff) << 1);
                    mm->baro_rate_valid = true;
                    mm->baro_rate = vr * 3.125;
                    p += 2;
                }
            }
            if (fspec[3] & 0x10){ // ID021/157 Geometric Vertical Rate
                if (*p & 0x80){ //range exceeded
                    p += 2;
                }
                else{
                    int16_t vr = ((*p & 0x7f) << 9) + ((*(p + 1) & 0xff) << 1);
                    mm->geom_rate_valid = true;
                    mm->geom_rate = vr * 3.125;
                    p += 2;
                }
            }
            if (fspec[3] & 0x8){ // ID021/160 Airborne Ground Vector
                if (*p & 0x80){ //range exceeded
                    p += 4;
                }
                else{
                    uint16_t gs = ((*p & 0x7f) << 8) + ((*(p + 1) & 0xff));
                    p += 2;
                    uint16_t ta = ((*p & 0xff) << 8) + ((*(p + 1) & 0xff));
                    p += 2;
                    mm->gs_valid = true;
                    mm->heading_valid = true;
                    mm->heading_type = HEADING_GROUND_TRACK;
                    mm->gs.v0 = gs * pow(2, -14) * 3600;
                    mm->heading = ta * (360 / pow(2, 16));
                }
            }
            if (fspec[3] & 0x4){ // ID021/165 Track Angle Rate
                p += 2;
            }
            if (fspec[3] & 0x2){ // ID021/077 Time of Report Transmission
                uint64_t tt = readAsterixTime(&p);
                if (mm->sysTimestamp == -1){
                    mm->sysTimestamp = tt;
                }
            }
            if (fspec[4] & 0x80){ // ID021/170 Target Identification
                uint64_t cs = ((uint64_t)(*p & 0xff) << 40) + ((uint64_t)(*(p + 1) & 0xff) << 32) + ((uint64_t)(*(p + 2) & 0xff) << 24) + ((uint64_t)(*(p + 3) & 0xff) << 16) + ((uint64_t)(*(p + 4) & 0xff) << 8) + (uint64_t)(*(p + 5) & 0xff);
                char *callsign = mm->callsign;
                callsign[0] = ais_charset[((cs & 0xFC0000000000) >> 42)];
                callsign[1] = ais_charset[((cs & 0x3F000000000) >> 36)];
                callsign[2] = ais_charset[((cs & 0xFC0000000) >> 30)];
                callsign[3] = ais_charset[((cs & 0x3F000000) >> 24)];
                callsign[4] = ais_charset[((cs & 0xFC0000) >> 18)];
                callsign[5] = ais_charset[((cs & 0x3F000) >> 12)];
                callsign[6] = ais_charset[((cs & 0xFC0) >> 6)];
                callsign[7] = ais_charset[(cs & 0x3F)];
                callsign[8] = 0;
                mm->callsign_valid = 1;
                for (int i = 0; i < 8; ++i) {
                    if (
                            (callsign[i] >= 'A' && callsign[i] <= 'Z')
                            // -./0123456789
                            || (callsign[i] >= '-' && callsign[i] <= '9')
                            || callsign[i] == ' '
                            || callsign[i] == '@'
                       ) {
                        // valid chars
                    } else {
                        mm->callsign_valid = 0;
                    }
                }
                p += 6;
            }
            if (fspec[4] & 0x40){ // ID021/020 Emitter Category
                int tc = 0;
                int ca = 0;
                uint8_t ecat = *p++ & 0xFF;
                switch (ecat) {
                    case 0:
                        tc = 0x0e;
                        ca = 0;
                        break;
                    case 1:
                    case 2:
                    case 3:
                    case 4:
                    case 5:
                    case 6:
                        tc = 4;
                        ca = ecat;
                        break;
                    case 10:
                        tc = 4;
                        ca = 7;
                        break;
                    case 11:
                        tc = 3;
                        ca = 1;
                        break;
                    case 12:
                        tc = 3;
                        ca = 2;
                        break;
                    case 13:
                        tc = 3;
                        ca = 6;
                        break;
                    case 14:
                        tc = 3;
                        ca = 7;
                        break;
                    case 15:
                        tc = 3;
                        ca = 4;
                        break;
                    case 16:
                        tc = 3;
                        ca = 3;
                        break;
                    case 20:
                        tc = 2;
                        ca = 1;
                        break;
                    case 21:
                        tc = 2;
                        ca = 3;
                        break;
                    case 22:
                        tc = 2;
                        ca = 4;
                        break;
                    case 23:
                        tc = 2;
                        ca = 5;
                        break;
                    case 24:
                        tc = 2;
                        ca = 6;
                        break;
                }
                mm->category = ((0x0E - tc) << 4) | ca;
                mm->category_valid = 1;
            }

            if (fspec[4] & 0x20) { // I021/220 Met Information
                uint8_t met[FSPEC_MAX];
                readFspec(met, &p, c->eod);
            }

            if (fspec[4] & 0x10) { // I021/146 Selected Altitude
                if (*p & 0x80 && *p & 0x60) {
                    int16_t alt = (*p & 0x1F) << 8;
                    alt += *(p + 1) & 0xFF;
                    if (alt < 0x1000){
                        if ((*p & 0x60) == 0x40) { // MCP
                            mm->nav.mcp_altitude_valid = 1;
                            mm->nav.mcp_altitude = alt * 25;
                        }
                        else if ((*p & 0x60) == 0x60) { //FMS
                            mm->nav.fms_altitude_valid = 1;
                            mm->nav.fms_altitude = alt * 25;
                        }
                    }
                }
                p += 2;
            }
            netUseMessage(mm);
            break;
    }
    if (mm->sysTimestamp == -1){
        mm->sysTimestamp = mstime();
    }
    //mm->decoded_nic = 0;
    //mm->decoded_rc = RC_UNKNOWN;
    return 0;
}



//
//=========================================================================
//
// Write ASTERIX output to TCP clients
//

void modesSendAsterixOutput(struct modesMessage *mm, struct net_writer *writer) {
    int64_t now = mstime();
    uint8_t category;

    unsigned char bytes[3 * 128];
    memset(bytes, 0x0, 3 * 128);

    uint8_t fspec[7];
    for (size_t i = 0; i < 7; i++)
    {
        fspec[i] = 0;
    }
    int p = 0;
    if (mm->from_mlat) // CAT 20
        return;
    if (mm->from_tisb)
        return;
    else { // CAT 21
        category = 21;

        // I021/010 Data Source Identification
        fspec[0] |= 1 << 7;
        bytes[p++] = 000; //SAC
        bytes[p++] = 001; //SIC

        // I021/040 Target Report Descriptor
        fspec[0] |= 1 << 6;
        if (mm->addr & MODES_NON_ICAO_ADDRESS){
            bytes[p] |= (3 << 5);
        }
        else if (mm->addrtype == ADDR_ADSB_OTHER || mm->addrtype == ADDR_TISB_OTHER || mm->addrtype == ADDR_ADSR_OTHER){
            bytes[p] |= (2 << 5);
        }

        if (mm->alt_q_bit == 0)
            bytes[p] |= (1 << 3);

        if (mm->airground == AG_GROUND)
            bytes[p + 1] |= 1 << 6;
        if (bytes[p + 1]){
            bytes[p] |= 1;
            p++;
        }
        p++;

        // I021/130 Position in WGS-84 co-ordinates
        if (mm->cpr_decoded || mm->sbs_pos_valid){
            fspec[0] |= 1 << 2;
            int32_t lat;
            int32_t lon;
            lat = mm->decoded_lat / (180 / pow(2,23));
            lon = mm->decoded_lon / (180 / pow(2,23));
            if (lat < 0){
                lat += 0x1000000;
            }
            if (lon < 0){
                lon += 0x1000000;
            }
            bytes[p++] = (lat & 0xFF0000) >> 16;
            bytes[p++] = (lat & 0xFF00) >> 8;
            bytes[p++] = (lat & 0xFF);
            bytes[p++] = (lon & 0xFF0000) >> 16;
            bytes[p++] = (lon & 0xFF00) >> 8;
            bytes[p++] = (lon & 0xFF);
        }

        // I021/131 Position in WGS-84 co-ordinates, high res.
        /*
           if (mm->cpr_decoded || mm->sbs_pos_valid){
           fspec[0] |= 1 << 1;
           int32_t lat;
           int32_t lon;
           lat = mm->decoded_lat / (180 / pow(2,30));
           lon = mm->decoded_lon / (180 / pow(2,30));
           bytes[p++] = (lat & 0xFF000000) >> 24;
           bytes[p++] = (lat & 0xFF0000) >> 16;
           bytes[p++] = (lat & 0xFF00) >> 8;
           bytes[p++] = (lat & 0xFF);
           bytes[p++] = (lon & 0xFF000000) >> 24;
           bytes[p++] = (lon & 0xFF0000) >> 16;
           bytes[p++] = (lon & 0xFF00) >> 8;
           bytes[p++] = (lon & 0xFF);
           }
           */
        // I021/150 Air Speed
        if(mm->ias_valid || mm->mach_valid){
            fspec[1] |= 1 << 6;
            uint16_t speedval;
            if (mm->mach_valid){
                bytes[p] = (1 << 7);
                speedval = mm->mach * 1000;
            }
            else{
                speedval = (mm->ias / 3600.0) * pow(2,14);
            }
            bytes[p++] |= (speedval & 0x7f00) >> 8;
            bytes[p++] = (speedval & 0xff);
        }

        // I021/151 True Air Speed
        if(mm->tas_valid){
            fspec[1] |= 1 << 5;
            bytes[p++] = (mm->tas & 0x7f00) >> 8;
            bytes[p++] = (mm->tas & 0xff);
        }

        // I021/080 Target Address
        fspec[1] |= 1 << 4;
        bytes[p++] = (mm->addr & 0xff0000) >> 16;
        bytes[p++] = (mm->addr & 0xff00) >> 8;
        bytes[p++] = (mm->addr & 0xff);
        struct aircraft *a = aircraftGet(mm->addr);
        if (!a) { // If it's a currently unknown aircraft....
            a = aircraftCreate(mm->addr); // ., create a new record for it,
        }

        // I021/073 Time of Message Reception of Position
        if (fspec[0] & 0b110){
            fspec[1] |= 1 << 3;
            long midnight = (long)(time(NULL) / 86400) * 86400000;
            int tsm = (mm->sysTimestamp) - midnight;
            if (tsm < 0)
                tsm += 86400000;
            tsm = (int)(tsm * 0.128);
            bytes[p++] = (tsm & 0xff0000) >> 16;
            bytes[p++] = (tsm & 0xff00) >> 8;
            bytes[p++] = tsm & 0xff;
        }

        //  I021/075 Time of Message Reception of Velocity
        if (mm->gs_valid && mm->heading_valid && mm->heading_type == HEADING_GROUND_TRACK){
            fspec[1] |= 1 << 1;
            long midnight = (long)(time(NULL) / 86400) * 86400000;
            int tsm = (mm->sysTimestamp) - midnight;
            if (tsm < 0)
                tsm += 86400000;
            tsm = (int)(tsm * 0.128);
            bytes[p++] = (tsm & 0xff0000) >> 16;
            bytes[p++] = (tsm & 0xff00) >> 8;
            bytes[p++] = tsm & 0xff;
        }

        // I021/140 Geometric Height
        if (mm->geom_alt_valid){
            fspec[2] |= 1 << 6;
            int16_t alt;
            if(mm->geom_alt_unit == UNIT_FEET)
                alt = mm->geom_alt / 6.25;
            else
                alt = mm->geom_alt / 20.5053;
            bytes[p++] = (alt & 0xff00) >> 8;
            bytes[p++] = alt & 0xff;
        }
        else if (mm->geom_delta_valid){
            fspec[2] |= 1 << 6;
            int16_t alt = (int)((a->baro_alt + mm->geom_delta) / 6.25);
            bytes[p++] = (alt & 0xff00) >> 8;
            bytes[p++] = alt & 0xff;
        }
        // I021/090 Quality Indicators
        fspec[2] |= 1 << 5;
        if (mm->accuracy.nac_v_valid)
            bytes[p] += mm->accuracy.nac_v << 5;
        if (mm->cpr_decoded)
            bytes[p] |= mm->cpr_nucp << 1;
        if (mm->accuracy.nic_baro_valid)
            bytes[p + 1] |= mm->accuracy.nic_baro << 7;
        if (mm->accuracy.sil_type != SIL_INVALID)
            bytes[p + 1] |= mm->accuracy.sil << 5;
        if (mm->accuracy.nac_p_valid)
            bytes[p + 1] |= mm->accuracy.nac_p << 1;
        if (bytes[p + 1]){
            bytes[p] |= 1;
            p++;
        }
        if (mm->accuracy.sil_type == SIL_PER_SAMPLE)
            bytes[p + 1] |= 1 << 5;
        if (mm->accuracy.sda_valid)
            bytes[p + 1] |= mm->accuracy.sda << 3;
        if (mm->accuracy.gva_valid)
            bytes[p + 1] |= mm->accuracy.gva << 1;
        if (bytes[p + 1]){
            bytes[p] |= 1;
            p++;
        }
        p++;

        // I021/210 MOPS Version
        if (mm->opstatus.valid){
            fspec[2] |= 1 << 4;

            if (mm->remote) {
                switch (mm->addrtype){
                    case ADDR_ADSB_ICAO:
                    case ADDR_ADSB_OTHER:
                        bytes[p] = 2;
                        break;
                    case ADDR_ADSR_ICAO:
                    case ADDR_ADSR_OTHER:
                        bytes[p] = 1;
                        break;
                    default:
                        bytes[p] = 0;
                        break;
                }
            }
            else {
                switch (mm->source){
                    case SOURCE_ADSB:
                        bytes[p] = 2;
                        break;
                    case SOURCE_ADSR:
                        bytes[p] = 1;
                        break;
                    default:
                        bytes[p] = 0;
                        break;
                }
            }
            bytes[p++] |= (mm->opstatus.version) << 3;
        }

        // I021/070 Mode 3/A Code
        if(mm->squawk_valid){
            fspec[2] |= 1 << 3;
            uint16_t squawk = mm->squawkHex;
            bytes[p]   |= ((squawk & 0x7000)) >> 11;
            bytes[p++] |= ((squawk & 0x0400)) >> 10;
            bytes[p]   |= ((squawk & 0x0300)) >> 2;
            bytes[p]   |= ((squawk & 0x0070)) >> 1;
            bytes[p++] |= ((squawk & 0x0007));
        }

        // I021/230 Roll Angle
        if(mm->roll_valid){
            fspec[2] |= 1 << 2;
            int16_t roll = mm->roll * 100;
            bytes[p++] = (roll & 0xFF00) >> 8;
            bytes[p++] = (roll & 0xFF);
        }

        // I021/145 Flight Level
        if(mm->baro_alt_valid){
            fspec[2] |= 1 << 1;
            int16_t value = mm->baro_alt / 25;
            if (mm->baro_alt_unit == UNIT_METERS)
                value = (int)(mm-> baro_alt * 3.2808);
            bytes[p++] = (value & 0xff00) >> 8;
            bytes[p++] = value & 0xff;
        }

        // I021/152 Magnetic Heading
        if(mm->heading_valid && mm->heading_type == HEADING_MAGNETIC){
            fspec[3] |= 1 << 7;
            double adj_trk = mm->heading * 182.0444;
            uint16_t trk = (int)adj_trk;
            bytes[p++] = (trk & 0xff00) >> 8;
            bytes[p++] = trk & 0xff;
        }

        // I021/200 Target Status
        if (mm->spi_valid || mm->alert_valid || mm->emergency_valid || mm->nav.modes_valid){
            fspec[3] |= 1 << 6;
            if (mm->nav.modes_valid){
                if (mm->nav.modes & 0b00000010)
                    bytes[p] |= 1 << 6;
            }
            if (mm->emergency_valid)
                bytes[p] |= (mm->emergency << 2);
            if (mm->alert_valid)
                bytes[p] |= (mm->alert);
            else if (mm->spi_valid && mm->spi)
                bytes[p] |=3;
            p++;
        }

        // I021/155 Barometric Vertical Rate
        if (mm->baro_rate_valid){
            fspec[3] |= 1 << 5;
            int value = ((int16_t)(mm->baro_rate / 3.125)) >> 1;
            bytes[p++] = (value & 0x7f00) >> 8;
            bytes[p++] = value & 0xff;
        }

        // I021/157 Geometric Vertical Rate
        if (mm->geom_rate_valid){
            fspec[3] |= 1 << 4;
            int value = ((int16_t)(mm->geom_rate / 3.125)) >> 1;
            bytes[p++] = (value & 0x7f00) >> 8;
            bytes[p++] = value & 0xff;
        }

        // I021/160 Airborne Ground Vector
        if (mm->gs_valid && mm->heading_valid && mm->heading_type == HEADING_GROUND_TRACK){
            fspec[3] |= 1 << 3;
            double adj_gs = mm->gs.v0 * 4.5511;
            bytes[p++] = ((int)adj_gs & 0x7f00) >> 8;
            bytes[p++] = (int)adj_gs & 0xff;
            double adj_trk = mm->heading * (pow(2,16) / 360.0);
            uint16_t trk = (int)adj_trk;
            bytes[p++] = (trk & 0xff00) >> 8;
            bytes[p++] = trk & 0xff;
        }

        // I021/077 Time of Report Transmission
        {
            fspec[3] |= 1 << 1;
            long midnight = (long)(time(NULL) / 86400) * 86400000;
            int tsm = now - midnight;
            if (tsm < 0)
                tsm += 86400000;
            tsm = (int)(tsm * 0.128);
            bytes[p++] = (tsm & 0xff0000) >> 16;
            bytes[p++] = (tsm & 0xff00) >> 8;
            bytes[p++] = tsm & 0xff;
        }

        // I021/170 Target Identification
        if(mm->callsign_valid){
            fspec[4] |= 1 << 7;
            uint64_t enc_callsign = 0;
            for (int i = 0; i <= 7; i++)
            {
                uint8_t ch = char_to_ais(mm->callsign[i]);
                enc_callsign = (enc_callsign << 6) + (ch & 0x3F);
            }
            bytes[p++] = (enc_callsign & 0xff0000000000) >> 40;
            bytes[p++] = (enc_callsign & 0xff00000000) >> 32;
            bytes[p++] = (enc_callsign & 0xff000000) >> 24;
            bytes[p++] = (enc_callsign & 0xff0000) >> 16;
            bytes[p++] = (enc_callsign & 0xff00) >> 8;
            bytes[p++] = (enc_callsign & 0xff);
        }

        // I021/020 Emitter Category
        if (mm->category_valid){
            fspec[4] |= 1 << 6;
            int tc = 0x0e - ((mm->category & 0x1F0) >> 4);
            int ca = mm->category & 7;
            if (ca){
                switch (tc) {
                    case 1:
                        break;
                    case 2:
                        switch (ca){
                            case 1:
                                bytes[p++] = 20;
                                break;
                            case 3:
                                bytes[p++] = 21;
                                break;
                            case 4:
                            case 5:
                            case 6:
                            case 7:
                                bytes[p++] = 22;
                                break;
                        }
                        break;
                    case 3:
                        switch (ca){
                            case 1:
                                bytes[p++] = 11;
                                break;
                            case 2:
                                bytes[p++] = 12;
                                break;
                            case 3:
                                bytes[p++] = 16;
                                break;
                            case 4:
                                bytes[p++] = 15;
                                break;
                            case 6:
                                bytes[p++] = 13;
                                break;
                            case 7:
                                bytes[p++] = 14;
                                break;
                        }
                        break;
                    case 4:
                        switch (ca){
                            case 1:
                            case 2:
                            case 3:
                            case 4:
                            case 5:
                            case 6:
                                bytes[p++] = ca;
                                break;
                            case 7:
                                bytes[p++] = 10;
                        }
                        break;
                }
            } else {
                bytes[p++] = 0;
            }
        }
        else if (!(a->category)){
            fspec[4] |= 1 << 6;
            bytes[p++] = 0;
        }
        // I021/220 Met Information
        //if (ac && ((now < ac->oat_updated + TRACK_EXPIRE) || (now < ac->wind_updated + TRACK_EXPIRE && abs(ac->wind_altitude - ac->baro_alt) < 500))){}
        if (mm->wind_valid || mm->oat_valid || mm->turbulence_valid || mm->static_pressure_valid || mm->humidity_valid) {
            fspec[4] |= 1 << 5;
            bool wind = false;
            bool temp = false;
            //if (now < ac->wind_updated + TRACK_EXPIRE && abs(ac->wind_altitude - ac->baro_alt) < 500){}
            if (mm->wind_valid) {
                bytes[p] |= 0xC0;
                wind = true;
            }
            //if (now < ac->oat_updated + TRACK_EXPIRE){}
            if (mm->oat_valid) {
                bytes[p] |= 0x20;
                temp = true;
            }
            p++;
            if (wind){
                uint16_t ws = (int)(mm->wind_speed);
                uint16_t wd = (int)(mm->wind_direction);
                bytes[p++] = (ws & 0xFF00) >> 8;
                bytes[p++] = ws & 0xFF;
                bytes[p++] = (wd & 0xFF00) >> 8;
                bytes[p++] = wd & 0xFF;
            }
            if (temp){
                int16_t oat = (int16_t)((mm->oat) * 4);
                bytes[p++] = (oat & 0xFF00) >> 8;
                bytes[p++] = oat & 0xFF;
            }
        }

        // I021/146 Selected Altitude
        if (mm->nav.fms_altitude_valid || mm->nav.mcp_altitude_valid){
            fspec[4] |= 1 << 4;
            int alt = 0;
            if (mm->nav.mcp_altitude_valid){
                alt = mm->nav.mcp_altitude;
                bytes[p] |= 0xC0;
            }
            else if (mm->nav.fms_altitude_valid){
                alt = mm->nav.fms_altitude;
                bytes[p] |= 0xE0;
            }
            alt /= 25;
            bytes[p++] |= (alt & 0x1F00) >> 8;
            bytes[p++] = (alt & 0xFF);
        }

        // I021/008 Aircraft Operational Status
        if (mm->opstatus.valid){
            if (mm->opstatus.om_acas_ra || mm->opstatus.cc_tc ||
                    mm->opstatus.cc_ts || mm->opstatus.cc_arv || mm->opstatus.cc_cdti
                    || (!mm->opstatus.cc_acas)){
                fspec[5] |= 1 << 7;
                bytes[p] |= (mm->opstatus.om_acas_ra & 0x1) << 7;
                bytes[p] |= (mm->opstatus.cc_tc & 0x3) << 5;
                bytes[p] |= (mm->opstatus.cc_ts & 0x1) << 4;
                bytes[p] |= (mm->opstatus.cc_arv & 0x1) << 3;
                bytes[p] |= (mm->opstatus.cc_cdti & 0x1) << 2;
                bytes[p] |= ((!mm->opstatus.cc_acas) & 0x1) << 1;
                p++;
            }
        }

        // I021/400 Receiver ID
        if (mm->receiverId){
            fspec[5] |= 1 << 2;
            bytes[p++] = (mm->receiverId) & 0xFF;
        }

        // I021/295 Data Ages
        /*
           if (fspec[4] == 0b100000){
           fspec[5] |= 1 << 1;
           bytes[p++] |= 1;
           bytes[p++] |= 1;
           bytes[p++] |= 1 << 2;
           if((now < ac->wind_updated + TRACK_EXPIRE && abs(ac->wind_altitude - ac->baro_alt) < 500) &&
           (now < ac->oat_updated + TRACK_EXPIRE)) {
           uint64_t wind_age = now - ac->wind_updated;
           uint64_t oat_age = now - ac->oat_updated;
           if (wind_age > oat_age){
           bytes[p++] = (int)(wind_age / 100);
           }
           else {
           bytes[p++] = (int)(oat_age / 100);
           }
           }
           else if (now < ac->wind_updated + TRACK_EXPIRE && abs(ac->wind_altitude - ac->baro_alt) < 500){
           uint64_t wind_age = now - ac->wind_updated;
           bytes[p++] = (int)(wind_age / 100);
           }
           else if (now < ac->oat_updated + TRACK_EXPIRE){
           uint64_t oat_age = now - ac->oat_updated;
           bytes[p++] = (int)(oat_age / 100);
           }
           }
           */

        int fspec_len = 1;
        for (int i = 5; i >= 0; i--)
        {
            if (fspec[i + 1]){
                fspec[i] |= 1;
                fspec_len++;
            }
        }

        if (p > 2 * 128) {
            fprintf(stderr, "ERROR: REPORT THIS BUG: asterix byte len too large: %d\n", p);
        }

        uint16_t msgLen = p + 3 + fspec_len;
        uint8_t msgLenA = (msgLen & 0xFF00) >> 8;
        uint8_t msgLenB = msgLen & 0xFF;
        char *w = prepareWrite(writer, msgLen);
        memcpy(w, &category, 1);
        w++;
        memcpy(w, &msgLenA, 1);
        memcpy(w + 1, &msgLenB, 1);
        w+=2;
        memcpy(w, &fspec, fspec_len);
        w += fspec_len;
        memcpy(w, &bytes, p);
        w += p;
        completeWrite(writer, w);

    }
}

//
//=========================================================================
//
// Read SBS input from TCP clients
//

int readAsterix(struct client *c, int64_t now, struct messageBuffer *mb) {

    while (c->som < c->eod) {
        if (c->eod - c->som < 3) {
            // incomplete header, wait for more data
            break;
        }
        char *p = c->som;
        // Cast to uint8_t before shifting to prevent sign-extension on systems
        // where char is signed — a byte value > 127 would otherwise sign-extend
        // to a negative int before the shift, corrupting msgLen.
        uint16_t msgLen = ((uint8_t)*(p + 1) << 8) | (uint8_t)*(p + 2);
        if (msgLen < 3 || c->som + msgLen > c->eod) {
            // invalid or incomplete messages
            break;
        }
        char *end = c->som + msgLen;
        c->som = end;
        if (c->service->read_handler(c, p, c->remote, now, mb)) {
            if (Modes.debug_net) {
                fprintf(stderr, "%s: Closing connection from %s port %s\n", c->service->descr, c->host, c->port);
            }
            modesCloseClient(c);
            return -1;
        }
    }
    return 0;
}

// Spec for Planefinder message.
// All messages begin with a DLE and end with a DLE, ETX. DLE cannot appear in the middle of a message unless it's escaped with another DLE (i.e., bit stuffing)
// Message format:
// Byte     Value       Notes
// 0        <DLE>       header
// 1        ID          Only packet id 0xc1 is recognized here
// 2 - n    Data        Depends on the packet type
// n+1      <DLE>       escape
// n+2      <ETX>       footer

