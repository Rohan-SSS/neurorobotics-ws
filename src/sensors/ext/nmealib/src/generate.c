/*
 *
 * NMEA library
 * URL: http://nmea.sourceforge.net
 * Author: Tim (xtimor@gmail.com)
 * Licence: http://www.gnu.org/licenses/lgpl.html
 * $Id: generate.c 17 2008-03-11 11:56:11Z xtimor $
 *
 */

#include "tok.h"
#include "sentence.h"
#include "generate.h"
#include "units.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>

void convert_decimal_to_degrees_minutes_second(double degrees, double* degrees_minutes_seconds){
    (*degrees_minutes_seconds) = 0.0;
    double deg = floor(degrees);
    double minutes = (degrees - deg)*60.0;
    //double minutes = floor((degrees - deg)*60.0);
    //double seconds = (degrees - deg - (minutes/60.0))*36.0;
    //(*degrees_minutes_seconds) = deg*100.0 + minutes + seconds;
    (*degrees_minutes_seconds) = deg*100.0 + minutes;
}

int nmea_gen_GPGGA(char *buff, int buff_sz, nmeaGPGGA *pack)
{
    //original
        // return nmea_printf(buff, buff_sz,        
        // "$GPGGA,%02d%02d%02d.%02d,%07.4f,%C,%07.4f,%C,%1d,%02d,%03.1f,%03.1f,%C,%03.1f,%C,%03.1f,%04d",
        // pack->utc.hour, pack->utc.min, pack->utc.sec, pack->utc.hsec,
        // pack->lat, pack->ns, pack->lon, pack->ew,
        // pack->sig, pack->satinuse, pack->HDOP, pack->elv, pack->elv_units,
        // pack->diff, pack->diff_units, pack->dgps_age, pack->dgps_sid);

        //nmeaString = "$GNGGA,063620.00,1906.47224,N,07301.72145,E,1,04,99.99,41.8,M,-65.7,M,,*6A\n
        /*
        pack->utc.hour = 6;
        
        pack->utc.min = 36;
        
        pack->utc.sec = 21;
        pack->utc.hsec = 0;
        */
        /*
        pack->lat = 1906.47224f;
        pack->ns = 'N';
        pack->lon = 7301.72145f;
        pack->ew = 'E';
        
        pack->sig = 1;
        pack->satinuse = 4;
        
        pack->HDOP = 99.99;
        pack->elv = 41.8;
        pack->elv_units = 'M';
        pack->diff = -65.7;
        pack->diff_units = 'M';
        */
        //pack->HDOP = 0.0;
        return nmea_printf(buff, buff_sz,        
        "$GNGGA,%02d%02d%02d.%02d,%010.5f,%C,%011.5f,%C,%1d,%02d,%.2f,%.1f,%C,%.1f,%C,,",
        pack->utc.hour, pack->utc.min, pack->utc.sec, pack->utc.hsec,
        pack->lat, pack->ns, pack->lon, pack->ew,
        pack->sig, pack->satinuse, pack->HDOP, pack->elv, pack->elv_units,
        pack->diff, pack->diff_units);
        // return nmea_printf(buff, buff_sz,        
        // "$GNGGA,%02d%02d%02d.%02d,%08.3f,%C,%09.3f,%C,%1d,%02d,%.2f,%.1f,%C,,,,",
        // pack->utc.hour, pack->utc.min, pack->utc.sec, pack->utc.hsec,
        // pack->lat, pack->ns, pack->lon, pack->ew,
        // pack->sig, pack->satinuse, pack->HDOP, pack->elv, pack->elv_units);
        
}

int nmea_gen_GPGSA(char *buff, int buff_sz, nmeaGPGSA *pack)
{
    return nmea_printf(buff, buff_sz,
        "$GPGSA,%C,%1d,%02d,%02d,%02d,%02d,%02d,%02d,%02d,%02d,%02d,%02d,%02d,%02d,%03.1f,%03.1f,%03.1f",
        pack->fix_mode, pack->fix_type,
        pack->sat_prn[0], pack->sat_prn[1], pack->sat_prn[2], pack->sat_prn[3], pack->sat_prn[4], pack->sat_prn[5],
        pack->sat_prn[6], pack->sat_prn[7], pack->sat_prn[8], pack->sat_prn[9], pack->sat_prn[10], pack->sat_prn[11],
        pack->PDOP, pack->HDOP, pack->VDOP);
}

int nmea_gen_GPGSV(char *buff, int buff_sz, nmeaGPGSV *pack)
{
    return nmea_printf(buff, buff_sz,
        "$GPGSV,%1d,%1d,%02d,"
        "%02d,%02d,%03d,%02d,"
        "%02d,%02d,%03d,%02d,"
        "%02d,%02d,%03d,%02d,"
        "%02d,%02d,%03d,%02d",
        pack->pack_count, pack->pack_index + 1, pack->sat_count,
        pack->sat_data[0].id, pack->sat_data[0].elv, pack->sat_data[0].azimuth, pack->sat_data[0].sig,
        pack->sat_data[1].id, pack->sat_data[1].elv, pack->sat_data[1].azimuth, pack->sat_data[1].sig,
        pack->sat_data[2].id, pack->sat_data[2].elv, pack->sat_data[2].azimuth, pack->sat_data[2].sig,
        pack->sat_data[3].id, pack->sat_data[3].elv, pack->sat_data[3].azimuth, pack->sat_data[3].sig);
}

int nmea_gen_GPRMC(char *buff, int buff_sz, nmeaGPRMC *pack)
{
        //original
        // return nmea_printf(buff, buff_sz,
        // "$GPRMC,%02d%02d%02d.%02d,%C,%07.4f,%C,%07.4f,%C,%03.1f,%03.1f,%02d%02d%02d,%03.1f,%C,%C",
        // pack->utc.hour, pack->utc.min, pack->utc.sec, pack->utc.hsec,
        // pack->status, pack->lat, pack->ns, pack->lon, pack->ew,
        // pack->speed, pack->direction,
        // pack->utc.day, pack->utc.mon + 1, pack->utc.year - 100,
        // pack->declination, pack->declin_ew, pack->mode);
        //$GNRMC,063621.00,A,1906.47224,N,07301.72145,E,0.000,,080822,,,A*67\n";
        
        /*
        static int  val = 10;
        val += 10; 
        */
        //currently just test with fixed time as time is not a part of GCS logic
        /*
        pack->utc.hour = 6;
        pack->utc.min = 36;
        
        pack->utc.sec = 21;
        
        pack->utc.hsec = 0;
        */
        /*
        pack->status = 'A';
        
        pack->lat = 1906.47224f;
        pack->ns = 'N';
        pack->lon = 7301.72145f;
        pack->ew = 'E';
        
        pack->speed = 0;
        
        pack->utc.day = 8;
        pack->utc.mon = 7;
        pack->utc.year = 122;
        
        pack->mode = 'A';
        */
        return nmea_printf(buff, buff_sz,
        "$GNRMC,%02d%02d%02d.%02d,%C,%010.5f,%C,%011.5f,%C,%.3f,%.3f,%02d%02d%02d,,,%C",
        pack->utc.hour, pack->utc.min, pack->utc.sec, pack->utc.hsec,
        pack->status, pack->lat, pack->ns, pack->lon, pack->ew,
        pack->speed, pack->direction, pack->utc.day, pack->utc.mon + 1, pack->utc.year - 100, pack->mode);
        
}

int nmea_gen_GPVTG(char *buff, int buff_sz, nmeaGPVTG *pack)
{
    return nmea_printf(buff, buff_sz,
        "$GPVTG,%.1f,%C,%.1f,%C,%.1f,%C,%.1f,%C",
        pack->dir, pack->dir_t,
        pack->dec, pack->dec_m,
        pack->spn, pack->spn_n,
        pack->spk, pack->spk_k);
}

void nmea_info2GPGGA(const nmeaINFO *info, nmeaGPGGA *pack)
{
    nmea_zero_GPGGA(pack);

    pack->utc = info->utc;
    pack->lat = fabs(info->lat);
    pack->ns = ((info->lat > 0)?'N':'S');
    pack->lon = fabs(info->lon);
    pack->ew = ((info->lon > 0)?'E':'W');
    pack->sig = info->sig;
    pack->satinuse = info->satinfo.inuse;
    pack->HDOP = info->HDOP;
    pack->elv = info->elv;
}

void nmea_info2GPGSA(const nmeaINFO *info, nmeaGPGSA *pack)
{
    int it;

    nmea_zero_GPGSA(pack);

    pack->fix_type = info->fix;
    pack->PDOP = info->PDOP;
    pack->HDOP = info->HDOP;
    pack->VDOP = info->VDOP;

    for(it = 0; it < NMEA_MAXSAT; ++it)
    {
        pack->sat_prn[it] =
            ((info->satinfo.sat[it].in_use)?info->satinfo.sat[it].id:0);
    }
}

int nmea_gsv_npack(int sat_count)
{
    int pack_count = (int)ceil(((double)sat_count) / NMEA_SATINPACK);

    if(0 == pack_count)
        pack_count = 1;

    return pack_count;
}

void nmea_info2GPGSV(const nmeaINFO *info, nmeaGPGSV *pack, int pack_idx)
{
    int sit, pit;

    nmea_zero_GPGSV(pack);

    pack->sat_count = (info->satinfo.inview <= NMEA_MAXSAT)?info->satinfo.inview:NMEA_MAXSAT;
    pack->pack_count = nmea_gsv_npack(pack->sat_count);

    if(pack->pack_count == 0)
        pack->pack_count = 1;

    if(pack_idx >= pack->pack_count)
        pack->pack_index = pack_idx % pack->pack_count;
    else
        pack->pack_index = pack_idx;

    for(pit = 0, sit = pack->pack_index * NMEA_SATINPACK; pit < NMEA_SATINPACK; ++pit, ++sit)
        pack->sat_data[pit] = info->satinfo.sat[sit];
}

void nmea_info2GPRMC(const nmeaINFO *info, nmeaGPRMC *pack)
{
    nmea_zero_GPRMC(pack);

    pack->utc = info->utc;
    pack->status = ((info->sig > 0)?'A':'V');
    pack->lat = fabs(info->lat);
    pack->ns = ((info->lat > 0)?'N':'S');
    pack->lon = fabs(info->lon);
    pack->ew = ((info->lon > 0)?'E':'W');
    pack->speed = info->speed / NMEA_TUD_KNOTS;
    pack->direction = info->direction;
    pack->declination = info->declination;
    pack->declin_ew = 'E';
    pack->mode = ((info->sig > 0)?'A':'N');
}

void nmea_info2GPVTG(const nmeaINFO *info, nmeaGPVTG *pack)
{
    nmea_zero_GPVTG(pack);

    pack->dir = info->direction;
    pack->dec = info->declination;
    pack->spn = info->speed / NMEA_TUD_KNOTS;
    pack->spk = info->speed;
}

int nmea_generate(
    char *buff, int buff_sz,
    const nmeaINFO *info,
    int generate_mask
    )
{
    int gen_count = 0, gsv_it, gsv_count;
    int pack_mask = generate_mask;

    nmeaGPGGA gga;
    nmeaGPGSA gsa;
    nmeaGPGSV gsv;
    nmeaGPRMC rmc;
    nmeaGPVTG vtg;

    if(!buff)
        return 0;

    while(pack_mask)
    {
        if(pack_mask & GPGGA)
        {
            nmea_info2GPGGA(info, &gga);
            gen_count += nmea_gen_GPGGA(buff + gen_count, buff_sz - gen_count, &gga);
            pack_mask &= ~GPGGA;
        }
        else if(pack_mask & GPGSA)
        {
            nmea_info2GPGSA(info, &gsa);
            gen_count += nmea_gen_GPGSA(buff + gen_count, buff_sz - gen_count, &gsa);
            pack_mask &= ~GPGSA;
        }
        else if(pack_mask & GPGSV)
        {
            gsv_count = nmea_gsv_npack(info->satinfo.inview);
            for(gsv_it = 0; gsv_it < gsv_count && buff_sz - gen_count > 0; ++gsv_it)
            {
                nmea_info2GPGSV(info, &gsv, gsv_it);
                gen_count += nmea_gen_GPGSV(buff + gen_count, buff_sz - gen_count, &gsv);
            }
            pack_mask &= ~GPGSV;
        }
        else if(pack_mask & GPRMC)
        {
            nmea_info2GPRMC(info, &rmc);
            gen_count += nmea_gen_GPRMC(buff + gen_count, buff_sz - gen_count, &rmc);
            pack_mask &= ~GPRMC;
        }
        else if(pack_mask & GPVTG)
        {
            nmea_info2GPVTG(info, &vtg);
            gen_count += nmea_gen_GPVTG(buff + gen_count, buff_sz - gen_count, &vtg);
            pack_mask &= ~GPVTG;
        }
        else
            break;

        if(buff_sz - gen_count <= 0)
            break;
    }

    return gen_count;
}

void nmea_info2GPGGA_dms(const nmeaINFO *info, nmeaGPGGA *pack)
{
    nmea_zero_GPGGA(pack);

    pack->utc = info->utc;
    double lat = fabs(info->lat);
    double lat_dms = 0.0;
    convert_decimal_to_degrees_minutes_second(lat,&lat_dms);
    pack->lat = lat_dms;
    pack->ns = ((info->lat > 0)?'N':'S');
    double lon = fabs(info->lon); 
    double lon_dms = 0.0;
    convert_decimal_to_degrees_minutes_second(lon, &lon_dms);
    pack->lon = lon_dms;
    pack->ew = ((info->lon > 0)?'E':'W');
    pack->sig = info->sig;
    pack->satinuse = info->satinfo.inuse;
    pack->HDOP = info->HDOP;
    pack->elv = info->elv;
}


void nmea_info2GPRMC_dms(const nmeaINFO *info, nmeaGPRMC *pack)
{
    nmea_zero_GPRMC(pack);

    pack->utc = info->utc;
    pack->status = ((info->sig > 0)?'A':'V');
    double lat = fabs(info->lat);
    double lat_dms = 0.0;
    convert_decimal_to_degrees_minutes_second(lat,&lat_dms);
    pack->lat = lat_dms;
    pack->ns = ((info->lat > 0)?'N':'S');
    double lon = fabs(info->lon); 
    double lon_dms = 0.0;
    convert_decimal_to_degrees_minutes_second(lon, &lon_dms);
    pack->lon = lon_dms;
    pack->ew = ((info->lon > 0)?'E':'W');
    pack->speed = info->speed / NMEA_TUD_KNOTS;
    pack->direction = info->direction;
    pack->declination = info->declination;
    pack->declin_ew = 'E';
    pack->mode = ((info->sig > 0)?'A':'N');
}


int nmea_generate_dms(
    char *buff, int buff_sz,
    const nmeaINFO *info,
    int generate_mask
    )
{
    int gen_count = 0, gsv_it, gsv_count;
    int pack_mask = generate_mask;

    nmeaGPGGA gga;
    nmeaGPGSA gsa;
    nmeaGPGSV gsv;
    nmeaGPRMC rmc;
    nmeaGPVTG vtg;

    if(!buff)
        return 0;

    while(pack_mask)
    {
        if(pack_mask & GPGGA)
        {
            nmea_info2GPGGA_dms(info, &gga);
            gen_count += nmea_gen_GPGGA(buff + gen_count, buff_sz - gen_count, &gga);
            pack_mask &= ~GPGGA;
        }
        else if(pack_mask & GPGSA)
        {
            nmea_info2GPGSA(info, &gsa);
            gen_count += nmea_gen_GPGSA(buff + gen_count, buff_sz - gen_count, &gsa);
            pack_mask &= ~GPGSA;
        }
        else if(pack_mask & GPGSV)
        {
            gsv_count = nmea_gsv_npack(info->satinfo.inview);
            for(gsv_it = 0; gsv_it < gsv_count && buff_sz - gen_count > 0; ++gsv_it)
            {
                nmea_info2GPGSV(info, &gsv, gsv_it);
                gen_count += nmea_gen_GPGSV(buff + gen_count, buff_sz - gen_count, &gsv);
            }
            pack_mask &= ~GPGSV;
        }
        else if(pack_mask & GPRMC)
        {
            nmea_info2GPRMC_dms(info, &rmc);
            gen_count += nmea_gen_GPRMC(buff + gen_count, buff_sz - gen_count, &rmc);
            pack_mask &= ~GPRMC;
        }
        else if(pack_mask & GPVTG)
        {
            nmea_info2GPVTG(info, &vtg);
            gen_count += nmea_gen_GPVTG(buff + gen_count, buff_sz - gen_count, &vtg);
            pack_mask &= ~GPVTG;
        }
        else
            break;

        if(buff_sz - gen_count <= 0)
            break;
    }

    return gen_count;
}
