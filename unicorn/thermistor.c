/*
 * Unicorn 3D Printer Firmware
 * thermistor.c
 *
 * Copyright (c) 2014 Truby Zong <truby.zong@gmail.com>
 *
 * Unicorn is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Unicorn is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package; if not, see <http://www.gnu.org/licenses/>.
 */

#include <unistd.h>

#include "common.h"
#include "thermistor.h"

struct convert_entry {
    unsigned int adc_value;
    double       celsius;
};

static const struct convert_entry thermistor_pt100[] = {
    { 4096, 7.0 },
    { 3893, 35.2 },
    { 3859, 38.3 },
    { 3834, 39.8 },
    { 3823, 41.1 },
    { 3800, 43.8 },
    { 3775, 44.2 },
    { 3732, 46.6 },
    { 3682, 49.1 },
    { 3616, 52.7 },
    { 3561, 55.0 }, 
    { 3493, 57.7 },
    { 3393, 61.7 },
    { 3284, 65.2 },
    { 3195, 68.7 },
    { 3102, 71.6 },
    { 3006, 75.2 },
    { 2772, 81.2 },
    { 2596, 86.5 },
    { 2526, 89.5 },
    { 2458, 91.2 },
    { 2353, 93.4 }, 
    { 2214, 97.1 },
    { 2146, 99.5 },
    { 2048, 102.3 },
    { 1982, 104.3 },
    { 1889, 106.8 },
    { 1795, 109.4 },
    { 1707, 111.8 },
    { 1622, 113.6 },
    { 1568, 116.5 },
    { 1488, 118.7 },
    { 1413, 121.9 },
    { 1340, 124.0 },
    { 1249, 127.6 },
    { 1206, 130.3 },
    { 1165, 132.0 },
    { 1104, 134.8 },
    { 1047, 136.9 },
    { 1013, 138.7 },
    { 978, 140.2 },
    { 928, 142.4 },
    { 899, 145.2 },
    { 853, 147.7 },
    { 769, 149.4 },
    { 744, 152.0 },
    { 719, 154.6 },
    { 696, 157.8 },
    { 678, 160.3 },
    { 651, 161.5 },
    { 630, 163.4 },
    { 610, 165.0 },
    { 580, 167.7 },
    { 562, 170.1 },
    { 535, 172.2 },
    { 510, 173.4 },
    { 501, 174.8 },
    { 485, 176.6 },
    { 469, 178.0 },
    { 473, 179.2 },
    { 448, 181.0 },
    { 441, 182.2 },
    { 428, 183.7 },
    { 414, 184.8 },
    { 407, 185.3 },
    { 396, 187.1 },
    { 385, 189.1 },
    { 369, 191.2 },
    { 357, 192.5 },
    { 353, 193.5 },
    { 341, 195.2 },
    { 332, 196.4 },
    { 328, 197.2 },
    { 309, 199.7 },
    { 305, 200.7 },
    { 298, 202.8 },
    { 294, 203.5 },
    { 284, 204.6 },
    { 282, 205.3 },
    { 278, 206.1 },
    { 273, 207.0 },
    { 271, 208.0 },
    { 264, 209.0 },
    { 259, 210.0 },
    { 253, 212.0 },
    { 250, 213.0 },
    { 241, 214.0 },
    { 234, 217.0 },
    { 232, 218.0 },
    { 225, 219.0 },
    { 223, 220.0 },
    { 216, 221.0 },
    { 214, 222.0 },
    { 209, 223.0 },
    { 200, 225.0 },
    { 198, 226.0 },
    { 196, 227.0 },
    { 191, 229.0 },
    { 189, 230.0 },
    { 184, 231.0 },
    { 182, 233.0 },
    { 180, 234.0 },
    { 173, 236.0 },
    { 164, 237.0 },
    { 159, 238.0 },
    { 155, 239.0 },
    { 148, 240.0 },
    { 146, 241.0 },
};

static const struct convert_entry thermistor_100k[] = {
    { 3900,   0.0 },  //FIXME
    { 3617,   0.0 },
    { 3612,   5.0 },
    { 3604,  10.0 },
    { 3595,  15.0 },
    { 3584,  20.0 },
    { 3571,  25.0 },
    { 3555,  30.0 },
    { 3536,  35.0 },
    { 3513,  40.0 },
    { 3486,  45.0 },
    { 3455,  50.0 },
    { 3419,  55.0 },
    { 3377,  60.0 },
    { 3330,  65.0 },
    { 3276,  70.0 },
    { 3216,  75.0 },
    { 3149,  80.0 },
    { 3075,  85.0 },
    { 2994,  90.0 },
    { 2907,  95.0 },
    { 2814, 100.0 },
    { 2715, 105.0 },
    { 2612, 110.0 },
    { 2504, 115.0 },
    { 2393, 120.0 },
    { 2279, 125.0 },
    { 2164, 130.0 },
    { 2049, 135.0 },
    { 1935, 140.0 },
    { 1822, 145.0 },
    { 1711, 150.0 },
    { 1604, 155.0 },
    { 1500, 160.0 },
    { 1400, 165.0 },
    { 1305, 170.0 },
    { 1215, 175.0 },
    { 1130, 180.0 },
    { 1049, 185.0 },
    {  974, 190.0 },
    {  904, 195.0 },
    {  838, 200.0 },
    {  777, 205.0 },
    {  720, 210.0 },
    {  668, 215.0 },
    {  619, 220.0 },
    {  574, 225.0 },
    {  532, 230.0 },
    {  494, 235.0 },
    {  459, 240.0 },
    {  396, 250.0 },
    {  369, 255.0 },
    {  343, 260.0 },
    {  298, 270.0 },
    {  278, 275.0 },
    {  260, 280.0 },
    {  243, 285.0 },
    {  227, 290.0 },
    {  212, 295.0 },
    {  199, 300.0 },
};

static const struct convert_entry thermistor_330k[] = {
    { 3650,  0.0 },
    { 3626, 21.0 },
    { 3601, 35.0 },
    { 3597, 37.0 },
    { 3571, 47.0 },
    { 3546, 54.0 },
    { 3534, 57.0 },
    { 3519, 60.0 },
    { 3485, 66.0 },
    { 3452, 71.0 },
    { 3412, 76.0 },
    { 3393, 78.4 },
    { 3375, 80.4 },
    { 3357, 82.1 },
    { 3342, 83.7 },
    { 3328, 85.2 },
    { 3240, 90.0 },
    { 3157, 95.0 },
    { 3061, 100.0 },
    { 2949, 105.0 },
    { 2822, 110.0 },
    { 2678, 115.0 },
    { 2520, 120.0 },
    { 2348, 125.0 },
    { 2165, 130.0 },
    { 1975, 135.0 },
    { 1782, 140.0 },
    { 1589, 145.0 },
    { 1401, 150.0 },
};

static int convert(const struct convert_entry *table, int entries, int adc, double *celsius)
{
    int idx;

    if (!celsius) {
        return -1;
    }
   
    for (idx = 0; idx < entries; idx++) {
        int delta_adc = adc - table[idx].adc_value;
        if (delta_adc > 0) {

            if (idx > 0) { 
                double celsius_ix      = table[idx].celsius;
                unsigned int adc_ix    = table[idx].adc_value;
                double delta_celsius   = celsius_ix - table[idx - 1].celsius; 
                unsigned int delta_adc = table[idx - 1].adc_value - adc_ix; 
                double rc = delta_celsius / delta_adc;

                *celsius = celsius_ix - (adc - adc_ix) * rc;
            } else {
                /*
                 *  Before first entry, assume open circuit or sensor fault
                 *  Return very high temperature to turn heaters off
                 */
                *celsius = 777.9;
            }

            break;
        } else if(delta_adc == 0) {
            *celsius = table[idx].celsius;
            break;
        } else if (idx == entries) {
            /*
             * After last entry, assume short circuit or sensor fault
             * return very high temperature to turn heaters off.
             */
            *celsius = 444.0;
        }
    }

    return 0;
}

int temp_convert_extruder(int adc, double *celsius)
{
    //return convert(thermistor_100k, NR_ITEMS(thermistor_100k), adc, celsius);
    return convert(thermistor_pt100, NR_ITEMS(thermistor_pt100), adc, celsius);
}

int temp_convert_bed(int adc, double *celsius)
{
    //return convert(thermistor_330k, NR_ITEMS(thermistor_330k), adc, celsius);
    return convert(thermistor_pt100, NR_ITEMS(thermistor_pt100), adc, celsius);
}
