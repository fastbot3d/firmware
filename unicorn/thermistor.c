/*
 * Unicorn 3D Printer Firmware
 * thermistor.c
*/

#include <unistd.h>

#include "common.h"
#include "thermistor.h"
#include "eeprom.h"
#include "parameter.h"

static const struct convert_entry thermistor_pt100[] = {
  	{ 4096, 0 },
 	 {4029.29 ,       2   },
     {4013.30 ,       5   },
     {3993.31 ,       10  },
     {3969.33 ,       15  },
     {3937.35 ,       20 },
     {3905.37 ,       25 },
     {3861.40 ,       30 },
     {3813.43 ,       35 },
     {3753.47 ,       40 },
	 {3685.52 ,       45 },
     {3609.57 ,       50 },
     {3521.63 ,       55 },
     {3425.69 ,       60 },
     {3317.77 ,       65 },
     {3201.84 ,       70 },
     {3077.93 ,       75 },
     {2946.02 ,       80 },
     {2806.11 ,       85 },
     {2658.21 ,       90 },
     {2510.31 ,       95 },
     {2362.41 ,       100},
     {2210.51 ,       105},
     {2062.61 ,       110},
	 {1918.71 ,       115},
     {1778.80 ,       120},
     {1642.89 ,       125},
     {1514.98 ,       130},
     {1391.06 ,       135},
     {1279.14 ,       140},
     {1171.21 ,       145},
     {1071.28 ,       150},
	 {979.34 ,       155},
     {895.39 ,       160},
     {819.44 ,       165},
     {747.49 ,       170},
     {683.54 ,       175},
     {623.58 ,       180},
     {571.61 ,       185},
	 {523.64 ,       190},
     {479.67 ,       195},
     {435.70 ,       200},
     {399.73 ,       205},
     {367.75	 , 210},
     {335.77	 , 215},
     {311.79	 , 220},
     {283.80	 , 225},
     {263.82	 , 230},
     {243.83	 , 235},
     {223.84	 , 240},
     {207.86	 , 245},
     {191.87	 , 250},
     {175.88	 , 255},
     {163.88	 , 260},
     {151.89	 , 265},
     {139.9	 , 270},
     {131.9	 , 275},
     {123.9	 , 280},
     {111.9	 , 285},
     {107.9	 , 290},
     {	99.9 , 295},
     { 91.9	 , 300},
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

int temp_convert_extruder1(int adc, double *celsius)
{
	// thermistor connection wire is break.
	if (adc > ERROR_MAX_ADC || adc < ERROR_MIN_ADC) { 
		return -1;
	}

	if (ext1_temp_curve.array_len >0) {
    	return convert(ext1_temp_curve.curve, ext1_temp_curve.array_len, adc, celsius);
	} else {
    	//return convert(thermistor_100k, NR_ITEMS(thermistor_100k), adc, celsius);
    	return convert(thermistor_pt100, NR_ITEMS(thermistor_pt100), adc, celsius);
	}
}

int temp_convert_extruder2(int adc, double *celsius)
{
	// thermistor connection wire is break.
	if (adc > ERROR_MAX_ADC || adc < ERROR_MIN_ADC) {
		return -1;
	}

	if (ext2_temp_curve.array_len >0) {
    	return convert(ext2_temp_curve.curve, ext2_temp_curve.array_len, adc, celsius);
	} else {
		//return convert(thermistor_100k, NR_ITEMS(thermistor_100k), adc, celsius);
		return convert(thermistor_pt100, NR_ITEMS(thermistor_pt100), adc, celsius);
	}
}

int temp_convert_extruder3(int adc, double *celsius)
{
	return read_max6675_thermocouple(celsius);
}

 
int temp_convert_extruder4(int adc, double *celsius)
{
	return read_ad597_thermocouple(AIN_CH_EXT4, celsius);
}

int temp_convert_extruder5(int adc, double *celsius)
{
	return read_ad597_thermocouple(AIN_CH_EXT5, celsius);
}

int temp_convert_extruder6(int adc, double *celsius)
{
	return read_ad597_thermocouple(AIN_CH_EXT6, celsius);
}

int temp_convert_bed(int adc, double *celsius)
{
	// thermistor connection wire is break.
	if (adc > ERROR_MAX_ADC || adc < ERROR_MIN_ADC) {
		return -1;
	}

	if(bed0_temp_curve.array_len >0) {
    	return convert(bed0_temp_curve.curve, bed0_temp_curve.array_len, adc, celsius);
	} else {
		//return convert(thermistor_330k, NR_ITEMS(thermistor_330k), adc, celsius);
		return convert(thermistor_pt100, NR_ITEMS(thermistor_pt100), adc, celsius);
	}
}
