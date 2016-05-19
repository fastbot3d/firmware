/*
 * Unicorn 3D Printer Firmware
 * thermistor.h
*/
#ifndef _THERMISTOR_H
#define _THERMISTOR_H


#if defined (__cplusplus)
extern "C" {
#endif

extern int temp_convert_extruder(int adc, double *celsius);
extern int temp_convert_bed(int adc, double *celsius);

#if defined (__cplusplus)
}
#endif
#endif
