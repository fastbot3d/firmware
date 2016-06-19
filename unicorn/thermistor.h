/*
 * Unicorn 3D Printer Firmware
 * thermistor.h
*/
#ifndef _THERMISTOR_H
#define _THERMISTOR_H


#if defined (__cplusplus)
extern "C" {
#endif

extern int temp_convert_extruder1(int adc, double *celsius);
extern int temp_convert_extruder2(int adc, double *celsius);
extern int temp_convert_extruder3(int adc, double *celsius);
extern int temp_convert_extruder4(int adc, double *celsius);
extern int temp_convert_extruder5(int adc, double *celsius);
extern int temp_convert_extruder6(int adc, double *celsius);
extern int temp_convert_bed(int adc, double *celsius);

#if defined (__cplusplus)
}
#endif
#endif
