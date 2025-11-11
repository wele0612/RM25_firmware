#ifndef __BUZZER_H
#define __BUZZER_H

#include <tim.h>
#include <main.h>

void buzzer_on();
void buzzer_off();
void buzzer_set_freq(float freq);

void buzzer_DJI_startup();
void buzzer_calibration_startup();
void buzzer_calibration_done();
void buzzer_motor_notconnected(int id);

void why_play_harunokage();

#define TUNE_C4            261.63f
#define TUNE_C4_SHARP_D4_FLAT 277.18f
#define TUNE_D4            293.66f
#define TUNE_D4_SHARP_E4_FLAT 311.13f
#define TUNE_E4            329.63f
#define TUNE_F4            349.23f
#define TUNE_F4_SHARP_G4_FLAT 369.99f
#define TUNE_G4            392.00f
#define TUNE_G4_SHARP_A4_FLAT 415.30f
#define TUNE_A4            440.00f
#define TUNE_A4_SHARP_B4_FLAT 466.16f
#define TUNE_B4            493.88f
#define TUNE_C5 523.25f
#define TUNE_C5_SHARP_D5_FLAT 554.37f
#define TUNE_D5 587.33f
#define TUNE_D5_SHARP_E5_FLAT 622.25f
#define TUNE_E5 659.25f
#define TUNE_F5 698.46f
#define TUNE_F5_SHARP_G5_FLAT 739.99f
#define TUNE_G5 783.99f
#define TUNE_G5_SHARP_A5_FLAT 830.61f
#define TUNE_A5 880.0f
#define TUNE_A5_SHARP_B5_FLAT 932.33f
#define TUNE_B5 987.77f
#define TUNE_C6 1046.5f
#define TUNE_C6_SHARP_D6_FLAT 1108.73f
#define TUNE_D6 1174.66f
#define TUNE_D6_SHARP_E6_FLAT 1244.51f
#define TUNE_E6 1318.51f
#define TUNE_F6 1396.91f
#define TUNE_F6_SHARP_G6_FLAT 1479.98f
#define TUNE_G6 1567.98f
#define TUNE_G6_SHARP_A6_FLAT 1661.22f
#define TUNE_A6 1760.0f
#define TUNE_A6_SHARP_B6_FLAT 1864.66f
#define TUNE_B6 1975.53f

#endif
