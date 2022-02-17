/*
 * File:   main.c
 * Author: Vincilir
 *
 * Created on February 1, 2022, 8:26 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include "p30Fxxxx.h"
#include "adc.h"
#include "driverGLCD.h"
#include "timer1.h"
#include "timer2.h"
#include "uart.h"



_FOSC(CSW_ON_FSCM_OFF & HS3_PLL4);
_FWDT(WDT_OFF);
_FGS(CODE_PROT_OFF);


char tempRX[5];
unsigned char temp;

unsigned int X, Y,x_vrednost, y_vrednost;
const unsigned int AD_Xmin =220;
const unsigned int AD_Xmax =3642;
const unsigned int AD_Ymin =520;
const unsigned int AD_Ymax =3450;
unsigned int vreme_paljenja = 0, vreme_gasenja = 0;
unsigned int touchx,touchy; //touch_adc
unsigned int adc_mq, adc_fo; //adc_promenljive
unsigned int brojac_ms = 0; //brojac_ms
unsigned int brojac_us = 0; //brojac_us
unsigned int broj,broj1,broj2,temp0,temp1, n; 

#define DRIVE_A PORTCbits.RC13
#define DRIVE_B PORTCbits.RC14



unsigned char const pass[1024] = {
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
 128,192,224,224,240,208,216,104,104, 48, 52, 52, 16, 16, 16, 16, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,192,240,248,254,127, 
  63, 15,  7,  3,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,128,192,224,240,208, 
 224, 96, 48, 16,  8,  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,192,254,255,255,255,  3,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,128,224,240,248,252,126, 63, 31, 15,  7,  3,  1, 
   0,  0,  0,224,128,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  1, 63,255,255,255,224,  0, 
   0,  0,  0,  0,  0,  0,  0,  1,  3,  7, 15, 30, 62,126,252,248, 
 252,124, 62, 31, 15,  7,  3,  1,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,128,240,255,255,  2,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  7, 15, 63,127, 
 252,248,240,224,192,128,128,  0,  0,  0,  0,  0,  0,  0,  0,  1, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,128,192,192,224,240,252, 
 126, 63, 31,  7,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  1,  1,  3,  7,  7, 15, 15, 31, 30, 30, 28, 60, 60, 60, 60, 
  60, 60, 60, 60, 60, 30, 30, 30, 15, 15, 15,  7,  3,  3,  1,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 
};

unsigned char const fail[1024] = {
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
 128,192,224,224,240,216,232,104,104, 48, 52, 20, 16, 16, 16, 16, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,192,240,252,254,127, 
  31, 15,  7,  3,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,128,192,224,224,192, 64, 32,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,224,255,255,255, 31,  1,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 96,252,248,240,224,192, 
 192,224,240,248,252,126, 63, 31, 15,  7,  3,  1,  0,  0,  0,  0, 
   0,  0,  0,224,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  3,127,255,255,252,192,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,240,248,253,127, 63, 31, 
  15, 31, 63,125,248,240,224,192,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,192,248,255,254,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  7, 15, 63,127, 
 252,248,240,224,192,128,  0,  0,  0,  0,  3,  1,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,128,192,224,224,248,252, 
 127, 63, 15,  7,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  1,  1,  3,  7,  7, 15, 15, 30, 30, 30, 28, 60, 60, 60, 60, 
  60, 60, 60, 60, 60, 30, 30, 30, 15, 15,  7,  7,  3,  3,  1,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 
};

const char  DAN [1024] = {
   0,  0,  0,  0,  0,  0,  0,  0,248,236,  4,  6,  6, 30, 56, 32, 
  32, 56, 30,  6,  6,  4,236,248,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,128,128,  0,  0, 
   0,  0,  0,  2,  4,  8, 16, 32,  0,  0,128,128, 64, 64, 64, 79, 
  64, 64,128,128,  0, 32, 16,  8,  4,  2,  1,  0,  0,  0,128,128, 
   0, 56, 62, 99, 97,195,  3,  3,  3,  1,128,224, 48, 24,  8, 12, 
  12,  8, 24, 48,224,128,  1,  3,  3,  3,193, 97,103, 62, 56,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1, 
   2,  2,  4,  0,  0,224, 24,  4,  6,  1,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  1,  2,  4, 24,224,  0,  2,  2,  1,  1,  0,  0, 
   0, 12,124,198,134,195,192,192,192,128,  1,  7, 12, 24, 16, 48, 
  48, 16, 24, 12,  7,  1,128,192,192,192,195,134,230,124, 28,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1, 
   1,  1,128, 64,  0,  7, 24, 32, 64,128,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,192, 64, 32, 24,  7,  0,  0,  0,129,129,  1,  1, 
   0,  0,  0,  0,  0,  0,  0,  0, 31, 51, 32, 96, 96, 56, 28,  4, 
   4, 28,120, 96, 32, 48, 23, 15,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  4,  2, 
   1,  1,  0,  0,  0,  0,192, 48,  8,  0,  1,  1,  2,  2,  2,242, 
   2,  2,  1,  1,  0,  0,  8, 16, 32, 64,128,  0,  0,  0,  1,  1, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  3, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 
};
void pinInit() { //Inicijalizacija pinova 
    
    
    //analogni
    ADPCFGbits.PCFG6 = 0; //pin_B6_analogni
    ADPCFGbits.PCFG7 = 1; //pin_B7_digitalni
    ADPCFGbits.PCFG8 = 0; //pin_B8_analogni
    ADPCFGbits.PCFG9 = 0; //pin_B9_analogni
    
    //senzori
    TRISBbits.TRISB6 = 1; //pin_B6_ulaz(senzor_mq)
    TRISBbits.TRISB7 = 1; //pin_B7_ulaz(fotootpornik)
    
    //TOUCH
    TRISCbits.TRISC13= 0;
    TRISCbits.TRISC14= 0;
    TRISBbits.TRISB8 = 1; //pin_B8_ulaz(touchX)
    TRISBbits.TRISB9 = 1; //pin_B9_ulaz(touchY)
    
    //buzzer
    TRISAbits.TRISA11 = 0; //pin_A11_izlaz(buzzer)
    
    //bcklight
    TRISBbits.TRISB10 = 0; //pin_B10_izlaz(lcd_bckl_pin)
    
    //PIR
    TRISDbits.TRISD8 = 1; //pin_D8_ulaz(pir_senzor)
    
    //senzor_zatvorenih_vrata
    TRISFbits.TRISF6= 0; //pin_F6_izlaz(prekidac_zatvorenih_vrata)
    
    //servo
    TRISDbits.TRISD9 = 0; //pin_D9_izlaz(servo)
    
    //led
    ADPCFGbits.PCFG11 = 1;
    ADPCFGbits.PCFG12 = 1;
    TRISBbits.TRISB11 = 0;
    TRISBbits.TRISB12 = 0;
    
    
}
void __attribute__((__interrupt__)) _ADCInterrupt(void) 
{
	adc_mq = ADCBUF0;
    //adc_fo = ADCBUF1;
	touchx=ADCBUF1;
	touchy=ADCBUF2;
    
    
	temp0=touchx;
	temp1=touchy;

    IFS0bits.ADIF = 0;
} 
void __attribute__((__interrupt__)) _U1RXInterrupt(void) 
{
    IFS0bits.U1RXIF = 0;
    temp = U1RXREG;
    if(temp == 'O'){
        
        tempRX[0] = temp;
        for(n = 1; n < 5; n++){   
            while(!U1STAbits.URXDA); 
            tempRX[n] = U1RXREG;
         } 
    }

}

void __attribute__((__interrupt__)) _T1Interrupt(void)
{
   	TMR1 =0;
	brojac_ms++;//brojac milisekundi
    IFS0bits.T1IF = 0;   
}

void __attribute__((__interrupt__)) _T2Interrupt(void)
{
   	TMR2 =0;
	brojac_us++;//brojac mikrosekundi
    IFS0bits.T2IF = 0;   
}

/*void lcd_bck_light()
{
    vreme_paljenja = adc_fo/100;
    vreme_gasenja = (2800 - adc_fo)/100;
        PORTBbits.RB10 = 1;
        Delay_us(vreme_gasenja);
        PORTBbits.RB10 = 0;
        Delay_us(vreme_paljenja);
        
}*/

void Delay(unsigned int N)
{
	unsigned int i;
	for(i=0;i<N;i++);
}

void Touch_Panel (void)
{
// vode horizontalni tranzistori
	DRIVE_A = 1;  
	DRIVE_B = 0;
    
     LATCbits.LATC13=1;
     LATCbits.LATC14=0;

	Delay(500); //cekamo jedno vreme da se odradi AD konverzija
				
	// ocitavamo x	
	x_vrednost = temp0;//temp0 je vrednost koji nam daje AD konvertor na BOTTOM pinu		

	// vode vertikalni tranzistori
     LATCbits.LATC13=0;
     LATCbits.LATC14=1;
	DRIVE_A = 0;  
	DRIVE_B = 1;

	Delay(500); //cekamo jedno vreme da se odradi AD konverzija
	
	// ocitavamo y	
	y_vrednost = temp1;// temp1 je vrednost koji nam daje AD konvertor na LEFT pinu	

//Skaliranje X-koordinate
    X=(x_vrednost-161)*0.03629;

//Skaliranje Y-koordinate
	Y= ((y_vrednost-500)*0.020725);

}

void Delay_ms (int vreme)//funkcija za kasnjenje u milisekundama
{
    brojac_ms = 0;
    while(brojac_ms < vreme);
}

void Delay_us (int vreme)//funkcija za kasnjenje u milisekundama
{
    brojac_us = 0;
    while(brojac_us < vreme);
}

void servo0(){
    for(n = 0; n < 5; n++)
    {    
        LATDbits.LATD9 = 1;
        Delay_ms(1);
        LATDbits.LATD9 = 0;
        Delay_ms(19);
    }
}


void servo90(){
    
        LATDbits.LATD9 = 1;
        Delay_ms(2);
        LATDbits.LATD9 = 0;
        Delay_ms(18);
}

void buzzer_stop()//signal buzzer prilikom prisustva alkohola
{  
    unsigned int i;
    for(i = 0; i < 50; i++){

        PORTAbits.RA11 = 0;
        Delay_us(2);
        PORTAbits.RA11 = 1;
        Delay_us(10); 
      
    } 
 }

void buzzer_open()//signal buzzer prilikom prolaska testa
{  
    unsigned int i;
    for(i = 0; i < 50; i++){

        PORTAbits.RA11 = 0;
        Delay_us(5);
        PORTAbits.RA11 = 1;
         Delay_us(20); 
    } 
 }


void pocetni_ekran_ispis(){
    
    GLCD_Rectangle (0,0,127,63);
    GoToXY(7,1);
    GLCD_Printf ("Za pokretanje testa");
    GoToXY(16,2);
    GLCD_Printf ("pritisnite START");
    GLCD_Rectangle (10,33,117,53);
    GoToXY(49,5);
    GLCD_Printf ("START");
    
}

void test_ispis(){
    GoToXY(16,2);
    GLCD_Printf ("Duvajte u pravcu");
    GoToXY(40,3);
    GLCD_Printf ("uredjaja");
    
}




int main(void) {
    
    unsigned int uslov = 0, flag1 = 0, flag2 = 0, i = 0;
    GLCD_LcdInit();
    ADCinit();
    pinInit();
    ConfigureLCDPins();
    initTIMER1(1000);
    initTIMER2(10);
    initUART1();
    GLCD_ClrScr();
    ADCON1bits.ADON=1;
    
    while(1){
        
        Touch_Panel();
        pocetni_ekran_ispis();
        //***********PROVERA STANJA************ 
        if(Y > 15 && Y < 40){
            uslov = 1;
            GLCD_ClrScr();
        }
        
        while(uslov == 1){
            
            test_ispis();
        
            if(PORTDbits.RD8 && flag1 == 0) {
            
                /*RS232_putst("Korisnik je ispred uredjaja");
                WriteUART1(13);*/
                LATFbits.LATF6 = 1;
                flag1 = 1;
                flag2 = 0;
                while(adc_mq < 100);
                Delay_ms(500);
                if(adc_mq < 400)
                    flag2 = 1;
                else 
                    flag2 = 2;
            }
          
            
            while(flag1 == 1){
 
                if(adc_mq > 400 && flag2 == 2){
                    
                    LATBbits.LATB11 = 1;
                    Delay_ms(500);
                    for(i = 0; i < 10; i++)
                    {
                        buzzer_stop();
                    }
                    /*RS232_putst("Korisnik nije prosao alko test");
                    WriteUART1(13);*/
                    GLCD_DisplayPicture(fail);
                    Delay_ms(2000);
                    flag1 = 0;
                    uslov = 0; 
                    LATBbits.LATB11 = 0;
                    LATFbits.LATF6 = 0;
                    GLCD_ClrScr();
                }
            
                if((adc_mq < 400 && adc_mq > 100) && flag2 == 1){

                    LATBbits.LATB12 = 1;
                    buzzer_open();
                    servo90();
                    Delay_ms(500);
                    GLCD_DisplayPicture(pass);
                    Delay_ms(2000);
                    flag2 = 0; 
                    /*RS232_putst("Korisnik prosao alko test");
                    WriteUART1(13);*/

                }
                
                if(PORTBbits.RB7 == 1){
                    
                    LATBbits.LATB12 = 0;
                    LATFbits.LATF6 = 0;
                    servo0();
                    Delay_ms(500);
                    GLCD_ClrScr();
                    /*RS232_putst("Vrata uspesno zatvorena");
                    WriteUART1(13*/
                    flag1 = 0;
                    uslov = 0;
                }
            }    
        }
            
        //otvaranje vrata pomocu uarta komandom open
        if(tempRX[0] == 'O' && tempRX[1] == 'P' && tempRX[2] == 'E' && tempRX[3] == 'N' && flag1 == 0){
            
            for(n = 0; n < 5; n++){
                tempRX[n] = ' ';
            }
            
             flag1 = 3;
             LATFbits.LATF6 = 1;
             RS232_putst("Vrata su otvorena komandom OPEN");
             WriteUART1(13);
             servo90();
             
         }
         
        if(PORTBbits.RB7 == 1 && flag1 == 3)
        {   
           
            RS232_putst("Vrata uspesno zatvorena");
            WriteUART1(13);
            servo0();      
            LATFbits.LATF6 = 0;
            flag1 = 0;
            
        }
    }
}
