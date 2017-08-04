/*********************************************************\
*  AVR MegaSquirt - 2003                                  *
*  (C) 2003 [Michael Kristensen, mik@caffrey.dk]          *
***********************************************************
*  -- History --                                          *
*    release 0.1 - 06-03-2003                             *
*            http://caffrey.dk/megasquirt                 *
*    initial Motorola version by Bowling & Grippo (2002)  *
*            http://www.bgsoflex.com/megasquirt.html      *
\*********************************************************/


/*
TODO:
check rpmk

There is a timeout during acceleration, this should be fixed.
Is the same timeout appering in the B&G asm code?

*/


#include "global.h"
#include "fuelcalc.h"
#include "helpers.h"
#include "adc.h"

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <string.h>

#warning "THERMFACTOR locate in flash"
uint8_t THERMFACTOR[] PROGMEM = {210, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254,
    250, 246, 242, 238, 235, 231, 228, 225, 222, 220, 217, 214, 212, 209, 207, 205, 203, 200, 198, 196, 194, 192, 191, 189, 187, 185, 184,
    182, 180, 179, 177, 176, 174, 173, 171, 170, 168, 167, 166, 164, 163, 162, 161, 159, 158, 157, 156, 155, 153, 152, 151, 150, 149, 148,
    147, 146, 145, 144, 143, 142, 141, 140, 139, 138, 137, 136, 135, 134, 133, 132, 131, 130, 130, 129, 128, 127, 126, 125, 124, 124, 123,
    122, 121, 120, 119, 119, 118, 117, 116, 115, 115, 114, 113, 112, 112, 111, 110, 109, 109, 108, 107, 106, 106, 105, 104, 103, 103, 102,
    101, 101, 100, 99, 98, 98, 97, 96, 96, 95, 94, 93, 93, 92, 91, 91, 90, 89, 89, 88, 87, 87, 86, 85, 84, 84, 83, 82, 82, 81, 80, 80, 79, 78,
    78, 77, 76, 75, 75, 74, 73, 73, 72, 71, 71, 70, 69, 68, 68, 67, 66, 66, 65, 64, 63, 63, 62, 61, 60, 60, 59, 58, 57, 57, 56, 55, 54, 54, 53,
    52, 51, 50, 49, 49, 48, 47, 46, 45, 44, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 25, 24, 23, 22, 20, 19, 18,
    16, 15, 14, 12, 10, 9, 7, 5, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 210};
//#include "airdenfactor.c"
#warning "AIRDENFACTOR locate in flash"
uint8_t AIRDENFACTOR[] PROGMEM = {100, 47, 54, 58, 61, 64, 65, 67, 68, 70, 71, 72, 73, 73, 74, 75, 76, 76, 77, 77, 78, 78, 79, 79, 80, 80, 81,
    81, 82, 82, 82, 83, 83, 83, 84, 84, 84, 85, 85, 85, 86, 86, 86, 86, 87, 87, 87, 87, 88, 88, 88, 88, 89, 89, 89, 89, 89, 90, 90, 90, 90,
    90, 91, 91, 91, 91, 91, 92, 92, 92, 92, 92, 93, 93, 93, 93, 93, 93, 94, 94, 94, 94, 94, 94, 95, 95, 95, 95, 95, 95, 96, 96, 96, 96, 96,
    96, 97, 97, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 98, 98, 99, 99, 99, 99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 101, 101, 101,
    101, 101, 101, 101, 102, 102, 102, 102, 102, 102, 102, 103, 103, 103, 103, 103, 103, 103, 104, 104, 104, 104, 104, 104, 104, 105, 105, 105,
    105, 105, 105, 105, 106, 106, 106, 106, 106, 106, 106, 107, 107, 107, 107, 107, 107, 108, 108, 108, 108, 108, 108, 108, 109, 109, 109, 109,
    109, 109, 110, 110, 110, 110, 110, 110, 111, 111, 111, 111, 111, 112, 112, 112, 112, 112, 113, 113, 113, 113, 113, 114, 114, 114, 114, 114,
    115, 115, 115, 115, 116, 116, 116, 116, 117, 117, 117, 117, 118, 118, 118, 119, 119, 119, 120, 120, 120, 121, 121, 121, 122, 122, 123, 123,
    124, 124, 125, 125, 126, 127, 128, 128, 129, 130, 131, 133, 134, 136, 139, 143, 100};
//#include "barofac4115.c"
/*#warning "BAROFAC4114 locate in flash"
uint8_t BAROFAC4115[] PROGMEM = {
    100,
    141,
    141,
    141,
    141,
    141,
    140,
    140,
    140,
    140,
    139,
    139,
    139,
    139,
    139,
    138,
    138,
    138,
    138,
    138,
    137,
    137,
    137,
    137,
    137,
    136,
    136,
    136,
    136,
    136,
    135,
    135,
    135,
    135,
    135,
    134,
    134,
    134,
    134,
    134,
    133,
    133,
    133,
    133,
    133,
    132,
    132,
    132,
    132,
    132,
    131,
    131,
    131,
    131,
    130,
    130,
    130,
    130,
    130,
    129,
    129,
    129,
    129,
    129,
    128,
    128,
    128,
    128,
    128,
    127,
    127,
    127,
    127,
    127,
    126,
    126,
    126,
    126,
    126,
    125,
    125,
    125,
    125,
    125,
    124,
    124,
    124,
    124,
    124,
    123,
    123,
    123,
    123,
    122,
    122,
    122,
    122,
    122,
    121,
    121,
    121,
    121,
    121,
    120,
    120,
    120,
    120,
    120,
    119,
    119,
    119,
    119,
    119,
    118,
    118,
    118,
    118,
    118,
    117,
    117,
    117,
    117,
    117,
    116,
    116,
    116,
    116,
    116,
    115,
    115,
    115,
    115,
    115,
    114,
    114,
    114,
    114,
    113,
    113,
    113,
    113,
    113,
    112,
    112,
    112,
    112,
    112,
    111,
    111,
    111,
    111,
    111,
    110,
    110,
    110,
    110,
    110,
    109,
    109,
    109,
    109,
    109,
    108,
    108,
    108,
    108,
    108,
    107,
    107,
    107,
    107,
    107,
    106,
    106,
    106,
    106,
    105,
    105,
    105,
    105,
    105,
    104,
    104,
    104,
    104,
    104,
    103,
    103,
    103,
    103,
    103,
    102,
    102,
    102,
    102,
    102,
    101,
    101,
    101,
    101,
    101,
    100,
    100,
    100,
    100,
    100,
    99,
    99,
    99,
    99,
    99,
    98,
    98,
    98,
    98,
    98,
    97,
    97,
    97,
    97,
    96,
    96,
    96,
    96,
    96,
    95,
    95,
    95,
    95,
    95,
    94,
    94,
    94,
    94,
    94,
    93,
    93,
    93,
    93,
    93,
    92,
    92,
    92,
    92,
    92,
    91,
    91,
    91,
    91,
    91,
    90,
    90,
    90,
    90,
    90,
    100};*/
//#include "barofac4250.c"
/*#warning "BAROFAC4250 locate in flash"
uint8_t BAROFAC4250[] PROGMEM = {
    100,
    141,
    141,
    140,
    140,
    139,
    139,
    139,
    138,
    138,
    137,
    137,
    136,
    136,
    135,
    135,
    134,
    134,
    134,
    133,
    133,
    132,
    132,
    131,
    131,
    130,
    130,
    129,
    129,
    128,
    128,
    128,
    127,
    127,
    126,
    126,
    125,
    125,
    124,
    124,
    123,
    123,
    122,
    122,
    122,
    121,
    121,
    120,
    120,
    119,
    119,
    118,
    118,
    117,
    117,
    116,
    116,
    116,
    115,
    115,
    114,
    114,
    113,
    113,
    112,
    112,
    111,
    111,
    110,
    110,
    110,
    109,
    109,
    108,
    108,
    107,
    107,
    106,
    106,
    105,
    105,
    104,
    104,
    104,
    103,
    103,
    102,
    102,
    101,
    101,
    100,
    100,
    99,
    99,
    98,
    98,
    98,
    97,
    97,
    96,
    96,
    95,
    95,
    94,
    94,
    93,
    93,
    92,
    92,
    92,
    91,
    91,
    90,
    90,
    89,
    89,
    88,
    88,
    87,
    87,
    87,
    86,
    86,
    85,
    85,
    84,
    84,
    83,
    83,
    82,
    82,
    81,
    81,
    81,
    80,
    80,
    79,
    79,
    78,
    78,
    77,
    77,
    76,
    76,
    75,
    75,
    75,
    74,
    74,
    73,
    73,
    72,
    72,
    71,
    71,
    70,
    70,
    69,
    69,
    69,
    68,
    68,
    67,
    67,
    66,
    66,
    65,
    65,
    64,
    64,
    63,
    63,
    63,
    62,
    62,
    61,
    61,
    60,
    60,
    59,
    59,
    58,
    58,
    57,
    57,
    57,
    56,
    56,
    55,
    55,
    54,
    54,
    53,
    53,
    52,
    52,
    51,
    51,
    51,
    50,
    50,
    49,
    49,
    48,
    48,
    47,
    47,
    46,
    46,
    45,
    45,
    45,
    44,
    44,
    43,
    43,
    42,
    42,
    41,
    41,
    40,
    40,
    40,
    39,
    39,
    38,
    38,
    37,
    37,
    36,
    36,
    35,
    35,
    34,
    34,
    34,
    33,
    33,
    32,
    32,
    31,
    31,
    30,
    30,
    29,
    29,
    28,
    28,
    28,
    27,
    27,
    27,
    27,
    27,
    27,
    100};*/
//#include "kpafactor4115.c"
/*#warning "KPAFACTOR4115 locate in flash"
uint8_t KPAFACTOR4115[] PROGMEM = {
    100,
    10,
    11,
    11,
    12,
    12,
    13,
    13,
    14,
    14,
    14,
    15,
    15,
    16,
    16,
    17,
    17,
    17,
    18,
    18,
    19,
    19,
    20,
    20,
    21,
    21,
    21,
    22,
    22,
    23,
    23,
    24,
    24,
    24,
    25,
    25,
    26,
    26,
    27,
    27,
    27,
    28,
    28,
    29,
    29,
    30,
    30,
    31,
    31,
    31,
    32,
    32,
    33,
    33,
    34,
    34,
    34,
    35,
    35,
    36,
    36,
    37,
    37,
    38,
    38,
    38,
    39,
    39,
    40,
    40,
    41,
    41,
    41,
    42,
    42,
    43,
    43,
    44,
    44,
    44,
    45,
    45,
    46,
    46,
    47,
    47,
    48,
    48,
    48,
    49,
    49,
    50,
    50,
    51,
    51,
    51,
    52,
    52,
    53,
    53,
    54,
    54,
    55,
    55,
    55,
    56,
    56,
    57,
    57,
    58,
    58,
    58,
    59,
    59,
    60,
    60,
    61,
    61,
    61,
    62,
    62,
    63,
    63,
    64,
    64,
    65,
    65,
    65,
    66,
    66,
    67,
    67,
    68,
    68,
    68,
    69,
    69,
    70,
    70,
    71,
    71,
    71,
    72,
    72,
    73,
    73,
    74,
    74,
    75,
    75,
    75,
    76,
    76,
    77,
    77,
    78,
    78,
    78,
    79,
    79,
    80,
    80,
    81,
    81,
    82,
    82,
    82,
    83,
    83,
    84,
    84,
    85,
    85,
    85,
    86,
    86,
    87,
    87,
    88,
    88,
    88,
    89,
    89,
    90,
    90,
    91,
    91,
    92,
    92,
    92,
    93,
    93,
    94,
    94,
    95,
    95,
    95,
    96,
    96,
    97,
    97,
    98,
    98,
    99,
    99,
    99,
    100,
    100,
    101,
    101,
    102,
    102,
    102,
    103,
    103,
    104,
    104,
    105,
    105,
    105,
    106,
    106,
    107,
    107,
    108,
    108,
    109,
    109,
    109,
    110,
    110,
    111,
    111,
    112,
    112,
    112,
    113,
    113,
    114,
    114,
    115,
    115,
    116,
    116,
    116,
    117,
    117,
    118,
    118,
    119,
    119,
    119,
    120,
    120,
    121,
    100};*/
//#include "kpafactor4250.c"
#warning "KPAFACTOR4250 locate in flash"
uint8_t KPAFACTOR4250[] PROGMEM = {100, 100, 33, 34, 35, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,
64, 65, 66, 67, 68, 69, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 86, 87, 88, 89, 90, 91, 92, 93, 94,
95, 96, 97, 98, 99, 100, 101, 102, 103, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120,
121, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 138, 139, 140, 141, 142, 143, 144,
145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169,
170, 171, 172, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 189, 190, 191, 192, 193,
194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218,
219, 220, 221, 222, 223, 224, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 241, 242,
243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
255, 255, 255, 255, 100};

volatile extern struct squirt_t inj_port1, inj_port2;
volatile extern struct engine_t engine;
volatile extern struct time_t rtc;
volatile extern uint8_t sensors[];
extern struct config_t config;

volatile uint8_t tpsaclk; // TPS enrichment timer clock in 0.1 second resolution

uint8_t tpsaclkcmp; // Comparison value for TPS acceleration time - from lookup table
//uint8_t tpsfuelcut; // TPS Fuel Cut (percent)

volatile uint8_t egocount; // Counter value for EGO step - incremented every ignition pulse
volatile uint8_t asecount; // Counter value for after-start enrichment counter - every ignition pulse

struct corr_t corr;


void init_fuelcalc(void) {
  corr.ego = 100;
  corr.air = 100;
  corr.warm = 100;
  corr.baro = 100;
  corr.gammae = 100;
  corr.ve = 100;
  corr.tpsaccel = 0;
  corr.tpsfuelcut = 100;

}


/***************************************************************************/
/*** warm-up and after-start enrichment                                  ***/
/***************************************************************************/
void warmup_enrich(void) {
  uint8_t warm_enrich;
  uint8_t my_status;
  struct search_table_t st;

  /* обогащение на прогреве */
  cli();
  my_status = engine.status;
  if (my_status & (uint8_t) _BV(crank)) {
    my_status &= ~_BV(crank);
    my_status |= _BV(startw) | _BV(warmup);
    engine.status = my_status;
    asecount = 0;
  }
  sei();

#warning "wwurange should be located in flash"

  /* используем температуру двигателя для нахождения wwurange, */
  /* "упакуем" 256 возможных значений сигнала датчика в [0-9] шкалу */
  search_table(config.wwurange, sizeof(config.wwurange), engine.coolant, &st);

  /* затем, используя интерполяцию по wwu-table, */
  /* а также шкалу [0-9] для нахождения значения обогащения */
  warm_enrich = linear_interp(st.lbound, st.ubound, config.wwu[st.index-1], 
                              config.wwu[st.index], engine.coolant);

  if (warm_enrich > 100) {
    //PORTA |= _BV(LED_WARMUP); /* зажечь светодиод - "прогрев" */
    
	cli();
    engine.status |= _BV(warmup);
    sei();
    
	/* обогащение после пуска двигателя */
	if (engine.status & _BV(startw)) {
        if (asecount <= config.awc) {
	      uint16_t wue;
	      wue = warm_enrich + linear_interp(0, config.awc, config.awev, 0, asecount);
	      if ((wue >> 8) != 0) /* проверка на переполнение */
	        wue = 0xFF;
	      warm_enrich = (uint8_t)wue;
        } else {
	      cli();
	      engine.status &= ~_BV(startw);
	      sei();
        }
    }
  } else {
    /* обогащения на прогреве закончилось */
    warm_enrich = 100;
    
	cli();
    engine.status &= ~( _BV(startw) | _BV(warmup) );
    sei();
    
	//PORTA &= ~_BV(LED_WARMUP);  /* погасить свтодиод - "прогрев" */
  }

  corr.warm = warm_enrich;
}


/***************************************************************************/
/*** Throttle posistion acceleration enrichment                          ***/
/***************************************************************************/
void tps_acc_enrich(void) {
  uint8_t tps_diff;
  uint8_t my_tps, my_last_tps;
  
  cli();
  my_tps = engine.tps;
  my_last_tps = engine.last_tps;
  sei();

  if (my_tps < my_last_tps) {                             /* замедление */
    tps_diff = my_last_tps - my_tps;
    
    if (tps_diff >= config.tps_thresh) {                  /* скорость изменения положения дросселя выше порога? */

        if (engine.status & _BV(tpsaen)) {
		    corr.tpsfuelcut = 100;
		    corr.tpsaccel = 0;
		    cli();
		    engine.status &= ~( _BV(tpsaen) | _BV(tpsden) );
		    sei();
		    //PORTA &= ~_BV(LED_ACCEL);                     /* погасить свтодиод - "ускорение" */
	  
        } else {                                          /* топливоограничение */
		    if (engine.rpm >= config.fuelcut_thres) {
			    corr.tpsfuelcut = config.tpsdq;
			    cli();
			    engine.status &= ~_BV(tpsaen);
			    engine.status |= _BV(tpsden);
			    sei();
			    //PORTA &= ~_BV(LED_ACCEL);                 /* погасить свтодиод - "ускорение" */
		    }
        }
    } else {                                              /* топливоограничение выполнено */
        if ((engine.status & _BV(tpsden)) && (engine.rpm < config.fuelcut_thres)) {            
		    cli();
		    engine.status &= ~( _BV(tpsaen) | _BV(tpsden) );
		    sei();
		    corr.tpsfuelcut = 100;
		    corr.tpsaccel = 0;
      }
    }
  } else {                                                /* ускорение */
    tps_diff = my_tps - my_last_tps;
    if (tps_diff >= config.tps_thresh) {
    /*
	  Вычисление ускорения должно быть переделано, это - неточно.
      Это очень долго, и вычисление обогащения начинается снова.
	*/
      if (engine.status & _BV(tpsaen)) {
		uint8_t acc_temp_offset, acc_temp_mult, acc_enrich;
		uint16_t mtmp;
		struct search_table_t st;

		/* multiplier amount based on cold temperature */
		acc_temp_mult = linear_interp(0, 205, config.acmult, 100, engine.coolant);
	
		/* lookup table amount based on tpsdot */
		search_table(config.tpsdotrate, sizeof(config.tpsdotrate), tps_diff, &st);
		acc_enrich = linear_interp(st.lbound, st.ubound, config.tpsaq[st.index-1], config.tpsaq[st.index], tps_diff);

		/* calculate acc-enrichment, catch overflow */
		mtmp = mult_div100(acc_temp_mult, acc_enrich);
		if ((mtmp >> 8) != 0) // will the result fit in a byte?
			mtmp = 200;         // ... no!

		/* extra fuel due to temperature */
		acc_temp_offset = linear_interp(0, 205, config.tpsacold, 0, engine.coolant);

		mtmp += acc_temp_offset;
		if ((mtmp >> 8) != 0) // will the result fit in a byte?
			mtmp = 0xFE;        // ... no!
	
		if (mtmp > corr.tpsaccel) 
			corr.tpsaccel = (uint8_t)mtmp; // tpsaccel is the result of this function

		if ((engine.status & _BV(tpsden)) || (tpsaclk >= tpsaclkcmp))  {
			cli();
			engine.status &= ~( _BV(tpsaen) | _BV(tpsden) );
			sei();
			corr.tpsfuelcut = 100;
			corr.tpsaccel = 0;
			//PORTA &= ~_BV(LED_ACCEL); 	/* погасить светодиод - "ускорение" */
		}
      
	  /* старт ускорения, инициализация некоторых переменных */
	  } else { 
		corr.tpsaccel = config.tpsaq[0];
		tpsaclk = 0;
		tpsaclkcmp = config.tpsasync;
		cli();
		engine.status |= _BV(tpsaen); 	/* установить флаг - ускорение */
		engine.status &= ~_BV(tpsden); 	/* сбросить флаг - замедление */
		sei();
		//PORTA |= _BV(LED_ACCEL); 		/* зажечь светодиод - "ускорение" */
      }
    
	/* ускорение выполненно */
	} else { 
      if ((engine.status & _BV(tpsden)) || (tpsaclk >= tpsaclkcmp)) {
		cli();
		engine.status &= ~( _BV(tpsaen) | _BV(tpsden) );
		sei();
		corr.tpsfuelcut = 100;
		corr.tpsaccel = 0;
		//PORTA &= ~_BV(LED_ACCEL);     	/* погасить свтодиод - "ускорение" */
      }
    }
  }
}


/***************************************************************************/
/*** Exhaust gas oxygen sensor measurement section                       ***/
/***************************************************************************/
void o2(void) {
  uint8_t limit, new_ego, o2_is_lean;

  if ( (config.egodelta == 0) ||                            /* EGO коррекция отключена */
       (engine.rpm < config.rpmoxlimit) ||                  /* обороты двигателя малы */
       (engine.status & (_BV(tpsaen) | _BV(tpsden))) ||     /* активно обогащение смеси */
       (engine.coolant < config.egotemp) ||                 /* температура двигателя мала */
       (engine.tps > O2_MAX_TPS) || 						/* тапка в пол! */
       (engine.status_ext & _BV(o2_not_ready)) ||           /* O2S не готов (не прогрет) */ 
       (engine.kpa > O2_MAX_KPA) ) {                        /* велика нагрузка на двигатель */

    corr.ego = 100;
    egocount = 0;

  } else {
    if (egocount > config.egocountcmp) {          /* расстояние (в событиях зажигания) между шагами EGO корректора */
      egocount = 0;

      // do we want variable AFR?
      // then search bin and interpolate(kpa)
    
        if (sensors[EGO] != config.voltoxtarget) { /* если равно, нет никакой необходимости в регулировании */
	        if (sensors[EGO] < config.voltoxtarget) {
	            if (config.config13 & _BV(O2_WB_SENSOR))
	                o2_is_lean = false;            /* характеристика ШДК имеет обратный наклон */
	            else
	                o2_is_lean = true;
	        } else {
	            if (config.config13 & _BV(O2_WB_SENSOR))
	                o2_is_lean = true;             /* характеристика ШДК имеет обратный наклон */
	            else
	                o2_is_lean = false;
	        }
	        if (o2_is_lean) {                      /* бедно */
	            limit = 100 + config.egolimit;
	            new_ego = corr.ego + config.egodelta;

	            if (new_ego > limit)
	                corr.ego = limit;
	            else
	                corr.ego = new_ego;
	        } else {      			               /* богато */
	            limit = 100 - config.egolimit;
	            new_ego = corr.ego - config.egodelta;

	            if (new_ego < limit)
	                corr.ego = limit;
	            else
	                corr.ego = new_ego;
	        }
        }
    }
  }
}


/***************************************************************************/
/*** VE table lookup                 objomnaja effektivnostj             ***/
/***************************************************************************/
void ve_table_lookup(void) {
  uint8_t ve_11, ve_12, ve_21, ve_22; // napolnenie
  uint8_t ve_low_kpa, ve_high_kpa; //kpa kiopaskali
  struct search_table_t kpa, rpm; //rpm oboroti

  if (config.config13 & _BV(CONTROL_STRATEGY)) engine.kpa = engine.tps; //tps polozhenie droseljnoj zaslonki
	//{ //Alpha-N
	//	engine.kpa = engine.tps; //tps polozhenie droseljnoj zaslonki
	//}

  search_table(config.kparangeve, sizeof(config.kparangeve), engine.kpa, &kpa);
  search_table(config.rpmrangeve, sizeof(config.rpmrangeve), engine.rpm, &rpm);

  ve_11 = *(config.VE+8*(kpa.index-1)+(rpm.index-1));
  ve_12 = *(config.VE+8*(kpa.index-1)+rpm.index);
  ve_21 = *(config.VE+8*kpa.index+(rpm.index-1));
  ve_22 = *(config.VE+8*kpa.index+rpm.index);
  
  ve_low_kpa = linear_interp(rpm.lbound, rpm.ubound, ve_11, ve_12, engine.rpm);
  ve_high_kpa = linear_interp(rpm.lbound, rpm.ubound, ve_21, ve_22, engine.rpm);

  corr.ve = linear_interp(kpa.lbound, kpa.ubound, ve_low_kpa, ve_high_kpa, engine.kpa);
}




/***************************************************************************/
/*** calc total enrichment                                               ***/
/***************************************************************************/
void calc_total_enrichment(void) {
  uint8_t fuel_tmp, batt_tmp, pw;
  uint8_t batt_high, batt_low;
  uint16_t res;

  // 8-bit x 16-bit multiplications
  res = (uint16_t)corr.warm;
  res = mult_div100(corr.tpsfuelcut, res);
  res = mult_div100(corr.air, res);
  res = mult_div100(corr.ego, res);
  res = mult_div100(corr.baro, res);

  if ((res >> 8) != 0)                             /* проверка на переполнение */
    corr.gammae = 0xFF;
  else
    corr.gammae = (uint8_t)res;                    /* используется для ведения логов */

  res = mult_div100(corr.ve, res);
  
  if (!(config.config13 & _BV(CONTROL_STRATEGY)))  // speed-density
    res = mult_div100(engine.kpa, res);

  res = mult_div100(config.req_fuel, res);

  if ((res >> 8) != 0)                             // противное переполнение "... синий экран смерти!"
    fuel_tmp = 0xFF;
  else
    fuel_tmp = (uint8_t)res;


  /* battery voltage compensation */
  /* вспомним, низкое напряжение бортовой сети только увеличивает время открытия форсунок */
  batt_low = config.injopen + config.battfac;

  if (config.injopen <= config.battfac)
    batt_high = 0;
  else
    batt_high = config.injopen - config.battfac;

#warning "this should be configurable via configuration struct"
  batt_tmp = linear_interp(61, 164, batt_low, batt_high, sensors[BATT]);

  // final pulsewidth calculation, wuhuw!
  if (fuel_tmp) {
    res = batt_tmp + fuel_tmp + corr.tpsaccel - config.injocfuel;
    if ((res >> 8) != 0)
      pw = 0xFF;
    else
      pw = (uint8_t)res;
  } else {
    pw = 0;
  }

  cli();
  inj_port1.pwcalc = pw;
  inj_port2.pwcalc = pw;
  sei();

}


/***************************************************************************/
/*** calc parameters                                                     ***/
/***************************************************************************/
void calc_parameters(void) {

  // Manifold Air Pressure in kiloPascals
  if (config.config11 & _BV(MAP_SENSOR))
    engine.kpa = PRG_RDB(&KPAFACTOR4250[sensors[MAP]]);
  else
    engine.kpa = PRG_RDB(&KPAFACTOR4250[sensors[MAP]]); //KPAFACTOR4115

  // Barometric correction enabled?
  /*if (config.config13 & _BV(BARO_CORRECTION)) {
    uint8_t val = sensors[BARO];

    // If the mcu resets while the engine is running, we don't believe 
    // 20 kPa being the atmospheric pressure.
    if ( (val < (config.baro - config.dbaro)) || 
	 (val > (config.baro + config.dbaro)) ) {
#warning "MegaTune wants the raw sensor value and will not discover the cheat!"
      corr.baro = config.baro;
      cli();
      engine.status_ext |= _BV(baro_problem);
      sei();
    } else {
      if (config.config11 & _BV(MAP_SENSOR))
	corr.baro = PRG_RDB(&BAROFAC4250[sensors[BARO]]);
      else
	corr.baro = PRG_RDB(&BAROFAC4250[sensors[BARO]]); //BAROFAC4115
    }
  } else {
    corr.baro = 100; // no, not enabled
  }*/
  corr.baro = 100; //otklju4il barometri4eskuju korrekciju
  // Coolant temperature in degrees F + 40
  engine.coolant = PRG_RDB(&THERMFACTOR[sensors[CLT]]);

  // Air Density Correction Factor
  corr.air = PRG_RDB(&AIRDENFACTOR[sensors[MAT]]);

  // Interpolate throttle position
  // this makes it really smooth to install the tps
  // Should the tps be scaled 0..255 or 0..100% ?
  engine.tps = linear_interp(config.tps_low, config.tps_high, 0, 255, sensors[TPS]);

}


/***************************************************************************/
/*** расчет rpm                                                          ***/
/***************************************************************************/
void calc_rpm(void) {
  uint16_t my_rpm, my_rpmk;

  /* необходимо чтобы оба байта переменнй были переданны атомарно */
  cli();
  my_rpm = engine.rpm_p;
  sei();

  my_rpmk = (config.rpmk_1 << 8) | config.rpmk_2;

  /* расчет rpm */
  if (my_rpm) {                  /* удостоверимся, что двигатель вращается */
      my_rpm = my_rpmk/my_rpm;

      if ((my_rpm >> 8) != 0)
	     engine.rpm = 255;       /* диапаона в 25500 об/мин " ... хватит каждому!" (Bill Gates '81) */
      else
	     engine.rpm = my_rpm & 0xFF;
  } else {
      engine.rpm = 0;
  }
}


/***************************************************************************/
/*** режим запуска двигателя                                             ***/
/***************************************************************************/
void cranking(void) {
  uint8_t pw;
  uint8_t my_status;

  cli();
  my_status = engine.status;
  my_status |= _BV(crank);
  my_status &= ~( _BV(startw) | _BV(warmup) );
  engine.status = my_status;
  sei();

  /* расчет обогащения при пуске */
  
  /* если TPS меньше 3 volt */
  if (engine.tps < FLOOD_CLEAR) { 
    
	/* расчет ширины импульса [cwl..cwh] и интерполяция по температуре ОЖ */
	pw = linear_interp(0, 205, config.cwl, config.cwh, engine.coolant);
    // battery correction?
  
  /* продувка камер сгорания*/
  } else { 
    pw = 0; 
  }
  
  cli();
  inj_port1.pwcalc = pw;
  inj_port2.pwcalc = pw;
  sei();
}


/***************************************************************************/
/*** primepulse                                                          ***/
/***************************************************************************/
void primepulse(void) {

  if (config.primep_cold || config.primep_warm) {
    uint8_t pw, tps;

    tps = linear_interp(config.tps_low, config.tps_high, 0, 255, sensors[TPS]);

    // don't prime if the engine is flooded
    if (tps < FLOOD_CLEAR) {
      pw = linear_interp(0, 205, config.primep_cold, config.primep_warm, PRG_RDB(&THERMFACTOR[sensors[CLT]]));
      // battery correction?
      inj_port1.pw = pw;
      inj_port2.pw = pw;
      inj_port1.pwrun = 0;
      inj_port2.pwrun = 0;
      inj_port1.status |= _BV(enabled) | _BV(scheduled);
      inj_port2.status |= _BV(enabled) | _BV(scheduled);
      engine.status |= _BV(running);
    }
  }
}
