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


#include "storage.h"
#include "global.h"
#include <inttypes.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

extern volatile struct squirt_t inj_port1, inj_port2;

struct config_t config;
struct config_t config_ee SEEPROM;

const uint8_t code_ver = 20;
extern uint8_t gammae;

void loadConfig(void) {

  /* ������ ���������� �� eeprom � sram */
  eeprom_read_block(&config, &config_ee, sizeof(struct config_t));

  /*
    the squirt-datastructure contains its own pwm constants,
    MegaTune supports _one_ shared set of constants. Copy that set
    to both data structures until configuration program is done.
  */
  
  inj_port1.pwm_delay = config.injpwmt;
  inj_port1.pwm_dc = config.injpwm;

  inj_port2.pwm_delay = config.injpwmt;
  inj_port2.pwm_dc = config.injpwm;

  /* 
     these are currently initialized statically,
     haven't had time for doing a Linux program 
     for configuration of these variables
  */

  config.fuelcut_thres = 0x0F;         /* ����� ������������������ �� �������� ��������� (1500 rpm) */
  config.cranking_thres = 3;           /* ����� ������ ����� �� �������� ��������� (300 rpm) */

  config.primep_warm = config.primep;  /* ������������ ����������������� �������� ��� -40 � */
  config.primep_cold = config.primep;  /* ������������ ����������������� �������� ��� +77 � */

  config.tps_low = 0; //12;            /* ����� ������ ��������� �������� ADC TPS */
  config.tps_high = 255; //220;        /* ����� ������� ��������� �������� ADC TPS */

  config.wwurange[0] = 0;              // should be stored in flash instead
  config.wwurange[1] = 20;
  config.wwurange[2] = 40;
  config.wwurange[3] = 60;
  config.wwurange[4] = 80;
  config.wwurange[5] = 100;
  config.wwurange[6] = 120;
  config.wwurange[7] = 140;
  config.wwurange[8] = 170;
  config.wwurange[9] = 200;

  config.tpsdotrate[0] = 5;            // should be stored in flash instead
  config.tpsdotrate[1] = 20;
  config.tpsdotrate[2] = 40;
  config.tpsdotrate[3] = 77;

  config.fan_temp = 234;               /* ����������� ��������� ����������� ���������� */
  config.fan_hyst = 5;                 /* ���������� ��� ���������/���������� ����������� ���������� */

  /* ��� ��������� ��� �������� ��������� ������� ��� ������ �������� �� ��������� ��������� */
  config.iac_step_seq = 0xD8;			/* ���������� ������������������ '�������' �� ��� */
  config.iac_conf = 0x90;				/* bit 7:4 - �������� ������ �� ��� [����/���] (0x90 == 9 ����/���) */
  config.iac_warm_idle = 9;				/* ������� �� ���������� ��������� [x100 rpm] */
  config.iac_cold_idle = 15;			/* ������� �� ������������ ��������� [x100 rpm] */
  config.iac_skip = 40;					/* ��������� �������� ��� ������ ������������� �� [raw TPS ADC] */
  config.iac_decel = 5;					/* ���������� ����� ��� ��������� ������� ��� ���������� */
  config.iac_step_coarse = 5;			/* ���������� ����� �� ���� �������� ��� ������ ����������� */
  config.iac_step_fine = 1;				/* ���������� ����� �� ���� �������� ��� ������ ����������� */
  config.iac_backoff_cold = 80;			/* ���������� ����� �� �������� ����� ������������� ��� -40 � */
  config.iac_backoff_warm = 20;			/* ���������� ����� �� �������� ����� ������������� ��� +77 � */
  config.iac_max_close = 100;			/* ���������� ����� ��� ������������� �������� ������� */

  config.baro = 100;                   /* mean barometric reading */
  config.dbaro = 20;                   /* max deviation from mean reading */

}

volatile uint8_t eeprom_store_idx;     /* ������� �������������� ���� ������*/
volatile uint8_t eeprom_store_busy;    /* ����, eeprom ������*/

/*
  **************************************************************************
  *** storeConfig                                                        ***
  **************************************************************************

  ��� ������� ��������� ������� ���������� ��������� ������������ �� sram � 
  eeprom � ���������� ���������� ����������,�������� ����������� ���������� 
  EEPROM ���������� ������ �� ���������� ������.
*/
uint8_t storeConfig(void) {

  if (eeprom_store_busy) {
    return -1; 				/* eeprom ������, ���������! */
  }

  eeprom_store_idx = 0;		/* �������� ������� �������������� ���� ������*/
  eeprom_store_busy = 1;	/* ���������� ����, eeprom ������*/
  
  EECR |= _BV(EERIE);		/* ��������� ���������� �� eeprom */
  return 0;
}


/*
  ************************************************************************
  *** INTERRUPT: EEPROM ready 										   ***
  ************************************************************************

  ����� ���������� ����� ���������� ����� ���� ���������� ������� � ������ 
  ����� ������ ��������, ��� ������ ���� ����������� �����������!
*/
ISR(EE_READY_vect) {
  uint8_t *pe, *ps;
  uint8_t ebyte;

  pe = (uint8_t *)&config_ee + eeprom_store_idx; /* ����� ������ � eeprom */
  ps = (uint8_t *)&config + eeprom_store_idx;    /* ����� ������ � sram */
  ebyte = eeprom_read_byte(pe);

  /* �������������� ������ � eeprom ����� ������ ���� ��� ���������� */

  /* 
    ������� ���������� �������� � sram � eeprom, ����� �� ����� 
	������������ �� ������� ���������� �������� � ������
  */	 
  while( (ebyte == *ps) && (eeprom_store_idx != (sizeof(struct config_t) - 1)) ) {
    eeprom_store_idx++;
    pe++;
    ps++;
    ebyte = eeprom_read_byte(pe);
  }

  /* ���������� ����� �������� � eeprom */
  if (ebyte != *ps)
    eeprom_write_byte(pe, *ps);

  /*
    ���� ��������� ������ � ��������� �� ���������� ��������� � ������ 
	���������� �������� ������ � sram � eeprom 
  */
  if (eeprom_store_idx == sizeof(struct config_t)-1) { 	/* ������ ��������� */
      EECR &= ~_BV(EERIE);     							/* ��������� ���������� �� eeprom */
      eeprom_store_busy = 0;							/* �������� ����, eeprom ������*/
      //PORTA &= ~_BV(LED_MISC); 							/* �������� ��������� */
  } else { 												/* ��������� � ���������� �������� */
      eeprom_store_idx++;
      //PORTA |= _BV(LED_MISC); 							/* ������ ��������� */
  /* 
    ���������� ��������, ���� ��������� ����� ����� 5 ��� ������� ����� ������� 
	� ����������� ����� �������� ����������� eeprom
  */
  }
}

