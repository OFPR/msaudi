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


// rs232 communication

/*
  ���� ���� ����������� ���������� ����������!

 ������ ����, ����� ��������� �������� ��������� ����� ��� ������ uart, 
 ������������ ��������� ��������� ����� ������� ������ ('command_queue').
 ���� ����� �������� ����� � ��������� ������� ������� ����� �������� 
 ����� ������ �� ������ ����� � ������.

 ��������, MegaTune �������� 'A'- ������ ���������� � �������� �������:
 1) ���������� Uart �������� ������� comm()
 2) comm() �������� ����� � ����� ���� ������� (22 �����) ��� sendRTvar()
    � ������� ������ � ��������� ���������� ����������� uart (tx-interrupt).
 3) � ����������� ��������� ����������� uart �������� ������� ������, 
    ����������� ������� � ����������� ����� ���� ������� (������� ����� 
	������������ � ������� ������). ���� ����� ���� ������� ����� ����, 
	������� ��������� �� �������. ���� ������� �����, �� ����������� 
	���������� uart	�����������.
*/

#include "global.h"
#include "comm.h"
#include "storage.h"
#include "actuators.h"

#include <avr/io.h>
#include <inttypes.h>
#include <avr/sfr_defs.h>
#include <avr/interrupt.h>

extern volatile uint8_t sensors[];
extern volatile struct squirt_t inj_port1, inj_port2;
extern volatile struct engine_t engine;
extern volatile struct config_t config;
extern volatile struct time_t rtc;
extern volatile struct step_t step;
extern struct corr_t corr;

extern uint8_t code_ver;

struct cmd_queue_t cmd_queue[CMD_QUEUE_SIZE];
uint8_t cmd_queue_head, cmd_queue_tail;

/* ������ ������ */
#define CMD    0
#define OFFSET 1
#define VALUE  2

/***************************************************************************/
/*** comm                                                                ***/
/***************************************************************************/
void comm(uint8_t rx_data) {
  static uint8_t comm_state = CMD;
  static uint8_t rx_offset = 0;

  switch (comm_state) {
	case OFFSET:
      rx_offset = rx_data;  				/* �������� �������� ��� ���������� ������ �������� ����� */
      comm_state = VALUE;
    break;

	case VALUE:
      storeConfigVar(rx_offset, rx_data);
      comm_state = CMD;
    break;

	case CMD:
      switch (rx_data) {
		
		case 'A': 							/* ������ ���������� � �������� ������� */	
		  pushfunc(&sendRTvar, 22);
		break;
		
		case 'B': 							/* �������� ������������ � eeprom */	
		  storeConfig();
		break;
		
		case 'C': 							/* �������� �������������� ���������� */	
		  pushfunc(&testComm, 1);
		break;
		
		case 'F':                           /* �������� ������ ����������� (non MS cmd) */    
		  UCSR0B &= ~_BV(UDRIE0);               	/* ��������� ���������� ����������� uart */
		  cmd_queue_head = cmd_queue_tail = 0;  	/* �������� ������� */
		break;
		
		case 'Q': 							/* ������ ������ ��� */	
          pushfunc(&codeVer, 1);
		break;
		
		case 's': 							/* ���������� ������� ��� �� 10 ticks (non MS cmd) */	
		  cli();
		  step.count = 10;
		  step.status &= ~_BV(direction);
		  step.status |= _BV(busy);
		  sei();
		break;
		
		case 'S': 							/* �������� ������� ��� �� 10 ticks (non MS cmd) */	
		  cli();
		  step.count = 10;
		  step.status |= _BV(direction);
		  step.status |= _BV(busy);
		  sei();
		break;
		
		case 'T': 							/* ������ ������ ������ ������������ (non MS cmd) */	
		  pushfunc(&sendConfigVar, sizeof(struct config_t));
		break;
		
		case 'V': 							/* ������ �������� � ������� VE */	
		  pushfunc(&sendConfigVar, 0x7c);
		break;
		
		case 'W': 							/* �������� ����� �������� VE ��� ��������� */		
		  comm_state = OFFSET;                  	/* �� ������� 'W'+<offset>+<newbyte> */
		break;
  
		default:
		  
		break;
    }
  }
}


/***************************************************************************/
/*** pushfunc                                                            ***/
/***************************************************************************/
uint8_t pushfunc(uint8_t (*f)(uint8_t i), uint8_t len) {
  uint8_t next_head;

  next_head = (cmd_queue_head + 1) % CMD_QUEUE_SIZE;
  if (next_head == cmd_queue_tail)
    return -1; 										// ���, ������� ���������

  /* len starts at 0 */
  if (len > 0) 										// this looks like a hack - it is!
      len = len - 1;

  cmd_queue[cmd_queue_head].func = f;
  cmd_queue[cmd_queue_head].len = len;
  cmd_queue[cmd_queue_head].count = 0;

  cmd_queue_head = next_head;
  
  UCSR0B |= _BV(UDRIE0); 							// enable transmit interrupt
  return 0;
}

/***************************************************************************/
/*** codeVer                                                             ***/
/***************************************************************************/
// return codeversion
uint8_t codeVer(uint8_t i) {
  return (code_ver);
}

/***************************************************************************/
/*** testComm                                                            ***/
/***************************************************************************/
// return low part of second-count
uint8_t testComm(uint8_t i) {
  return (rtc.sec & 0xFF);
}

/***************************************************************************/
/*** sendRTvar                                                           ***/
/***************************************************************************/
// convert datastructures to megasquirt compatible format
uint8_t sendRTvar(uint8_t i) {
  uint8_t a, b;

  switch (i) {
  case 0: 
    return (uint8_t)rtc.sec;  		// secl

  case 1:                           // squirt
    // � ������� squirt_t ���� ���� ����������� ���� ���������, ��������� ��� ����
    a = inj_port1.status;
    b = inj_port2.status;

    return ((a & 0x01) << 0) | ((a & 0x02) << 1) | ((a & 0x04) << 1) |
           ((b & 0x01) << 1) | ((b & 0x02) << 3) | ((b & 0x04) << 3); 

  case 2:  return engine.status;  	// engine
  case 3:  return sensors[BARO];  	// baro
  case 4:  return sensors[MAP];   	// map
  case 5:  return sensors[MAT];   	// mat
  case 6:  return sensors[CLT];   	// clt
  case 7:  return sensors[TPS];   	// tps
  case 8:  return sensors[BATT];  	// batt
  case 9:  return sensors[EGO];   	// ego
  case 10: return corr.ego;       	// egocorr
  case 11: return corr.air;       	// aircorr
  case 12: return corr.warm;      	// warmcorr
  case 13: return engine.rpm;     	// rpm
  case 14: return inj_port1.pw;   	// pw
  case 15: return corr.tpsaccel;  	// tpsaccel
  case 16: return corr.baro;      	// barocorr
  case 17: return corr.gammae;    	// gammae
  case 18: return corr.ve;        	// vecorr
  case 19: return step.count;              	// ������ ���� 1
  case 20: return 2;              	// ������ ���� 2
  case 21: return 3;              	// ������ ���� 3

  default: return 0;
  }
}

/***************************************************************************/
/*** storeConfigVar                                                      ***/
/***************************************************************************/
void storeConfigVar(uint8_t addr, uint8_t value) {
  
  if (addr < sizeof(struct config_t)) {       /* � ���������� � ������� �� ��������! */
    *( (uint8_t *)&config + addr ) = value;

    if (addr == 0x5F) {                       /* Injector PWM duty cycle at current limit */
      inj_port1.pwm_dc = value;
      inj_port2.pwm_dc = value;
    } else if (addr == 0x60) {                /* Injector PWM mmillisec time at which to activate */
      inj_port1.pwm_delay = value;
      inj_port2.pwm_delay = value;
    }
  }
}

/***************************************************************************/
/*** sendConfigVar                                                       ***/
/***************************************************************************/
// ������� ���������� ���� �������� �� ������� ���������������� ������
// �������� �� ������������ ������ �����������
uint8_t sendConfigVar(uint8_t i) {
  return *( (uint8_t *)&config + i );
}

/***************************************************************************/
/*** initUART                                                            ***/
/***************************************************************************/
void initUART(void) {

  /* rx enable, tx enable */
  //UCSR0B = _BV(RXCIE0) | _BV(UDRIE0) | _BV(RXEN0) | _BV(TXEN0);
  UCSR0B = _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0);
  /* 8 databit, 1 stopbit */
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);

  /* 9600 baud @��16MHz */
  UBRR0H = 0;
  UBRR0L = 103;

  cmd_queue_head = cmd_queue_tail = 0;
  
}

/***************************************************************************/
/*** INTERRUPT: UART rx complete                                         ***/
/***************************************************************************/
// a small receivebuffer is still waiting to be implemented
ISR(USART_RX_vect) {
    
  /* ��������� ������� ������� UART �� ������ ������������ ��� ������ ��� ������ ������ ����� */
  if ( UCSR0A & (_BV(FE0) | _BV(DOR0)) ) { 
	volatile uint8_t dummy;
	dummy = UDR0; 		/* ���������� ��������� ��� � ���� ������ �� ����� */
	return;
  } else {
	comm(UDR0);
  }
}

/***************************************************************************/
/*** INTERRUPT: UART tx empty                                            ***/
/***************************************************************************/
ISR(USART_UDRE_vect) {

	//call function and get data to send
	UDR0 = (*cmd_queue[cmd_queue_tail].func)(cmd_queue[cmd_queue_tail].count);

	if (cmd_queue[cmd_queue_tail].count < cmd_queue[cmd_queue_tail].len) {
		cmd_queue[cmd_queue_tail].count++;
	} else {
		// we're done with this function
		cmd_queue_tail = (cmd_queue_tail + 1) % CMD_QUEUE_SIZE;

		if (cmd_queue_head == cmd_queue_tail)    /* ������ ���� */
			UCSR0B &= ~_BV(UDRIE0);              /* ��������� ���������� ����������� uart */
	}
}





