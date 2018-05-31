/*
 * stk500.h
 *
 *  Created on: May 24, 2018
 *      Author: Marek Bel
 */

#ifndef STK500_H_
#define STK500_H_

//************************************************************************
//* LED on pin "PROGLED_PIN" on port "PROGLED_PORT"
//* indicates that bootloader is active
//* PG2 -> LED on Wiring board
//************************************************************************

#define PROGLED_PORT    PORTD
#define PROGLED_DDR     DDRD
#define PROGLED_PIN     PIND5

void processCommand(void);
unsigned char msgParsed(unsigned char c);
void replyMsg(void);


#endif /* STK500_H_ */
