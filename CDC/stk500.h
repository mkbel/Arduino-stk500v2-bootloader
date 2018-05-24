/*
 * stk500.h
 *
 *  Created on: May 24, 2018
 *      Author: Marek Bel
 */

#ifndef STK500_H_
#define STK500_H_

#define ST_PROCESS 7

unsigned char parseMsg(unsigned char c);
void processCommand(void);
void replyMsg(void);


#endif /* STK500_H_ */
