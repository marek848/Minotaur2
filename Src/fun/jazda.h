/*
 * jazda.h
 *
 *  Created on: Dec 4, 2015
 *      Author: Marek
 */

#ifndef JAZDA_H_
#define JAZDA_H_

#include "stm32f1xx_hal.h"

#define dystans 180 // rozmiar komórki
#define SR_Tresh 130
#define SL_Tresh 270
#define SF_Tresh 130

#define ORI_START 3
#define XMAZE 11
#define YMAZE 11 // Rozmiar labiryntu + 2
#define TARGET_1 8
#define TARGET_2 9

extern int x,y;
extern int i,j;
extern int ori;
extern int state;
extern char cell[XMAZE][YMAZE];
extern char walls[XMAZE][YMAZE];
extern char target[2];
extern char path[256];
extern int szyb;
extern int przejazd;

//void anglefun();
//void rotary_left(int);
//void rotary_right(int);
//void rotary(int);
void drive(int);


#endif

#endif /* JAZDA_H_ */
