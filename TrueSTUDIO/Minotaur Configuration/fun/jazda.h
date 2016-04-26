/*
 * jazda.h
 *
 *  Created on: Dec 4, 2015
 *      Author: Marek
 */

#ifndef JAZDA_H_
#define JAZDA_H_

#define XMAZE 18
#define YMAZE 18// Rozmiar labiryntu + 2
#define ORI_START 1
#define VEL 250
#define VELR 700
#define TARGET_1 8
#define TARGET_2 9

/* ProgramStaus=*/
#define DRIVE_STATUS 2
#define PAUSE_STATUS 1
#define STOP_STATUS 0

//#define K_drive 5/2
//#define I_drive 1/2000
//#define D_drive 300

#define K_drive 5/2
#define I_drive 0
#define D_drive 250

#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim4;

volatile uint16_t adcData[6];
extern volatile uint32_t adctmp[6];
extern volatile int32_t SensorTab[6][5];
extern volatile uint16_t dys0[6];
extern uint8_t counter;
extern volatile int32_t lin_vel,rot_vel;
uint32_t test,test2;
int32_t test3[5];
volatile uint16_t MinMax[6][2];
volatile uint16_t Max3[6];
volatile uint16_t Min3[6];
extern int k;
extern int l;
extern uint8_t Status;
extern volatile int32_t angle,distance,angle1;

extern int32_t speed[2];
extern int8_t indexer;
int32_t propocjonal,integral,derivative;

extern int8_t tryb;
extern int32_t dryf;
extern int8_t x,y;
extern int8_t i,j;
extern int8_t ori;
extern int8_t state;
extern int8_t cell[XMAZE][YMAZE];
extern int8_t walls[XMAZE][YMAZE];
extern int8_t target[2];
extern int8_t path[256];

extern uint8_t change_wall;
extern uint8_t change_wall_1;

volatile uint8_t Transmit;
volatile uint8_t TxBuffer[34];
volatile uint8_t RxBuffer[8];

int32_t error;
int32_t error2;
int32_t regulator;

#define STALA 0
#define KATNAST 100
#define WALLSOFF 0
#define DISTANCE 159500 // rozmiar kom�rki
#define SSR_Tresh -100
#define SSL_Tresh -100
#define SR_Tresh -150
#define SL_Tresh -150
#define SF_Tresh -220

void rotary(int , int32_t);
void drive(int);

int istarget(int , int );
void findPath();
void readPath();
void set();
void rstdrive();
void mapCell();
int8_t highestNeighbourCell(int , int );
int8_t lowestNeighbourCell(int , int );
void flood();

extern I2C_HandleTypeDef hi2c1;
void Send_Gyro(uint8_t Register, uint8_t Value);
int8_t Read_Gyro(uint8_t Register);
int16_t Read_AXIS(uint8_t Register);
void calibration();
int32_t abs(int32_t);

#endif /* JAZDA_H_ */
