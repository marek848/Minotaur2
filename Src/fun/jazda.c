/*
 * jazda.c
 *
 *  Created on: Dec 4, 2015
 *      Author: Marek
 */

#include "stm32f1xx_hal.h"

// void rotary_left(int power)
// {
//   int moc,pom;
//   impulsA=0;
//   impulsB=0;
//   angle=0;
//
//    digitalWrite(AIN1,0);
//    digitalWrite(AIN2,1);
//    digitalWrite(BIN1,0);
//    digitalWrite(BIN2,1);
//  while (angle<90)
//  {
//    pom=abs((90-(int)angle)/4+16);
//
//    moc=abs(pom+(impulsA+impulsB)*2);
//    if(moc >=255) moc=255;
//    analogWrite(PWMA,moc);
//
//    moc=abs(pom-(impulsA+impulsB)*2);
//    if(moc >=255) moc=255;
//    analogWrite(PWMB,moc);
//  anglefun();
//  }
//  analogWrite(PWMA,0);
//  analogWrite(PWMB,0);
//  ori+=3;
//  if(ori>4) ori-=4;
//
// }
// /******************************************************************/
// void rotary_right(int power)
// {
//  int moc,pom;
//  impulsA=0;
//  impulsB=0;
//  angle=0;
//
//   digitalWrite(AIN1,1);
//   digitalWrite(AIN2,0);
//   digitalWrite(BIN1,1);
//   digitalWrite(BIN2,0);
//
//  while (angle>-90)
//  {
//    pom=abs((-90-(int)angle)/4-16);
//
//    moc=abs(pom-(impulsA+impulsB)*2);
//    if(moc >=255) moc=255;
//    analogWrite(PWMA,moc);
//    moc=abs(pom+(impulsA+impulsB)*2);
//    if(moc >=255) moc=255;
//    analogWrite(PWMB,moc);
//
//  anglefun();
// }
//  analogWrite(PWMA,0);
//  analogWrite(PWMB,0);
//  ori+=1;
//  if(ori>4) ori-=4;
//
// }
// /************************************************************/
// void rotary(int power)
// {
//   int moc,pom;
//   impulsA=0;
//   impulsB=0;
//   angle=0;
//
//   digitalWrite(AIN1,0);
//   digitalWrite(AIN2,1);
//   digitalWrite(BIN1,0);
//   digitalWrite(BIN2,1);
//  while (angle<180)
//  {
//
//    pom=abs((180-(int)angle)/10+16);
//
//    moc=abs(pom+(impulsA+impulsB)*2);
//    if(moc >=255) moc=255;
//    analogWrite(PWMA,moc);
//
//    moc=abs(pom-(impulsA+impulsB)*2);
//    if(moc >=255) moc=255;
//    analogWrite(PWMB,moc);
//
//  anglefun();
//  }
//  analogWrite(PWMA,0);
//  analogWrite(PWMB,0);
//  ori+=2;
//  if(ori>4) ori-=4;
//
// }

 void drive(int power)
 {
//	   int sciankal,sciankar,zmiana_scianki;
//	   int sl,sr,serror,moc;
//	   impulsA=0;
//	   impulsB=0;
//	   angle=0;

	   HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,1);
	   HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,0);
	   HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,1);
	   HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,0);
		  TIM1->CCR1=50;
		  TIM1->CCR2=50;

//	   sl=analogRead(SL);
//	   sr=analogRead(SR);
//
//	   if (sl<SL_Tresh) sciankal=0;
//	   else sciankal=1;
//	   if (sr<SR_Tresh) sciankar=0;
//	   else sciankar=1;
//
//	   zmiana_scianki=0;
//
//	  while ((impulsA+impulsB)/2< dystans && analogRead(SF)<480) //
//	  {
//		anglefun();
//		sl=analogRead(SL);
//		sr=analogRead(SR);
//
//		if(zmiana_scianki==0)
//		{
//		  if(sl<SL_Tresh && sciankal==1)
//		  {
//			impulsA=dystans/2;
//			impulsB=dystans/2;
//			zmiana_scianki=1;
//		  }
//		   if(sl>SL_Tresh && sciankal==0)
//		  {
//			impulsA=dystans/2;
//			impulsB=dystans/2;
//			zmiana_scianki=1;
//		  }
//		   if(sr<SR_Tresh && sciankar==1)
//		  {
//			impulsA=dystans/2;
//			impulsB=dystans/2;
//			zmiana_scianki=1;
//		  }
//		   if(sr>SR_Tresh && sciankar==0)
//		  {
//			impulsA=dystans/2;
//			impulsB=dystans/2;
//			zmiana_scianki=1;
//		  }
//		}
//
//		if(sl<SL_Tresh && sr<SR_Tresh) serror=angle*3;
//		else if(sl<SL_Tresh) serror=(esr-sr)/25 + angle*3;
//		else if(sr<SR_Tresh) serror=-(esl-sl)/25 + angle*3;
//		else serror=-(esl-sl)/24 + (esr-sr)/24 + angle*3;
//		moc=abs(power+serror-(impulsA-impulsB)/2);
//		if(moc >=255) moc=255;
//		analogWrite(PWMA,moc);
//
//		moc=abs(power-serror+(impulsA-impulsB)/2);
//		if(moc >=255) moc=255;
//		analogWrite(PWMB,moc);
//	   }
//	   analogWrite(PWMA,0);
//	   analogWrite(PWMB,0);
//	   if (ori==1) y++;
//	   else if(ori==2) x++;
//	   else if(ori==3) y--;
//	   else if(ori==4) x--;

 }
