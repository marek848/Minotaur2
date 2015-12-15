/*
 * jazda.c
 *
 *  Created on: Dec 4, 2015
 *      Author: Marek
 */

#include "stm32f1xx_hal.h"
#include "jazda.h"


 void rotary_left(int power)
 {
	 int32_t speed;
	 	 int32_t obrot=90000;//4500;
	 	 HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,0);
		 HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,1);
		 HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,1);
		 HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,0);
	 	 angle=0;
	 	 TIM3->CNT=0;
	 	 TIM2->CNT=0;
	 	 while(angle<obrot-1000 || angle>obrot+1000)//(TIM2->CNT<710)
	 	 {
	 		 speed=(obrot-angle)/10;

	 		 if(speed<0)
	 		 {
	 			 HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,1);
				 HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,0);
				 HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,0);
				 HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,1);
	 		 }
	 		 else
	 		 {

	 			HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,0);
	 			HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,1);
				HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,1);
				HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,0);
	 		 }

	 		 if(speed<0) speed=speed*(-1);
	 		 if(speed>power) speed=power;
//	 		 if(speed<-power) speed=-power;

	 		 TIM1->CCR1=speed;
	 		 TIM1->CCR2=speed;
	 	 }
// 	 HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,0);
// 	 HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,0);
// 	 HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,0);
// 	 HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,0);
 	 TIM1->CCR1=0;
 	 TIM1->CCR2=0;

  ori+=3;
  if(ori>4) ori-=4;

 }
 /******************************************************************/
 void rotary_right(int power)
 {
	 int32_t speed;
	 int16_t obrot=4600;
	 HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,1);
	 HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,0);
	 HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,0);
	 HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,1);
	 angle=0;
	 TIM3->CNT=0;
	 TIM2->CNT=0;
	 while(angle<-obrot-100 || angle>-obrot+100)//(TIM2->CNT<710)
	 {
		 speed=(obrot-angle)/5;
		 if(speed>power) speed=power;
		 if(speed<-power) speed=-power;

		 if(speed<0)
		 {
			 HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,0);
			 HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,1);
			 HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,1);
			 HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,0);
		 }
		 else
		 {
			 HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,1);
			 HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,0);
			 HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,0);
			 HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,1);
		 }

		 TIM1->CCR1=speed;
		 TIM1->CCR2=speed;
	 }
//	 HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,0);
//	 HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,0);
//	 HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,0);
//	 HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,0);
	 TIM1->CCR1=0;
	 TIM1->CCR2=0;

  ori+=1;
  if(ori>4) ori-=4;

 }
// /************************************************************/
 void rotary(int power)
 {
	 int32_t speed;
	 	 int16_t obrot=9500;
	 	 HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,1);
	 	 HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,0);
	 	 HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,0);
	 	 HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,1);
	 	 angle=0;
	 	 TIM3->CNT=0;
	 	 TIM2->CNT=0;
	 	 while(angle<-obrot-100 || angle>-obrot+100)//(TIM2->CNT<710)
	 	 {
	 		 speed=(obrot-angle)/5;
	 		 if(speed>power) speed=power;
	 		 if(speed<-power) speed=-power;

	 		 if(speed<0)
	 		 {
	 			 HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,0);
	 			 HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,1);
	 			 HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,1);
	 			 HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,0);
	 		 }
	 		 else
	 		 {
	 			 HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,1);
	 			 HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,0);
	 			 HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,0);
	 			 HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,1);
	 		 }

	 		 TIM1->CCR1=speed;
	 		 TIM1->CCR2=speed;
	 	 }
//	 HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,0);
//	 HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,0);
//	 HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,0);
//	 HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,0);
	 TIM1->CCR1=0;
	 TIM1->CCR2=0;

	 ori+=2;
	 if(ori>4) ori-=4;

 }

 void drive(int power)
 {
	int32_t speed[2]={0,0};
	int16_t enkoder[2]={0,0};
	uint8_t wall_left, wall_right, change_wall;

	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,0);
	HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,1);
	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,0);
	HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,1);
	TIM3->CNT=65535;
	TIM2->CNT=65535;

	if(SensorTab[2]<SL_Tresh) wall_left=0;
	else wall_left=1;
	if(SensorTab[3]<SR_Tresh) wall_right=0;
	else wall_right=1;

		   change_wall=0;

   enkoder[0]=65535-TIM3->CNT;
   enkoder[1]=65535-TIM2->CNT;
	while ((enkoder[0]+enkoder[1])/2<DISTANCE /*&& SensorTab[5]<0*/)
	{
	enkoder[0]=65535-TIM3->CNT;
	enkoder[1]=65535-TIM2->CNT;
//		test3=65535-TIM3->CNT;
		if(change_wall==0)
		{
			if(SensorTab[2]<SL_Tresh && wall_left==1)
			{
				//snapowanie
				change_wall=1;
			}
			if(SensorTab[2]>SL_Tresh && wall_left==0)
			{
				//snapowanie
				change_wall=1;
			}
			if(SensorTab[3]<SR_Tresh && wall_right==1)
			{
				//snapowanie
				change_wall=1;
			}
			if(SensorTab[3]>SR_Tresh && wall_right==0)
			{
				//snapowanie
				change_wall=1;
			}
		}
//		else
		if (SensorTab[2]<SL_Tresh && SensorTab[3]<SR_Tresh) //enkodery
		{
			HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,0);
			HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,0);
			speed[0]=power+(TIM3->CNT-TIM2->CNT);
			speed[1]=power-(TIM3->CNT-TIM2->CNT);
		}
//		else if((SensorTab[3]+SensorTab[1])/2>SR_Tresh && (SensorTab[0]+SensorTab[2])/2>SR_Tresh )
		else if(SensorTab[3]>SR_Tresh && SensorTab[1]>SR_Tresh && SensorTab[0]>SL_Tresh && SensorTab[2]>SL_Tresh )
		{
			HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
			HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
			speed[0]=power-error[0]+(TIM3->CNT-TIM2->CNT)-((SensorTab[3]+SensorTab[1])/2-(SensorTab[0]+SensorTab[2])/2)/5;
			speed[1]=power+error[0]-(TIM3->CNT-TIM2->CNT)+((SensorTab[3]+SensorTab[1])/2-(SensorTab[0]+SensorTab[2])/2)/5;
		}
		else if (SensorTab[3]>SR_Tresh && SensorTab[1]>SR_Tresh)
		{
			HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,0);
			HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
			speed[0]=power-error[1]+(TIM3->CNT-TIM2->CNT)-(SensorTab[3]+SensorTab[1])/15;
			speed[1]=power+error[1]-(TIM3->CNT-TIM2->CNT)+(SensorTab[3]+SensorTab[1])/15;
		}
		else if (SensorTab[0]>SL_Tresh && SensorTab[2]>SL_Tresh)
		{
			HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
			HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,0);
			speed[0]=power-error[0]+(TIM3->CNT-TIM2->CNT);
			speed[1]=power+error[0]-(TIM3->CNT-TIM2->CNT);
		}



	   if(speed[0]>999) speed[0]=999;
	   if(speed[0]<0) speed[0]=0;
	   if(speed[1]>999) speed[1]=999;
	   if(speed[1]<0) speed[1]=0;

	   TIM1->CCR1=speed[0];
	   TIM1->CCR2=speed[1];
	}
//	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,0);
//	HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,0);
//	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,0);
//	HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,0);
	TIM1->CCR1=0;
	TIM1->CCR2=0;

	if (ori==1) y++;
	else if(ori==2) x++;
	else if(ori==3) y--;
	else if(ori==4) x--;

 }

/****************************************************/
int istarget(int i1, int j1)
{

	if (target[0]==target[1])
	{
		if(target[0]==i1&&target[0]==j1)return 1;
		else return 0;
	}
	else
	{
		if(target[0]==i1&&target[0]==j1)return 1;
		else if(target[1]==i1&&target[0]==j1)return 1;
		else if(target[0]==i1&&target[1]==j1)return 1;
		else if(target[1]==i1&&target[1]==j1)return 1;
		else return 0;
	}
}
/****************************************************/
void findPath()
{
	int curx=x;
	int cury=y;
	int iter=0;
	int v;
	for (v=0; v<256; v++) path[v]=-1;
	while(1)
	{
		path[iter]=lowestNeighbourCell(curx, cury);
                if( path[iter]==1) cury++;
                else if (path[iter]==2) curx++;
                else if (path[iter]==3) cury--;
                else if (path[iter]==4) curx--;
		if(istarget(curx,cury)==1) break;
		iter++;
	}
	start=0; //
}
/****************************************************/
void readPath()
{
	int iter=0;
	while(1)
	{
		if (path[iter]>0)
		{
			state=path[iter];
			path[iter]=-1;
			break;
		}
		iter++;
		if (iter>255)
		{
			state=-1;
			break;
		}
	}
}
/****************************************************/
void set()
{
	if (state-ori==1 || state-ori==-3) rotary_right(VELR);
	else if (state-ori==2 || state-ori==-2) rotary(VELR);
	else if (state-ori==3 || state-ori==-1) rotary_left(VELR);
}
/****************************************************/
void rstdrive()
{
//    static int licznik=0;
        mapCell();
	x=1;
	y=1;
	ori=ORI_START;
        flood();
	findPath();
//	while (start!=0) HAL_Delay(50);
//        HAL_Delay(1000);
        start=0;
//        szyb+=20;
//        licznik++;
//        if(licznik>1) szyb+=20;
//        esl=analogRead(SL);
//        esr=analogRead(SR);
}
/****************************************************/
void mapCell()
{
	walls[x][y]=0;
	if(ori==1)
	{
		if (SensorTab[2] > SL_Tresh && SensorTab[0]> SL_Tresh) walls[x][y]+=8;
		if (SensorTab[4] > SF_Tresh && SensorTab[5]> SF_Tresh) walls[x][y]+=1;
		if (SensorTab[3] > SR_Tresh && SensorTab[1]> SR_Tresh) walls[x][y]+=2;
	}
	else if(ori==2)
	{
		if (SensorTab[2] > SL_Tresh && SensorTab[0]> SL_Tresh) walls[x][y]+=1;
		if (SensorTab[4] > SF_Tresh && SensorTab[5]> SF_Tresh) walls[x][y]+=2;
		if (SensorTab[3] > SR_Tresh && SensorTab[1]> SR_Tresh) walls[x][y]+=4;
	}
	else if(ori==3)
	{
		if (SensorTab[2] > SL_Tresh && SensorTab[0]> SL_Tresh) walls[x][y]+=2;
		if (SensorTab[4] > SF_Tresh && SensorTab[5]> SF_Tresh) walls[x][y]+=4;
		if (SensorTab[3] > SR_Tresh && SensorTab[1]> SR_Tresh) walls[x][y]+=8;
	}
	else if(ori==4)
	{
		if (SensorTab[2] > SL_Tresh && SensorTab[0]> SL_Tresh) walls[x][y]+=4;
		if (SensorTab[4] > SF_Tresh && SensorTab[5]> SF_Tresh) walls[x][y]+=8;
		if (SensorTab[3] > SR_Tresh && SensorTab[1]> SR_Tresh) walls[x][y]+=1;
	}
}
/****************************************************/
int8_t highestNeighbourCell(int i1, int j1)
{
   int a=-1;
	if(walls[i1][j1]<8   || walls[i1][j1]==-1) if(cell[i1-1][j1] > a) a=cell[i1-1][j1];
	if(walls[i1][j1]%2<1 || walls[i1][j1]==-1) if(cell[i1][j1+1] > a) a=cell[i1][j1+1];
	if(walls[i1][j1]%4<2 || walls[i1][j1]==-1) if(cell[i1+1][j1] > a) a=cell[i1+1][j1];
	if(walls[i1][j1]%8<4 || walls[i1][j1]==-1) if(cell[i1][j1-1] > a) a=cell[i1][j1-1];

	return a;
}
/****************************************************/
int8_t lowestNeighbourCell(int i1, int j1)
{
    int opt_direction=0;
	int a=cell[i1][j1];
	if(walls[i1][j1]<8 || walls[i1][j1]==-1) if(cell[i1-1][j1] < a && cell[i1-1][j1]  > -1)
                                                {
                                                  opt_direction=4;
                                                  a=cell[i1-1][j1];
                                                }
	if(walls[i1][j1]%2<1 || walls[i1][j1]==-1) if(cell[i1][j1+1] < a && cell[i1][j1+1]  > -1)
                                                {
                                                  opt_direction=1;
                                                  a=cell[i1][j1+1];
                                                }
	if(walls[i1][j1]%4<2 || walls[i1][j1]==-1) if(cell[i1+1][j1] < a && cell[i1+1][j1]  > -1)
                                                {
                                                  opt_direction=2;
                                                  a=cell[i1+1][j1];
                                                }
	if(walls[i1][j1]%8<4 || walls[i1][j1]==-1) if(cell[i1][j1-1] < a && cell[i1][j1-1]  > -1)
                                                {
                                                  opt_direction=3;
                                                  a=cell[i1][j1-1];
                                                }
	return opt_direction;
}
/****************************************************/
void flood()
{
	int8_t cell_temp[XMAZE][YMAZE];
	int8_t PathDist = 1; // This is how far the 'water' has flowed
	int8_t i2,j2;
	for(i2 = 0; i2 < XMAZE; i2++)  //Creating a loop which scans the whole maze
		for(j2 = 0; j2 < YMAZE; j2++)
                {
                  cell[i2][j2] = -1;
                  cell_temp[i2][j2]=-1;
                }
	for(i2 = target[0]; i2<=target[1]; i2++)
		for(j2 = target[0]; j2<=target[1]; j2++)
                  {
                    cell[i2][j2] = 1;
                    cell_temp[i2][j2]=1;
                  }

	while(PathDist>0)
	{
		PathDist++;  //Increment the distance because we are scanning again.
		for(i2 = 1; i2 < XMAZE-1; i2++) { //Creating a loop which scans the whole maze
			for(j2 = 1; j2 < YMAZE-1; j2++) {
				if(cell[i2][j2] != -1) //If the cell has already been reached, then continue to the next cell
					continue;
				if(highestNeighbourCell(i2,j2) != -1) //If there is a neighbouring cell which has been
					cell_temp[i2][j2] = PathDist;   //reached, then you have reached the current cell
											        //so give it a value
			}
		}

              for(i2 = 0; i2 < XMAZE; i2++)  //Creating a loop which scans the whole maze
		  for(j2 = 0; j2 < YMAZE; j2++)
                    cell[i2][j2] =cell_temp[i2][j2];

		if(cell[x][y] != -1) break;
	}
}
/****************************************************/
void Send_Gyro(uint8_t Register, uint8_t Value)
{
	uint8_t ToSend[2]={Register,Value};
	HAL_I2C_Master_Transmit(&hi2c1,0x6B<<1,ToSend,2,10);
}

/****************************************************/
int8_t Read_Gyro(uint8_t Register)
{
	uint8_t Read=0;
	HAL_I2C_Master_Transmit(&hi2c1,0x6B<<1,&Register,1,10);
	HAL_I2C_Master_Receive(&hi2c1,0x6B<<1,&Read,1,10);

	return Read;
}
/****************************************************/
uint16_t Read_AXIS(uint8_t Register)
{
	int16_t Measurement=0;
	int8_t LSB=0;
	int8_t MSB=0;

	LSB=Read_Gyro(Register);
	MSB=Read_Gyro(Register+1);

	Measurement=(MSB<<8)+LSB;
	return Measurement;
}
/****************************************************/
