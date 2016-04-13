/*
 * jazda.c
 *
 *  Created on: Dec 4, 2015
 *      Author: Marek
 */

#include "stm32f1xx_hal.h"
#include "jazda.h"

 /**************************************************************/
 void rotary(int power, int32_t obrot)
  {
	 int32_t speed1;
	 obrot*=-1;
	 test3[1]=obrot;
	 angle=0;
	 test3[2]=0;

	 Transmit=0;

	 while(test3[2]<20)
	 {
		 speed1=(obrot-angle)/80;

		 if(speed1<0)
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

		 if(speed1<0) speed1=speed1*(-1);
		 if(speed1>power) speed1=power;

		 TIM1->CCR1=speed1+100;
		 TIM1->CCR2=speed1+100;
	 }
	 TIM1->CCR1=0;
	 TIM1->CCR2=0;

	 if (obrot>45000 && obrot<135000) ori+=3;
	 if (obrot<-45000 && obrot>-135000) ori+=1;
	 if ((obrot<-135000 && obrot>-225000) || (obrot>135000 && obrot<225000))  ori+=2;

	 if(ori>4) ori-=4;
	 angle=0;
	 angle1=0;
	 Transmit=1;
//
}
/**************************************************************/
 void drive(int power)
 {
	uint8_t wall_left, wall_right;

	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,0);
	HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,1);
	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,0);
	HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,1);

	if(SensorTab[2][indexer]-dys0[2]<SL_Tresh && SensorTab[0][indexer]-dys0[0]<SL_Tresh) wall_left=0;
	else wall_left=1;
	if(SensorTab[3][indexer]-dys0[3]<SR_Tresh && SensorTab[1][indexer]-dys0[1]<SR_Tresh) wall_right=0;
	else wall_right=1;

	change_wall=0;
	distance=0;

	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,0);
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,0);

	tryb=1;
	while(distance<DISTANCE)
	{

//		if(change_wall==0)
//		{
//			if(SensorTab[2][indexer]-dys0[2]<SL_Tresh && wall_left==1) change_wall=1;
//			if(SensorTab[2][indexer]-dys0[2]>SL_Tresh && wall_left==0) change_wall=1;
//			if(SensorTab[3][indexer]-dys0[3]<SR_Tresh && wall_right==1) change_wall=1;
//			if(SensorTab[3][indexer]-dys0[3]>SR_Tresh && wall_right==0) change_wall=1;
//
//			if (change_wall==1)
//			{
//				HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
//				HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
//				distance=69000;
//			}
//		}
	}

   	tryb=0;

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
//	start=0;
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
	if (state-ori==1 || state-ori==-3) rotary(VELR,90000/*+angle/2*/);
	else if (state-ori==2 || state-ori==-2) rotary(VELR,180000/*+angle/2*/);
	else if (state-ori==3 || state-ori==-1) rotary(VELR,-95000/*+angle/2*/);
}
/****************************************************/
void rstdrive()
{
//	int8_t i1;
//    static int licznik=0;
    mapCell();

	x=1;
	y=1;
	ori=ORI_START;
        flood();
	findPath();
	Status=STOP_STATUS;
//	while (Status==STOP_STATUS) HAL_Delay(50);

//	for(i1=0;i1<4;i1++) dys0[i1]=SensorTab[i1];
	HAL_Delay(1000);
	angle=0;

}
/****************************************************/
void mapCell()
{
	int8_t i;
	int32_t czujnik[6];

	for (i=0;i<6;i++) czujnik[i]=SensorTab[i][indexer]-dys0[i];
	walls[x][y]=0;
	if(ori==1)
	{
		if (czujnik[2] > SL_Tresh && czujnik[0] > SL_Tresh) walls[x][y]+=8;
		if (czujnik[4] > SF_Tresh && czujnik[5] > SF_Tresh) walls[x][y]+=1;
		if (czujnik[3] > SR_Tresh && czujnik[1] > SR_Tresh) walls[x][y]+=2;
	}
	else if(ori==2)
	{
		if (czujnik[2] > SL_Tresh && czujnik[0] > SL_Tresh) walls[x][y]+=1;
		if (czujnik[4] > SF_Tresh && czujnik[5] > SF_Tresh) walls[x][y]+=2;
		if (czujnik[3] > SR_Tresh && czujnik[1] > SR_Tresh) walls[x][y]+=4;
	}
	else if(ori==3)
	{
		if (czujnik[2] > SL_Tresh && czujnik[0] > SL_Tresh) walls[x][y]+=2;
		if (czujnik[4] > SF_Tresh && czujnik[5] > SF_Tresh) walls[x][y]+=4;
		if (czujnik[3] > SR_Tresh && czujnik[1] > SR_Tresh) walls[x][y]+=8;
	}
	else if(ori==4)
	{
		if (czujnik[2] > SL_Tresh && czujnik[0] > SL_Tresh) walls[x][y]+=4;
		if (czujnik[4] > SF_Tresh && czujnik[5] > SF_Tresh) walls[x][y]+=8;
		if (czujnik[3] > SR_Tresh && czujnik[1] > SR_Tresh) walls[x][y]+=1;
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
int16_t Read_AXIS(uint8_t Register)
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
void calibration()
{
	int16_t pomoc=0;
	uint8_t i1;

	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,0);
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
	HAL_Delay(1000);

	Send_Gyro(0x20,0xDF);//4F
	Send_Gyro(0x21,0x00);
	Send_Gyro(0x22,0x00);
	Send_Gyro(0x23,0x20);// 0x00 - 250dps(8.75 mdps/digit); 0x10 - 500 dps(17.5 mdps/digit); 0x20 - 2000 dps(70 mdps/digit)
	Send_Gyro(0x24,0x00);

	for(i1=0;i1<100;i1++) pomoc+=((Read_AXIS(0x2C)-dryf)*700)/10000;
	pomoc/=100;
	HAL_Delay(1000);
  	while(pomoc>2 || pomoc<-2)
  	{
		 for(i1=0;i1<100;i1++)
		 {
			dryf += Read_AXIS(0x2C);
		 }
		 dryf/=100;

		 pomoc=0;
		 for(i1=0;i1<100;i1++) pomoc+=((Read_AXIS(0x2C)-dryf)*700)/10000;
		 pomoc/=100;
  	}

	 TIM3->CNT=16384;
	 TIM2->CNT=16384;

	 HAL_TIM_Base_Start_IT(&htim4);
	 HAL_Delay(100);

	 rotary(VELR,-95000);
	 HAL_Delay(1000);
	 dys0[4]=SensorTab[4][indexer];
	 dys0[5]=SensorTab[5][indexer];

	 rotary(VELR,90000);
	 HAL_Delay(100);
  	 for(i1=0;i1<4;i1++) dys0[i1]=SensorTab[i1][indexer];


//  	rotary_new(VELR,90000);
//  	start=0;
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,0);
}
int32_t abs(int32_t a)
{
	if(a>0) return a;
	else return-a;
}
