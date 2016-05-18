/*
 * jazda.c
 *
 *  Created on: Dec 4, 2015
 *      Author: Marek
 */

#include "stm32f1xx_hal.h"
#include "jazda.h"
int32_t lef_back, rig_back, lef_fr, rig_fr, front_right,front_left;

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
	 integral=0;
	 Transmit=1;
//
}
 /**************************************************************/
 void align(int power)
 {
	 int error3;

	 int le_fr=SensorTab[2]-dys0[2];
	 int le_back=SensorTab[0]-dys0[0];
	 int ri_fr=SensorTab[3]-dys0[3];
	 int ri_back=SensorTab[1]-dys0[1];
	 int fr_le=SensorTab[4]-dys0[4];
	 int fr_ri=SensorTab[5]-dys0[5];

	 if (fr_le>SF_Tresh && fr_ri>SR_Tresh && abs(fr_ri-fr_le)<100 && abs(fr_ri-fr_le)>50)
	 {
		 fr_le=SensorTab[4]-dys0[4];
		 fr_ri=SensorTab[5]-dys0[5];
		 error3=fr_le-fr_ri;
		 while(abs(error3)>2)
		 {
			 fr_le=SensorTab[4]-dys0[4];
			 fr_ri=SensorTab[5]-dys0[5];

			 error3=fr_le-fr_ri;
			 if(error3<0)
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

			 if(error3<0) error3=error3*(-1);
			 if(error3>power) error3=power;

			 TIM1->CCR1=error3+100;
			 TIM1->CCR2=error3+100;
		 }
	 }
	 else if (ri_fr>SR_Tresh && ri_back>SR_Tresh && abs(ri_fr-ri_back)<100 && abs(ri_fr-ri_back)>50)
	 {
		 error3=ri_fr-ri_back;
		 while(abs(error3)>2)
		 {
			 le_fr=SensorTab[2]-dys0[2];
			 le_back=SensorTab[0]-dys0[0];
			 ri_fr=SensorTab[3]-dys0[3];
			 ri_back=SensorTab[1]-dys0[1];
			 error3=ri_fr-ri_back;
			 if(error3<0)
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

			 if(error3<0) error3=error3*(-1);
			 if(error3>power) error3=power;

			 TIM1->CCR1=error3+100;
			 TIM1->CCR2=error3+100;
		 }
	 }
	 else if(le_fr>SL_Tresh &&  le_back>SL_Tresh && abs(le_fr-le_back)<100 && abs(le_back-le_fr)>50)
	 {
		 error3=le_back-le_fr;
		 while(abs(error3)>5)
		 {
			 le_fr=SensorTab[2]-dys0[2];
			 le_back=SensorTab[0]-dys0[0];
			 ri_fr=SensorTab[3]-dys0[3];
			 ri_back=SensorTab[1]-dys0[1];
			 error3=le_back-le_fr;
			 if(error3<0)
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

			 if(error3<0) error3=error3*(-1);
			 if(error3>power) error3=power;

			 TIM1->CCR1=error3+100;
			 TIM1->CCR2=error3+100;
		 }
	 }
	 TIM1->CCR1=0;
	 TIM1->CCR2=0;

	 angle=0;
	 angle1=0;
 }

/**************************************************************/
 void drive(int power)
 {
	uint8_t wall_left, wall_right, wall_flag;

	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,0);
	HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,1);
	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,0);
	HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,1);

	if(SensorTab[2]-dys0[2]<SL_Tresh && SensorTab[0]-dys0[0]<SL_Tresh) wall_left=0;
	else wall_left=1;
	if(SensorTab[3]-dys0[3]<SR_Tresh && SensorTab[1]-dys0[1]<SR_Tresh) wall_right=0;
	else wall_right=1;

	change_wall=0;
	change_wall_1=0;
	column=0;
	distance=0;
	angle1=0;

	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,0);
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,0);

//	if(wall_left==1)
//	{
//		angle1=SensorTab[0][indexer]-dys0[0]-(SensorTab[2][indexer]-dys0[2]);
//		angle1=angle1*5;
//	}
//	if(wall_right==1)
//	{
//		angle1=SensorTab[3][indexer]-dys0[3]-(SensorTab[1][indexer]-dys0[1]);
//		angle1=angle1*5;
//	}

	tryb=1;
	while(distance<DISTANCE)
	{
		lef_fr=SensorTab[2]-dys0[2];
		lef_back=SensorTab[0]-dys0[0];
		rig_fr=SensorTab[3]-dys0[3];
		rig_back=SensorTab[1]-dys0[1];
		front_left=SensorTab[4]-dys0[4];
		front_right=SensorTab[5]-dys0[5];

		if(change_wall==0)
		{
			if(SensorTab[2]-dys0[2]<SL_Tresh && wall_left==1 && SensorTab[0]-dys0[0]>SL_Tresh) change_wall=1;
			if(SensorTab[2]-dys0[2]>SL_Tresh && wall_left==0) change_wall=0;
			if(SensorTab[3]-dys0[3]<SR_Tresh && wall_right==1 && SensorTab[1]-dys0[1]>SR_Tresh) change_wall=2;
			if(SensorTab[3]-dys0[3]>SR_Tresh && wall_right==0) change_wall=0;

			if (change_wall==1)
			{
				HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
				HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,0);
				distance=80000;
			}
			if (change_wall==2)
			{
				HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
				HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,0);
				distance=80000;
			}
		}
		if(change_wall_1==0)
		{
			if(lef_fr>SL_Tresh && wall_left==0 && lef_back>SL_Tresh) change_wall_1=1;
			if(rig_fr>SR_Tresh && wall_right==0 && rig_back>SR_Tresh) change_wall_1=2;

			if (change_wall_1==1 || change_wall_1==2)
			{
				HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,0);
				HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
				distance=105000;
			}
		}
		if(column==0)
		{
			if(lef_fr>SL_Tresh && wall_left==0 && lef_back<SL_Tresh) column=1;
			if(rig_fr>SR_Tresh && wall_right==0 && rig_back<SR_Tresh) column=2;
		}
		if(column==1 && lef_fr<SL_Tresh && wall_left==0 && lef_back<SL_Tresh) column=3;
		if(column==2 && rig_fr<SR_Tresh && wall_right==0 && rig_back<SR_Tresh) column=3;
		if (column==3)
		{
			HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,1);
			HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
			distance=75000;
			column=4;
		}

		if (front_left >5 || front_right >5 )
		{
			wall_flag=1;
			tryb=0;
			TIM1->CCR1=0;
			TIM1->CCR2=0;

			HAL_Delay(1000);
			break;
		}
	}

   	tryb=0;

	TIM1->CCR1=0;
	TIM1->CCR2=0;
	if(wall_flag==0)
	{
		if (ori==1) y++;
		else if(ori==2) x++;
		else if(ori==3) y--;
		else if(ori==4) x--;
	}
	else if (distance > 120000)
	{
		if (ori==1) y++;
		else if(ori==2) x++;
		else if(ori==3) y--;
		else if(ori==4) x--;
	}
	else
	{
		mapCell();

		HAL_TIM_Base_Stop_IT(&htim4);

		flood();
		findPath();

		HAL_TIM_Base_Start_IT(&htim4);
	}

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
	if (state-ori==1 || state-ori==-3) {
		align(VELR);
		rotary(VELR,90000/*+angle/2*/);
	}
	else if (state-ori==2 || state-ori==-2) {
		align(VELR);
		rotary(VELR,180000/*+angle/2*/);
	}
	else if (state-ori==3 || state-ori==-1) {
		align(VELR);
		rotary(VELR,-95000/*+angle/2*/);
	}
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

	for (i=0;i<6;i++) czujnik[i]=SensorTab[i]-dys0[i];
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
	uint8_t i1,j1;

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

	 int32_t tmp[6][4],dys1[6],pom,sum;
	 uint8_t index[6];

	 //dokonywanie pomiarow
	 for(i1=0;i1<4;i1++)
	 {
		 for(j1=0;j1<6;j1++)
		 {
			 tmp[j1][i1]=SensorTab[j1];
		 }
		 HAL_Delay(100);
		 rotary(VELR,-95000);
	 }

	 //wyznaczanie minimalnych odczytow
	 for(j1=0;j1<6;j1++)
	 {
		 pom=4096;
		 for(i1=0;i1<4;i1++)
		 {
			 if(pom>tmp[j1][i1])
			 {
				 pom=tmp[j1][i1];
				 index[j1]=i1;
			 }
		 }
		 dys1[j1]=pom;
	 }

	 //wyznaczanie dys 0 SSR
	 for(j1=0;j1<6;j1++)
	 {
		 sum=0;
		 for(i1=0;i1<4;i1++)
		 {
			 if(index[j1]!=i1)
			 {
				 sum+=tmp[j1][i1];
			 }
		 }
		 dys0[j1]=sum/3;
	 }

	 //treshe
	 SR_Tresh=(dys1[1]-dys0[1]+dys1[3]-dys0[3])/2;
	 SL_Tresh=(dys1[0]-dys0[0]+dys1[2]-dys0[2])/2;
	 SF_Tresh=(dys1[4]-dys0[4]+dys1[5]-dys0[5])/2;

	 SR_Tresh=SR_Tresh*7/10;
	 SL_Tresh=SL_Tresh*7/10;
	 SF_Tresh=SF_Tresh*7/10;

	 SSR_Tresh=SR_Tresh*5/10;
	 SSL_Tresh=SL_Tresh*5/10;

 /***********************************************************************************/
//	Treshe sa na sztywno w tym momencie
//	 rotary(VELR,-95000);
//	 HAL_Delay(2000);
//	 dys0[4]=SensorTab[4];
//	 dys0[5]=SensorTab[5];
//
//	 rotary(VELR,90000);
//	 HAL_Delay(100);
//	 if (SensorTab[4]-dys0[4] > SF_Tresh && SensorTab[5]-dys0[5] > SF_Tresh)
//	 {
//		 rotary(VELR,90000);
//		 HAL_Delay(100);
//	 }
//
//  	 for(i1=0;i1<4;i1++) dys0[i1]=SensorTab[i1];
/**************************************************************************************/
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,0);
}
int32_t abs(int32_t a)
{
	if(a>0) return a;
	else return-a;
}
