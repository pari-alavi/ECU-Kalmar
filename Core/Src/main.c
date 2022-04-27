/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
//#include <eeprom.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/////////////////////////////////////////////////////////////////////
//#define READ_INPUT10()   HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_3)//PG3--I10--pin21->N.C
//#define READ_INPUT11()   HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_5)//PG5--->N.C                       /////   N.C pins
//#define READ_INPUT12()   HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_7)//PG7--->N.C
//#define READ_INPUT13()   HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11)//PC11--->N.C
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define SPREAD_STOP_SEN()   HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_2)//PG2--->30-35 Stop
#define DAMP_SEN()  HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_12)//PC12---> Damping 20/40  
                   /////// lock/ unlock valve //////////           
#define UNLOCK_L_SEN()   HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_6)//PG6---> Unlock sensor Left
#define LOCK_L_SEN()   HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_8)//PG8---> Lock sensor Left
                 //////***************************//////////////
#define UNLOCK_R_SEN()  HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4)//PD4---> Unlok sensor right
#define LOCK_R_SEN()   HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10)//PC10---> Lock sensor Right                             /////reading GPIOS status
                 //////// seat sensors feed back ////////
#define SEAT_L_FRONT()  HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2)//PD2- I12--pin13> Seat sensor Front/Left
#define SEAT_R_FRONT()  HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_5)//PD5--I8-pin40> Seat Sensor Front/Right
#define SEAT_R_REAR()  HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_3)//PD3--I4-pin26> Seat sensor Rear/Right
#define SEAT_L_REAR()  HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6)//PD6--I5-pin27>Seat Sensor  Rear/Left


/////////////////////////////////////////////////////////////////////////////////////////////////
#define LOCK_VALVE_HIGH()    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET)//Q13---PC0---LOCK TW VALVE 
#define OUT_TEST_HIGH()    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_11,GPIO_PIN_SET)//Q3---PF11--                       1             
#define SEAT_LIGHT_HIGH()    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12,GPIO_PIN_SET)//Q6---PF12---SEAT LIGHT Orange
#define WORK_LIGHT_HIGH()    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_SET)//Q1-DO4--PF13---WORK LIGHT MAIN
#define SHIFT_RIGHT_HIGH()    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_SET)//Q10---PF14---SIDE SHIFT RIGHT VALVE
#define SHIFT_LEFT_HIGH()    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_SET)//Q11---PF15---SIDE SHIFT LEFT VALVE
#define TILT_LOCK_HIGH()    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET)//Q14---PE7---TILT LOCK VALVE
#define UNLOCK_VALVE_HIGH()    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET) //Q12---PE8---UNLOCK TW VALVE                                     /////SET GPIOS
#define TILT_LOCK1_HIGH()    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_SET)//Q9---PE10---TILT LOCK VALVE
#define LOCKED_GREEN_HIGH()    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET)//Q7---PE12---LOCK TW LIGHT Green 
#define RUN_LED_HIGH()   HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET)//Q8---PE15--RUN LED
#define UNLOCKED_RED_HIGH()   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET) //Q4---PB12---UNLOCK TW LIGHT Red
#define CAN_ST_LED_HIGH()   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET) //Q2---PB13---Can Status LED
#define SPREAD_ALARM_HIGH()   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET) //Q5---PB14---AUTO SPAREADER ALARM

///////////////////////////////////////////////////////////////////////////////////////////////////////
#define LOCK_VALVE_LOW()    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET)//PC0
#define OUT_TEST_LOW()    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_11,GPIO_PIN_RESET)//PF11  res
#define SEAT_LIGHT_LOW()    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_12,GPIO_PIN_RESET)//PF12
#define WORK_LIGHT_LOW()    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_13,GPIO_PIN_RESET)//PF13
#define SHIFT_RIGHT_LOW()    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_RESET)//PF14
#define SHIFT_LEFT_LOW()    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_15,GPIO_PIN_RESET)//PF15
#define TILT_LOCK_LOW()    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET)//PE7
#define UNLOCK_VALVE_LOW()    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET)//PE8
#define TILT_LOCK1_LOW()    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET) //PE10        ////RESET GPIOS 
#define LOCKED_GREEN_LOW()    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET)//PE12
#define RUN_LED_LOW()   HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_RESET) //PE15
#define UNLOCKED_RED_LOW()   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET) //PB12
#define CAN_ST_LED_LOW()   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET) //PB13
#define SPREAD_ALARM_LOW()   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET) //PB14

#define BitSet(x, BitPosition) ((x) |= (1 << (BitPosition)))
#define BitClear(x, BitPosition) ((x) &=~(1 << (BitPosition)))
#define BitRead(x, BitPosition) (((x) >>BitPosition) & 1)
#define BitWrite(x,BitPosition, value)((value)==1?BitSet(x, BitPosition):BitClear(x,BitPosition))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
CAN_FilterTypeDef    sFilterConfig; //CAN Bus Filter
CAN_RxHeaderTypeDef  RxHeader_Can_ST;//COBID 0x00
CAN_RxHeaderTypeDef  RxHeader;       //COBID 0x381
CAN_RxHeaderTypeDef  RxHeader_t;     //COBID 0x382
CAN_TxHeaderTypeDef  TxHeader_heart_bit; //0x208
CAN_TxHeaderTypeDef  TxHeader_status; //COBID 0x26C

uint8_t  RxData_Can_ST[8]={0,0,0,0,0,0,0,0};//COBID 0x00
uint8_t  RxData[8]={0,0,0,0,0,0,0,0};//COBID 0x381
uint8_t  RxData_t[8]={0,0,0,0,0,0,0,0};//COBID 0x382
uint8_t  TxData_heart_bit[8]={0,0,0,0,0,0,0,0};//0x208-----> 791-1
uint8_t  TxData_status[8]={0,0,0,0,0,0,0,0};// COBID 26C
uint32_t TxMailbox; 
uint16_t pwm1_buff_D1,pwm2_CW_D2,pwm3_CCW_D3,pwm4_buff_D4;
//uint16_t PWM1_20ft_D1;// pwm buffers;
//uint16_t pwm2_buff_D2;
//uint16_t pwm3_buff_D3;
//uint16_t PWM4_40ft_D4;
/////////////////////////////////////////////////////////ecu gpio flags and other defines /////////////////////////////

char pistol_flag=0;
char Auto_20_40ft=0;
char panel_lock_flag=0;
char to20ft_CMD=0, to40ft_CMD=0;
char spreader_light=0;
char tilt_lock=0;
_Bool flag_40ft,flag_20ft,spread_flag;
uint8_t lock_safty_time=0;
uint16_t Timer5=0;
uint16_t ccw_pwm=0,cw_pwm=0;
uint16_t Vol_Rotation;
_Bool lock_button_flag=0,CW_flag=0,CCW_flag=0;
_Bool lock_edge_flag=0;
_Bool Lock_flag=0;
_Bool unLock_flag=0;
_Bool CAN_ST_flag=0;
_Bool Test_flag=0;
_Bool SEAT_L_FRONT,SEAT_R_FRONT,SEAT_R_REAR,SEAT_L_REAR, SPREAD_STOP_SEN,DAMP_SEN,UNLOCK_L_SEN,LOCK_L_SEN,UNLOCK_R_SEN,LOCK_R_SEN;
_Bool unlock_lock_flag;
_Bool valve40ft,valve20ft,valve_Unlock,valve_lock;
_Bool seat_flag,Locked_flag, Unlocked_flag;
uint8_t RUN_Timer=0;
uint8_t CAN_Timer=0;
uint16_t Timer1=0 , Timer2=0 , Timer3=0 , Timer4=0;
/////////////////////////eeprom variables//////////////////
//uint16_t write_eeprom_value;
//uint16_t read_eeprom_value[256];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_IWDG_Init(void);
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
 //__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
  if(RxHeader.StdId==0x00)   //Heart bit from 790/0
    { 
    if(RxData[1] ==0x89)
      CAN_ST_flag=1;
       else
      CAN_ST_flag=0;
    }
  ///////////////////////////////////// COB ID 381 ////////////////////////////////////////////////////
  if(RxHeader.StdId==0x381)   //baraye fal sazy PWM   ----- VOLUMES ON JOY STICK
               {  
        Vol_Rotation =RxData[4] <<8 | RxData[5] ;   ////set pwm3 
           
                 }
 /////////////////////////////////////////////  COB ID 382 /////////////////////////////////////////////////////////////////////// 
   if(RxHeader.StdId==0x382)    // baraye fala sazy khorojy ha tavasot CAN_BUS
      { 
       ///////////FIRST WORD /////////////////////// 
      if(BitRead(RxData[0],0) && pistol_flag ==0)//COBID=382 Data=0 Bit=0
            SHIFT_LEFT_HIGH();
            else                                                     
            SHIFT_LEFT_LOW(); 
        
  if(BitRead(RxData[0],1) && pistol_flag ==0)	//COBID=382 Data=0 Bit=1	
           SHIFT_RIGHT_HIGH(); 
           else                                                          
           SHIFT_RIGHT_LOW();        
  ///////////////////////////////////******spreader---20foot-40foot*****////////////////////////////////////////////////////                 
       if(BitRead(RxData[0],0) && pistol_flag ==1)//COBID=382 Data=0 Bit=0
          to40ft_CMD=1;
          else                         
          to40ft_CMD=0;
         
         if(BitRead(RxData[0],1) && pistol_flag ==1)  //COBID=382 Data=0 Bit=1                                  
          to20ft_CMD=1;
          else                     
          to20ft_CMD=0;  
      
	if(BitRead(RxData[1],3))//COBID=382 Data=1 Bit=3
         Auto_20_40ft=1;
         else                      
         Auto_20_40ft=0;
    
 /////////////////////////////////////pistol////////////////////////////////////////////////            
	 if(BitRead(RxData[0],2))//COBID=382 Data=0 Bit=2
           pistol_flag=1; 
           else                   
           pistol_flag=0;  
                 
    //////////// UNLOCK/lock TW  /// button on joy stick----toggle mode
	 if(BitRead(RxData[0],3))////COBID=382 Data=0 Bit=3 
                  {  
            if(SEAT_L_FRONT() ==1 & SEAT_R_FRONT() ==1 & SEAT_R_REAR()==1 & SEAT_L_REAR()==1) 
                        {
                 if(lock_edge_flag==0)
                            {
                  lock_edge_flag=1;   
                  lock_button_flag ^=1 ;   //Toggle    
                            }
                        }
                  }
                      else
                  lock_edge_flag=0; 
          
	if(BitRead(RxData[1],1))//COBID=382 Data=1 Bit=1
           panel_lock_flag=1; 
           else                      //// TW Auto-LOCK key on panel
           panel_lock_flag=0;   
           
//////////////////////////////////////////////////////////////////////////////////////////////          
    
	if(BitRead(RxData[0],5))//COBID=382 Data=0 Bit=5
          {
           tilt_lock=1;
          }
       else
          {
           tilt_lock=0;             // LEVEL LOCK BIT
          }
       
////////////SECOND WORD ////////////////////////////////////////////////////////////
///////////////////////////five word//////////////////////////////////////////////         
 
	if(BitRead(RxData[5],6)) //COBID=382 Data=5 Bit=6
           spreader_light=1;    
           else                 ////spreader Work light on
           spreader_light=0;
      }  
 }
////////////////////////////////////////////// HEART BIT COBID 208 /////////////////////////
void heartbit (void) //Timer 4
     {
   TxData_heart_bit[0]=0;
   TxData_heart_bit[1]=137;  /////COBID 208 0.030 sec
   HAL_CAN_AddTxMessage(&hcan,&TxHeader_heart_bit,TxData_heart_bit,&TxMailbox); //CAN_BUS heart bit ECU 791-1
       }

void Status (void)//Timer 3
          { 	 
    HAL_CAN_AddTxMessage(&hcan,&TxHeader_status,TxData_status,&TxMailbox); //COBID 0x26C
 
  //////////////////// CAN STATUS LED ////////////////////
      CAN_Timer++;
     if(CAN_Timer==160)  
       CAN_Timer=0; 
     if(CAN_Timer>80 & CAN_ST_flag==1)
       CAN_ST_LED_HIGH();
         else  
       CAN_ST_LED_LOW();   
  
    //COBID=26C Data=0 Bit0 (UNLOCKED , LOCKED Light Status)
	 if( Locked_flag==1 | Unlocked_flag==1)
	                {
          BitWrite(TxData_status[0], 0,1);        
			}
		     else
	  BitWrite(TxData_status[0], 0,0); 
							
	//COBID=26C Data=0 Bit1 (seat Status)		
            if(seat_flag==1)	
		   {
           BitWrite(TxData_status[0], 1,1);          
	           }
		 else
	   BitWrite(TxData_status[0], 1,0);          	
								
	//COBID=26C Data=0 Bit2 (com valve spreader Y6003)				
	if(valve40ft==1 | valve20ft==1 | flag_40ft==1 | flag_20ft==1 | valve_Unlock==1 | valve_lock==1 | CW_flag==1 | CCW_flag==1)
	          {
	 BitWrite(TxData_status[0], 2,1);          
		   }
		  else
	BitWrite(TxData_status[0], 2,0); 
					
  }    
   void Rotation (void)  //Timer 2
           {
  ////////////////////////// CW ROTATION ///////////////////
    if( Vol_Rotation <1800 &Vol_Rotation >560 & pistol_flag==0)
            {
       CW_flag=1;
     pwm2_CW_D2 = map(Vol_Rotation,1800,560,100,900);
    }
     else
     {
        pwm2_CW_D2=0;
       CW_flag=0;
     }
     ///////////////////// CCW ROTATION ///////////////////
   
    if( Vol_Rotation >3200 & Vol_Rotation <4500 & pistol_flag==0)
    {
      CCW_flag=1;
    pwm3_CCW_D3 = map(Vol_Rotation,3200,4440,100,900);
    }
    else
    {
      pwm3_CCW_D3=0;
      CCW_flag=0;
    }
    
   if((Vol_Rotation <3201 & Vol_Rotation >1799) |Vol_Rotation <560 | Vol_Rotation >4500 )
   {
       pwm2_CW_D2=0;
        pwm3_CCW_D3=0;
   }
  
  
}
   
void readgpio (void)  //Timer 6
    {                
   __HAL_IWDG_RELOAD_COUNTER(&hiwdg);//rest watch dog  
      lock_safty_time ++; 
       ///////////////////// RUN LED  ///////////////////
    RUN_Timer++;
       if(RUN_Timer==100) 
            {
        RUN_Timer=0;
            }
     if(RUN_Timer>50) 
    RUN_LED_HIGH();  
    else
   RUN_LED_LOW(); 
///////////////////////////////////////////////////////////////////

      
      
   if(SEAT_L_FRONT() ==1 & SEAT_R_FRONT() ==1 & SEAT_R_REAR()==1 & SEAT_L_REAR()==1)  //seat sensors   
      {
       SEAT_LIGHT_HIGH();
        seat_flag=1;
      //if(read_eeprom_value[0] ==0) 
			if(lock_button_flag==0)
           {
            if(LOCK_L_SEN()==0 | LOCK_R_SEN()==0)
                  {
                 lock_safty_time=0;
                LOCK_VALVE_HIGH();        //lock
                 UNLOCK_VALVE_LOW();   
                valve_lock=1;
                  }
                else
                  {
               LOCK_VALVE_LOW();
               valve_lock=0;
                  }        
           }
					 
       //if( read_eeprom_value[0] ==1) 
	 if(lock_button_flag==1)
               {
        if(UNLOCK_L_SEN()==0 & UNLOCK_R_SEN()==0)
                   {   
                 lock_safty_time=0;
                 UNLOCK_VALVE_HIGH();      //unlock
                 LOCK_VALVE_LOW();
		 valve_Unlock=1;
                  }
                 else{            
               UNLOCK_VALVE_LOW();
	             valve_Unlock=0;
                     } 
                }  
         }
            
      else if(SEAT_L_FRONT() ==0 | SEAT_R_FRONT() ==0 | SEAT_R_REAR()==0 | SEAT_L_REAR()==0)   
              { 
          SEAT_LIGHT_LOW(); 
	seat_flag=0;			
              }
 ////////////////////////////////////  Locked Lamp Green  //////////////////////////////////////      
         if(LOCK_L_SEN()==1 & LOCK_R_SEN()== 1)
              {
            LOCKED_GREEN_HIGH();
	     Locked_flag=1;									
              }
             else 
                 {
           LOCKED_GREEN_LOW();
	   Locked_flag=0;		 
                 }
 //////////////////////////////////   Unlocked lamp RED    /////////////////////////////////////                            
         if(UNLOCK_L_SEN()==1 & UNLOCK_R_SEN()==1)
                {
            UNLOCKED_RED_HIGH();  
	    Unlocked_flag=1;			
                }
              else 
                  {
            UNLOCKED_RED_LOW();
	    Unlocked_flag=0;					
                  }
      
        if(lock_safty_time >=120)
                {        
              UNLOCK_VALVE_LOW();
              LOCK_VALVE_LOW();     
                }
        
 //////////////////////////////////Work_Light function////////////////////////////////////     
      
          if(spreader_light ==1)
                {
             WORK_LIGHT_HIGH(); ////Turn on work light
                }   
          else if(spreader_light==0)
                {
              WORK_LIGHT_LOW(); //// Turn off work light
                }
      
 ///////////////////////////////Tilt_lock//////////////////////////////////////////////////    
      
            if(tilt_lock ==1)
                 {
              TILT_LOCK_LOW();//tilt lock
              TILT_LOCK1_LOW(); //tilt lock1 
                 }
           else if(tilt_lock==0)
                   {
              TILT_LOCK_HIGH();//tilt lock
              TILT_LOCK1_HIGH();//tilt lock1
                   }
  
 /////////////////////////// to 40ft hand ////////////////////////////////////////////////
 if(Auto_20_40ft==0)
 {
    if( to40ft_CMD==1 & SEAT_L_FRONT() ==0 &SEAT_R_FRONT() ==0 &SEAT_R_REAR()==0 &SEAT_L_REAR()==0 &UNLOCK_L_SEN()==1 &UNLOCK_R_SEN()==1)
{
   Timer1++;
	valve40ft=1;
if(DAMP_SEN() ==1)    
pwm4_buff_D4=800;
if(DAMP_SEN() ==0)   
pwm4_buff_D4=1000;    
 }
  if(to40ft_CMD==0 || (SPREAD_STOP_SEN() ==1& Timer1>30))
    {  
pwm4_buff_D4=0;
   valve40ft=0; 
    }
  if(to40ft_CMD==0 )
    {
   Timer1=0; 
   }
 
///////////////////////////to 20ft Hand///////////////////////////////////////////////////////////
if(to20ft_CMD==1 &SEAT_L_FRONT() ==0 &SEAT_R_FRONT() ==0 &SEAT_R_REAR() ==0 &SEAT_L_REAR() ==0 &UNLOCK_L_SEN() ==1 &UNLOCK_R_SEN() ==1)
  {
  Timer2++;
valve20ft=1;
 if(DAMP_SEN() ==1)
    {
pwm1_buff_D1=800;
    }
if(DAMP_SEN() ==0)
    {
pwm1_buff_D1=1000;
    }
  }
  if(to20ft_CMD==0 | (SPREAD_STOP_SEN() ==1 & Timer2>30))
    { 			
pwm1_buff_D1=0;
valve20ft=0;		
    }
  if(to20ft_CMD==0 )
   {
   Timer2=0; 
  }
  }

//////////////////////////////// Auto 40ft /////////////////////////////
  if(Auto_20_40ft==1 )
  {
  if(to40ft_CMD==1 & SEAT_L_FRONT()==0 &SEAT_R_FRONT()==0 &SEAT_R_REAR()==0 &SEAT_L_REAR()==0 &UNLOCK_L_SEN()==1 &UNLOCK_R_SEN()==1)
     {
   flag_40ft=1;
   flag_20ft=0;
	pwm1_buff_D1=0;		 
     }
   if(flag_40ft==1)
     {
    SPREAD_ALARM_HIGH();
  Timer4++;
 if(DAMP_SEN() ==1)
     {
pwm4_buff_D4=800;
     }
if(DAMP_SEN() ==0)
     {
pwm4_buff_D4=1000;
     }
 }
  if(flag_40ft==0 | (SPREAD_STOP_SEN() ==1 & Timer4>30))
     {  
pwm4_buff_D4=0;
flag_40ft=0;
SPREAD_ALARM_LOW();
     }
  if(flag_40ft==0 )
    {
   Timer4=0; 
    }
  //////////////////////////////////////////// Auto 20ft ////////////////////
  if(to20ft_CMD==1 & SEAT_L_FRONT()==0 &SEAT_R_FRONT()==0 &SEAT_R_REAR()==0 &SEAT_L_REAR()==0 & UNLOCK_L_SEN()==1 & UNLOCK_R_SEN()==1) 
     {
  flag_20ft=1;
  flag_40ft=0;
  pwm4_buff_D4=0;		 
     }
   if(flag_20ft==1)
   {
    SPREAD_ALARM_HIGH();
  Timer3++;
 if(DAMP_SEN() ==1)
     {
pwm1_buff_D1=800;
     }
if(DAMP_SEN() ==0)
     {
pwm1_buff_D1=1000;
     }
 }
  if(Auto_20_40ft==0 | (SPREAD_STOP_SEN() ==1 & Timer3>30))
     {  
pwm1_buff_D1=0;
flag_20ft=0;
SPREAD_ALARM_LOW();
     }
  if(flag_20ft==0 )
    {
   Timer3=0; 
    }
  }
  if(Auto_20_40ft==0)
    {
  flag_20ft=0 ;
  flag_40ft=0;
  SPREAD_ALARM_LOW();
   }		
		
 ///////////////////////////MONITORING/////////////////////////////////////////////     
UNLOCK_L_SEN =HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_6);//PG6---> Unlock TW Left
LOCK_L_SEN =HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_8);//PG8---> Lock TW Left
                 //////***************************//////////////
UNLOCK_R_SEN =HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4);//PD4---> Unlok TW right
LOCK_R_SEN =HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10);//PC10---> Lock TW Right                             /////reading GPIOS status
                 //////// seat sensors feed back ////////
SEAT_L_FRONT =HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2);//PD2---> Seat sensor Front/Left
SEAT_R_FRONT =HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_5);//PD5---> Seat Sensor Front/Right
SEAT_R_REAR =HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_3);//PD3---> Seat sensor Rear/Right
SEAT_L_REAR =HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6);//PD6--->Seat Sensor  Rear/Left
   //////////////////////////20 foot-40 foot sensors ////////////////////////////////////   
SPREAD_STOP_SEN =HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_2);//PG2--->30-35 Stop
DAMP_SEN =HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_12);//PC12---> Damping 20/40      
  }
      
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
   
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  sFilterConfig.FilterBank=0;
  sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh=0x0000;
  sFilterConfig.FilterIdLow=0x0000;
  sFilterConfig.FilterMaskIdHigh=0x0000;
  sFilterConfig.FilterMaskIdLow=0x0000;
  sFilterConfig.FilterFIFOAssignment=CAN_RX_FIFO0; 
  sFilterConfig.FilterActivation=ENABLE;
  sFilterConfig.SlaveStartFilterBank=14;
  
   if(HAL_CAN_ConfigFilter(&hcan,&sFilterConfig)!=HAL_OK)
     {
      Error_Handler();
     }
  
  if(HAL_CAN_Start(&hcan)!=HAL_OK)
    {
     Error_Handler();
    }
   
   RxHeader.StdId =0x381;
   RxHeader.RTR  = CAN_RTR_DATA;
   RxHeader.IDE = CAN_ID_STD ;
   RxHeader.DLC = 8;
   
   RxHeader_t.StdId =0x382;
   RxHeader_t.RTR  = CAN_RTR_DATA;
   RxHeader_t.IDE = CAN_ID_STD ;
   RxHeader_t.DLC = 8;
   
   RxHeader_Can_ST.StdId =0x00;
   RxHeader_Can_ST.RTR  = CAN_RTR_DATA;
   RxHeader_Can_ST.IDE = CAN_ID_STD ;
  RxHeader_Can_ST.DLC = 8;
   /////////////////////////////////////////////////////// HEART BIT TX HEADER////////////////////////////////////
   TxHeader_heart_bit.StdId = 0x208; //208=791-1 heart bit
   TxHeader_heart_bit.RTR = CAN_RTR_DATA; //tx3
   TxHeader_heart_bit.IDE = CAN_ID_STD;
   TxHeader_heart_bit.DLC = 2; 
   TxHeader_heart_bit.TransmitGlobalTime = DISABLE;
   //*********************************************************************************************************//
   TxHeader_status.StdId = 0x26C;
   TxHeader_status.RTR = CAN_RTR_DATA; //seats status
   TxHeader_status.IDE = CAN_ID_STD;
   TxHeader_status.DLC = 1; 
   TxHeader_status.TransmitGlobalTime = DISABLE;
 
  //////////////////////////////////////////////////////////////////////////////////////////////////// 
   HAL_CAN_ConfigFilter(&hcan,&sFilterConfig); //Initialize CAN Filter
   HAL_CAN_Start(&hcan); //Initialize CAN Bus
   HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING); 
   HAL_TIM_Base_Start_IT(&htim6);
   HAL_TIM_Base_Start_IT(&htim4);
   HAL_TIM_Base_Start_IT(&htim3);
   HAL_TIM_Base_Start_IT(&htim2);
  ////////////////////////////////////////////pwms enable
   HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1); 
   HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2); 
   HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3); 
   HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4); 
  //////////////////////////////////////////// 
   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);

////////////////////eeprom initialize//////////////////////////////
//HAL_FLASH_Unlock();
//EE_Init();
//////////////////////////////////////////////////////   
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   //__HAL_IWDG_RELOAD_COUNTER(&hiwdg);

 ////////////////////////////SET PWMS ////////////////////////////////////////// 
     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pwm1_buff_D1);//Pin19 to 20ft PE9
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,pwm2_CW_D2);//Pin16 Rotation CW PE11
     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,pwm3_CCW_D3);//Pin17 Rotation CCW PE13
     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,pwm4_buff_D4);//Pin18 to 40ft PE14
//////////////////////////////// BOOM UP/////////////////////////////////////////////////// 

    }
 
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 100;//100
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1100;//1100
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;//disable
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;//1000
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 10;//310
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7200;//7200
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100;//100
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 7200;//7200
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 300;//300
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7200;//7200
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 1000;//1000
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 7200;//7200
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 10;//10
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 7200;//7200
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UNLOCK_TW_VALVE_GPIO_Port, UNLOCK_TW_VALVE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, WORK_LIGHT_Pin|SEAT_LAMP_Pin|WORK_LIGHTF13_Pin|SIDE_SHIFT_R_Pin
                          |SIDE_SHIFT_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, TILT_LOCK_VALVE_Pin|LOCK_TW_VALVE_Pin|TILT_LOCK_VALVEE10_Pin|LOCK_TW_LIGHT_Pin
                          |WORK_LIGHTE15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, UNLOCK_TW_LIGHT_Pin|CENTRAL_LUBRICANT_Pin|SPREADER_ALARM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : UNLOCK_TW_VALVE_Pin */
  GPIO_InitStruct.Pin = UNLOCK_TW_VALVE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UNLOCK_TW_VALVE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : WORK_LIGHT_Pin SEAT_LAMP_Pin WORK_LIGHTF13_Pin SIDE_SHIFT_R_Pin
                           SIDE_SHIFT_L_Pin */
  GPIO_InitStruct.Pin = WORK_LIGHT_Pin|SEAT_LAMP_Pin|WORK_LIGHTF13_Pin|SIDE_SHIFT_R_Pin
                          |SIDE_SHIFT_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : TILT_LOCK_VALVE_Pin LOCK_TW_VALVE_Pin TILT_LOCK_VALVEE10_Pin LOCK_TW_LIGHT_Pin
                           WORK_LIGHTE15_Pin */
  GPIO_InitStruct.Pin = TILT_LOCK_VALVE_Pin|LOCK_TW_VALVE_Pin|TILT_LOCK_VALVEE10_Pin|LOCK_TW_LIGHT_Pin
                          |WORK_LIGHTE15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : UNLOCK_TW_LIGHT_Pin CENTRAL_LUBRICANT_Pin SPREADER_ALARM_Pin */
  GPIO_InitStruct.Pin = UNLOCK_TW_LIGHT_Pin|CENTRAL_LUBRICANT_Pin|SPREADER_ALARM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR_30_35_STOP_Pin UNLOCKED_TW__LEFT_Pin LOCKED_TW_LEFT_Pin */
  GPIO_InitStruct.Pin = SENSOR_30_35_STOP_Pin|UNLOCKED_TW__LEFT_Pin|LOCKED_TW_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LOCKED_TW_RIGHT_Pin DAMPING_20_40_FOOT_Pin */
  GPIO_InitStruct.Pin = LOCKED_TW_RIGHT_Pin|DAMPING_20_40_FOOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD2 SENSOR_SEAT_REAR__RIGHT_Pin UNLOCKED_TW_RIGHT_Pin SENSOR_SEAT_FRONT_RIGHT_Pin
                           SENSOR_SEAT_REAR_LEFT_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|SENSOR_SEAT_REAR__RIGHT_Pin|UNLOCKED_TW_RIGHT_Pin|SENSOR_SEAT_FRONT_RIGHT_Pin
                          |SENSOR_SEAT_REAR_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
