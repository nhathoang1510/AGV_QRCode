/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stdio.h"
#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"
#include "math.h"

#define DUONG_A         HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET)
#define AM_A            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET)
#define DUONG_B         HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET)
#define AM_B            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET)
#define PWMA(x)         __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,x)
#define PWMB(y)         __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,y)

#define STOP 0
#define START 1
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//bien van toc
float va=0.0,vb=0.0,vnow=0.0,v=0.0,vout=0.0,omega;
//bien gia toc
float acc;
float dcc;
float t;
//bien la ban mpu6050
char send_data='a';
int16_t angleMpu;
uint8_t mpu[10];
uint8_t mpu_send,flag=1;
//bien FIX goc
int limitSpeed=100;
float vqMpu,vq,speedW,speedWout;
float fix,val;
int16_t E_angle,E,delta;
uint8_t vv=0;
int16_t temp_mpu;
//pid
//float Kp=0.3,Ki,Kd=0.05;
//float PidOut, PidLastOut;
float T = 0.01;
//bien goc
int16_t angleRobot;  
bool k=false;
//bien ps2
uint8_t data[7],RX,RY,LX,LY;
//bien truyen nhan data Pi 4
uint8_t send_pi[10];
uint8_t receive_pi[11];
// mode run
uint8_t moderun;
bool DONE=false;
bool RUN=false;
bool HOME=false;
uint8_t tt=0;
//button from web
uint8_t ButtonStart=0,ButtonStop=0,ButtonHome=0;
//bien Position
uint16_t Position=11;
typedef struct 
{
  uint8_t X;
  uint8_t Y;
} position;
position Pos_Tar,Pos_Cur;
uint16_t Pos=11;
bool Axis;
int16_t Delta_Pos;
uint8_t divvv;
//bien adc
uint32_t adc;
bool people=false;
//bien reset arm
uint8_t rst;
//bien check timer
int tim2,tim3;
int ccccc=0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void rstArm(void);
void pwm_gen(void);
void acceleration(void);
void angleFix(int _limitSpeed, int16_t a_want);
float acceleration_Rotate(float _v,float _accW,float _dccW,float _T);
void run_main(void);
void robotRotate(int16_t aRobot);
void robotRun(float vRun,float _acc, float _dcc, float _t);
void robotLock(void);
void control_ps2(void);
void robotRunMap(float vRun, float dccRun, uint8_t X_tar, uint8_t Y_tar, uint8_t X_cur, uint8_t Y_cur);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void rstArm(void)
{
  if(rst==1)SCB->AIRCR=0x05FA<<16|0x04;
  rst=0;
}
void pwm_gen(void)
{
  if(va>0)DUONG_A;
  else AM_A;
  
  if(vb>0) DUONG_B;
  else AM_B;
  
  PWMA((uint16_t)fabs(va));
  PWMB((uint16_t)fabs(vb));
}
void acceleration(void)
{
  if(vnow<v)
  {
    vout=acc*t;
    t+=0.01;
    vnow+=vout;
    if(vnow>v)
    {
      vnow=v;
      t=0;
      vout=0;
    }
  }
  if(vnow>v)
  {
    vout=dcc*t;
    t+=0.01;
    vnow-=vout;
    if(vnow<v)
    {
      vnow=v;
      t=0;
      vout=0;
    }
  }
}
//void angleFix(int _limitSpeed, int16_t a_want)
//{ 
//  int16_t pos_nega, temp;
//  delta = a_want - (angleMpu);
// 
////  if(E_angle>2600)E=900;
////  else if(E_angle<-2600)E=-900;
////  else E=E_angle;
//  
//  if(delta>0){
//      pos_nega= (delta+1800)/3600;
//      E_angle=delta- (3600*pos_nega);
//  }
//  else{
//      temp=((int16_t)fabs(delta)+ 1800);
//      pos_nega=temp/3600;
//      E_angle=delta+ (3600*pos_nega);
//  }
//  if(fabs(E_angle)>100) fix=4;
//  else if(fabs(E_angle)>50) fix = 3;
//  else if(fabs(E_angle)>25) fix = 2;
//  else if(fabs(E_angle)>10) fix = 1.5;
//  
//  vqMpu = fix*E_angle;
//
//  if(vqMpu > _limitSpeed)
//  {
//    vqMpu = _limitSpeed;
//  }
//  if(vqMpu < -_limitSpeed)
//  {
//    vqMpu = -_limitSpeed;
//  }
//}
void angleFix(int _limitSpeed, int16_t a_want)
{ 
  E_angle = a_want - (angleMpu);
  
//  if(E_angle>2600)E=900;
//  else if(E_angle<-2600)E=-900;
//  else E=E_angle;
  
  if(fabs(E_angle)>100) fix=3;
  else if(fabs(E_angle)>50) fix = 2;
  else if(fabs(E_angle)>25) fix = 0.9;
  else fix=1.5;

  vqMpu = fix*E_angle;

  if(vqMpu > _limitSpeed)
  {
    vqMpu = _limitSpeed;
  }
  if(vqMpu < -_limitSpeed)
  {
    vqMpu = -_limitSpeed;
  }
}
float acceleration_Rotate(float _v,float _accW,float _dccW,float _T)
{
  if(fabs(_v)>50)vv=1;
  else vv=2;
  if (_v != speedW)
  {
    if(vv==1)
    {
      if(speedW<_v)
      {
        speedWout=_accW*_T;
        speedW+=speedWout;
        _T+=0.01;
        if(speedW>_v)
        {
          _T=0;
          speedWout=0;
        }
      }
      else
      {
        speedWout=_dccW*_T;
        speedW+=speedWout;
        _T+=0.01;
        if(speedW<_v)
        {
          _T=0;
          speedWout=0;
        }
      }
    }
    if(vv==2)
    {
      if (_v > speedW) speedW += _accW;
      else speedW += _dccW;
    }
  }
  return speedW;
}
void run_main(void)
{
  if(k==true)angleFix(limitSpeed, angleRobot);
  else vqMpu=0;
  
  if(fabs(vqMpu)>75) val=1.5;
  else if(fabs(vqMpu)>50) val = 0.9;
  else if(fabs(vqMpu)>25) val = 0.5;
  else val=0.3;
  
  vq=acceleration_Rotate(vqMpu,val,-val,0.005);

  acceleration();
  
  va=vnow-vq;
  vb=vnow+vq;
  
  pwm_gen(); 
}
void robotRotate(int16_t aRobot)
{
  //k=true;
  angleRobot=aRobot;
}
void robotRun(float vRun,float _acc, float _dcc, float _t)
{
  //k=false;
  if(people==true){
    v=2;
    acc=_acc;
    dcc=_dcc;
    t=_t;
    robotRotate(angleRobot);
  }
  else {
//    if((int16_t)fabs(angleMpu)>30)k=true;
//    else k=false;
    v=vRun;
    acc=_acc;
    dcc=_dcc;
    t=_t;
    robotRotate(angleRobot);
  }
}
void robotLock(void)
{
  robotRun(2,6,2,0.01);
  robotRotate(angleRobot);
}
void robotRunMap(float vRun, float dccRun, uint8_t X_tar, uint8_t Y_tar, uint8_t X_cur, uint8_t Y_cur)
{
  //static uint8_t dem=0;
  static uint8_t step=0;
    //========= Button Stop =============//
  if(ButtonStop==1) 
  {
    RUN=STOP;
    DONE=true;
    step=0;
  }
  if(step==0)
  {
    if(ButtonStart==1|ButtonHome==1) RUN=START;
    if(RUN==START)
    {
      step=1;
      DONE=false;
    }
    else robotLock();
  }
  if(step==1&&DONE==false)
  { 
    if(Y_tar!=Y_cur)
    {
      Delta_Pos = Y_tar - Y_cur;
      if(Delta_Pos>0) 
      {
        if(angleMpu<80&&angleMpu>-20)
        {
          k=false;
          robotRun(vRun,6,dccRun,0.01);
        }
        else
        {
          k=true;
          robotRotate(30);
        }
      }
      else
      {
        if(temp_mpu<1870&&temp_mpu>1770)
        {
          k=false;
          robotRun(vRun,6,dccRun,0.01);
        }
        else
        {
          k=true;
          robotRotate(1820); 
          tt=0;
        }
      }
    }
    else step=2;
  }
  if(step==2)
  {
    robotRun(2,6,dccRun,0.01);
    if(vnow==2&&Y_cur==Y_tar)step=3;
  }
  if(step==3)
  {
    if(X_tar!=X_cur)
    {
      Delta_Pos = X_tar - X_cur;
      if(Delta_Pos>0) 
      {
        if(temp_mpu<950&&temp_mpu>850)
        {
          k=false;
          robotRun(vRun,6,dccRun,0.01);
        }
        else
        {
          k=true;
          robotRotate(-900);
        }
      }
      else
      {
        if(temp_mpu<990&&temp_mpu>890)
        {
          k=false;
          robotRun(vRun,6,dccRun,0.01);
        }
        else
        {
          k=true;
          robotRotate(940);
        }
      }
    }
    else step=4;
  }
  if(step==4)
  {
    robotRun(2,6,dccRun,0.01);
    if(vnow==2&&X_cur==X_tar)step=5;
  }
  if(step==5)
  {
    if(X_tar == X_cur&&Y_tar==Y_cur)
    {
//      if(X_cur==1&&Y_cur==1){
//        step=6;       
//      }
      DONE=true;
      RUN=STOP;  
    }
    else 
    {
      step=1;
      DONE=false;  
    }
  }
//  if(step==6){
//    k=true;
//    robotRotate(0);
//    if(angleMpu<-3550&&angleMpu>-3650)
//    {
//      HAL_Delay(1000);
//      step=7;  
//    }
//  }
//  if(step==7){
//    flag=0;
//    HAL_Delay(200);
//    //rst=1;
//    flag=1;
//    step=8;
//  }
//  if(step==8){
//    if(X_tar == X_cur&&Y_tar==Y_cur)
//    {
//      DONE=true;
//      RUN=STOP;  
//    }
//    else 
//    {
//      step=1;
//      DONE=false;  
//    }
//  }
}
void robotRunHome(float vRun, float dccRun)
{  
  robotRunMap(vRun,dccRun,1,1,Pos_Cur.X,Pos_Cur.Y);
  if(Pos_Tar.X == Pos_Cur.X&&Pos_Tar.Y==Pos_Cur.Y) HOME=false;
}
//void control_ps2(void)
//{
//  if(data[0]==1)
//  {
//      if(LX==128&&LY==128&&RX==128&&RY==128){robotLock();}
//      else
//      {
//        if(LY<120) {k=false;robotRun(100,7,30,0.01);}
//        if(LY>130) {k=false;robotRun(-100,7,30,0.01);}
//        
//        if(RX<120) {k=true;robotRotate(900);}
//        if(RX>130) {k=true;robotRotate(-900);}
//        if(RY<120) {k=true;robotRotate(0);}
//        if(RY>130) {k=true;robotRotate(1840);}
//      }
//  }
//} 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==htim2.Instance)
  { 
    //tim2++;
    //============ truyen nhan data PS2 =============//
    if(data[0]!=1)
    {
      HAL_UART_DMAStop(&huart1);
      HAL_UART_Receive_DMA(&huart1,data,7);    
    }
    RX=data[3];
    RY=data[4];
    LX=data[5];
    LY=data[6];
    //============ truyen nhan data MPU6050 =============//
    if(flag==1) send_data='z';
    else send_data ='a';
    HAL_UART_Transmit_IT(&huart2,(uint8_t *)&send_data,1);
    angleMpu=mpu[0]<<8|mpu[1];
     //============ truyen nhan data Pi =============//
    HAL_UART_Transmit_IT(&huart3,(uint8_t*)&send_pi,sizeof(send_pi));
    HAL_UART_Receive_DMA(&huart3,(uint8_t*)receive_pi,sizeof(receive_pi));
    if(receive_pi[0]!=0x46|receive_pi[0]!=0x45)
    {
      HAL_UART_DMAStop(&huart3);
      HAL_UART_Receive_DMA(&huart3,(uint8_t*)receive_pi,sizeof(receive_pi));
    }
    if(receive_pi[0]==0x45&&receive_pi[10]==0x45){
      ButtonHome=receive_pi[1]-0x30;
      ButtonStart=receive_pi[2]-0x30;
      ButtonStop=receive_pi[3]-0x30;
      Pos_Tar.X=receive_pi[7]-0x30;
      Pos_Tar.Y=receive_pi[9]-0x30;
    }
    if(receive_pi[0]==0x46&&receive_pi[10]==0x46){
      Pos_Cur.X = receive_pi[4]-0x30;
      Pos_Cur.Y = receive_pi[6]-0x30;
    }
  }  
  if(htim->Instance==htim3.Instance)
  { 
    //tim3++;
    if(adc<20)people=true;
    else people=false;
    run_main();
    rstArm();
    temp_mpu = ((int16_t)fabs(angleMpu));
    //control_ps2();
  }    
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
//  MX_GPIO_Init();
//  MX_DMA_Init();
//  MX_TIM1_Init();
//  MX_USART1_UART_Init();
//  MX_TIM2_Init();
//  MX_TIM3_Init();
//  MX_USART2_UART_Init();
//  MX_USART3_UART_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  
  HAL_UART_Receive_DMA(&huart1,(uint8_t*)data,7); 
  HAL_UART_Receive_DMA(&huart2,(uint8_t*)mpu,10);
  
  HAL_UART_Transmit_IT(&huart3,(uint8_t*)&send_pi,sizeof(send_pi));
  HAL_UART_Receive_DMA(&huart3,(uint8_t*)receive_pi,sizeof(receive_pi));
  
  HAL_ADC_Start_DMA(&hadc1,&adc,1);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//    static uint8_t kk=0;
//    if(data[1]==128)kk=1;
//    if(data[1]==32)kk=0;
//    if(kk==1){
//      robotAuto();
//    }
//    if(kk==0){
//      robotLock();
//    }
    if(ButtonHome==false) robotRunMap(50,1.8,Pos_Tar.X,Pos_Tar.Y,Pos_Cur.X,Pos_Cur.Y);
    else robotRunHome(50,2.0);
    //HAL_Delay(2);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Prescaler = 9;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
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
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
