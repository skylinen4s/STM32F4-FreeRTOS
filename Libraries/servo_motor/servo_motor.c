#include "FreeRTOS.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "servo_motor.h"

void coordinate(int x, int y)
{ 
  //60 -> -90' MIN
  //150 -> 0'  MID
  //240 -> 90' MAX
  if(x < MOTOR_SG90_MIN) x = MOTOR_SG90_MIN;
  if(y < MOTOR_SG90_MIN) y = MOTOR_SG90_MIN;
  
  if(x > MOTOR_SG90_MAX) x = MOTOR_SG90_MAX;
  if(y > MOTOR_SG90_MAX) y = MOTOR_SG90_MAX;

  TIM_SetCompare1(TIM3, x);
	TIM_SetCompare2(TIM3, y);
}

void init_ServoMotor()
{
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE );
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE );

  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_OCInitTypeDef TIM_OCInitStruct;
    
  GPIO_StructInit(&GPIO_InitStructure); // Reset init structure

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);
    
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7| GPIO_Pin_8| GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;// Alt Function - Push Pull
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init( GPIOC, &GPIO_InitStructure );  
  
  //  84MHz /(period * prescaler) = 50Hz ,that is 0.2ms
  TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
  TIM_TimeBaseInitStruct.TIM_Period = 2000 - 1;
  TIM_TimeBaseInitStruct.TIM_Prescaler = 840 - 1;
  TIM_TimeBaseInit( TIM3, &TIM_TimeBaseInitStruct );
  

  TIM_OCStructInit( &TIM_OCInitStruct );
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;

  TIM_OC1Init( TIM3, &TIM_OCInitStruct ); // Channel 1  LED
  TIM_OC2Init( TIM3, &TIM_OCInitStruct ); // Channel 2  LED
  TIM_OC3Init( TIM3, &TIM_OCInitStruct ); // Channel 3  LED
  TIM_OC4Init( TIM3, &TIM_OCInitStruct ); // Channel 4  LED

  TIM_Cmd( TIM3, ENABLE );
}
