/*
 * C_TO.cpp
 *
 *  Created on: Apr 18, 2020
 *      Author: gilg
 */


#include "main.h"
#include "ros.h"
#include <nav_msgs/Odometry.h>


extern TIM_HandleTypeDef htim2,htim3,
htim4,htim5,htim8;

nav_msgs::Odometry odom;

ros::NodeHandle nh;


extern "C"
{

void setup()
{
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1);

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim5);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

	nh.initNode();

}

void loop()
{
	motor_4_left();
	TIM8->CCR3=0;
	HAL_Delay(5000);
	motor_4_right();
	TIM8->CCR3=0;
	HAL_Delay(5000);


	nh.spinOnce();
//	HAL_Delay(1000);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim2)
	{

	}
	if(htim==&htim3)
	{

	}
	if(htim==&htim4)
	{

	}
	if(htim==&htim5)
	{

	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	nh.getHardware()->reset_rbuf();
}

}
