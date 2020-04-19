/*
 * C_TO.cpp
 *
 *  Created on: Apr 18, 2020
 *      Author: gilg
 */


#include "main.h"
#include "ros.h"
#include "Wheel.h"
#include <cstdio>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

extern TIM_HandleTypeDef htim2,htim3,
htim4,htim5,htim8;


ros::NodeHandle nh;
nav_msgs::Odometry odom;
ros::Publisher pub("odom",&odom);

void power_wheel_1(const std_msgs::Float32& msg);
void power_wheel_2(const std_msgs::Float32& msg);
void power_wheel_3(const std_msgs::Float32& msg);
void power_wheel_4(const std_msgs::Float32& msg);

ros::Subscriber<std_msgs::Float32> power_wheel_1_sub("wheel1",&power_wheel_1);
ros::Subscriber<std_msgs::Float32> power_wheel_2_sub("wheel2",&power_wheel_2);
ros::Subscriber<std_msgs::Float32> power_wheel_3_sub("wheel3",&power_wheel_3);
ros::Subscriber<std_msgs::Float32> power_wheel_4_sub("wheel4",&power_wheel_4);

Wheel* wheel[4];

#undef DEBUG
#ifdef DEBUG
uint8_t str[200];
int strl;
#endif

extern "C"
{

void setup()
{
	wheel[0]=new Wheel(
			&htim8,TIM_CHANNEL_2,
			MOTOR1_1_GPIO_Port,MOTOR1_1_Pin,
			MOTOR1_2_GPIO_Port,MOTOR1_2_Pin,
			&htim4);
	wheel[1]=new Wheel(
			&htim8,TIM_CHANNEL_1,
			MOTOR2_1_GPIO_Port,MOTOR2_1_Pin,
			MOTOR2_2_GPIO_Port,MOTOR2_2_Pin,
			&htim5);

	wheel[2]=new Wheel(
			&htim8,TIM_CHANNEL_4,
			MOTOR3_1_GPIO_Port,MOTOR3_1_Pin,
			MOTOR3_2_GPIO_Port,MOTOR3_2_Pin,
			&htim3);
	wheel[3]=new Wheel(
			&htim8,TIM_CHANNEL_3,
			MOTOR4_1_GPIO_Port,MOTOR4_1_Pin,
			MOTOR4_2_GPIO_Port,MOTOR4_2_Pin,
			&htim2);

	for(uint8_t i=0;i<4;i++)
	{
		wheel[i]->init();
	}

	nh.initNode();
	nh.advertise(pub);
	nh.subscribe(power_wheel_1_sub);
	nh.subscribe(power_wheel_2_sub);
	nh.subscribe(power_wheel_3_sub);
	nh.subscribe(power_wheel_3_sub);

}

void loop()
{
	//Need feel
	odom.pose.pose.position.x=wheel[3]->get_way();
	//-----------------------
	pub.publish(&odom);
	nh.spinOnce();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	for(uint8_t i=0;i<4;i++)
	{
		if(htim==wheel[i]->encoder)
		{
			wheel[i]->elapsed_cnt();

		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
#ifdef DEBUG
	strl=sprintf(reinterpret_cast<char*>(str),"enc: %8lu %li %f \r\n",
			wheel[3]->encoder->Instance->CNT,wheel[3]->numbes_of_turns,wheel[3]->get_way());
	HAL_UART_Transmit_DMA(&huart2, str, strl);
#endif
#ifndef DEBUG
	nh.getHardware()->flush();
#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	nh.getHardware()->reset_rbuf();
}
}
void power_wheel_1(const std_msgs::Float32& msg)
{
	wheel[0]->set_power(msg.data);

}

void power_wheel_2(const std_msgs::Float32& msg)
{
	wheel[1]->set_power(msg.data);

}
void power_wheel_3(const std_msgs::Float32& msg)
{
	wheel[2]->set_power(msg.data);

}
void power_wheel_4(const std_msgs::Float32& msg)
{
	wheel[3]->set_power(msg.data);

}


