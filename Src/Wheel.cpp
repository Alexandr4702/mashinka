/*
 * wheel.cpp
 *
 *  Created on: Apr 19, 2020
 *      Author: gilg
 */

#include "Wheel.h"

Wheel::Wheel(TIM_HandleTypeDef *control_tim_, uint32_t control_chanel_,
		GPIO_TypeDef *direction_port_1_, uint16_t direction_pin_1_,
		GPIO_TypeDef *direction_port_2_, uint16_t direction_pin_2_,
		TIM_HandleTypeDef *encoder_):
		control_tim(control_tim_),control_chanel(control_chanel_),
		direction_port_1(direction_port_1_),direction_pin_1(direction_pin_1_),
		direction_port_2(direction_port_2_),direction_pin_2(direction_pin_2_),
		encoder(encoder_)
{
}

Wheel::~Wheel()
{
	HAL_TIM_PWM_Stop(control_tim, control_chanel);
}

void Wheel::init()
{
	__HAL_TIM_SET_COMPARE(control_tim,control_chanel,0);
	HAL_TIM_PWM_Start(control_tim, control_chanel);
	HAL_TIM_Encoder_Start(encoder, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(encoder, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(encoder);
}

void Wheel::elapsed_cnt()
{
	numbes_of_turns+=__HAL_TIM_IS_TIM_COUNTING_DOWN(encoder)?-1:1;
}
/*
 * brief: set power's motor
 * param:
 * power - power of motor -1:1
 *
 */
void Wheel::set_power(float power)
{
	if(power>=0.0f)
	{forward();}
	else {back();}
	if(power>1.0f||power<-1.0f)return;
	__HAL_TIM_SET_COMPARE(control_tim,control_chanel,control_tim->Instance->ARR*fabs(power));

}




inline void Wheel::forward()
{
	HAL_GPIO_WritePin(direction_port_2,direction_pin_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(direction_port_1, direction_pin_1, GPIO_PIN_SET);
}

inline void Wheel::back()
{
	HAL_GPIO_WritePin(direction_port_1, direction_pin_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(direction_port_2,direction_pin_2, GPIO_PIN_SET);
}

float Wheel::get_way()
{
	way=static_cast<float>(numbes_of_turns)*static_cast<float>(control_tim->Instance->ARR);
	way+=static_cast<float>(encoder->Instance->CNT);
	return way;
}
