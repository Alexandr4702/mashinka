
#include "main.h"
#include "cmath"

class Wheel
{
public:
	TIM_HandleTypeDef* control_tim;
	uint32_t control_chanel;
	GPIO_TypeDef* direction_port_1;
	uint16_t direction_pin_1;
	GPIO_TypeDef* direction_port_2;
	uint16_t direction_pin_2;
	TIM_HandleTypeDef* encoder;

	int32_t numbes_of_turns=-1;//вот -1)
	float way=0;

	Wheel(TIM_HandleTypeDef* control_tim_,uint32_t control_chanel_,
			GPIO_TypeDef* direction_port_1_,uint16_t direction_pin_1_,
			GPIO_TypeDef* direction_port_2_,uint16_t direction_pin_2_,
			TIM_HandleTypeDef* encoder_);
	~Wheel();
	void init();
	void elapsed_cnt();
	void set_power(float power);
	float get_way();
private:
	void forward();
	void back();

};
