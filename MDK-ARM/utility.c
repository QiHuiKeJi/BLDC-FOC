#include "main.h"




void led_blink(void)
{
	static uint32_t last_tick = 0;

	if(((TIM2->CNT) - last_tick) < 500 )
	{
		last_tick = TIM2->CNT;
		LL_GPIO_TogglePin(GPIOA ,LL_GPIO_PIN_12);
		LL_GPIO_TogglePin(GPIOA ,LL_GPIO_PIN_2);
	}

}