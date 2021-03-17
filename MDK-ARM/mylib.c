#include "mylib.h"


uint32_t time1;
void Delay_us(uint32_t delay)
{
	time1 = TIM2->CNT;
	while( ((TIM2->CNT) - time1) < delay) {}
}


void Delay_ms(uint32_t delay)
{
	time1 = TIM2->CNT;
	while( ((TIM2->CNT) - time1) < 1000*delay) {}	
}
	






float moving_average(moving_average_type* filter_x, float input, float window_f)
{

	if(!filter_x->init)
	{
		if(window_f > FILTER_BUF - 1)
			{
				filter_x->error=1;
				while(1){}
			}
				
		filter_x->init=1;		
			
	}
	
		if(!filter_x->filled)
		{
			
			
			filter_x->buffer[filter_x->counter1] = input ;
			filter_x->sum = filter_x->sum + filter_x->buffer[filter_x->counter1];
				if(filter_x->counter1 < window_f - 1) 
				{
					filter_x->counter1++;
				}
				else
				{
					filter_x->output  = filter_x->sum / window_f ; // out of the filter
					filter_x->filled = 1;
					filter_x->counter2=0;
					
					
				}
			 
		}
		
		// 2) start filtering
		
		else
		{
			
			
		
			filter_x->sum = filter_x->sum - filter_x->buffer[filter_x->counter2] + input;
			filter_x->output  = filter_x->sum / window_f ; // out of the filter
			filter_x->buffer[filter_x->counter2] = input; // substitute thrown out value with new value for cycling
			filter_x->counter2++;
			if(filter_x->counter2 >= window_f) filter_x->counter2=0; // array loop
			
			
		}
	
	
	return filter_x->output;
	
}








