#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "AD.h"
#include "Serial.h"
#include "IR.h"

int main(void)
{
	uint32_t time_ms = 0;					//软件计时，每行+100ms
	
	/*模块初始化*/
	Serial_Init();							//USART1，PA9/PA10，115200
	IR_Init();								//PB1输出高电平，使能红外管
	AD_Init();								//ADC1扫描8通道 + DMA循环采集
	
	/*开机提示*/
	Serial_Printf("\r\nS2 8CH IR ADC+DMA\r\n");
	Serial_Printf("D1-D7: PA1-PA7, D8: PC0, IR: PB1\r\n");
	Serial_Printf("t_ms,raw0,raw1,raw2,raw3,raw4,raw5,raw6,raw7\r\n");
	
	while (1)
	{
		/*打印8路原始值，10Hz，方便看黑白变化*/
		Serial_Printf("%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
					  (unsigned)time_ms,
					  (unsigned)AD_Value[0],
					  (unsigned)AD_Value[1],
					  (unsigned)AD_Value[2],
					  (unsigned)AD_Value[3],
					  (unsigned)AD_Value[4],
					  (unsigned)AD_Value[5],
					  (unsigned)AD_Value[6],
					  (unsigned)AD_Value[7]);
		
		time_ms += 100;
		Delay_ms(100);
	}
}
