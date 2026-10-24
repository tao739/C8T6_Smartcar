#include "stm32f10x.h"                  // Device header

/**
  * 函    数：红外模块使能引脚初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：IR接PB1，高电平使能所有红外管；低电平关闭。
  */
void IR_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//开启GPIOB时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//PB1推挽输出
	
	GPIO_SetBits(GPIOB, GPIO_Pin_1);						//输出高电平，使能红外管
}

/**
  * 函    数：打开红外管
  */
void IR_On(void)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_1);
}

/**
  * 函    数：关闭红外管
  */
void IR_Off(void)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
}
