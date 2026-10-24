#include "stm32f10x.h"                  // Device header

/*
 * 已确认实物：STM32F103VET6（LQFP100）
 *
 * 按原理图接：
 *   D1~D7 -> PA1~PA7 -> ADC1_IN1~IN7
 *   D8     -> PC0     -> ADC1_IN10
 *   IR     -> PB1     -> 普通GPIO输出，高电平使能
 */
#define IR_D8_GPIO_CLK       RCC_APB2Periph_GPIOC
#define IR_D8_GPIO_PORT      GPIOC
#define IR_D8_GPIO_PIN       GPIO_Pin_0
#define IR_D8_ADC_CHANNEL    ADC_Channel_10

volatile uint16_t AD_Value[8];			//DMA会直接改这个数组，必须加volatile

/**
  * 函    数：8路红外AD初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：D1~D7 -> PA1~PA7 -> ADC1_IN1~IN7
  *           D8     -> PC0      -> ADC1_IN10
  *           IR     -> PB1      -> 普通GPIO输出，高电平使能
  */
void AD_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);	//开启ADC1的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//开启GPIOA的时钟
	RCC_APB2PeriphClockCmd(IR_D8_GPIO_CLK, ENABLE);			//开启D8所在端口的时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		//开启DMA1的时钟
	
	/*设置ADC时钟*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);						//ADCCLK = 72MHz / 6 = 12MHz，不超过14MHz
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 |
								  GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//PA1~PA7初始化为模拟输入
	
	GPIO_InitStructure.GPIO_Pin = IR_D8_GPIO_PIN;
	GPIO_Init(IR_D8_GPIO_PORT, &GPIO_InitStructure);		//D8初始化为模拟输入（PC0）
	
	/*规则组通道配置*/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);	//D1 -> PA1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_55Cycles5);	//D2 -> PA2
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_55Cycles5);	//D3 -> PA3
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 4, ADC_SampleTime_55Cycles5);	//D4 -> PA4
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 5, ADC_SampleTime_55Cycles5);	//D5 -> PA5
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 6, ADC_SampleTime_55Cycles5);	//D6 -> PA6
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 7, ADC_SampleTime_55Cycles5);	//D7 -> PA7
	ADC_RegularChannelConfig(ADC1, IR_D8_ADC_CHANNEL, 8, ADC_SampleTime_55Cycles5);	//D8 -> PC0
	
	/*ADC初始化*/
	ADC_InitTypeDef ADC_InitStructure;											//定义结构体变量
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;							//独立模式，单独使用ADC1
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;						//数据右对齐
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;			//软件触发
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;							//连续转换
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;								//扫描模式
	ADC_InitStructure.ADC_NbrOfChannel = 8;										//规则组8个通道
	ADC_Init(ADC1, &ADC_InitStructure);											//配置ADC1
	
	/*DMA初始化*/
	DMA_InitTypeDef DMA_InitStructure;											//定义结构体变量
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;				//源：ADC1数据寄存器
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//外设数据宽度：半字
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设地址不自增
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)AD_Value;					//目的：AD_Value数组
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;			//存储器数据宽度：半字
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;						//存储器地址自增
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;							//外设 -> 存储器
	DMA_InitStructure.DMA_BufferSize = 8;										//8个通道，转运8次
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;								//循环模式
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;								//失能存储器到存储器
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;						//中等优先级
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);								//配置DMA1通道1
	
	/*DMA和ADC使能*/
	DMA_Cmd(DMA1_Channel1, ENABLE);							//DMA1通道1使能
	ADC_DMACmd(ADC1, ENABLE);								//ADC1的DMA请求使能
	ADC_Cmd(ADC1, ENABLE);									//ADC1使能
	
	/*ADC校准*/
	ADC_ResetCalibration(ADC1);								//复位校准
	while (ADC_GetResetCalibrationStatus(ADC1) == SET);		//等待复位校准完成
	ADC_StartCalibration(ADC1);								//开始校准
	while (ADC_GetCalibrationStatus(ADC1) == SET);			//等待校准完成
	
	/*ADC触发*/
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);					//软件触发一次，之后连续转换
}
