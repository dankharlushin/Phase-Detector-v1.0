#ifndef PIN_DESCRIPTION_H_
#define PIN_DESCRIPTION_H_

#include "stm32f4xx_hal.h"

struct {
	GPIO_TypeDef * const port;
	uint16_t const pin;
	uint8_t invert;
	GPIO_PinState lastState;
	GPIO_InitTypeDef init;
} typedef PinDescription;

/*
********* PIN DESCRIPTION FOR WIZNET *********
*/
typedef enum
{
	INT,
	RST,
	CS,
	WIZNET_PIN_COUNT
}WizNet_SPI;

static PinDescription Bus_WizNet_SPI[WIZNET_PIN_COUNT]  =
{
		{GPIOB, GPIO_PIN_3  ,  0, 0, {GPIO_PIN_3 ,  GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, 	GPIO_SPEED_FREQ_VERY_HIGH}}, //INT PB3
		{GPIOD, GPIO_PIN_2  ,  0, 0, {GPIO_PIN_2 ,  GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, 	GPIO_SPEED_FREQ_VERY_HIGH}}, //RST PD2
		{GPIOA, GPIO_PIN_15 ,  0, 0, {GPIO_PIN_15,  GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, 	GPIO_SPEED_FREQ_VERY_HIGH}}, //CS PA15
};

/*
********* PIN DESCRIPTION FOR MIXER *********
*/
typedef enum
{
	PRUPX,
	PRUPZ,
	PRUPY,
	PRUPA,
	PHASEX,
	PHASEZ,
	PHASEY,
	PHASEA,
	MIXER_PIN_COUNT
}Mixer;

static PinDescription Bus_Mixer[MIXER_PIN_COUNT]  =
{
		{GPIOB, GPIO_PIN_1 ,  0, 0, {GPIO_PIN_1 ,  GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, 	GPIO_SPEED_FREQ_VERY_HIGH}}, //PRUPX PB1
		{GPIOC, GPIO_PIN_5 ,  0, 0, {GPIO_PIN_5 ,  GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, 	GPIO_SPEED_FREQ_VERY_HIGH}}, //PRUPY PC5
		{GPIOC, GPIO_PIN_4 ,  0, 0, {GPIO_PIN_4 ,  GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, 	GPIO_SPEED_FREQ_VERY_HIGH}}, //PRUPZ PC4
		{GPIOB, GPIO_PIN_0 ,  0, 0, {GPIO_PIN_0 ,  GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, 	GPIO_SPEED_FREQ_VERY_HIGH}}, //PRUPA PB0

		{GPIOA, GPIO_PIN_2 ,  0, 0, {GPIO_PIN_2 ,  GPIO_MODE_IT_RISING, GPIO_NOPULL, 	GPIO_SPEED_FREQ_VERY_HIGH}}, //PHASEX PA2
		{GPIOA, GPIO_PIN_1 ,  0, 0, {GPIO_PIN_1 ,  GPIO_MODE_IT_RISING, GPIO_NOPULL, 	GPIO_SPEED_FREQ_VERY_HIGH}}, //PHASEY PA1
		{GPIOA, GPIO_PIN_0 ,  0, 0, {GPIO_PIN_0 ,  GPIO_MODE_IT_RISING, GPIO_NOPULL, 	GPIO_SPEED_FREQ_VERY_HIGH}}, //PHASEZ PA0
		{GPIOA, GPIO_PIN_3 ,  0, 0, {GPIO_PIN_3 ,  GPIO_MODE_IT_RISING, GPIO_NOPULL, 	GPIO_SPEED_FREQ_VERY_HIGH}}, //PHASEA PA3
};




#define GPIO_PIN_SET(pinDesc) do{HAL_GPIO_WritePin((pinDesc)->port,(pinDesc)->pin,GPIO_PIN_SET^(pinDesc)->invert);}while(0)
#define GPIO_PIN_RESET(pinDesc) do{HAL_GPIO_WritePin((pinDesc)->port,(pinDesc)->pin,GPIO_PIN_RESET^(pinDesc)->invert);}while(0)
#define GPIO_PIN_TOGGLE(pinDesc) do{HAL_GPIO_TogglePin((pinDesc)->port,(pinDesc)->pin);}while(0)
#define GPIO_PIN_WRITE(pinDesc,state) do{HAL_GPIO_WritePin((pinDesc)->port,(pinDesc)->pin,state^(pinDesc)->invert);}while(0)
#define GPIO_PIN_READ(pinDesc) (HAL_GPIO_ReadPin((pinDesc)->port,(pinDesc)->pin)^(pinDesc)->invert)
#define GPIO_PIN_INIT(pinDesc) do{HAL_GPIO_DeInit((pinDesc)->port,(pinDesc)->pin); HAL_GPIO_Init((pinDesc)->port, &((pinDesc)->init));}while(0)


#endif
