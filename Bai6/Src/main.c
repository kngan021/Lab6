/* File name: Lab6_1
 *
 * Description: Nhấn nút B2 để đảo trạng thái đèn và còi
 *
 *
 * Last Changed By:  $Author: Nhóm 1
 * Last Changed:     $Date: 09/05/2023
 *
 *
 */

#include<stdio.h>
#include<stdint.h>
#include<stm32f401re.h>
#include "stm32f401re_rcc.h"
#include "stm32f401re_gpio.h"



#define BUTTON_PRESS						0
#define BUTTON_RELEASE						1


//Define GPIO_PIN--------------------------------------------------------------
#define LED_GPIO_PORT						GPIOA
#define LED_GPIO_PIN                     	GPIO_Pin_0
#define LED_GPIO_CLOCK_EN					RCC_AHB1Periph_GPIOA

#define BUZZER_GPIO_PORT					GPIOC
#define BUZZER_GPIO_PIN 					GPIO_Pin_9
#define BUZZER_GPIO_CLOCK_EN				RCC_AHB1Periph_GPIOC

#define BUTTON_GPIO_PORT					GPIOB
#define BUTTON_GPIO_PIN 					GPIO_Pin_3
#define BUTTON_GPIO_CLOCK_EN				RCC_AHB1Periph_GPIOB

//function declaration---------------------------------------------------------
static void mainInit(void);
static void led_Init(void);
static void buttonB2_Init(void);
static void buzzer_Init(void);


//-----------------------------------------------------------------------------
int main(void)
{

	mainInit();

	while(1)
	{
		if (GPIO_ReadInputDataBit(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN) == BUTTON_PRESS)
		{
			GPIO_SetBits(LED_GPIO_PORT, LED_GPIO_PIN);
			GPIO_SetBits(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN);
		}


		else
		{
			GPIO_ResetBits(LED_GPIO_PORT, LED_GPIO_PIN);
			GPIO_ResetBits(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN);
		}
	}
}


static
void mainInit(void){
	SystemInit();
	buttonB2_Init();
	buzzer_Init();
	led_Init();
}
/**
 * @func   Initializes GPIO Use Led
 * @brief  Led_Init
 * @param  None
 * @retval None
 */
static
void led_Init(void)
{
	//Declare type variable GPIO Struc-----------------------------------------
	GPIO_InitTypeDef GPIO_InitStructure;

	//Enable Clock GPIOA-------------------------------------------------------
	RCC_AHB1PeriphClockCmd(LED_GPIO_CLOCK_EN , ENABLE);

	//Choose Pin Led-----------------------------------------------------------
	GPIO_InitStructure.GPIO_Pin = LED_GPIO_PIN;

	//Choose Pin Led as Out----------------------------------------------------
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;

	//Choose Speed Pin---------------------------------------------------------
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	//Select type--------------------------------------------------------------
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

	//Select status------------------------------------------------------------
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;

	//The function initializes all of the above values-------------------------
	GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);
}

/**
 * @func   Initializes GPIO Use Buzzer
 * @brief  Buzzer_Init
 * @param  None
 * @retval None
 */
static
void buzzer_Init(void)
{
	//Declare type variable GPIO Struc-----------------------------------------
	GPIO_InitTypeDef GPIO_InitStructure;

	//Enable Clock GPIOC-------------------------------------------------------
	RCC_AHB1PeriphClockCmd(BUZZER_GPIO_CLOCK_EN, ENABLE);
//Choose Pin Buzzer--------------------------------------------------------
	GPIO_InitStructure.GPIO_Pin = BUZZER_GPIO_PIN;

	//Choose Pin Buzzer as Out-------------------------------------------------
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;

	//Choose Speed Pin---------------------------------------------------------
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	//Select type--------------------------------------------------------------
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

	//Select status------------------------------------------------------------
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;

	//The function initializes all of the above values-------------------------
	GPIO_Init(BUZZER_GPIO_PORT, &GPIO_InitStructure);
}

/**
 * @func   Initializes GPIO Use Button
 * @brief  ButtonInit
 * @param  None
 * @retval None
 */
static
void buttonB2_Init(void)
{
	//Declare type variable GPIO Struct----------------------------------------
	GPIO_InitTypeDef GPIO_InitStructure ;

	//Enable Clock GPIOC-------------------------------------------------------
	RCC_AHB1PeriphClockCmd(BUTTON_GPIO_CLOCK_EN, ENABLE);

	//Choose Pin BUTTON--------------------------------------------------------
	GPIO_InitStructure.GPIO_Pin = BUTTON_GPIO_PIN;

	//Choose Pin Led as Input--------------------------------------------------
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;

	//Select status------------------------------------------------------------
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;

	//The function initializes all of the above values-------------------------
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
