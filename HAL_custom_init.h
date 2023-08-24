#ifndef __HAL_CUSTOM_INIT_H__
#define __HAL_CUSTOM_INIT_H__


// Set PA_0 and PA_1 as alternate function 1 -> TIM2
/*
void AF_Tim2_pins_encoder(){
    GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
*/


/***************************************************************/
/*                            Encoder                          */
/***************************************************************/


// Set PA_0 and PA_1 as alternate function 1 -> TIM5
void AF_Tim5_pins_encoder(){
    GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_TIM5_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// Set PC_6 and PC_7 as alternate function 2 -> TIM3
void AF_Tim3_pins_encoder(){
    GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/***************************************************************/
/*                             PWM                             */
/***************************************************************/

void AF_Tim2_pwm(){
    GPIO_InitTypeDef GPIO_InitStructA;
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStructA.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_15;
	GPIO_InitStructA.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructA.Pull = GPIO_NOPULL;
  	GPIO_InitStructA.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructA);


	GPIO_InitTypeDef GPIO_InitStructB;
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStructB.Pin = GPIO_PIN_3;
	GPIO_InitStructB.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructB.Pull = GPIO_NOPULL;
  	GPIO_InitStructB.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructB);
}




#endif