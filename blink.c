/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "stm32f4xx_hal_gpio.h"

void delay (int a) {
    volatile int i,j;
    for (i=0 ; i < a ; i++) {
        j++;
    }
    return;
}

int main(){
        //----------------------------------------------------------
        //PA5 LED output initialization:

        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        // GPIOA Periph clock enable

        GPIOA->MODER |= GPIO_MODER_MODER5_0 ;
        // Configure PA5 in output  mode

        GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5 ;
        // Ensure push pull mode selected--default

        GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5;
        //Ensure maximum speed setting (even though it is unnecessary)

        GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR5;
        //Ensure all pull up pull down resistors are disabled

        while (1) {
                GPIOA->BSRR = 0x0020; //light up the LED on PA5
                delay(50000);
                GPIOA->ODR = 0x0000; //turn the LED off
       }
    return 0;
}

