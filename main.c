//PIR sensor code
#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#include "stdio.h"

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
        /* GPIOA Periph clock enable */

        GPIOA->MODER |= GPIO_MODER_MODER5_0 ;
        /* Configure PA5 in output  mode  */

        GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5 ;
        // Ensure push pull mode selected--default

        GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5;
        //Ensure maximum speed setting (even though it is unnecessary)

        GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR5;
        //Ensure all pull up pull down resistors are disabled
        //----------------------------------------------------------
        //PC8 input from PIR sensor initialization:

        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
        /* GPIOC Periph clock enable */

        GPIOC->MODER &= 0x00000000 ;
        /* Configure PC8 in input  mode  */

        GPIOC->OTYPER &= ~GPIO_OTYPER_OT_8 ;
        // Ensure push pull mode selected--default

        GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8;
        // Ensure maximum speed setting (even though it is unnecessary)

        GPIOC->PUPDR |= 0x00010000;
        // PC8 is pull-up
        //----------------------------------------------------------

        while (1) {
            if (GPIOC->IDR == 0x0100) { //if motion is detected on PC8
                GPIOA->BSRR = 0x0020; //light up the LED on PA5
                printf("motion detected!\n");
                //GPIOA_ODR |= 0x0020;
            }
            else {
                GPIOA->ODR = 0x0000; //turn the LED off
            }

            delay(5000);
        } // end while loop
    return 0;
}
