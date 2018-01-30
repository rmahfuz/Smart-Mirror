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
			
#define BSRR_VAL        0x0030

void delay (int a);

int main(void){
    /* GPIOA Periph clock enable */
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

        GPIOA->MODER |= (GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0) ;
        /* Configure PC8 and PC9 in output  mode  */

        GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_4 | GPIO_OTYPER_OT_5) ;
        // Ensure push pull mode selected--default

        GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR4|GPIO_OSPEEDER_OSPEEDR5);
        //Ensure maximum speed setting (even though it is unnecessary)

        GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4|GPIO_PUPDR_PUPDR5);
        //Ensure all pull up pull down resistors are disabled

        while (1)
        {
            /* Set PC8 and PC9 */
            GPIOA->BSRR = BSRR_VAL;
            delay(500000);
            /* Reset PC8 and PC9 */
            //GPIOA->BSRR = 0x0000; //changed this
            GPIOA->ODR = 0x0000; //changed this
            delay(500000);
        }

        return 0;
    }

void delay (int a)
{
    volatile int i,j;

    for (i=0 ; i < a ; i++)
    {
        j++;
    }

    return;
}
