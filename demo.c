//------------------------------------------------------------------------------
// M0S - Tiny 1024 bytes RTOS for Cortex M0 devices (and other Ms)
// Author: Dumitru Stama
// Date: 2016-11-30
// License: Public Domain
//------------------------------------------------------------------------------

#include "demo.h"

//-----------------------------------------------------
void task1(){
    uint32_t test = 0;
    //uint8_t *my_mem1;
    //uint8_t *my_mem2;
    

    while(1) {
        //my_mem1 = malloc(13);           // it will be rounded to 16
        //memset(my_mem1, 0x41, 4);
        //my_mem2 = malloc(16);
        //memset(my_mem2, 0x42, 4);
        //if (my_mem1 != 0) free(my_mem1);
        flip_bit(GPIOA, 5);
        sleep(40);

        //if (my_mem2 != 0) free(my_mem2);

        flip_bit(GPIOA, 5);
        sleep(460);                     // during this sleep the idle task is supposed to combine the blocks

        test++;
        // kill task2 after 5 seconds
        if (test == 15) kill(get_task_id(&task2));
    }
}

//-----------------------------------------------------
void task2(){
    while(1) {
        flip_bit(GPIOB, 5);
        sleep(100);

        flip_bit(GPIOB, 5);
        sleep(900);
    }
}

//-----------------------------------------------------
// Let's init the clocks, PLL, GPIO ports and tasks
//-----------------------------------------------------
void init() {

    uint32_t tmpval;

    *(uint32_t *)(RCC + RCC_CFGR) = 0;
    *(uint32_t *)(RCC + RCC_CFGR) = 0x00280002;

    tmpval = *(uint32_t *)(RCC + RCC_CR);
    *(uint32_t *)(RCC + RCC_CR) = tmpval | (1 << 24);

    *(uint32_t *)(RCC + RCC_AHBENR) |= CLK_A | CLK_B;
    *(uint32_t *)(GPIOA) |= 0x400;
    *(uint32_t *)(GPIOB) |= 0x400;

    create_task(&task1);
    create_task(&task2);
}

//---------------------------------------------------
void flip_bit(uint32_t port, uint32_t bit) {
    uint32_t val;

    val = *(uint32_t *)((uint8_t *)port+ODR);
    val ^= 1 << bit;
    *(uint32_t *)((uint8_t *)port+ODR) = val;
}

