//------------------------------------------------------------------------------
// M0S - Tiny 1024 bytes RTOS for Cortex M0 devices (and other Ms)
// Author: Dumitru Stama
// Date: 2016-11-30
// License: Public Domain
//------------------------------------------------------------------------------

#ifndef DEMO_H
#define DEMO_H

#include <stdint.h>

//------------------------------------------------------------------------------
// Internal functions
//
void flip_bit(uint32_t port, uint32_t bit);
void set_bit(uint32_t port, uint32_t bit);
void reset_bit(uint32_t port, uint32_t bit);
void task1();
void task2();

//------------------------------------------------------------------------------
// External functions
//
uint32_t create_task        (void *task_address);   // returns task id or -1 in case of error
void     sleep              (uint32_t miliseconds);
void     sleep_task         (uint32_t task_id, uint32_t msecs);
void     kill               (uint32_t task_id);
void     exit               ();
uint32_t get_task_id        (void *task_address);
uint8_t *malloc             (uint32_t size);
uint32_t free               (uint8_t *memory);
uint32_t create_task        (void *task_address);
void     mutex_lock         (uint32_t mutex_id);
uint32_t mutex_try_lock     (uint32_t mutex_id);    // returns 0 if mutex was locked and 1 if it was not locked
void     mutex_unlock       (uint32_t mutex_id);
void     memset             (uint8_t *buffer, uint32_t value, uint32_t length); // length is specified in words !!!
uint32_t set_system_timer   (uint32_t value);       // returns the old value of the system tick counter
uint32_t get_system_timer   ();                     // returns the current value of the system tick counter
uint32_t get_number_of_tasks();                     // returns the current number of active tasks in array
uint32_t get_current_task_id();                     // returns the current task id
void     ipc_send           (uint32_t task_id, uint32_t val);
uint32_t ipc_read           ();                     // returns the current IPC value from the TASK STRUCT


#define     TS_CAL1             0x1FFFF7B8
#define     TS_CAL2             0x1FFFF7C2
#define     VREFINT_CAL         0x1FFFF7BA
#define     NVIC_INT_CTRL       0xE000ED04

//------------------------------------------------------------
// Systick registers
//
#define     SYSTICK_CTRL        0xE000E010
#define     SYSTICK_LOAD        0x0004          // offset to add to first reg
#define     SYSTICK_VAL         0x0008          // same
#define     SYSTICK_CALIB       0x000C          // same

//------------------------------------------------------------
// BUS registers
//
#define     FLASH_BASE          0x08000000
#define     SRAM_BASE           0x20000000
#define     PERIPHERALS_BASE    0x40000000

//------------------------------------------------------------
// APB
//
#define     APB_BASE            PERIPHERALS_BASE
#define     TIM2                APB_BASE + 0x00000      // 1KB
#define     TIM3                APB_BASE + 0x00400      // 1KB
#define     TIM6                APB_BASE + 0x01000      // 1KB
#define     TIM7                APB_BASE + 0x01400      // 1KB
#define     TIM14               APB_BASE + 0x02000      // 1KB
#define     RTC                 APB_BASE + 0x02800      // 1KB
#define     WWDG                APB_BASE + 0x02C00      // 1KB
#define     IWDG                APB_BASE + 0x03000      // 1KB
#define     SPI2                APB_BASE + 0x03800      // 1KB
#define     USART2              APB_BASE + 0x04400      // 1KB
#define     USART3              APB_BASE + 0x04800      // 1KB
#define     USART4              APB_BASE + 0x04C00      // 1KB
#define     I2C1                APB_BASE + 0x05400      // 1KB
#define     I2C2                APB_BASE + 0x05800      // 1KB
#define     USB                 APB_BASE + 0x05C00      // 1KB
#define     USB_CAN_RAM         APB_BASE + 0x06000      // 1KB
#define     BxCAN               APB_BASE + 0x06400      // 1KB
#define     CRS                 APB_BASE + 0x06C00      // 1KB
#define     PWR                 APB_BASE + 0x07000      // 1KB
#define     DAC                 APB_BASE + 0x07400      // 1KB
#define     CEC                 APB_BASE + 0x07800      // 1KB
#define     SYSCFG_COMP         APB_BASE + 0x10000      // 1KB
#define     EXTI                APB_BASE + 0x10400      // 1KB
#define     ADC                 APB_BASE + 0x12400      // 1KB
#define     TIM1                APB_BASE + 0x12C00      // 1KB
#define     SPI1                APB_BASE + 0x13000      // 1KB
#define     I2S1                SPI1                        // 1KB, SPI1 shares the port with I2S1
#define     USART1              APB_BASE + 0x13800      // 1KB
#define     TIM15               APB_BASE + 0x14000      // 1KB
#define     TIM16               APB_BASE + 0x14400      // 1KB
#define     TIM17               APB_BASE + 0x14800      // 1KB
#define     DBGMCU              APB_BASE + 0x15800      // 1KB


// AHB1
#define     AHB1_BASE           PERIPHERALS_BASE + 0x20000
#define     DMA                 AHB1_BASE + 0x00000     // 1KB
#define     RCC                 AHB1_BASE + 0x01000     // 1KB
#define     FLASH_MEMORY_INT    AHB1_BASE + 0x02000     // 1KB
#define     CRC                 AHB1_BASE + 0x03000     // 1KB
#define     TSC                 AHB1_BASE + 0x04000     // 1KB

// AHB2
#define     AHB2_BASE           PERIPHERALS_BASE + 0x08000000
#define     GPIOA               AHB2_BASE + 0x00000     // 1KB
#define     GPIOB               AHB2_BASE + 0x00400     // 1KB
#define     GPIOC               AHB2_BASE + 0x00800     // 1KB
#define     GPIOD               AHB2_BASE + 0x00C00     // 1KB
#define     GPIOE               AHB2_BASE + 0x01000     // 1KB
#define     GPIOF               AHB2_BASE + 0x01400     // 1KB

// offsets for GPIO registers
#define     MODER               0x0000
#define     OTYPER              0x0004
#define     OSPEEDR             0x0008
#define     PUPDR               0x000C
#define     IDR                 0x0010
#define     ODR                 0x0014
#define     BSRR                0x0018
#define     LCKR                0x001C
#define     AFRL                0x0020
#define     AFRH                0x0024
#define     BRR                 0x0028

//------------------------------------------------------------
// RCC - Clock Control
//
#define     RCC_CR              0x0000 
#define     RCC_CFGR            0x0004
#define     RCC_CIR             0x0008
#define     RCC_APB2RSTR        0x000C
#define     RCC_APB1RSTR        0x0010
#define     RCC_AHBENR          0x0014
#define     RCC_APB2ENR         0x0018
#define     RCC_APB1ENR         0x001C
#define     RCC_BDCR            0x0020
#define     RCC_CSR             0x0024
#define     RCC_AHBRSTR         0x0028
#define     RCC_CFGR2           0x002C
#define     RCC_CFGR3           0x0030
#define     RCC_CR2             0x0034

// These masks can be OR-ed together to activate the clocks
#define     CLK_A               0x020000
#define     CLK_B               0x040000
#define     CLK_C               0x080000
#define     CLK_D               0x100000
#define     CLK_E               0x200000
#define     CLK_F               0x400000

#endif

