.equ	TS_CAL1				, 0x1FFFF7B8
.equ	TS_CAL2				, 0x1FFFF7C2
.equ	VREFINT_CAL			, 0x1FFFF7BA
.equ	NVIC_INT_CTRL		, 0xE000ED04

//------------------------------------------------------------
// Systick registers
//
.equ	SYSTICK_CTRL		, 0xE000E010
.equ	SYSTICK_LOAD		, 0x0004			// offset to add to first reg
.equ	SYSTICK_VAL			, 0x0008			// same
.equ	SYSTICK_CALIB		, 0x000C			// same

//------------------------------------------------------------
// BUS registers
//
.equ	FLASH_BASE			, 0x08000000
.equ	SRAM_BASE			, 0x20000000
.equ	PERIPHERALS_BASE	, 0x40000000

//------------------------------------------------------------
// APB
//
.equ	APB_BASE			, PERIPHERALS_BASE
.equ	TIM2				, APB_BASE + 0x00000		// 1KB
.equ	TIM3				, APB_BASE + 0x00400		// 1KB
.equ	TIM6				, APB_BASE + 0x01000		// 1KB
.equ	TIM7				, APB_BASE + 0x01400		// 1KB
.equ	TIM14				, APB_BASE + 0x02000		// 1KB
.equ	RTC					, APB_BASE + 0x02800		// 1KB
.equ	WWDG				, APB_BASE + 0x02C00		// 1KB
.equ	IWDG				, APB_BASE + 0x03000		// 1KB
.equ	SPI2				, APB_BASE + 0x03800		// 1KB
.equ	USART2				, APB_BASE + 0x04400		// 1KB
.equ	USART3				, APB_BASE + 0x04800		// 1KB
.equ	USART4				, APB_BASE + 0x04C00		// 1KB
.equ	I2C1				, APB_BASE + 0x05400		// 1KB
.equ	I2C2				, APB_BASE + 0x05800		// 1KB
.equ	USB					, APB_BASE + 0x05C00		// 1KB
.equ	USB_CAN_RAM			, APB_BASE + 0x06000		// 1KB
.equ	BxCAN				, APB_BASE + 0x06400		// 1KB
.equ	CRS					, APB_BASE + 0x06C00		// 1KB
.equ	PWR					, APB_BASE + 0x07000		// 1KB
.equ	DAC					, APB_BASE + 0x07400		// 1KB
.equ	CEC					, APB_BASE + 0x07800		// 1KB
.equ	SYSCFG_COMP			, APB_BASE + 0x10000		// 1KB
.equ	EXTI				, APB_BASE + 0x10400		// 1KB
.equ	ADC					, APB_BASE + 0x12400		// 1KB
.equ	TIM1				, APB_BASE + 0x12C00		// 1KB
.equ	SPI1				, APB_BASE + 0x13000		// 1KB
.equ	I2S1				, SPI1						// 1KB, SPI1 shares the port with I2S1
.equ	USART1				, APB_BASE + 0x13800		// 1KB
.equ	TIM15				, APB_BASE + 0x14000		// 1KB
.equ	TIM16				, APB_BASE + 0x14400		// 1KB
.equ	TIM17				, APB_BASE + 0x14800		// 1KB
.equ	DBGMCU				, APB_BASE + 0x15800		// 1KB

// AHB1
.equ	AHB1_BASE			, PERIPHERALS_BASE + 0x20000
.equ	DMA					, AHB1_BASE + 0x00000		// 1KB
.equ	RCC					, AHB1_BASE + 0x01000		// 1KB
.equ	FLASH_MEMORY_INT	, AHB1_BASE + 0x02000		// 1KB
.equ	CRC					, AHB1_BASE + 0x03000		// 1KB
.equ	TSC					, AHB1_BASE + 0x04000		// 1KB

// AHB2
.equ	AHB2_BASE			, PERIPHERALS_BASE + 0x08000000
.equ	GPIOA				, AHB2_BASE + 0x00000		// 1KB
.equ	GPIOB				, AHB2_BASE + 0x00400		// 1KB
.equ	GPIOC				, AHB2_BASE + 0x00800		// 1KB
.equ	GPIOD				, AHB2_BASE + 0x00C00		// 1KB
.equ	GPIOE				, AHB2_BASE + 0x01000		// 1KB
.equ	GPIOF				, AHB2_BASE + 0x01400		// 1KB

// offsets for GPIO registers
.equ	MODER				, 0x0000
.equ	OTYPER				, 0x0004
.equ	OSPEEDR				, 0x0008
.equ	PUPDR				, 0x000C
.equ	IDR					, 0x0010
.equ	ODR					, 0x0014
.equ	BSRR				, 0x0018
.equ	LCKR				, 0x001C
.equ	AFRL				, 0x0020
.equ	AFRH				, 0x0024
.equ	BRR					, 0x0028

//------------------------------------------------------------
// RCC - Clock Control
//
.equ	RCC_CR				, 0x0000 
.equ	RCC_CFGR			, 0x0004
.equ	RCC_CIR				, 0x0008
.equ	RCC_APB2RSTR		, 0x000C
.equ	RCC_APB1RSTR		, 0x0010
.equ	RCC_AHBENR			, 0x0014
.equ	RCC_APB2ENR			, 0x0018
.equ	RCC_APB1ENR			, 0x001C
.equ	RCC_BDCR			, 0x0020
.equ	RCC_CSR				, 0x0024
.equ	RCC_AHBRSTR			, 0x0028
.equ	RCC_CFGR2			, 0x002C
.equ	RCC_CFGR3			, 0x0030
.equ	RCC_CR2				, 0x0034

// These masks can be OR-ed together to activate the clocks
.equ	CLK_A				, 0x020000
.equ	CLK_B				, 0x040000
.equ	CLK_C				, 0x080000
.equ	CLK_D				, 0x100000
.equ	CLK_E				, 0x200000
.equ	CLK_F				, 0x400000


//------------------------------------------------------------
//------------------------------------------------------------
// M0S defines
//------------------------------------------------------------
//------------------------------------------------------------
.equ	KERNEL_MEMORY_START	, 0x20000000
.equ	TOTAL_MEMORY		, 0x4000		// 16 KB in my case
.equ	MAX_KERNEL_MEMORY	, 512			// it's in bytes
.equ	KERNEL_MEMORY_SHIFTL, 9				// 2^9 = 512
.equ	ALL_MEMORY_SHIFT_L	, 14			// 2^14 = 16KB of memory
.equ	KERNEL_STACK_TOP	, 0x20000400	// top of kernel stack
.equ	KERNEL_STACK_SIZE	, 256			// kernel needs only 256 bytes for the stack
.equ	IDLE_TASK_STACK_TOP	, 0x20000300	// top of idle task stack
.equ	IDLE_TASK_STACK_SIZE, 256			// only 256 bytes of stack size for idle task

.equ	MAX_TASKS			, 4
.equ	MAX_MUTEX			, 8
.equ	OFS_CURRENT_TASK	, 0x0000
.equ	OFS_NUM_TASKS		, 0x0004
.equ	OFS_SYS_TIMER		, 0x0008		// 49 days rollover if systick period is 1ms
.equ	OFS_IDLE_STACK_POS	, 0x000C		// here I store the idle task stack top
.equ	OFS_IDLE_NR_SCHED	, 0x0010		// here I store how many times i scheduled the idle task
.equ	OFS_MUTEX_AREA		, OFS_TASK_ARRAY + (MAX_TASKS * SIZE_TASK_ENTRY)

.equ	OFS_TASK_ARRAY		, 0x0020		// task array starts at 0x20000020
.equ	SIZE_TASK_ENTRY		, 0x0020		// 32 bytes / entry
.equ	TASK_ENTRY_SHIFT_L	, 5				// how many bits to shift left to multiply by entry size; 2^5 = 32
.equ	TASK_STACK_SIZE		, 0x0400		// 1024 bytes / stack / thread
.equ	TASK_STACK_SHIFT_L	, 10			// how many bits to shift left to multiply by stack size; 2^10 = 1024
.equ	FREE_MEMORY_START	, KERNEL_MEMORY_START + MAX_KERNEL_MEMORY + KERNEL_STACK_SIZE + IDLE_TASK_STACK_SIZE	// Start of available memory (>=)
.equ	FREE_MEMORY_END		, KERNEL_MEMORY_START + TOTAL_MEMORY - (MAX_TASKS * TASK_STACK_SIZE)					// End of available memory (<) 

// Offsets in a TASK ENTRY
.equ	TASK_ENTRY_STACK	, 0x0000
.equ	TASK_ENTRY_PC		, 0x0004
.equ	TASK_ENTRY_IPC		, 0x0008
.equ	TASK_ENTRY_STATE	, 0x000C		// When a task in non-existent, this dword should be 0 !!!!
											// 00 = inexistent
											// 01 = not yet scheduled
											// 10 = running
											// 11 = sleeping
.equ	unused				, 0x0010
.equ	TASK_ENTRY_TARGET	, 0x0014		// stores the target value for sleeping tasks
.equ	TASK_ENTRY_NR_SCHED	, 0x0018		// how many times was this thread scheduled
.equ	TASK_ENTRY_MUTEX	, 0x001C

.equ	STATE_INACTIVE		, 0x0000
.equ	STATE_UNSCHEDULED	, 0x0001
.equ	STATE_RUNNING		, 0x0002
.equ	STATE_SLEEPING		, 0x0003
.equ	STATE_SENT_TO_SLEEP	, 0x0004
.equ	STATE_WAIT_FOR_MUTEX, 0x0005

// offsets in the stack frame saved automatically by interrupts
.equ	FRAME_R0			, 0x0000
.equ	FRAME_R1			, 0x0004
.equ	FRAME_R2			, 0x0008
.equ	FRAME_R3			, 0x000C
.equ	FRAME_R12			, 0x0010
.equ	FRAME_LR			, 0x0014
.equ	FRAME_PC			, 0x0018
.equ	FRAME_XPSR			, 0x001C

