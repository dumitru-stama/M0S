//------------------------------------------------------------------------------
// M0S - Tiny 1024 bytes RTOS for Cortex M0 devices (and other Ms)
// Author: Dumitru Stama
// Date: 2016-11-30
// License: Public Domain
//------------------------------------------------------------------------------
.thumb
.syntax unified
.include "stm32f072.inc"
.text

//------------------- usefull macros --------------------
.macro ret
    bx lr
.endm

.macro call address
    bl \address
.endm

//-----------------------------------------------------------------------------
// This is the vector table. I didn't include all the upper vectors since
// this is a 1KB contest and I don't intend to use peripheral interrupts
//-----------------------------------------------------------------------------
Vector_Table:    
    .word  0x20000400                   // Top of stack (same as kernel stack)
    .word  _start+1                     // Start address 
    .word  default_handler+1            // Odd addresses = Thumb addresses
    .word  default_handler+1
    .word  0
    .word  0
    .word  0
    .word  0
    .word  0
    .word  0
    .word  0
    .word  default_handler+1
    .word  0
    .word  0
    .word  default_handler+1
    .word  scheduler+1
    .word  default_handler+1            // Window WatchDog
//  .word  default_handler+1            // PVD and VDDIO2 through EXTI Line detect
//  .word  default_handler+1            // RTC through EXTI line
//  .word  default_handler+1            // FLASH
//  .word  default_handler+1            // RCC and CRS
//  .word  default_handler+1            // EXTI Lines 0, 1
//  .word  default_handler+1            // EXTI Lines 2, 3
//  .word  default_handler+1            // EXTI Lines 4..15
//  .word  default_handler+1            // TSC
//  .word  default_handler+1            // DMA1 Channel 1
//  .word  default_handler+1            // DMA1 Channels 2, 3
//  .word  default_handler+1            // DMA1 Channels 4, 5, 6, 7
//  .word  default_handler+1            // ADC1, COMP1, COMP2
//  .word  default_handler+1            // TIM1 Break, Update, Trigger
//  .word  default_handler+1            // TIM1 Capture Compare
//  .word  default_handler+1            // TIM2
//  .word  default_handler+1            // TIM3
//  .word  default_handler+1            // TIM6 and DAC
//  .word  default_handler+1            // TIM7
//  .word  default_handler+1            // TIM14
//  .word  default_handler+1            // TIM15
//  .word  default_handler+1            // TIM16
//  .word  default_handler+1            // TIM17
//  .word  default_handler+1            // I2C1
//  .word  default_handler+1            // I2C2
//  .word  default_handler+1            // SPI1
//  .word  default_handler+1            // SPI2
//  .word  default_handler+1            // USART1
//  .word  default_handler+1            // USART2
//  .word  default_handler+1            // USART3 and USART4
//  .word  default_handler+1            // CEC and CAN
//  .word  default_handler+1            // USB

default_handler:
	//mrs r7,PSP
    b default_handler

//------------------------------------------------------------------------------
scheduler:
    ldr r0, =0x20000000             // r0 = ram_start
    
    ldr r1,[r0,OFS_SYS_TIMER]       // Increment system timer
    adds r1,1
    str r1,[r0,OFS_SYS_TIMER]

    ldr r1,[r0,OFS_NUM_TASKS]
    orrs r1,r1                      // if the number of tasks is 0 then
    beq scheduler_exit              // we do not use task switching

    mrs r1,PSP                      // get user mode stack
    subs r1,8*4                     // make space for registers r4-r11
    stmia r1!,{r4-r7}               // store r4-r7 and increment r1 for each one
    mov r2,r8                       // r2, r3, r4 and r5 are already saved
    mov r3,r9                       // so we can use them as intermediary regs
    mov r4,r10
    mov r5,r11
    stmia r1!,{r2-r5}               // registers are in order on stack r4...r11
    subs r1,8*4                     // adjust stack pointer back
    msr PSP,r1                      // set PSP to the new level
    
    ldr r6,[r0,OFS_CURRENT_TASK]    // get current task in r6 and keep it
    orrs r6,r6                      // will affect N flag if it's the "idle task"
    uxtb r6,r6                      // this instruction does not affect the flags
    bpl continue_normal_task        // jump if last task was not the "idle task"

    str r1,[r0,OFS_IDLE_STACK_POS]  // store it
    b sleeping_tasks                // go to process sleeping tasks

continue_normal_task:
    lsls r1,r6,TASK_ENTRY_SHIFT_L   // multiply by the size of a TASK ENTRY
    adds r1,OFS_TASK_ARRAY          // r1 = offset of the current task in array
    adds r1,r0                      // add RAM start address or kernel memory start
                                    // now r1 points to the task entry in the array
    ldr r2,[r1,TASK_ENTRY_STATE]    // Let's check its state
    //lsrs r2,30
    cmp r2,STATE_UNSCHEDULED        // if it's "not yet scheduled" or "unused"
    bls sleeping_tasks              // we don't save stack; it's not theirs
                                    // Save stack if running or sleeping
    mrs r2,PSP
    str r2,[r1,TASK_ENTRY_STACK]    // save stack in the current task structure
                                    // r0 = ram_start
                                    // r6 = current task id
sleeping_tasks:
    movs r1,OFS_TASK_ARRAY
    adds r1,r0                      // r1 = start of the task array
    movs r2,MAX_TASKS-1             // r2 = id of last task in array

process_sleeping_tasks:
    lsls r3,r2,TASK_ENTRY_SHIFT_L   // get the task offset in array
    adds r3,r1                      // r3 holds the address of the task in array
    ldr r4,[r3,TASK_ENTRY_STATE]    // get its state
    
    cmp r4,STATE_WAIT_FOR_MUTEX     // is it waiting for a MUTEX ?
    bne maybe_is_sleeping           // if not, maybe it's sleeping

check_to_see_if_mutex_is_free:
    ldr r5,[r3,TASK_ENTRY_MUTEX]    // It is waiting for a MUTEX so we check it
	ldr r4,[r0,OFS_MUTEX_STORAGE]	// get the value storin mutexes
	movs r7,1
	lsls r7,r5
	tst r4,r7						// see if mutex is available
    bne advance_counter             // we jump to the next task

	orrs r4,r7						// It was available so lock it
	str r4,[r0,OFS_MUTEX_STORAGE]
    b set_state_to_running          // to running so we can schedule the task

maybe_is_sleeping:
    cmp r4,STATE_SLEEPING           // is the task sleeping ?
    beq adjust_sleeping_value       // if so, jump to adjust its sleep counter
    cmp r4,STATE_SENT_TO_SLEEP      // was it put to sleep by other task ?
    bne advance_counter             // if not then we process the next task

adjust_sleeping_value:
    ldr r4,[r3,TASK_ENTRY_TARGET]   // increment the sleep counter
    subs r4,1
    str r4,[r3,TASK_ENTRY_TARGET]
    bne advance_counter             // if it's not zero we check the other tasks

    ldr r5,[r3,TASK_ENTRY_STATE]    // Sleep is over so we should make the task
    cmp r5,STATE_SENT_TO_SLEEP      // schedulable but we do not advance PC
    beq do_not_advance_pc           // if it was sent to sleep by other task

set_state_to_running:
    ldr r5,[r3,TASK_ENTRY_STACK]    // get the task's stack
    ldr r4,[r5,FRAME_PC+0x20]       // 0x20 with the r8-r11 registers overhead
    adds r4,2                       // skip the "b ." instruction to allow the
    str r4,[r5,FRAME_PC+0x20]       // task to continue execution
    dsb								// make sure the data is synced/written
    
do_not_advance_pc:
    movs r4,STATE_RUNNING           // set the state to "running"
    str r4,[r3,TASK_ENTRY_STATE]

advance_counter:
    subs r2,1                       // process the next task
    bpl process_sleeping_tasks      // after all task were processed we continue

//------------------------------------------------------------------------------
// Now let's see which task should be scheduled
// We only care about running ones or not_yet_scheduled ones
// For the later we don't have to restore r4-r11 since they were not yet saved
//------------------------------------------------------------------------------
find_next_task_init:
    movs r5,MAX_TASKS               // we check all tasks again
    movs r3,0                       // set the flag "not first time scheduled"
                                    // r0 = ram_start
                                    // r5 = 5
                                    // r6 = current task id
find_next_task:
    adds r6,1                       // go to the next task
    movs r2,MAX_TASKS-1             // r2 = 3
    ands r6,r2                      // r6 = next task id (modulo MAX_TASKS)

    lsls r2,r6,TASK_ENTRY_SHIFT_L
    adds r2,r0  
    adds r2,OFS_TASK_ARRAY          // r2 = next task address in array

    ldr r1,[r2,TASK_ENTRY_STATE]
    cmp r1, STATE_RUNNING           // if its current state is running we
    beq task_found                  // schedule it
    cmp r1, STATE_UNSCHEDULED       // if it's a brand new task we still
    beq it_is_an_unscheduled_task   // schedule it but we do not restore the
                                    // stack since it was never interrupted
                                    // by the scheduler
    subs r5,1                       // we check all tasks, including the current
    bne find_next_task              // one because it might be the only 
                                    // schedulable task in our list
schedule_idle_task:
    ldr r1,[r0,OFS_IDLE_NR_SCHED]   // r0 = ram start
    adds r1,1                       // increment the counter of schedules for
    str r1,[r0,OFS_IDLE_NR_SCHED]   // the idle task

    ldr r1,[r0,OFS_CURRENT_TASK]
    movs r2,1
    lsls r2,31
    orrs r1,r2                      // set bit 31 to indicate it's the idle task
    str r1,[r0,OFS_CURRENT_TASK]

    ldr r0,[r0,OFS_IDLE_STACK_POS]  // get the top of idle task stack
    ldr r1,=idle_task+1             // always start idle task from the begining
    str r1,[r0,FRAME_PC+0x20]       // set the PC in the stack saved context
    b restore_stack                 // schedule idle task

it_is_an_unscheduled_task:
    movs r3,1                       // set the flag "first time scheduled"
    movs r1,STATE_RUNNING           // set it's state to "running"
    str r1,[r2,TASK_ENTRY_STATE]
                                    // r2 = address on next task entry 
                                    // r6 = next task id
task_found:
                                    // GREAT NEWS EVERYBODY !
    movs r1,r6                      // We found a task to schedule
    str r1,[r0,OFS_CURRENT_TASK]    // store id in the current scheduled task id

    ldr r1,[r2,TASK_ENTRY_NR_SCHED] // increment the schedule counter 
    adds r1,1                       // this is just for statistics
    str r1,[r2,TASK_ENTRY_NR_SCHED]
    
    ldr r1,[r2,TASK_ENTRY_STACK]    // set the PSP stack
    msr PSP, r1

    orrs r3,r3                      // if it was scheduled for the first time
    bne scheduler_exit              // we do not restore r4-r11 registers

    mrs r0,PSP                      // get user mode stack

restore_stack:
    ldmia r0!,{r4-r7}               // load registers r4, r5, r6 and r7
    ldr r1,[r0]                     // use r1, r2, r3 as temporary regs
    ldr r2,[r0,0x04]                // since they will be restored from the 
    ldr r3,[r0,0x08]                // interrupt context from the stack
    mov r8,r1
    mov r9,r2
    mov r10,r3
    ldr r1,[r0,0x0c]
    mov r11,r1                      // registers r4-r11 were restored

    adds r0,0x10                    // the other 0x10 were added bt ldmia r0!
    msr PSP,r0                      // set PSP to the new level

scheduler_exit:
    isb                           // might be useful, kills instruction cache
    ret                             // go back to the scheduled task

//------------------------------------------------------------------------------
// Idle task will be scheduled in case there is no other available task
// in a schedulable state (eg. all of them are sleeping)
// It will try to concatenate two consecutive free memory blocks and then
// spin in an infinite loop.
// Might also be used to put the system to sleep for example until the next
// tick event (scheduler timer) comes along; saves energy
//------------------------------------------------------------------------------
idle_task:
    ldr r5,=FREE_MEMORY_START   // unscheduled while combining memory blocks
    ldr r4,=FREE_MEMORY_END     // Highly unlikely since it executes only once
    movs r0,0                   // Try to lock the memory mutex
    call mutex_try_lock         // if it's unsuccessful it will enter the
    orrs r0,r0                  // infinite loop
    bne enable_ints             // Mutex is used just in case a thread is 
                                // currently allocating memory and was unsched.
                                // while having the mutex; we should'nt combine
idle_loop:                      // Mutex was aquired so we are ok to process mem
    cpsid i                     // Disable interrupts. We don't want to be
    ldr r1,[r5]                 // get the first control word
    orrs r1,r1                  // set the flags
    bmi check_next_block        // if it's negative (vit 31 set) it's used
    beq idle_release_locks      // if it's 0 it was never used and there is
                                // nothing to combine
    adds r2,r1,4                // if it's positive we skip the control word
    adds r2,r5                  // add the base address
    cmp r2,r4                   // and check to see if it's higher than max
    bhs idle_release_locks      // If it is then we finished and we infinit-loop

    ldr r3,[r2]                 // Check to see if next block is free as well
    orrs r3,r3                  // If it's used we go back to treat it as the
    bmi check_next_block        // first block and check it against the next
    bne combine_zones           // if it's free as well we try to combine them
                                // If it's 0 then it's unallocated so we get
    subs r3,r4,r2               // the remaining size
    subs r3,4                   // the remaining size minus the control word

combine_zones:
    adds r1,r3
    adds r1,4
    str r1,[r5]                 // combine zones and store the result back

check_next_block:
    uxth r1,r1                  // sign extend the lower 16 bits into the word
    adds r1,4                   // skip the control word
    adds r5,r1                  // add the base address

    cmp r5,r4                   // if it's less than max we go and check it out
    blo idle_loop               // If it's more or equal we are done

idle_release_locks:
    movs r0,0                   // release the mutex
    call mutex_unlock

enable_ints:
    cpsie i                     // Enable interrupts
    b .                         // infinite loop
    //wfi					    // A much better option than infinite loop
    							// it will go to sleep and save energy
    							// It needs setup though so back to inf. loop

//==============================================================================
// This is the main entry point as mentioned in the second interrupt vector
// This code inits memory and stacks and systick and calls "init()" which
// should be present in the C code
//==============================================================================
_start:
    cpsid i                     // Disable interrupts
    ldr r0, =KERNEL_STACK_TOP   // set MSP
    msr MSP, r0                 // set MSP and PSP to the same stack for now
    msr PSP, r0
    mrs r0, CONTROL             // change thread mode stack to PSP
    movs r1, 2
    orrs r0, r1
    msr CONTROL, r0
    isb                         // sync instruction cache as mentioned in manual
    ldr r0, =0x20000000         // r0 = start of RAM and also kernel memory
    movs r1,0                   // we clear memory with 0
    movs r2,1
    lsls r2,ALL_MEMORY_SHIFT_L - 2  // we store one "word" (4 bytes) at a time
    call memset

    ldr r0, =SYSTICK_CTRL
    ldr r1,[r0,SYSTICK_LOAD]    // set reload value
    ldr r1,=0xBB80              // 48000 = 1ms based on the fact that PLL will 
    //ldr r1,=0x75300              // 480000 = 10ms based on the fact that PLL will 
    str r1,[r0,SYSTICK_LOAD]    // be activated in init()
    ldr r1,[r0,SYSTICK_VAL]     // clear current value
    movs r1, 0
    str r1,[r0,SYSTICK_VAL]

    // we can safely skip the calibration since these micros come pre-calibrated
    // If it's needed, we can just uncomment
    //ldr r1,[r0,SYSTICK_CALIB]     // calibration
    //ldr r1, =0x80001770
    //str r1,[r0,SYSTICK_CALIB]

    movs r1, 7                  // using reference clock
    str r1,[r0]                 // enable systick counting

    call init_idle_task

    call init_task_area

    cpsie i                     // Enable interrupts

//-------------------------------------- finished the initialization -----------

    call init                   // Calls init() which is currently defined as
                                // weak reference and points to the loop below

init:
                                // This is a weak reference to make sure the os 
                                // compiles without C; it's here just allow me 
                                // to check M0S's raw size 
infinite_loop:
        b infinite_loop         // Useless infinite loop waiting for tasks to
                                // be scheduled

//------------------------------------------------------------------------------
// It sends a word (1 ... 2^32-1) to another task through IPC
// 0 is forbidden since it means no value is currently available
// Input:   r0 = task id
//          r1 = word to send
//------------------------------------------------------------------------------
ipc_send:
    ldr r2,=0x20000000
    lsls r0,TASK_ENTRY_SHIFT_L
    adds r0,OFS_TASK_ARRAY
    adds r0,r2                  // r0 points to the task entry in the array
    str r1,[r0,TASK_ENTRY_IPC]  // Store the value
    ret

//------------------------------------------------------------------------------
// Reads current IPC value
//------------------------------------------------------------------------------
ipc_read:
    push {lr}
    ldr r1,=0x20000000
    call get_current_task_id
    lsls r0,TASK_ENTRY_SHIFT_L
    adds r0,OFS_TASK_ARRAY
    adds r0,r1
    ldr r0,[r0,TASK_ENTRY_IPC]
    pop {pc}

//------------------------------------------------------------------------------
// Arranges the stack for the first schedule of the idle task by simulating that 
// it was already scheduled at least once
//------------------------------------------------------------------------------
init_idle_task:
    ldr r0,=IDLE_TASK_STACK_TOP
    movs r1,0xC1
    rev r1,r1                   // little endian to big endian
    subs r0,8                   // to easily access last two values
    str r1,[r0,4]               // set XPSR
    ldr r1,=idle_task+1
    subs r2,4
    str r1,[r0]                 // set PC
    subs r0,64-8                // to simulate a return from interrupt (32 bytes
    ldr r1,=0x20000000          // for r4-r11 and 32 for usual layout)
    str r0,[r1,OFS_IDLE_STACK_POS]  // set new position to allow a schedule the first time
    ret

//------------------------------------------------------------------------------
// Initializes the task array area with default values and correct stack 
// pointers for all. Default PC is set to init
// *WARNING* NO TASK WILL BE SCHEDULED BY THIS ROUTINE
//------------------------------------------------------------------------------
init_task_area:
    push {r0-r4,lr}             // Save lr along since we already save some regs
    ldr r0,=0x20000000
    adds r0,OFS_TASK_ARRAY
    movs r1,MAX_TASKS-1         // Start from the last task

init_loop:
    lsls r2,r1, TASK_ENTRY_SHIFT_L
    adds r2,r0                  // r2 = address of the task entry

    movs r3,STATE_INACTIVE      // Sets its status to "inactive"
    str r3,[r2,TASK_ENTRY_STATE]
    ldr r3, =infinite_loop+1    // bogus PC for all tasks
    str r3,[r2,TASK_ENTRY_PC]
    lsls r3,r1, TASK_STACK_SHIFT_L  // the size of the stack for one task
    ldr r4,=0x20004000          // top of memory
    subs r4,r3                  // calculate the stack address for this task
    subs r4,0x20                // stack frame automatically saved by systick
    str r4,[r2,TASK_ENTRY_STACK]

    subs r1,1                   // decrement current task id
    bpl init_loop               // and loop back if value >= 0
    pop {r0-r4,pc}              // restore registers and go back

//------------------------------------------------------------------------------
// Creates a task
// Input:   r0 = address of task
// Returns task id or -1 if error
//------------------------------------------------------------------------------
create_task:
    push {r4,lr}
    cpsid i                         // disable interrupts (CRITICAL SECTION !)
    ldr r1, =0x20000000
    movs r2, MAX_TASKS-1            // start with the last task

create_loop:
    lsls r3, r2, TASK_ENTRY_SHIFT_L // r3 = offset of the task in array
    adds r3, r1
    adds r3, OFS_TASK_ARRAY         // r3 = address of the task entry

    ldr r4, [r3,TASK_ENTRY_STATE]
    cmp r4, STATE_INACTIVE          // if the current state is not "inactive"
    bne create_next                 // then try the next task
                                    // we found a free task, let's populate

    adds r0, 1                      // make it a thumb address :)
    str r0, [r3,TASK_ENTRY_PC]      // set PC in the task entry
    ldr r1, [r3,TASK_ENTRY_STACK]   // get the stack
    str r0, [r1,FRAME_PC]           // set PC on the stack frame

    // This part here is optional, I only need PC and XPSR set
    // I can omit this part and leave registers uninitialized
    //movs r0,0                     // all init regs are 0
    //mvns r0,r0

    //str r2,[r1,FRAME_R0]          // R0 = task id
    //str r0,[r1,FRAME_R1]          // R1
    //str r0,[r1,FRAME_R2]          // R2
    //str r0,[r1,FRAME_R3]          // R3
    //str r0,[r1,FRAME_R12]         // R12
    //str r0,[r1,FRAME_LR]          // LR
    movs r0,0xC1                    // A default value which works for XPSR
    rev r0,r0                       // 0xC1000000 (le <-> be)
    str r0,[r1,FRAME_XPSR]          // set XPSR

    movs r0,STATE_UNSCHEDULED
    str r0,[r3,TASK_ENTRY_STATE]    // set state to "not yet scheduled"

    ldr r1,=0x20000000
    ldr r4, [r1,OFS_NUM_TASKS]      // increment the number of tasks
    adds r4,1
    str r4, [r1,OFS_NUM_TASKS]

    movs r0,r2                      // return the task id (0..3)
    b create_exit

create_next:
    subs r2,1                       // go to the previous task entry
    bpl create_loop

    movs r0,0
    mvns r0,r0                      // return -1 since all tasks are used

create_exit:
    cpsie i                         // re-enable interrupts and go back
    pop {r4,pc}

//------------------------------------------------------------------------------
// Puts a task identified by its task id to sleep for specified number of ticks
// If the task is already sleeping, we overwrite the ms counter with the new val
// Input:   r0 = task id
//          r1 = how many miliseconds to wait
//------------------------------------------------------------------------------
sleep_task:
    cpsid i                         // CRITICAL SECTION !!!
    
    ldr r2,=0x20000000
    lsls r0,TASK_ENTRY_SHIFT_L
    adds r2,OFS_TASK_ARRAY
    adds r2,r0                      // r2 points to the task entry in array

    str r1,[r2,TASK_ENTRY_TARGET]   // store the target number of ticks
    ldr r1,[r2,TASK_ENTRY_STATE]    // get current state
    cmp r1,STATE_SLEEPING           // if it's already sleeping we do nothing
    beq sleeping

    movs r1,STATE_SENT_TO_SLEEP
    str r1,[r2,TASK_ENTRY_STATE]    // set sleep mode on

sleeping:
    cpsie i
    ret

//------------------------------------------------------------------------------
// Put current task to sleep for specified number of ticks
// Input:   r0 = how many miliseconds to wait
//------------------------------------------------------------------------------
sleep:
    ldr r2,=0x20000000
    movs r1,r0
    ldr r0,[r2,OFS_CURRENT_TASK]    // get current task id

    lsls r0,TASK_ENTRY_SHIFT_L
    adds r2,OFS_TASK_ARRAY
    adds r2,r0                      // r2 points to the task entry in array

    str r1,[r2,TASK_ENTRY_TARGET]   // store the target ticks value
    movs r1,STATE_SLEEPING
    str r1,[r2,TASK_ENTRY_STATE]    // set sleep mode on

    b .                             // PC will be adjusted over this instruction
                                    // in scheduler when time will be up
    ret

//------------------------------------------------------------------------------
// Finds out a task id using the address of it's subroutine
// Input:   r0 = start subroutine
// Return:  task id or -1 if error
// WARNING ! It returns the first id it finds which matches the address.
//           If you created two tasks with the same address then... good luck :)
//------------------------------------------------------------------------------
get_task_id:
    adds r0,1
    movs r1,MAX_TASKS-1             // start from the last task

get_task_loop:
    ldr r3,=0x20000000
    lsls r2,r1,TASK_ENTRY_SHIFT_L
    adds r2,OFS_TASK_ARRAY
    adds r2,r3                      // r2 = address of the task in array
    
    ldr r3,[r2,TASK_ENTRY_PC]
    cmp r0,r3                       // we compare the supplied value with the
    beq found_it                    // PC value stored in the TASK STRUCT

    subs r1,1                       // loop through all tasks
    bpl get_task_loop

found_it:
    movs r0,r1                      // return task id or -1
    ret

//------------------------------------------------------------------------------
// Returns number of running tasks
//------------------------------------------------------------------------------
get_number_of_tasks:
    ldr r0,=0x20000000
    ldr r0,[r0,OFS_NUM_TASKS]
    ret

//------------------------------------------------------------------------------
// Returns current task id
//------------------------------------------------------------------------------
get_current_task_id:
    ldr r0,=0x20000000
    ldr r0,[r0,OFS_CURRENT_TASK]
    ret

//------------------------------------------------------------------------------
// Returns system timer (jiffie counter or ticks counter)
//------------------------------------------------------------------------------
get_system_timer:
    ldr r0,=0x20000000
    ldr r0,[r0,OFS_SYS_TIMER]
    ret

//------------------------------------------------------------------------------
// Sets system timer (jiffie counter) to a specified value
// Input: r0 = value
// Output: old timer value
//------------------------------------------------------------------------------
set_system_timer:
    ldr r1,=0x20000000
    ldr r2,[r1,OFS_SYS_TIMER]
    str r0,[r1,OFS_SYS_TIMER]
    movs r0,r2
    ret

//------------------------------------------------------------------------------
// Removes a task from the scheduling array
// Input:   r0 = thread id
//------------------------------------------------------------------------------
kill:
    ldr r1,=0x20000000
    lsls r2,r0,TASK_ENTRY_SHIFT_L
    adds r2,OFS_TASK_ARRAY
    adds r2,r1                      // now r2 points to the thread array

    cpsid i                         // disable interrupts; CRITICAL SECTION
    movs r3,STATE_INACTIVE
    str r3,[r2,TASK_ENTRY_STATE]    // set state to inactive

    lsls r3, r0, TASK_STACK_SHIFT_L
    ldr r4,=0x20004000              // TOP of memory
    subs r4,r3
    subs r4, 0x20                   // task frame for returning to stack
    str r4, [r2,TASK_ENTRY_STACK]   // set default stack

    ldr r3, =infinite_loop+1        // first task will be used as fake starting point for all tasks
    str r3, [r2,TASK_ENTRY_PC]

    ldr r3,[r1,OFS_NUM_TASKS]
    subs r3,1                       // decrement the number of active tasks
    bpl keep_going                  // if the value is negative we start crying
                                    // WORRY! PANIC! TANTRUM! CRY! BLAME OTHERS!
                                    // Point of no return, there is no recovery
    b .                             // THE END.

keep_going:
    str r3,[r1,OFS_NUM_TASKS]       // set the new number of tasks
    cpsie i

    ret

//------------------------------------------------------------------------------
// It will be called by a thread to stop itself from executing
//------------------------------------------------------------------------------
exit:
    call get_current_task_id
    call kill
    b .                             // Oh wait, there is another end right here

//------------------------------------------------------------------------------
// It fills the buffer with passed value
// Input:   r0 = address of buffer
//          r1 = value to use for filling
//          r2 = size of buffer in words
//------------------------------------------------------------------------------
memset:
    stmia r0!,{r1}
    subs r2,1
    bne memset
    ret

//------------------------------------------------------------------------------
// Allocates memory
// Input:   r0 = size in bytes (max 65532 or 0xFFFC)
//               16 bits - 4 because we allocate word aligned memory)
// Returns the address of the newly allocated memory or 0 if error
// WARNING! There is no input validation so be a good sport and pass good values
//------------------------------------------------------------------------------
malloc:
    push {r4-r5,lr}                 // lr has to be saved because of calls
    movs r4,r0                      // save parameter to make sure it will
    movs r0,0                       // not get trashed in the next call
    call mutex_lock                 // lock mutex 0

    movs r0,r4                      // restore the parameter
    movs r1,3                       // make sure all memory is word aligned
    adds r0,3                       // If the addresses are not aligned
    bics r0,r1                      // the code will trigger an exception
    ldr r1,=FREE_MEMORY_START
    ldr r3,=FREE_MEMORY_END

check_block:
    ldr r2,[r1]                     // get the control word which should be
    orrs r2,r2                      // present before all allocated blocks
    bmi next_block                  // block is in use, check next one
    bne found_a_block               // found a free block

it_was_never_allocated:
    // if it's 0 it was never allocated, set block to max_size - control_word
    ldr r2,=FREE_MEMORY_END - FREE_MEMORY_START-4       

found_a_block:
    cmp r0,r2                       // Can it fit the requested size ?
    bhi next_block                  // If it can't fit we skip it
                                    // If it can the we use it
    subs r5,r2,r0                   // calculate the remaining size
    movs r4,4                       // has to be higher than the size of the
    cmp r5,r4                       // control block.
    bls next_block                  // if it's <= we skip it

    subs r5,r4                      // r5 holds the actual remaining memory
    movs r4,0x80
    rev r4,r4                       // 0x80 becomes 0x80000000 (LE <-> BE)
    orrs r0,r4                      // set the bit 31 in size to indicate
    str r0,[r1]                     // an used zone and store it
    adds r1,4                       // skip the control word; this is the value we want to return

    uxth r0,r0                      // clear its 31st bit to be able to add it in the next instructions
    adds r0,r1                      // go to the start of the remaining memory from this block
    str r5,[r0]                     // store the remaining size

    b exit_malloc

next_block:
    uxth r2,r2                      // clear bit 31
    adds r1,4                       // skip the control word
    adds r1,r2                      // go to next block
    cmp r1,r3                       // have we reached the end ?
    blo check_block

exit_with_null_pointer:
    movs r1,0                       // return null pointer if errors

exit_malloc:
    movs r4,r1                      // save return value
    movs r0,0
    call mutex_unlock

    movs r0,r4                      // restore return value             
    pop {r4-r5,pc}
    ret

//------------------------------------------------------------------------------
// It unlocks a mutex; It doesn't check if the current task actually
// holds the mutex since there is no mechanism in place for this :)
// Input:   r0 = mutex id
//------------------------------------------------------------------------------
mutex_unlock:
	ldr r1,=0x20000000
	movs r3,1
	lsls r3,r0
	mvns r3,r3
	
	cpsid i
	
	ldr r2,[r1,OFS_MUTEX_STORAGE]
	ands r2,r3
	str r2,[r1,OFS_MUTEX_STORAGE]

	cpsie i

	ret

//------------------------------------------------------------------------------
// Tries to lock a mutex. If it fails it returns without blocking
// Input:   r0 = mutex id (0-7)
// Return:  r0 = 0 if mutex was locked and 1 if it could not be locked
// WARNING! No validation on input parameter. Make sure you sanitize
//------------------------------------------------------------------------------
mutex_try_lock:
	ldr r1,=0x20000000
	movs r3,1
	lsls r3,r0
	
	cpsid i
	
	ldr r2,[r1,OFS_MUTEX_STORAGE]
	movs r0,r2
	ands r0,r3
	beq mutex_is_free
	
	cpsie i

	movs r0,1						// return fail
	ret

//------------------------------------------------------------------------------
// Tries to lock a mutex. If it fails it waits for the mutex release
// This function blocks and only returns if the mutex was locked
// Input:   r0 = mutex id (0-6) because we use mutex 7 for memory allocations
// No return is needed since it blocks if lock was unsuccessful
//------------------------------------------------------------------------------
mutex_lock:
	ldr r1,=0x20000000
	movs r3,1
	lsls r3,r0
	
	cpsid i
	
	ldr r2,[r1,OFS_MUTEX_STORAGE]
	ands r2,r3
	beq mutex_is_free

	cpsie i

    ldr r2,[r1]                     // get current task id
    lsls r2,TASK_ENTRY_SHIFT_L
    adds r2,OFS_TASK_ARRAY
    adds r2,r1                      // r2 points to the current task structure

    str r0,[r2,TASK_ENTRY_MUTEX]    // store the mutex in task structure
    movs r0,STATE_WAIT_FOR_MUTEX    // in the mutex reserved area
    str r0,[r2,TASK_ENTRY_STATE]    // set the state to "wait for mutex"
    
    b .

    ret


mutex_is_free:
	ldr r2,[r1,OFS_MUTEX_STORAGE]
	orrs r2,r3
	str r2,[r1,OFS_MUTEX_STORAGE]

	cpsie i
	movs r0,0						// return success
	ret


//------------------------------------------------------------------------------
// Frees allocated memory
// Input:   r0 = address of the allocated memory
// Returns 0 on error
// There is no need for mutexes here since the operation is kind of atomic
//------------------------------------------------------------------------------
free:
    subs r0,4                       // reach the control word
    ldr r1,=FREE_MEMORY_START
    cmp r0,r1                       // is its address less than the start
    blo error_freeing               // of memory ? if so, jump

    ldr r1,[r0]
    uxth r1,r1                      // reset bit 31 to make this an unused
    str r1,[r0]                     // memory block
    ret

error_freeing:
    movs r0,0
    ret

//---------------------------------------- END OF CODE ------------------------
// The assembler should put the literal constants in this section after .pool
.pool

//-----------------------------------------------------------------------------
// This string is here to fill in the remaining space till 1024 bytes
// Comes after the .pool section so it's the last available thing in .bin
// Update: string doesn't fit anymore, I updated the mutexes to work better
//.ascii "M0S-1K HaD Challenge"

//------------------------------------------------------------------------------
// I define this here as weak because most likely I will define it again in the C code
.weak init

//------------------------------------------------------------------------------
// Here are the exported functions
.global _start
.global sleep
.global create_task
.global mutex_lock
.global mutex_try_lock
.global mutex_unlock
.global memset
.global exit
.global kill
.global set_system_timer
.global get_system_timer
.global get_current_task_id
.global get_task_id
.global get_number_of_tasks
.global sleep
.global sleep_task
.global malloc
.global free
.global ipc_send
.global ipc_read

