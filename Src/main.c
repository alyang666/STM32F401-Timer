/**
 ******************************************************************************
 * @file           : main.c
 * @author         : NL
 * @brief          : Main program body
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "main.h"
#include "usart.h"
#include "timer.h" 
#include "system.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define STACK_SIZE 1024	//stack size in word (4ko)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
char stdin_buffer[20];
char stdout_buffer[20];
//static memroy allocation for TCB
struct TCB_t TCB[3];	// space for 3 TCB
//memory allocation for T0 stack 8 bytes aligned
uint32_t stack0[STACK_SIZE]__attribute__ ((aligned (8)));
uint32_t stack1[STACK_SIZE] __attribute__ ((aligned (8)));
uint32_t stack2[STACK_SIZE] __attribute__ ((aligned (8)));;
/* Private function prototypes -----------------------------------------------*/
void T0(int a, int b, int c, int d); //task prototype with parameters
// task prototype no parameters and non return value
void T1(void);
void T2(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  small waiting functions (time is really approx.)
  * @param  None
  * @retval None
  */
static void  __attribute__((optimize("O1"))) simple_wait_ms(uint32_t ms){
	ms =(SystemCoreClock/1000/4)*ms;
	while(ms--)
		asm("");	//prevent from optimization out
}

void task_yield(void);
void enter_critical_section(void);
void exit_critical_section(void);
/**
  * @brief  main application
  * @param  None
  * @retval should not return
  */
int main(void)
{

	/* for debug purpose active all fault exceptions with priority 5 */
	NVIC_SetPriority(MemoryManagement_IRQn, 5);
	NVIC_SetPriority(BusFault_IRQn, 5);
	NVIC_SetPriority(UsageFault_IRQn, 5);
	SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk; //Set bit 16
	SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA_Msk; //Set bit 17
	SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk; //Set bit 18


	/* Peripherals initialisation if needed */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;

	/******************* PINS PA2 and PA3 for USART2 **************************/
	/* PA2 et 3 in alternate function N°7 */
	GPIOA->AFR[0] &= ~(0xF << (2*4) );	/* clear the 4 bits */
	GPIOA->AFR[0] |= (7 << (2*4) ); 	/* set alternate function Nbr 7*/
	/* RX on PA3 alternate function 7 */
	GPIOA->AFR[0] &= ~(0xF << (3*4) );	/* clear the 4 bits */
	GPIOA->AFR[0] |= (7 << (3*4) );		/* set alternate function Nbr 7*/
	/* Configure alternate function for UART2 RX (PIN3) and TX (PIN2) */
	GPIOA->MODER &= ~(3 << (2 * 2) );	/*TX*/
	GPIOA->MODER &= ~(3 << (3 * 2) );	/*RX*/
	GPIOA->MODER |= (2 << (2 * 2) );	/*TX*/
	GPIOA->MODER |= (2 << (3 * 2) );	/*RX*/
	/************************** PINS PA2 and PA3 ******************************/
	USART2_Init(115200);

	USART2_Transmit("Start of application\n", 22);

	/* init TIMER 5 */
	TIM5_set_periodic_event(500);//500ms


	/************ Task configuration and initialization ***********/
	/* initial value of stack pointer for the T0 stack
	 * The stack is full descending so the initial pointer is
	 * given by stack0 + sizeof(stack0)
	 * if the stack is initialized with a task context a full
	 * stack frame must be initialized and the stack pointer
	 * will point 18 saved registers below the initial stack pointer address
	 * r0->r12 + r14 + return address (r15) + xpsr + control + EXC_RETURN
	 * note that the sp must be 8 bytes aligned
	 */
	TCB[0].stack =  stack0  + STACK_SIZE - (2+16);
	TCB[1].stack =  stack1  + STACK_SIZE - (2+16);
	TCB[2].stack =  stack2  + STACK_SIZE - (2+16);

	// initial value for the stacks frame (FPU not in use at startup of task)
/* EXEC_RETURN[31:5] = 1;
   EXEC_RETURN[4]= 1:8 words stack frame, 0 : 26 words stack frame (FPU)
   EXEC_RETURN[3]= 1 : return to Thread mode, 0 return to Handler mode
   EXEC_RETURN[2]= 1 : return with PSP (only Thread mode), 0 : return with MSP
   EXEC_RETURN[1]= 0 (reserved)
   EXEC_RETURN[0]= 1 (reserved)
*/

/*  CONTROL[2] (FPCA) = 0 : FP register don't need saving, 1 : FP need saving
	CONTROL[1] (SPEL) = 1 : PSP, 0 MSP
	CONTROL[0] (nPriv) = 0 : privileged, 1 : unprivileged
*/
	/* Task 0 */
	TCB[0].stack[0] = 0xFFFFFFFDUL;	// initial EXC_RETURN
	TCB[0].stack[1] = 0x3;	// initial CONTROL : unprivileged, PSP, no FPU
	TCB[0].stack[16] = (uint32_t) T0;	// initial pc
	TCB[0].stack[17] = 0x01000000; // initial xPSR (thumb mode T=1)
	/* parameters of task */
	TCB[0].stack[10] = 0; // first parameter (in r0)
	TCB[0].stack[11] = 1; // second parameter(in r1)
	TCB[0].stack[12] = 2; // third parameter (in r2)
	TCB[0].stack[13] = 3; // fourth parameter (in r3)

	/* Task 1 */
	TCB[1].stack[0] = 0xFFFFFFFDUL;;	// initial EXC_RETURN
	TCB[1].stack[1] = 0x3;	// initial CONTROL : unprivileged, PSP, no FPU
	TCB[1].stack[16] = (uint32_t) T1;	// initial pc
	TCB[1].stack[17] = 0x01000000; // initial xPSR (thumb mode T=1)

	/* Task2 */
	TCB[2].stack[0] = 0xFFFFFFFDUL;;	// initial EXC_RETURN
	TCB[2].stack[1] = 0x3;	// initial CONTROL : unprivileged, PSP, no FPU
	TCB[2].stack[16] = (uint32_t) T2;	// initial pc
	TCB[2].stack[17] = 0x01000000; // initial xPSR (thumb mode T=1)

	// link TCB and set current_tcb to TCB[0]
	TCB[0].next = &TCB[1];
	TCB[1].next = &TCB[2];
	TCB[2].next = &TCB[0];

	current_tcb = &TCB[0];	// task0 TCB

	/************ system interrupt configuration ***********/
	/* pendSV is used to switch tasks when no other interrupts are pending
	   or active. PendSV interrupt priority at the lowest priority */
	NVIC_SetPriority ( PendSV_IRQn, 15); /* lowest */
	NVIC_EnableIRQ ( PendSV_IRQn);
	/* active SysTick interrupt (every 10ms)*/
	NVIC_SetPriority ( SysTick_IRQn, 15); /* system interrupt 14 */
	SysTick_Config(SystemCoreClock / 1000 ); // quantum of 1ms/task
	/*Note that for the SVC exception, if the SVC instruction is accidentally
	 * used in an exception handler that has the same or higher priority than
	 * the SVC exception itself, it will cause the HardFault exception handler
	 * to execute.*/
	NVIC_SetPriority(SVCall_IRQn,12);
	NVIC_EnableIRQ ( SVCall_IRQn);
	/* start Scheduler and task : system service N°0 */
	SVC(0);

#if 0
	/* An other method to start the first task :
	 * switch the currens stack (MSP) to PSP and the main() becomes the first
	 * task to run, PSP is initialized with the top of the stack
	   note : TCB[0] initial value won't be used  */
	__set_PSP(__get_MSP());	// PSP = MSP, better before switching stack
	__set_CONTROL(0x2); // Switch to use Process Stack, privileged
	__ISB();// Execute ISB after changing CONTROL (architectural recommendation)
	__NOP();
	__set_PSP(stack0  + STACK_SIZE);
	//__set_PSP(TCB[0].stack[18]); // top of the stack 0
	/* The stack has switched the following code must not use local variable declared in main*/
	/* start scheduler and go to  task 0 main loop */
	schedule=1; // unlock the task switching
	/* go to task 0 */
	Task0();
#endif


	for(;;){
		/* should never go there */
		__NOP();
	}
    return 1;
}



/**
  * @brief  Task T0, blink the led
  * @param  None
  * @retval should not return
  */
void T0(int a, int b, int c, int d){
	int i, attente, len;
	GPIO_TypeDef  *PA = GPIOA, *PC = GPIOC;
	char str[100];

	USART2_Transmit("Start of Task 0\n", 17);
	// Affichage des paramètre de la tâches, on passe par la fonction
	// de converstion de la stdlib sprintf pour les afficher en ascii
	len = snprintf(str,100,"first param : %d, second parm : %d, "
			"third param : %d, fourth parm : %d \n", a, b, c, d);
	// send it
	USART2_Transmit(str, len);

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;

	/* PA5 : sortie, vitesse lente, push-pull, no pull
	 * la pin PC13 est par défaut configurée en entrée
	 */
	PA->MODER &= ~GPIO_MODER_MODER5_Msk;
	PA->MODER |= GPIO_MODER_MODER5_0;

	/* application : boucle infinie, l'appli ne doit pas revenir*/
	for(;;){

		/* test de la pin PC13 pour définir les durées de maintien
		 * Le bouton génère un 1 lorsque relaché, et un 0 lorsque appuyé
		 */
		if (PC->IDR & GPIO_IDR_ID13){
			/* bouton relache */
			attente = 300;
		}else{
			/* boutton appuye */
			attente = 100;	// environ 100ms de période
		}
		/* clignotement */
		// led on
		PA->BSRR = GPIO_BSRR_BS5; 	//Write only
		/* boucle d'attente pour maintenir la led allumée */
		simple_wait_ms(attente);
		// led off
		PA->BSRR = GPIO_BSRR_BR5;	//Write only
		simple_wait_ms(attente);
		task_yield();
	}
}

/**
  * @brief  Task T1, send data on the serial line 2
  * @param  None
  * @retval should not return
  */
void T1()
{
	//USART2_Transmit("Start of Task 1 \n", 17);
	for(;;){
		enter_critical_section();
		USART2_Transmit("Hello from Task T1 \n", 20);
		exit_critical_section();
		simple_wait_ms(500);
		task_yield();
	}

}

/**
  * @brief  Task T2, send data on the serial line 2
  * @param  None
  * @retval should not return
  */
void T2()
{
	//USART2_Transmit("Start of Task 2 \n", 17);
	for(;;){
		enter_critical_section();
		USART2_Transmit("Hello from Task T2 \n", 20);
		exit_critical_section();
		//TIM5_wait_for_periodic_event();
		simple_wait_ms(300);
		task_yield();
	}
	// no return
}

/**
  * @brief  System call N°3 : TaskYield, immediat yield of the processor
  * 		by the calling task
  * @param  None
  * @retval none
  */
void task_yield(void)
{
	SVC(3);
}

/**
  * @brief  System call N°1 : Enter critical section, the scheduler is
  * 		suspended
  * @param  None
  * @retval none
  */
void enter_critical_section(void)
{
	SVC(1);
}

/**
  * @brief  System call N°2 : Enter critical section, the scheduler is
  * 		restarted if none nested critical section
  * @param  None
  * @retval none
  */
void exit_critical_section(void)
{
	SVC(2);
}
