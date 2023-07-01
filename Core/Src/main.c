extern void runFirstThread(void);
#include "kernel.h"
#include "main.h"
#include<stdio.h>
UART_HandleTypeDef huart2;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void thread1(void);
static void thread2(void);
static uint32_t* stackptr;
static uint32_t z = (uint32_t)'z';
static uint32_t* MSP_INIT_VAL;

#define MAX_THREADS 10
#define STACK_SIZE 1024
#define SHPR2 *(uint32_t*)0xE000ED1C //for setting SVC priority, bits 31-24
#define SHPR3 *(uint32_t*)0xE000ED20 // PendSV is bits 23-16
#define _ICSR *(uint32_t*)0xE000ED04

uint32_t numThreadsRunning;
uint32_t currentThread;

typedef struct k_thread{
	uint32_t* sp; //stack pointer
	void (*thread_function)(void*); //function pointers
}thread;

struct k_thread threadArray[MAX_THREADS];

void osKernelInitialize(void) {
    // Initialize global variables here
    // Example:
	numThreadsRunning = 0;
    currentThread = -1;
    SHPR3 |= 0xFE << 16; //shift the constant 0xFE 16 bits to set PendSV priority
    SHPR2 |= 0xFDU << 24;
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    // You can also initialize the elements of threadsArray if needed
    MSP_INIT_VAL = *(uint32_t**)0x0;
    stackptr = (uint32_t*)((uint32_t)MSP_INIT_VAL - 0x200);

}

int osCreateThread(void (*threadFunction)(void)) {
    // Check if the maximum number of threads has been reached
    if (numThreadsRunning >= MAX_THREADS) {
        return 1;
    }

    // Allocate a new stack for the thread
    if (stackptr == NULL) {
        return 1;
    }

    stackptr = (uint32_t*)((uint32_t)stackptr - 0x200);

    *(--stackptr) = 1<<24; //A magic number, this is xPSR
    *(--stackptr) = (uint32_t)threadFunction; //the function name
    *(--stackptr) = (uint32_t)'a'; //An arbitrary number
    *(--stackptr) = (uint32_t)'b'; //An arbitrary number
    *(--stackptr) = (uint32_t)'c'; //An arbitrary number
    *(--stackptr) = (uint32_t)'d'; //An arbitrary number
    *(--stackptr) = (uint32_t)'e'; //An arbitrary number
    *(--stackptr) = (uint32_t)&z; //An arbitrary character
    *(--stackptr) = (uint32_t)'f'; //An arbitrary number
    *(--stackptr) = (uint32_t)'g'; //An arbitrary number
    *(--stackptr) = (uint32_t)'h'; //An arbitrary number
    *(--stackptr) = (uint32_t)'i'; //An arbitrary number
    *(--stackptr) = (uint32_t)'j'; //An arbitrary number
    *(--stackptr) = (uint32_t)'k'; //An arbitrary number
    *(--stackptr) = (uint32_t)'l'; //An arbitrary number
    *(--stackptr) = (uint32_t)'m'; //An arbitrary number

    // Update the kernel data structures to keep track of the new thread
    threadArray[numThreadsRunning].sp = stackptr;

    // Increment the thread count
    numThreadsRunning++;
    currentThread++;
	__set_PSP((uint32_t)stackptr);

    return 0;
}

void osKernelStart(void){
	__asm("SVC #3");
}

void osSched(void) {
    threadArray[currentThread].sp = (uint32_t*)(__get_PSP() - 8 * 4);
    currentThread = (currentThread + 1) % numThreadsRunning;
    __set_PSP((uint32_t)threadArray[currentThread].sp);
}

void osYield(void){
	__asm("SVC #9");
}

int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,HAL_MAX_DELAY);
	return ch;
}

int main(void)
{
  osKernelInitialize();
  osCreateThread(thread1);
  osCreateThread(thread2);
  osKernelStart();
  while (1)
  {

  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

void jumpAssembly(void* fcn)
{
	__asm("MOV PC, R0");
}

void print_I(void)
{
	__asm("SVC #2");
}

void print_next(void)
{
	__asm("SVC #6");
}

void thread1(void){
	while(1) {
		printf("Thread 1\r\n");
		osYield();
	}
}

void thread2(void){
	while(1) {
		printf("Thread 2\r\n");
		osYield();
	}
}
void SVC_Handler_Main( unsigned int *svc_args )
{
	unsigned int svc_number;
/*
* Stack contains:
* r0, r1, r2, r3, r12, r14, the return address and xPSR
* First argument (r0) is svc_args[0]
*/
	svc_number = ( ( char * )svc_args[ 6 ] )[ -2 ] ;
	void* z = (void *)svc_args[0];

	switch( svc_number )
	{
		case 2:
			printf("I ");
			break;
		case 3:
			runFirstThread();
			break;
		case 6:
			printf("will pass the course!\r\n");
			break;
		case 7:
			__set_PSP((uint32_t)stackptr);
			while(1){
				printf("%c",*(char*)z);
			}
			break;
		case 9:
			_ICSR |= 1<<28;
			__asm("isb");
			break;
		case 17:  //17 is sort of arbitrarily chosen
			printf("Success!\r\n");
			break;
		default:    /* unknown SVC */
			break;
	}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */


























