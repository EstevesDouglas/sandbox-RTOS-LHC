/**
 * @brief LHC RTOS main application file
 * http://mcuxpresso.nxp.com/apidoc/2.0/group__debug__console.html
 */

/*
 * FIXE mudar a de lugar a declaracao da variavel queueFree no arquivo OsQueue.c
 */
#include "board.h"
#include "clock_config.h"
#include "pin_mux.h"
#include "fsl_tsi_v4.h"
#include "fsl_tpm.h"
#include "uLipeRtos4.h"
#include <stdint.h>
#include "gpio/hal_gpio.h"
#include "leds/leds.h"
#include "lcd/lcd.h"
#include "delay/hal_delay.h"
#include "xprintf/xprintf.h"
#include "fsl_debug_console.h"
#include "fsl_lpsci.h"
#include <stdbool.h>
#include <stdio.h>

void swiec(void);
void opz(int ile);


typedef struct
{
	uint8_t *msg;
	uint16_t size;
	uint8_t x;
	uint8_t y;
}msgLCD_t;

/* reserve space for stack used on threads in words */
/*uint32_t led_stk[192];
uint32_t touch_stk[192];

uint32_t dly_stk[64];
uint32_t teste_stk[64];

uint32_t teste_stk2[64];
uint32_t teste_stk3[64];

uint32_t Leds_stk[64];
uint32_t LCD_stk[128];*/


/* various kernel objects used in this demo */
OsHandler_t sema;
OsHandler_t que;
OsHandler_t queLCD;
OsHandler_t flags;
OsHandler_t mtx;

#define TPM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_PllFllSelClk)


OsTCB_t test_task;

Sem_t test_sema;
FlagsGrp_t test_flags;
Queue_t test_msg;
Mutex_t test_mtx;


volatile uint16_t xcnt1 = 0;
volatile uint16_t xcnt3 = 0;

volatile uint16_t xcnt4 = 0;
volatile uint16_t xcnt5 = 0;


volatile uint16_t errSupend = 0;
volatile uint16_t errResume = 0;

void setMsgLCD(msgLCD_t *msg, uint8_t *buff, uint16_t size, uint8_t x, uint8_t y);

void setMsgLCD(msgLCD_t *msg, uint8_t *buff, uint16_t size, uint8_t x, uint8_t y)
{
	msg->msg = buff;
	msg->size = size;
	msg->x = x;
	msg->y = y;
}


static void teste_task2(void *args)
{
	volatile uint16_t *tsi_data;

	msgLCD_t *xMsg;
	for(;;)
	{
		uLipeTaskDelay(1000);
	//	tsi_data = ((uint16_t *)uLipeQueueRemove(que,OS_Q_BLOCK_EMPTY,0,NULL));
		xcnt5++;
	//	uLipeSemGive(sema, 1);
	}
}

/**
 * @brief Just a periodic task
 */
static void dly_task(void *args)
{
	for(;;)
	{
		uLipeTaskDelay(10);
		xcnt3++;
	}
}

static void leds_task(void *args)
{
	leds_attach(N_LED1, PB18);
	leds_attach(N_LED2, PB19);
	leds_attach(N_LED3, PD1);
	leds_set(N_LED1 | N_LED3, LED_OFF);
	leds_set(N_LED2, LED_BLINK_SLOW);

	for(;;)
	{
		uLipeTaskDelay(100);
		leds_action_isr_100ms();
	}
}

static void LCD_task(void *args)
{
	uint16_t *tsi_data;
	lcd_attach(PA13, PD5, PA4, PA5, PC8, PC9);

	lcd_gotoxy(1,1);
	xprintf(lcd_putc, "RTOS LHC");

	for(;;)
	{
		//uLipeSemTake(sema, 0);

		tsi_data = ((uint16_t *)uLipeQueueRemove(que,OS_Q_BLOCK_EMPTY,0,NULL));
		if(tsi_data != NULL)
		{
			lcd_gotoxy(1,2);
			xprintf(lcd_putc, "\fCount:%u", *tsi_data);
		}
	}
}


/**
 * @brief led task triggered by queue contents
 */
static void led_task(void *args)
{
	tpm_chnl_pwm_signal_param_t pwm_param[3] = {0};
	tpm_config_t pwm_config = {0};
	uint16_t *tsi_data;

	pwm_param[0].chnlNumber = 0;
	pwm_param[0].dutyCyclePercent =0;
	pwm_param[0].level = kTPM_LowTrue;

	pwm_param[1].chnlNumber = 1;
	pwm_param[1].dutyCyclePercent = 0;
	pwm_param[1].level = kTPM_LowTrue;

	pwm_param[2].chnlNumber = 1;
	pwm_param[2].dutyCyclePercent = 0;
	pwm_param[2].level = kTPM_LowTrue;

	CLOCK_SetTpmClock(1);

	/* inits and configures the tpm timers */
	CLOCK_EnableClock(kCLOCK_Tpm2);
	CLOCK_EnableClock(kCLOCK_Tpm0);
	TPM_GetDefaultConfig(&pwm_config);
	TPM_Init(TPM0, &pwm_config);
	TPM_Init(TPM2, &pwm_config);

	TPM_SetupPwm(TPM2, &pwm_param[0], 2, kTPM_EdgeAlignedPwm, 12000, TPM_SOURCE_CLOCK);
	TPM_SetupPwm(TPM0, &pwm_param[2], 1, kTPM_EdgeAlignedPwm, 12000, TPM_SOURCE_CLOCK);

	TPM_StartTimer(TPM2, kTPM_SystemClock);
	TPM_StartTimer(TPM0, kTPM_SystemClock);


	/*set the initial duty and starts the pwm */
	TPM_UpdatePwmDutycycle(TPM2,0,kTPM_EdgeAlignedPwm, 64);
	TPM_UpdatePwmDutycycle(TPM2,1,kTPM_EdgeAlignedPwm, 64);
	TPM_UpdatePwmDutycycle(TPM0,1,kTPM_EdgeAlignedPwm, 64);

	for(;;)
	{
		tsi_data = ((uint16_t *)uLipeQueueRemove(que,OS_Q_BLOCK_EMPTY,0,NULL));

		uLipeTaskDelay(100);


		if(tsi_data != NULL)
		{
			uint16_t value = *tsi_data;

			value >>= 5;
			TPM_UpdatePwmDutycycle(TPM2,0,kTPM_EdgeAlignedPwm, value );
			TPM_UpdatePwmDutycycle(TPM2,1,kTPM_EdgeAlignedPwm, (64 - value > 64)? 0 : (64 - value) );
			TPM_UpdatePwmDutycycle(TPM0,1,kTPM_EdgeAlignedPwm, (value/2) );
		}
	}
}


/**
 * @brief deferrable touch task, triggered when touch event is complete
 */
static void touch_task(void *args)
{
	tsi_config_t config = {0};
	uint16_t measure;

	/* configure and inits tsi */
	CLOCK_EnableClock(kCLOCK_Tsi0);
	TSI_EnableModule(TSI0, false);
	TSI_Deinit(TSI0);
	TSI_GetNormalModeDefaultConfig(&config);
	TSI_Init(TSI0, &config);

	/* set TSI interrupt */
	NVIC_DisableIRQ(TSI0_IRQn);
	NVIC_SetPriority(TSI0_IRQn, 0xFD);
	TSI_EnableInterrupts(TSI0, kTSI_EndOfScanInterruptEnable | kTSI_GlobalInterruptEnable);

	/* Enable the TSI */
	TSI_EnableModule(TSI0, true);

	/* starts the touch */
	NVIC_EnableIRQ(TSI0_IRQn);
	TSI_SetMeasuredChannelNumber(TSI0, BOARD_TSI_ELECTRODE_1_CHANNEL);
	TSI_StartSoftwareTrigger(TSI0);

	msgLCD_t xMsg;
	uint8_t buffer[16];
	for(;;)
	{
		//uLipeSemTake(sema, 0);
		uLipeFlagsPend(flags, 0x01, OS_FLAGS_PEND_ANY | OS_FLAGS_CONSUME, 0);
		measure = TSI_GetCounter(TSI0);
		uLipeTaskDelay(800);

		sprintf(buffer, "Touch: %d", measure);
		setMsgLCD(&xMsg, buffer, strlen(buffer), 1, 1);

		uLipeQueueInsert(que,&measure,OS_Q_NON_BLOCK,0);
		TSI_SetMeasuredChannelNumber(TSI0, BOARD_TSI_ELECTRODE_1_CHANNEL);
		TSI_StartSoftwareTrigger(TSI0);
	}

}

/**
 * @brief touch interrrupt server routine, responsible to defer the TOUCH event
 */
void TSI0_IRQHandler(void)
{
	uLipeKernelIrqIn();
	uint32_t status = TSI_GetStatusFlags(TSI0);

	if(status & kTSI_EndOfScanFlag)
	{
		/* wake up the touch task */
		//uLipeSemGive(sema, 1);
		uLipeFlagsPost(flags, 0x80);
	}

	TSI_ClearStatusFlags(TSI0, status | kTSI_GlobalInterruptEnable);
	uLipeKernelIrqOut();
}

void opz(int ile)
{
	int p;
	for(p=0;p<ile;p++)
	{

	}
}

void jrg_enableLeds(void)
{

	//======================================
	// configura leds
	//======================================


	PORTB->PCR[18] =PORTB->PCR[18] &~(1<<10) &~(1<<9) | (1<<8);
	GPIOB->PDDR=GPIOB->PDDR | (1<<18);
	GPIOB->PDOR |= (1<<18);

	PORTB->PCR[19]=PORTB->PCR[19] &~(1<<10) &~(1<<9) | (1<<8);
	GPIOB->PDDR=GPIOB->PDDR | (1<<19);
	GPIOB->PDOR |= (1<<19);

	PORTD->PCR[1]=PORTB->PCR[1] &~(1<<10) &~(1<<9) | (1<<8);
	GPIOD->PDDR=GPIOB->PDDR | (1<<1);
	GPIOD->PDOR |= (1<<1);

	//======================================
	// configura LCD
	//======================================

	PORTA->PCR[4] =PORTA->PCR[4] &~(1<<10) &~(1<<9) | (1<<8);
	GPIOA->PDDR=GPIOA->PDDR | (1<<4);
	GPIOA->PDOR |= (1<<4);

	PORTA->PCR[5] =PORTA->PCR[5] &~(1<<10) &~(1<<9) | (1<<8);
	GPIOA->PDDR=GPIOA->PDDR | (1<<5);
	GPIOA->PDOR |= (1<<5);

	PORTA->PCR[13] =PORTA->PCR[13] &~(1<<10) &~(1<<9) | (1<<8);
	GPIOA->PDDR=GPIOA->PDDR | (1<<13);
	GPIOA->PDOR |= (1<<13);


	PORTC->PCR[8] =PORTC->PCR[8] &~(1<<10) &~(1<<9) | (1<<8);
	GPIOC->PDDR=GPIOC->PDDR | (1<<8);
	GPIOC->PDOR |= (1<<8);

	PORTC->PCR[9] =PORTC->PCR[9] &~(1<<10) &~(1<<9) | (1<<8);
	GPIOC->PDDR=GPIOC->PDDR | (1<<9);
	GPIOC->PDOR |= (1<<9);


	PORTD->PCR[0] =PORTD->PCR[0] &~(1<<10) &~(1<<9) | (1<<8);
	GPIOD->PDDR=GPIOD->PDDR | (1<<0);
	GPIOD->PDOR |= (1<<0);

	PORTD->PCR[5] =PORTD->PCR[5] &~(1<<10) &~(1<<9) | (1<<8);
	GPIOD->PDDR=GPIOD->PDDR | (1<<5);
	GPIOD->PDOR |= (1<<5);
}

#include "fsl_uart.h"

#define RING_BUFFER_SIZE 64
#define RX_DATA_SIZE     32
uart_handle_t g_uartHandle;
uart_config_t user_config;
uart_transfer_t sendXfer;
uart_transfer_t receiveXfer;
volatile bool txFinished;
volatile bool rxFinished;
uint8_t receiveData[RX_DATA_SIZE];
uint8_t ringBuffer[RING_BUFFER_SIZE];

void UART_UserCallback(uart_handle_t *handle, status_t status, void *userData)
{
	userData = userData;
	if (kStatus_UART_RxIdle == status)
	{
		rxFinished = true;
	}
}

QueueData_t _queue[2];

/**
 * @brief the "old but gold" main function
 */
int main(void)
{

	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDEBUG_UART();
	BOARD_InitTOUCH();
	BOARD_InitLEDs();
	BOARD_InitLCD();
	BOARD_InitACCEL();
	jrg_enableLeds();

//	uint32_t uartClkSrcFreq = CLOCK_GetFreq(BOARD_DEBUG_UART_CLKSRC);
//	DbgConsole_Init(BOARD_DEBUG_UART_BASEADDR, BOARD_DEBUG_UART_BAUDRATE, DEBUG_CONSOLE_DEVICE_TYPE_UART, uartClkSrcFreq);
//
//	PRINTF("Ola Mundo!\n");

	uLipeRtosInit(); //bota o kernel no estado inicial


	//======================================
	//
	//======================================

	sema = uLipeSemCreate(0, 1, NULL);
	que = uLipeQueueCreate(6, NULL);

	flags = uLipeFlagsCreate(NULL);
	mtx = uLipeMutexCreate(NULL);


	//======================================
	// Tarefas que usam resume e ...
	//======================================
	if(uLipeTaskCreate(&dly_task, 64, 2, NULL) != kStatusOk)
	{
		while(1);
	}

	//======================================
	//
	//======================================
	if(uLipeTaskCreate(&teste_task2, 64, 3, NULL) != kStatusOk)
	{
		while(1);
	}

	//======================================
	// Tarefa Leds
	//======================================
	if(uLipeTaskCreate(&leds_task, 64, 4, NULL) != kStatusOk)
	{
		while(1);
	}

	//======================================
	// Tarefa Touch
	//======================================
	if(uLipeTaskCreate(&touch_task, 128, 5, NULL) != kStatusOk)
	{
		while(1);
	}

	//======================================
	// Tarefa Display LCD
	//======================================
	if(uLipeTaskCreate(&LCD_task, 128, 6, NULL) != kStatusOk)
	{
		while(1);
	}

	//======================================
	//
	//======================================



	uLipeRtosStart();
	return 0;
}










//int main(void)
//{
//
//	OS_QUEUE_RESERVE(qdata, 1);
//
//	BOARD_InitPins();
//	BOARD_BootClockRUN();
//	BOARD_InitTOUCH();
//
//	uLipeRtosInit(); //bota o kernel no estado inicial
//
//	// Tarefas que usam resume e ...
//	uLipeTaskCreate(&dly_task, &dly_stk[0], 64,     3, NULL);
//	uLipeTaskCreate(&teste_task, &teste_stk[0], 64, 2, NULL);
//
//
//	// Tarefas que usam semaforo
//	uLipeTaskCreate(&teste_task2, &teste_stk2[0], 64, 4, NULL);
//	uLipeTaskCreate(&teste_task3, &teste_stk3[0], 64, 5, NULL);
//
//
//
//	//    uLipeTaskCreate(&led_task,&led_stk[0],128,1,NULL);
//	//    uLipeTaskCreate(&touch_task,&touch_stk[0],128,4,NULL);
//	//	  uLipeTaskCreate(&dly_task,&dly_stk[0],64,3,NULL);
//
//	//    que = uLipeQueueCreate(qdata,1,NULL);
//	    sema = uLipeSemCreate(0,1,NULL);
//	//    flags = uLipeFlagsCreate(NULL);
//	//    mtx = uLipeMutexCreate(NULL);
//
//	uLipeRtosStart();
//	return 0;
//}

//http://mcuxpresso.nxp.com/apidoc/2.0/group__debug__console.html

