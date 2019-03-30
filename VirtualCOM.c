/*------------------------------------------------------------------------------
 * MDK Middleware - Component ::USB:Device
 * Copyright (c) 2004-2016 ARM Germany GmbH. All rights reserved.
 *------------------------------------------------------------------------------
 * Name:    VirtualCOM.c
 * Purpose: USB Device Virtual COM Port demonstration
 *----------------------------------------------------------------------------*/

#include "cmsis_os.h"
#include "rl_usb.h"

#include "Board_GLCD.h"
#include "GLCD_Config.h"
#include "LPC17xx.h"                    // Device header

extern    GLCD_FONT GLCD_Font_6x8;
extern    GLCD_FONT GLCD_Font_16x24;

#define THRE        (1<<5) //Transmit Holding Register Empty
#define MULVAL      15
#define DIVADDVAL   2
#define Ux_FIFO_EN  (1<<0)
#define Rx_FIFO_RST (1<<1)
#define Tx_FIFO_RST (1<<2)
#define DLAB_BIT    (1<<7)
#define LINE_FEED   0x0A //LF, For Linux, MAC and Windows Terminals
#define CARRIAGE_RETURN 0x0D //CR, For Windows Terminals (CR+LF).
//---------------------------
#define SBIT_CNTEN     0
#define SBIT_PWMEN     2
#define SBIT_PWMMR0R   1
#define SBIT_LEN0      0
#define SBIT_LEN1      1
#define SBIT_PWMENA1   9
#define PWM_1          0 //P2_0 (0-1 Bits of PINSEL4)
#define SBIT_TIMER0  1
#define SBIT_TIMER1  2
#define SBIT_MR0I    0
#define SBIT_MR0R    1
#define SBIT_CNTEN   0
#define PCLK_TIMER0  2
#define PCLK_TIMER1  4
//***********************************************
//***********************************************My Variables and consts
#define period  518
#define Sample 200

int buffer[Sample];
 uint8_t b[Sample*4+2];
int counter = 0;
int counterUART = 0;
uint32_t s1=0,flv;

//***********************************************

void CDC0_ACM_UART_to_USB_Threadm (void const *arg) {
  //(void)(arg);

while (1) {
	// UART - > USB
	if (flv == Sample*4+2) {
		USBD_CDC_ACM_WriteData(0U, b, (uint32_t)Sample*4+2);
		flv=0;
	}
	//osDelay(10U);
  }
}
extern const osThreadDef_t os_thread_def_CDC0_ACM_UART_to_USB_Threadm;
osThreadDef (CDC0_ACM_UART_to_USB_Threadm, osPriorityNormal, 1U, 0U);




int i=0,j=0;

int kk0=0;


void changePWM(uint32_t valMR0,uint32_t valMR1,int kx)
{
	LPC_PWM1->MR0 = valMR0;                /* set PWM cycle(Ton+Toff)=100) */
	LPC_PWM1->MR1 = valMR1;                 /* Set 50% Duty Cycle for all four channels */
	/* Trigger the latch Enable Bits to load the new Match Values */
	LPC_PWM1->LER =  (1<<SBIT_LEN1);
	/* Enable the PWM output pins for PWM_1-PWM_4(P2_0 - P2_3) */
	LPC_PWM1->PCR = (1<<SBIT_PWMENA1);
	if(kx>50){
		kk0=400000;
		while(kk0--);
	}
}


void initTIM0(void){
	//LPC_SC->PCONP |= (1<<SBIT_TIMER0) | (1<<SBIT_TIMER1); /* Power ON Timer0,1 */
	LPC_TIM0->MCR  = (1<<SBIT_MR0I) | (1<<SBIT_MR0R);     /* Clear TC on MR0 match and Generate Interrupt*/
	LPC_TIM0->PR   = 24999;
	LPC_TIM0->MR0  = 100;                 /* Load timer value to generate 100ms delay*/
	LPC_TIM0->TCR  = (1 <<SBIT_CNTEN);                    /* Start timer by setting the Counter Enable*/
	NVIC_EnableIRQ(TIMER0_IRQn);                          /* Enable Timer0 Interrupt */
}

void TIMER0_IRQHandler(void)
{
	unsigned int isrMask;
	isrMask = LPC_TIM0->IR;
	LPC_TIM0->IR = isrMask;         /* Clear the Interrupt Bit */
	LPC_TIM0->MR0  = 100;                 /* Load timer value to generate 100ms delay*/
	LPC_TIM0->TCR  = (1 <<SBIT_CNTEN);

	//changePWM(period,period/2,100);
	LPC_ADC->ADCR  |= (1<<16u); 	// ADC
}


void initPWM(void){
	LPC_PINCON->PINSEL4 = (1<<PWM_1);
	LPC_PINCON->PINMODE4 &= (2<<PWM_1);

   /* Enable Counters,PWM module */
	LPC_PWM1->TCR = (1<<SBIT_CNTEN) | (1<<SBIT_PWMEN);
	LPC_PWM1->PR  =  24;               //25mhz
	LPC_PWM1->MCR = (1<<SBIT_PWMMR0R);  /*Reset on PWMMR0, reset TC if it matches MR0 */
	LPC_PWM1->MR0 = period;                /* set PWM cycle(Ton+Toff)=100) */
	LPC_PWM1->MR1 = period>>1;                 /* Set 50% Duty Cycle for all four channels */
	/* Trigger the latch Enable Bits to load the new Match Values */
	LPC_PWM1->LER = (1<<SBIT_LEN0) | (1<<SBIT_LEN1);
	/* Enable the PWM output pins for PWM_1-PWM_4(P2_0 - P2_3) */
	LPC_PWM1->PCR = (1<<SBIT_PWMENA1);
}

void initADC(){
	LPC_SC->PCONP |= (1 << 12);
	LPC_PINCON->PINSEL1 &= ~(3 << 14);    // connect pin to ADC  p0.23
	LPC_PINCON->PINSEL1 |= (1 << 14);
	LPC_PINCON->PINMODE1 &= ~(3<<14);
	LPC_PINCON->PINMODE1 |= (2<<14);

	LPC_ADC->ADINTEN = 1;
	LPC_ADC->ADCR = (1<<21u) | 1;
	NVIC_EnableIRQ(ADC_IRQn);
}





void send0(){
	//changePWM(period,period+1,0);

	counter++;
		for(i=0;i<Sample;i++){
			s1= buffer[i];
			b[(i*4)+3]= (s1 %10) + 0x30;
			b[(i*4)+2]= ((s1 /10) % 10) + 0x30;
			b[(i*4)+1]= ((s1 /100) % 10) + 0x30;
			b[(i*4)+0]= ((s1 /1000)) + 0x30;
		}

		//send UART
		counterUART = 0;
		flv= Sample*4+2;
		//USBD_CDC_ACM_PutChar(0U,60);
		//USBD_CDC_ACM_WriteData(0U, b, Sample*4+2);
//		while( counterUART < Sample*4+2)
//		{
//			//U0Write(b[counterUART]);
//			counterUART++;
//		}
}

void ADC_IRQHandler(void)
{

	if( counter <Sample){
		buffer[counter] = (LPC_ADC->ADDR0 >> 4) & 0xFFF;
		counter++;
	}
	else if(counter == Sample){
		LPC_ADC->ADCR &= ~(1<<16u);			//stop ADC burst
		send0();
	}
	else{
		s1  = (LPC_ADC->ADDR0 >> 4) & 0xFFF;
		counter = 0;
	}

}



int main (void) {
	USBD_Initialize         (0U);         // USB Device 0 Initialization
  USBD_Connect            (0U);         // USB Device 0 Connect

	osDelay(10000U);							// this delay is neccessary for working USB properly and refusing HardFault error

	b[Sample*4]= 65;
	b[Sample*4+1]= 65;

	initTIM0();
	initPWM();
	initADC();



	osThreadCreate (osThread (CDC0_ACM_UART_to_USB_Threadm), NULL);
	while (1) {
		osSignalWait (0U, osWaitForever);
	}
}
