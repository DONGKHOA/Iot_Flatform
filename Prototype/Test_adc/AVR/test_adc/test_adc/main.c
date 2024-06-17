/*
 * main.c
 *
 * Created: 6/12/2024 4:52:01 PM
 * Author : DONGKHOA
 */ 
#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include "AD.h"
#include "uart.h"

uint16_t data;
char str_data[20];

void ADC_Init()
{
	ADMUX = (1<<REFS0);			//avcc
	ADCSRA = (1<<ADEN)|(1<<ADSC)|(1<<ADATE)|(1<<ADPS2)|(1<<ADPS1);
}

int main(void)
{
	DDRA |= 0x00;
	PORTA = 0;
    usart0_init();
	ADC_Init();
	
    while (1) 
    {
		data = adc_read(PINA0);
		sprintf(str_data,"%d\n", data);
		/*itoa(data, str_data, 10);*/
		usart0_send_string(str_data);
		_delay_ms(500);
    }
}

