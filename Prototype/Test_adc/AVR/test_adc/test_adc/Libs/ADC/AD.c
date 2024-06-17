#include <avr/io.h>
#include <avr/interrupt.h>
#include "AD.h"


void init_adc(uint8_t channel)
{
	ADMUX &= 0xD0;
	ADMUX |= (channel & 0x0F);
	//Prescaler of 128. 16Mhz/128 = 125kHz
	//ADC mode Freerun
	ADCSRA |= (1<<ADEN)|(1<<ADATE)|(1<<ADPS2)|(1<<ADPS1);
}


uint16_t adc_read(unsigned char channel)
{
    // Start ADC conversion by setting the ADSC bit in ADCSRA
    ADCSRA |= (1 << ADSC);

    // Wait for conversion to complete (ADIF bit in ADCSRA becomes 1)
    while (!(ADCSRA & (1 << ADIF)));

    // Clear ADIF by writing a 1 to it (this is done by writing a 1 to the ADIF bit)
    ADCSRA |= (1 << ADIF);

    return (ADCH << 8) | ADCL;
}

double voltage(unsigned char channel)
{
	return adc_read(channel) * 0.004003910068f;
}
