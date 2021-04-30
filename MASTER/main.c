/*
* slave.c
*
* Criado: 06/08/2016 15:45:04
* Autor : Marco Antonio de Oliveira
* email : marcoadeoli@outlook.com
*/
//
#include <avr/io.h>
#ifndef F_CPU
# define F_CPU 1000000UL
#endif
#include <util/delay.h>
#include "ds1307.h"

// Initialize SPI Master Device (without interrupt)
void spi_init_master (void)
{
	// Set MOSI, SCK as Output
	DDRB = (1<<5)|(1<<3);
	
	// Enable SPI, Set as Master
	//Prescaler: Fosc/16
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}

//Function to send data
void spi_envia (int data)
{
	// Load data into the buffer
	SPDR = data;
	
	//Wait until transmission complete
	while(!(SPSR & (1<<SPIF)));
}

void adc_init()
{
	// AREF = AVCC
	ADMUX = (1<<REFS0);
	
	// ADC Enable and prescaler of 128
	// 16000000/128 = 125000
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

uint16_t adc_read(uint8_t ch)
{
	// select the corresponding channel 0~7
	// ANDing with ’7? will always keep the value
	// of ‘ch’ between 0 and 7
	ch &= 0b00000111;  // AND operation with 7
	ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing
	
	// start single convertion
	// write ’1? to ADSC
	ADCSRA |= (1<<ADSC);
	
	// wait for conversion to complete
	// ADSC becomes ’0? again
	// till then, run loop continuously
	while(ADCSRA & (1<<ADSC));
	
	return (ADC);
}
//
void enviaByte(char byte)
{
	for (int a = 1; a <= 11; a++)
	{
		if ( a ==  1) {PORTB &=~(1 << PINB7);}
		if ( a ==  2) {if (0b00000001 & byte) PORTB |= (1 << PINB7); else PORTB &=~(1 << PINB7);}
		if ( a ==  3) {if (0b00000010 & byte) PORTB |= (1 << PINB7); else PORTB &=~(1 << PINB7);}
		if ( a ==  4) {if (0b00000100 & byte) PORTB |= (1 << PINB7); else PORTB &=~(1 << PINB7);}
		if ( a ==  5) {if (0b00001000 & byte) PORTB |= (1 << PINB7); else PORTB &=~(1 << PINB7);}
		if ( a ==  6) {if (0b00010000 & byte) PORTB |= (1 << PINB7); else PORTB &=~(1 << PINB7);}
		if ( a ==  7) {if (0b00100000 & byte) PORTB |= (1 << PINB7); else PORTB &=~(1 << PINB7);}
		if ( a ==  8) {if (0b01000000 & byte) PORTB |= (1 << PINB7); else PORTB &=~(1 << PINB7);}
		if ( a ==  9) {if (0b10000000 & byte) PORTB |= (1 << PINB7); else PORTB &=~(1 << PINB7);}
		if ((a == 10) || (a == 11)) {PORTB |= (1 << PINB7);}
		PORTB &=~(1 << PINB6);
		_delay_us(75);
		PORTB |= (1 << PINB6);
		_delay_us(25);
	}
}
//
void ligaDesligaNaSequencia()
{
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x3c);
	_delay_ms(100);
	enviaByte(0x2c);
	_delay_ms(100);
	enviaByte(0x45); //
	_delay_ms(100);
	enviaByte(0x16); //
	_delay_ms(100);
	enviaByte(0x29);
	_delay_ms(100);
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x31);
	_delay_ms(100);
	enviaByte(0x5a);
	_delay_ms(250);
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x3c);
	_delay_ms(100);
	enviaByte(0x2c);
	_delay_ms(100);
	enviaByte(0x45); //
	_delay_ms(100);
	enviaByte(0x16); //
	_delay_ms(100);
	enviaByte(0x29);
	_delay_ms(100);
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x2B);
	_delay_ms(100);
	enviaByte(0x2B);
	_delay_ms(100);
	enviaByte(0x5a);
	_delay_ms(250);
	//
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x3c);
	_delay_ms(100);
	enviaByte(0x2c);
	_delay_ms(100);
	enviaByte(0x45); //
	_delay_ms(100);
	enviaByte(0x1E); //
	_delay_ms(100);
	enviaByte(0x29);
	_delay_ms(100);
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x31);
	_delay_ms(100);
	enviaByte(0x5a);
	_delay_ms(250);
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x3c);
	_delay_ms(100);
	enviaByte(0x2c);
	_delay_ms(100);
	enviaByte(0x45); //
	_delay_ms(100);
	enviaByte(0x1E); //
	_delay_ms(100);
	enviaByte(0x29);
	_delay_ms(100);
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x2B);
	_delay_ms(100);
	enviaByte(0x2B);
	_delay_ms(100);
	enviaByte(0x5a);
	_delay_ms(250);
	//
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x3c);
	_delay_ms(100);
	enviaByte(0x2c);
	_delay_ms(100);
	enviaByte(0x45); //
	_delay_ms(100);
	enviaByte(0x26); //
	_delay_ms(100);
	enviaByte(0x29);
	_delay_ms(100);
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x31);
	_delay_ms(100);
	enviaByte(0x5a);
	_delay_ms(250);
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x3c);
	_delay_ms(100);
	enviaByte(0x2c);
	_delay_ms(100);
	enviaByte(0x45); //
	_delay_ms(100);
	enviaByte(0x26); //
	_delay_ms(100);
	enviaByte(0x29);
	_delay_ms(100);
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x2B);
	_delay_ms(100);
	enviaByte(0x2B);
	_delay_ms(100);
	enviaByte(0x5a);
	_delay_ms(250);
	//
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x3c);
	_delay_ms(100);
	enviaByte(0x2c);
	_delay_ms(100);
	enviaByte(0x45); //
	_delay_ms(100);
	enviaByte(0x25); //
	_delay_ms(100);
	enviaByte(0x29);
	_delay_ms(100);
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x31);
	_delay_ms(100);
	enviaByte(0x5a);
	_delay_ms(250);
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x3c);
	_delay_ms(100);
	enviaByte(0x2c);
	_delay_ms(100);
	enviaByte(0x45); //
	_delay_ms(100);
	enviaByte(0x25); //
	_delay_ms(100);
	enviaByte(0x29);
	_delay_ms(100);
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x2B);
	_delay_ms(100);
	enviaByte(0x2B);
	_delay_ms(100);
	enviaByte(0x5a);
	_delay_ms(250);
	//
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x3c);
	_delay_ms(100);
	enviaByte(0x2c);
	_delay_ms(100);
	enviaByte(0x45); //
	_delay_ms(100);
	enviaByte(0x2E); //
	_delay_ms(100);
	enviaByte(0x29);
	_delay_ms(100);
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x31);
	_delay_ms(100);
	enviaByte(0x5a);
	_delay_ms(250);
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x3c);
	_delay_ms(100);
	enviaByte(0x2c);
	_delay_ms(100);
	enviaByte(0x45); //
	_delay_ms(100);
	enviaByte(0x2E); //
	_delay_ms(100);
	enviaByte(0x29);
	_delay_ms(100);
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x2B);
	_delay_ms(100);
	enviaByte(0x2B);
	_delay_ms(100);
	enviaByte(0x5a);
	_delay_ms(250);
	//
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x3c);
	_delay_ms(100);
	enviaByte(0x2c);
	_delay_ms(100);
	enviaByte(0x45); //
	_delay_ms(100);
	enviaByte(0x36); //
	_delay_ms(100);
	enviaByte(0x29);
	_delay_ms(100);
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x31);
	_delay_ms(100);
	enviaByte(0x5a);
	_delay_ms(250);
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x3c);
	_delay_ms(100);
	enviaByte(0x2c);
	_delay_ms(100);
	enviaByte(0x45); //
	_delay_ms(100);
	enviaByte(0x36); //
	_delay_ms(100);
	enviaByte(0x29);
	_delay_ms(100);
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x2B);
	_delay_ms(100);
	enviaByte(0x2B);
	_delay_ms(100);
	enviaByte(0x5a);
	_delay_ms(250);
	//
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x3c);
	_delay_ms(100);
	enviaByte(0x2c);
	_delay_ms(100);
	enviaByte(0x45); //
	_delay_ms(100);
	enviaByte(0x3D); //
	_delay_ms(100);
	enviaByte(0x29);
	_delay_ms(100);
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x31);
	_delay_ms(100);
	enviaByte(0x5a);
	_delay_ms(250);
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x3c);
	_delay_ms(100);
	enviaByte(0x2c);
	_delay_ms(100);
	enviaByte(0x45); //
	_delay_ms(100);
	enviaByte(0x3D); //
	_delay_ms(100);
	enviaByte(0x29);
	_delay_ms(100);
	enviaByte(0x44);
	_delay_ms(100);
	enviaByte(0x2B);
	_delay_ms(100);
	enviaByte(0x2B);
	_delay_ms(100);
	enviaByte(0x5a);
	_delay_ms(250);
}
//
int main(void)
{
	DDRB = 0xFF;
	PORTB = 0xFF;
	_delay_ms(15000);

	// Envia alguns backspaces para resetar o simulador no Proteus
	for (int i = 0; i < 9; i++)
	{
		enviaByte(0x66); // <-- BS
		_delay_ms(100);
		enviaByte(0x66); // <-- BS
		_delay_ms(100);
		enviaByte(0x66); // <-- BS
		_delay_ms(100);
		enviaByte(0x66); // <-- BS
		_delay_ms(100);
		enviaByte(0x66); // <-- BS
		_delay_ms(100);
		enviaByte(0x66); // <-- BS
		_delay_ms(100);
		enviaByte(0x66); // <-- BS
		_delay_ms(100);
		enviaByte(0x66); // <-- BS
		_delay_ms(100);
	}	
	//
	ligaDesligaNaSequencia();
	//
	for (;;)
	{
		enviaByte(0x44); // O
		_delay_ms(100);
		enviaByte(0x3c); // U
		_delay_ms(100);
		enviaByte(0x2c); // T
		_delay_ms(100);
		enviaByte(0x16); // 1
		_delay_ms(100);
		enviaByte(0x16); // 1
		_delay_ms(100);
		enviaByte(0x29); // Espaço
		_delay_ms(100);
		enviaByte(0x44); // O
		_delay_ms(100);
		enviaByte(0x31); // N
		_delay_ms(100);
		enviaByte(0x5a); // Enter
		_delay_ms(1000);

		enviaByte(0x43); // I
		_delay_ms(100);
		enviaByte(0x31); // N
		_delay_ms(100);
		enviaByte(0x1E); // 2
		_delay_ms(100);
		enviaByte(0x1E); // 2
		_delay_ms(100);
		enviaByte(0x5a); // Enter
		_delay_ms(3000);

		enviaByte(0x44); // O
		_delay_ms(100);
		enviaByte(0x3c); // U
		_delay_ms(100);
		enviaByte(0x2c); // T
		_delay_ms(100);
		enviaByte(0x16); // 1
		_delay_ms(100);
		enviaByte(0x16); // 1
		_delay_ms(100);
		enviaByte(0x29); // Espaço
		_delay_ms(100);
		enviaByte(0x44); // O
		_delay_ms(100);
		enviaByte(0x2B); // F
		_delay_ms(100);
		enviaByte(0x2B); // F
		_delay_ms(100);
		enviaByte(0x5a); // Enter
		_delay_ms(1000);

		enviaByte(0x43); // I
		_delay_ms(100);
		enviaByte(0x31); // N
		_delay_ms(100);
		enviaByte(0x1E); // 2
		_delay_ms(100);
		enviaByte(0x1E); // 2
		_delay_ms(100);
		enviaByte(0x5a); // Enter
		_delay_ms(3000);

		enviaByte(0x44); // O
		_delay_ms(100);
		enviaByte(0x3c); // U
		_delay_ms(100);
		enviaByte(0x2c); // T
		_delay_ms(100);
		enviaByte(0x45); // 0
		_delay_ms(100);
		enviaByte(0x1E); // 2
		_delay_ms(100);
		enviaByte(0x29); // Espaço
		_delay_ms(100);
		enviaByte(0x44); // O
		_delay_ms(100);
		enviaByte(0x31); // N
		_delay_ms(100);
		enviaByte(0x5a); // Enter
		_delay_ms(1000);

		enviaByte(0x43); // I
		_delay_ms(100);
		enviaByte(0x31); // N
		_delay_ms(100);
		enviaByte(0x26); // 3
		_delay_ms(100);
		enviaByte(0x26); // 3
		_delay_ms(100);
		enviaByte(0x5a); // Enter
		_delay_ms(3000);

		enviaByte(0x44); // O
		_delay_ms(100);
		enviaByte(0x3c); // U
		_delay_ms(100);
		enviaByte(0x2c); // T
		_delay_ms(100);
		enviaByte(0x45); // 0
		_delay_ms(100);
		enviaByte(0x1E); // 2
		_delay_ms(100);
		enviaByte(0x29); // Espaço
		_delay_ms(100);
		enviaByte(0x44); // O
		_delay_ms(100);
		enviaByte(0x2B); // F
		_delay_ms(100);
		enviaByte(0x2B); // F
		_delay_ms(100);
		enviaByte(0x5a); // Enter
		_delay_ms(1000);

		enviaByte(0x43); // I
		_delay_ms(100);
		enviaByte(0x31); // N
		_delay_ms(100);
		enviaByte(0x26); // 3
		_delay_ms(100);
		enviaByte(0x26); // 3
		_delay_ms(100);
		enviaByte(0x5a); // Enter
		_delay_ms(3000);
	}
}
