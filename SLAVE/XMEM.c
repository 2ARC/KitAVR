// XMEM Example
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
// Define baud rate
#define USART0_BAUD         115200ul
#define USART0_UBBR_VALUE   ((F_CPU/(USART0_BAUD<<4))-1)
#define BUFFER_SIZE 255
#define PRINTF(format, ...) printf_P(PSTR(format), ## __VA_ARGS__)
void USART0_Init(void)
{
    // Set baud rate
    UBRR0H = (uint8_t)(USART0_UBBR_VALUE>>8);
    UBRR0L = (uint8_t)USART0_UBBR_VALUE;
    // Set frame format to 8 data bits, no parity, 1 stop bit
    UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
    // Enable receiver and transmitter
    UCSR0B |= (1<<RXEN)|(1<<TXEN);   
}
int usart0_put_char(char data, FILE *stream)
{
    // Recursive function to prepend a carriage return before a new line
    if(data == '\n')
    {
        usart0_put_char('\r',stream);
    }
    // Wait if a byte is being transmitted
    while((UCSR0A&(1<<UDRE0)) == 0)
    {
        ;
    }
    // Transmit data
    UDR0 = data;
    return 0;
}
// Create PRINTF Stream structure
FILE printf_stream = FDEV_SETUP_STREAM(usart0_put_char, NULL, _FDEV_SETUP_WRITE);
//initialize stream
void printf_init(void)
{
    // Initialise USART
    USART0_Init();
    // Route stdout stream to printf_stream
    stdout = &printf_stream;
}
void XMEM_init(void)
{
    // External memory interface enable 
	MCUCR |= (1<<SRE);   
    XMCRA = 0;
	//PC7..PC5 released pins
    XMCRB |= (1<<XMM1)|(1<<XMM0);
}
int main(void)
{
    uint8_t *mem;
	uint8_t index;
	uint8_t data=1;
    // Initialise USART
 	printf_init();
    // Send string
	PRINTF("AVR XMEM test\n");
	XMEM_init();
	PRINTF("XMEM init\n");
	mem = malloc(BUFFER_SIZE);
	PRINTF("Allocating 256 bytes space in ext ram\n");   
	PRINTF("%d Byte buffer (starting at 0x%04X) filled with incrementing numbers:\n",BUFFER_SIZE,mem);
	// Fill memory incrementing values
	for(index = 0; index < BUFFER_SIZE; index++)
	{
    	mem[index] = data++;
	}
	// Display memory block
	for(index = 0; index < BUFFER_SIZE; index++)
	{
		PRINTF("%02X ",mem[index]);
		if((index&0x0F) == 0x0F)
		{
			PRINTF("\n");
		}
	}
	//free allocated memory
	free(mem);
    // Repeat indefinitely
    while(1)
    {
        
    }
}
