/*
* slave.c
*
* Criado: 06/08/2016 15:45:04
* Autor : Marco Antonio de Oliveira
* email : marcoadeoli@outlook.com
*/
//
#include <stdio.h> // sscanf converte string para int
#include <stdlib.h> // utoa converte int para string
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/power.h>
//
#ifndef F_CPU
# define F_CPU 11059200UL
#endif
//
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "hd44780.h"
#include "scancodes.h"
// Pinos do teclado PS2
#define clockPS2 PIND3
#define dadosPS2 PIND4
// Tamanho do buffer do teclado PS2
#define KB_BUFF_SIZE 64
//
#define		BOOL		unsigned char
#define		TRUE		127 // Para evitar conflito com comparação a 1 em algumas funções e permitir o uso de outros valores nas variáveis em sequencia. Ex.: = FALSE, = TRUE ou = 2
#define		FALSE		0
#define		ON			1
#define		OFF			0
#define		ENTRADA		1
#define		SAIDA		0
//
BOOL blink = FALSE; // Avisa para piscar as saídas na velocidade do cursor
BOOL debug = FALSE; // Avisa para mostrar a representação em HEX do scan code recebido no LCD
static volatile uint_fast8_t chave = F12; // Seleciona modo de operação no momento. F12 => reinicia o teclado
static volatile uint_fast8_t chavePS2 = 0; // Marca quando se deve alterar o estado do led
static volatile uint_fast8_t contaBITS = 11; // Bits de um byte recebido do teclado PS2
static volatile uint_fast8_t indice = 0; // Índice para meuBuffer[]
static volatile uint_fast8_t indice2 = 0; // Índice para debug na linha 2 do LCD
static volatile unsigned char caractere = 0; // Guarda o caractere recebido para tratamento em main()
static volatile unsigned char meuBuffer[KB_BUFF_SIZE] = {0x00}; // Buffer do teclado e UART
volatile BOOL argumentoPS2 = FALSE; // Avisa que será transmitido o argumento do comando para o teclado
volatile BOOL capsLock = FALSE; // Estado do led do teclado
volatile BOOL mostraCaractere = FALSE; // Avisa em main() para guardar o caractere recebino em  meuBuffer[indice], depois incrementa o indice, mostra no LCD. Evita que o caractere seja apagado logo em seguida
volatile BOOL numLock = FALSE; // Estado do led do teclado
volatile BOOL paraPS2 = FALSE; // Avisa quando é para iniciar uma transmissão de comando ao teclado
volatile BOOL paridade = FALSE; // Paridade do byte a ser transmitido para o teclado
volatile BOOL rampa = TRUE; // FALSE = rampa de descida.  TRUE = rampa de subida
volatile BOOL resposta = FALSE; // Verifica resposta das funções saída e entrada
volatile BOOL scrollLock = FALSE; // Estado do led do teclado
volatile BOOL softwareReset = FALSE; // Reseta a placa através de CTRL + ALT + DEL
volatile uint_fast8_t comandoPS2 = 0; // Comando do avr para o teclado
volatile uint_fast8_t ledPS2 = 0; // Garda o estado dos leds do teclado
unsigned char estado = OFF; // Saída ON ou OFF
unsigned char unidade = 0, dezena = 0, centena = 0, milhar = 0, total = 0; // Identifica número das saídas e entradas
// Saídas
#define		OUT1		0b00000000
#define		OUT2		0b00001000
#define		OUT3		0b00010000
#define		OUT4		0b00011000
#define		OUT5		0b00100000
#define		OUT6		0b00101000
#define		OUT7		0b00110000
#define		OUT8		0b00111000
//
#define		OUT9		0b00000001
#define		OUT10		0b00001001
#define		OUT11		0b00010001
#define		OUT12		0b00011001
#define		OUT13		0b00100001
#define		OUT14		0b00000010
#define		OUT15		0b00001010
#define		OUT16		0b00010010
//
#define		OUT17		0b00011010
#define		OUT18		0b00100010
#define		OUT19		0b00101010
#define		OUT20		0b00110010
// Entradas
#define		IN1			0b00000000
#define		IN2			0b00001000
#define		IN3			0b00010000
#define		IN4			0b00011000
#define		IN5			0b00100000
#define		IN6			0b00101000
#define		IN7			0b00110000
#define		IN8			0b00101000
//
#define		IN9			0b00000001
#define		IN10		0b00001001
#define		IN11		0b00010001
#define		IN12		0b00011001
#define		IN13		0b00100001
#define		IN14		0b00101001
#define		IN15		0b00110001
#define		IN16		0b00101001
//
#define		IN17		0b00000010
#define		IN18		0b00001010
#define		IN19		0b00010010
#define		IN20		0b00011010
#define		IN21		0b00100010
#define		IN22		0b00101010
#define		IN23		0b00110010
#define		IN24		0b00111010
//
#define		IN25		0b00000011
#define		IN26		0b00001011
#define		IN27		0b00010011
#define		IN28		0b00011011
#define		IN29		0b00100011
#define		IN30		0b00101011
#define		IN31		0b00110011
#define		IN32		0b00111011
//
#define		IN33		0b00000100
#define		IN34		0b00001100
#define		IN35		0b00010100
#define		IN36		0b00011100
#define		IN37		0b00100100
#define		IN38		0b00101100
#define		IN39		0b00110100
#define		IN40		0b00111100
// Ativado
#define		INPUT		PORTE &=~(1 << PINE2)
#define		OUTPUT		PORTE &=~(1 << PINE0)
#define		DATA		PORTB |= (1 << PINB0)
// Desativado
#define		_INPUT		PORTE |= (1 << PINE2)
#define		_OUTPUT		PORTE |= (1 << PINE0)
#define		_DATA		PORTB &=~(1 << PINB0)
//
BOOL erroDeSintaxe(unsigned char);
BOOL testaMemoriaExterna(void);
BOOL trataEntradaUsuario(void);
BOOL verificaEntrada(unsigned char);
char spiReceive(void);
static void bitDeParidade(unsigned char);
static void enviaPS2(unsigned char, int);
static void salvaBuffer(unsigned char);
unsigned char eepromRead(unsigned int);
unsigned char leDaMemoriaExterna (unsigned int);
unsigned char paraMaiuscula(unsigned char);
void beep(unsigned int);
void configura(void);
void decodificaScanCode(unsigned char);
void delay(int);
void eepromWrite(unsigned int, unsigned char);
void enviaUart0(unsigned char);
void escreveNaMemoriaExterna (unsigned int, unsigned int);
void imprimeValorDoByteNoLCD (unsigned char);
void limpaSaida(void);
void MC14499(unsigned char, unsigned char, unsigned char);
void mostraConteudoEEPROM(void);
void setaSaida(unsigned char, unsigned char);
void spi(void);
void spiInit(void);
void resetaBarramento(void);
void USART0_Init(unsigned int);
//
int main(void)
{
	BOOL cursor = TRUE; // Seleciona cursor ou espaço (apaga o cursor). Inicia mostrando o cursor
	unsigned long int tempoCursor = 0; // Tempo que o cursor permanece piscando.
	unsigned long int limitetempoCursor = 25000; // Limite do tempo que o cursor permanece piscando, varia conforme a carga
	configura();
	limpaSaida();
	resetaBarramento();
	lcd_clrscr();
	lcd_puts("marcoadeoli@");
	lcd_gotoxy(0,1);
	lcd_puts("outlook.com");
	_delay_ms(1500);
	lcd_clrscr();
	lcd_puts("KIT de desenvol-");
	lcd_gotoxy(0,1);
	lcd_puts("vimento para AVR");
	_delay_ms(1500);
	lcd_clrscr();
	lcd_puts("mega162 escravo");
	lcd_gotoxy(0,1);
	lcd_puts("mega8 mestre");
	_delay_ms(1500);
	lcd_clrscr();
	lcd_puts("Digite ? p/ exi-");
	lcd_gotoxy(0,1);
	lcd_puts("bir a ajuda");
	_delay_ms(1500);
	lcd_clrscr();
	indice = 0;	// Evita que cursor apareça no segundo lugar quando reiniciado pela segunda vez após gravação do firmware
	softwareReset = FALSE;
	for (;;)
	{
		if (mostraCaractere == TRUE)
		{
			mostraCaractere = FALSE;
			if (capsLock == TRUE)
			{
				caractere = paraMaiuscula(caractere);
			}
			lcd_putc(caractere);
			meuBuffer[indice++] = paraMaiuscula(caractere);
		}
		//
		tempoCursor++;
		if (tempoCursor > limitetempoCursor)
		{
			tempoCursor = 0;
			if (cursor == TRUE)
			{
				cursor = FALSE;
				lcd_gotoxy(indice,0);
				lcd_puts("_");
				if (blink == TRUE)
				{
					limitetempoCursor = 5000;
					for (int i = 0; i < 255; i++)
					{
						// Liga tudo
						for (int i = 0; i < 255; i++)
						{
							PORTC = i;
							OUTPUT;
							DATA;
						}
						// Volta condição original
						_OUTPUT;
						_DATA;
					}
				}
				else
				{
					limitetempoCursor = 15000;
				}
			}
			else
			{
				cursor = TRUE;
				lcd_gotoxy(indice,0);
				// Quando se aperta backspace muito rápido, ainda aparece o cursor, para evitar, usar 16 espaços vazios
				lcd_puts("                ");
				if (blink == TRUE)
				{
					limitetempoCursor = 5000;
					for (int i = 0; i < 255; i++)
					{
						// Desliga tudo
						for (int i = 0; i < 255; i++)
						{
							PORTC = i;
							OUTPUT;
							_DATA;
						}
						// Volta condição original
						_OUTPUT;
					}
				}
				else
				{
					limitetempoCursor = 15000;
				}
			}
			// Retorna o cursor novamente, caso contrário o próximo caractere vai aparecer na frente do cursor
			lcd_gotoxy(indice,0);
		}
		// Foi pressionado enter ou passou do limite do buffer
		if ((chave == ENTER) || (indice > KB_BUFF_SIZE))
		{
			lcd_gotoxy(0,1); // Apaga o caractere ^ na segunda linha que indica o erro de sintaxe após a correção. Evita que aparecam vários ^ caso haja mais de um erro
			lcd_puts("                ");
			lcd_gotoxy(indice,0);
			resposta = trataEntradaUsuario();
			if (resposta == TRUE)
			{
				resetaBarramento(); // Evita que o cursor desapareça depois de retornar da função
				lcd_clrscr();
				resposta = FALSE;
				chave = 0;
				indice = 0;
				contaBITS = 11;
				// Evita que sucessivos ENTER repitam a última ação
				for (int i = 0; i < KB_BUFF_SIZE; i++)
				{
					meuBuffer[i] = 0x00;
				}
			}
			else // Houve erro de sintaxe, retorna para correção
			{
				chave = 0;
				contaBITS = 11;
			}
			// Obrigatório reabilitar a interrupção global aqui, pois pode haver retorno antes do fim da função
			sei();
		}
		else if (chave == F12) // Reseta o teclado ao (re) iniciar o mcu ou a cada acionamento da tecla F12
		{
			/*
				0xED Control code of keyboard LED indicator
				Bit 0 : Scrollock 1 on/0 off
				Bit 1 : Numlock   1 on/0 off
				Bit 2 : Capslock  1 on/0 off
				All other bits should be 0.

				0xEE Echo command. Returns 0xEE to port 0x60 as a diagnostic test
			
				0xF0 Set alternate scan code set (PS/2 Only). This command sets the scan code set to use.
				Bit 0: Returns current scan code set
				Bit 1: Sets scan code set 1
				Bit 2: Sets scan code set 2
				Bit 3: Sets scan code set 3
				All other bits should be 0.
								
				0xF2 Send 2 byte keyboard ID code as the next two bytes
				0xF3 Set autorepeat delay and repeat rate
				0xF4 Enable keyboard
				0xF5 Reset to power on condition and wait for enable command
				0xF6 Reset to power on condition and begin scanning keyboard
				0xF7 Set all keys to autorepeat (PS/2 only)
				0xF8 Set all keys to send make code and break code (PS/2 only)
				0xF9 Set all keys to generate only make codes
				0xFA Set all keys to autorepeat and generate make/break codes
				0xFB Set a single key to autorepeat
				0xFC Set a single key to generate make and break codes
				0xFD Set a single key to generate only break codes
				0xFE Resend last result
				0xFF Reset keyboard to power on state and start self test
			*/
			chave = 0;
			ledPS2 = 0;
			numLock = FALSE;
			capsLock = FALSE;
			scrollLock = FALSE;
			enviaPS2(0xFF, 0);
// 			enviaPS2(0xF2, 70);
// 			enviaPS2(0xF0, 70);
// 			enviaPS2(2, 0);
// 			enviaPS2(0xF6, 70);
// 			enviaPS2(0xF9, 70);
			chave = NUM_LOCK; // Seta condição inicial NUM_LOCK ativo
			enviaPS2(0xED, 70); // Embora tenha lido em manual do protocolo que o mínimo é 100 ms, parece funcionar bem com 70 ms
		}
		else if (chave == F11)
		{
			chave = 0;
			enviaPS2(0xEE,70);
		}
		//
		if (chavePS2 == 1)
		{
			chavePS2 = 0;
			enviaPS2(0xED, 70); // O mínimo parece ser 70
		}
		else if (chavePS2 == 2)
		{
			chavePS2 = 0;
			enviaPS2(ledPS2, 0);
		}
		//
		if (softwareReset == TRUE) // CTRL + ALT + DEL
		{
			// Desabilita a interrupção global
			cli();
			WDTCR=0x18;
			WDTCR=0x08;
			asm("wdr");
			// Aguarda até que ocorra o reset
			while(1);
		}
	} // Fim for(;;)
}
// Evita erro de compilação de delay.h por não aceitar variável no argumento, apenas integral
void delay(int atraso)
{
	for (int i = 0; i < atraso; i++)
	{
		_delay_ms(1);
	}
}
//
unsigned char paraMaiuscula(unsigned char caractere)
{
	if ((caractere >= 97) && (caractere <= 122))
	caractere -= 32;

	return caractere;
}
//
static void enviaPS2(unsigned char byte, int atraso)
{
	// Desabilita apenas a interrupção 1
	GICR &= ~(1 << INT1);
	// Necessário atraso antes de 0xED para fixar o estado do led
	delay(atraso);
	// Coloca o byte a ser transmitido no buffer
	comandoPS2 = byte;
	// Verifica a paridade de byte
	bitDeParidade(comandoPS2);
	// Garante que começará do zero
	contaBITS = 0;
	// Avisa para setar a transmissão na ISR (INT1_vect)
	paraPS2 = TRUE;
	// Seta porta de clock para nível alto (há resistores de pull-up na placa e o teclado tem coletor aberto)
	DDRD |= (1 << clockPS2);
	// Seta pino de clock para nível baixo
	PORTD &= ~(1 << clockPS2);
	// Espera 100 us para avisar ao teclado para entrar em modo de leitura
	// Alguns sites dizem que tem que ser mais que 60 us, mas em outros 100 us, os dois funcionam. Por garantia deixei em 120 us
	// O teclado verifica essa possibilidade em média a cada 10 ms
	_delay_us(120);
	// Seta porta de dados para nível alto, mesmo caso da porta de clock. Assim será possível usar este pino para escrever na linha
	DDRD |= (1 << dadosPS2);
	// Seta pino de dados para nível baixo
	PORTD &= ~(1 << dadosPS2);
	// Libera linha de clock. Agora o teclado vai começar a transmitir o clock. Os bits serão transmitidos na rampa de subida
	DDRD &= ~(1 << clockPS2);
	// Reabilita a interrupção 1
	GICR |= (1 << INT1);
	// Aguarda até que a transmissão tenha acabado. Evita sucessivas transmissões dentro da ISR (INT1_vect)
	while (paraPS2 == TRUE)
	;
}
// Usado para debug
void imprimeValorDoByteNoLCD (unsigned char byte)
{
	unsigned char nibbleAlto = 0, nibbleBaixo = 0;
	//
	nibbleAlto = byte & 0xF0;
	nibbleAlto >>= 4;
	nibbleAlto += '0';
	//
	if (nibbleAlto > '9')
	{
		nibbleAlto += 7;
	}
	//
	nibbleBaixo = (byte & 0x0F) + '0';
	if (nibbleBaixo > '9')
	{
		nibbleBaixo += 7;
	}
	//
	lcd_gotoxy(indice2,1); // Imprime na linha de baixo
	lcd_putc(nibbleAlto);
	lcd_putc(nibbleBaixo);
	lcd_gotoxy(indice,0); // Volta o cursor à posição anterior da linha de cima
	indice2 = indice2 + 3;
	if (indice2 > 16)
	{
		indice2 = 0;
	}
}
//
void mostraConteudoEEPROM(void)
{
	resetaBarramento();
	unsigned int finalEEPROM = 0;
	unsigned int valorLidoEEPROM = (((eepromRead(0x1FF)) << 8) | (eepromRead(0x200))); // endereço máximo a ser alcançado pelo laço for(int; expression; counter)
	char strValorLidoEEPROM[9] = {0x00};
	if (valorLidoEEPROM > 0x200)
	{
		valorLidoEEPROM = 0x00; // Zera para na próxima reinicialização não mostrar nada
		lcd_puts("Buffer overflow");
		_delay_ms(500);
	}
	else // Se for um valor dentro do máximo (512 bytes), mostra o conteúdo no LCD
	{
		finalEEPROM = valorLidoEEPROM;
		for (int i = 0; i <= finalEEPROM; i++)
		{
			lcd_gotoxy(0,1);
			lcd_puts("EEPROM contents");
			utoa(i, strValorLidoEEPROM, 16);
			lcd_gotoxy(0,0);
			lcd_puts("ADR ");
			lcd_puts(strValorLidoEEPROM);
			valorLidoEEPROM = eepromRead(i);
			utoa(valorLidoEEPROM, strValorLidoEEPROM, 16);
			lcd_puts(" | HEX ");
			lcd_puts(strValorLidoEEPROM);
			lcd_puts("  "); // Apaga qualquer caractere que restou da última vizualização
			_delay_ms(125);
		}
		lcd_clrscr();
	}
}
//
static void bitDeParidade(unsigned char byte)
{
	unsigned char contador = 7;
	paridade = 1;
	
	do
	{
		if (byte & (1 << contador))
		{
			paridade ^= 1;
		}
	}	while (contador--);

	if (paridade == 1)
	{
		paridade = TRUE;
	}
}
//
void eepromWrite(unsigned int uiAddress, unsigned char ucData)
{
	// Desabilita a interrupção global
	cli();
	/* Wait for completion of previous write */
	while(EECR & (1<<EEWE))
	;
	/* Set up address and data registers */
	EEAR = uiAddress;
	EEDR = ucData;
	/* Write logical one to EEMWE */
	EECR |= (1<<EEMWE);
	/* Start eeprom write by setting EEWE */
	EECR |= (1<<EEWE);
	// Reabilita a interrupção global
	sei();
}
//
unsigned char eepromRead(unsigned int uiAddress)
{
	
	// Desabilita a interrupção global
	cli();
	/* Wait for completion of previous write */
	while(EECR & (1<<EEWE))
	;
	/* Set up address register */
	EEAR = uiAddress;
	/* Start eeprom read by writing EERE */
	EECR |= (1<<EERE);
	// Reabilita a interrupção global. Deve vir antes do return (óbvio?)
	sei();
	/* Return data from data register */
	return EEDR;
}
//
ISR (INT1_vect)
{
	static unsigned char dadoLido = 0;
	// No modo leitura a variável contaBITS é decrementada de 11 a 0
	if (rampa == FALSE) // É rampa de subida
	{
		if (contaBITS < 11 && contaBITS > 2) // Do bit 3 ao bit 10 é o byte. Bits de paridade, start e stop são ignorados.
		{
			if (paraPS2 == FALSE)
			{
				// É ignorado quando se trata de transmissão
				dadoLido = (dadoLido >> 1); // Começa pelo bit menos significante, então é necessário inverter
				if(PIND & (1 << dadosPS2))
				{
					dadoLido = dadoLido | 0b10000000; // Salva um "1" no byte na posição atual equivalente ao contaBITS
				}
			}
		}
		//
		MCUCR |= (1 << ISC11) | (1 << ISC10);
		rampa = TRUE;
	}
	else // É rampa de descida
	{
		MCUCR &= ~(1 << ISC10);
		rampa = FALSE;
		
		// No modo escrita a variável contaBITS é incrementada de 0 a 11
		if (paraPS2 == TRUE) // Sim = envia comando e argumento para o teclado na rampa de descida conforme manual do protocolo PS2
		{
			if (contaBITS < 8) // Start bit foi transmitido em enviaPS2(unsigned char)
			{
				if (comandoPS2 & (1 << contaBITS))
				{
					PORTD |= (1 << dadosPS2);
				}
				else
				{
					PORTD &= ~(1 << dadosPS2);
				}
			}
			else if (contaBITS ==  8) // Bit de paridade
			{
				if (paridade == TRUE)
				{
					PORTD |= (1 << dadosPS2);
				}
				else
				{
					PORTD &= ~(1 << dadosPS2);
				}
			}
			else if (contaBITS == 9) // Stop bit
			{
				// Stop bit = a 1
				PORTD |= (1 << dadosPS2);
				// Muda direção da porta de dados para que seja possível receber novos dados (modo de leitura no AVR)
				DDRD &= ~(1 << dadosPS2);
			}
			else if (contaBITS == 10) // ACK bit. Ignorado. Recebido pelo MCU. Por isso a alteração da direção da porta no passo 9
			{
				// Libera porta de dados do teclado
				PORTD &= ~(1 << dadosPS2);
				// Se for solicitação de alteração do estado de algum led do teclado
				if (comandoPS2 == 0xED)
				{
					argumentoPS2 = TRUE;
				}
				// A próxima interrupção será leitura e não escrita
				paraPS2 = FALSE;
			}
			//
			contaBITS++;
		}
		else if (--contaBITS == 0) // Todos os bits foram recebidos
		{
			if ((dadoLido == NUM_LOCK) || (dadoLido == CAPS_LOCK) || (dadoLido == SCROLL_LOCK))
			{
				chave = dadoLido;
				chavePS2 = 1;
			}
			else if (argumentoPS2 == TRUE)
			{
				if (chave == NUM_LOCK)
				{
					if (numLock == FALSE)
					{
						//numLock = TRUE; // Comentado. trava NUM_LOCK ligado
						ledPS2 = ledPS2 | 0b00000010;
					}
					else
					{
						numLock = FALSE;
						ledPS2 = ledPS2 & 0b00000101;
					}
				} else if (chave == CAPS_LOCK)
				{
					if (capsLock == FALSE)
					{
						capsLock = TRUE;
						ledPS2 = ledPS2 | 0b00000100;
					}
					else
					{
						capsLock = FALSE;
						ledPS2 = ledPS2 & 0b00000011;
					}
				} else if (chave == SCROLL_LOCK)
				{
					if (scrollLock == FALSE)
					{
						scrollLock = TRUE;
						ledPS2 = ledPS2 | 0b00000001;
					}
					else
					{
						scrollLock = FALSE;
						ledPS2 = ledPS2 & 0b00000110;
					}
				}

				argumentoPS2 = FALSE;
				chave = 0;
				chavePS2 = 2;
			}
			else if (dadoLido == F12)
			{
				chave = F12;
			}
			else if (dadoLido == F11)
			{
				chave = F11;
			}
			else
			{
				// Se for em main() mostra o código ASCII e não o scan code
				if (debug == TRUE)
				{
					imprimeValorDoByteNoLCD(dadoLido);
				}
				contaBITS = 11;
				decodificaScanCode(dadoLido);
			}
		}
	}
}
//
void decodificaScanCode(unsigned char sc)
{
	static unsigned char is_up = 0, shift = 0, ext=0;
	unsigned char i;

	if (!is_up)                // previous data received was the up-key identifier
	{
		switch (sc)
		{
			case 0xF0 :        // The up-key identifier
			is_up = 1;
			break;

			case 0xE0:		//do a lookup of extended keys
			ext = 1;
			break;
			
			case 0x12 :        // Left SHIFT
			shift = 1;
			break;

			case 0x59 :        // Right SHIFT
			shift = 1;
			break;
			
			default:
			
			if(ext) { //extended key lookup
				
				for(i = 0; (pgm_read_byte(&extended[i][0])!=sc) && pgm_read_byte(&extended[i][0]); i++)
				;
				if (pgm_read_byte(&extended[i][0]) == sc)
				salvaBuffer(pgm_read_byte(&extended[i][1]));
			}
			else {
				if(!shift)           // If shift not pressed, do a table look-up
				{
					for(i = 0; (pgm_read_byte(&unshifted[i][0])!=sc) && pgm_read_byte(&unshifted[i][0]); i++)
					;
					if (pgm_read_byte(&unshifted[i][0])== sc)
					salvaBuffer(pgm_read_byte(&unshifted[i][1]));
				}
				else {               // If shift pressed

					for(i = 0; (pgm_read_byte(&shifted[i][0])!=sc) && pgm_read_byte(&shifted[i][0]); i++)
					;
					if (pgm_read_byte(&shifted[i][0])== sc)
					salvaBuffer(pgm_read_byte(&shifted[i][1]));
				}
			}
		} //switch(sc)
	}
	else {			// is_up = 1
		
		is_up = 0;  // Two 0xF0 in a row not allowed
		ext=0;
		switch (sc)
		{
			case 0x12 :                        // Left SHIFT
			shift = 0;
			break;
			
			case 0x59 :                        // Right SHIFT
			shift = 0;
			break;
			
		}
	}
}
//
static void salvaBuffer(unsigned char c)
{
	// Exceto estes, qualquer caractere deve ser escrito no buffer
	if ((c != BACKSPACE) && (c != ENTER) && (c != ESC) && (c != CTRL) && (c != ALT) && (c != DEL) && (indice < 16) &&
	(c != INS) &&  (c != HOME) && (c != PGUP) && (c != END) && (c != PGDN) && (c != L_WINDOWS) && (c != L_WINDOWS) &&
	(c != U_ARROW) && (c != D_ARROW) && (c != L_ARROW) && (c != R_ARROW))
	{
		caractere = c;
		mostraCaractere = TRUE;		
	}
	// Se for backspace, retorne o cursor um espaço à esquerda. Em main(), o caractere atual será apagado
	if (c == BACKSPACE)
	{
		if (indice > 0)
		{
			indice--;
		}
	}
	// Se for enter, habilita verificaEntradaUsuario()
	else if (c == ENTER)
	{
		chave = ENTER;
	}
	//
	else if (c == ESC)
	{
		resetaBarramento();
		lcd_clrscr();
		resposta = FALSE;
		chave = 0;
		indice = 0;
		contaBITS = 11;
	}
	// Procura pela sequencia CTRL ALT DEL em qualquer posição no meuBuffer[KB_BUFF_SIZE]
	else if (c == CTRL)
	{
		softwareReset = 1;
	}
	//
	else if (c == ALT)
	{
		if (softwareReset == 1) // Já passou por CTRL, obrigatório ser ALT
		{
			softwareReset = 2;
		}
		else
		{
			softwareReset = FALSE;
		}
	}
	//
	else if (c == DEL)
	{
		if (softwareReset == 2) // Já passou por CTRL, depois ALT, obrigatório ser DEL
		{
			softwareReset = TRUE;
		}
		else
		{
			softwareReset = FALSE;
		}
	}
}
//
BOOL testaMemoriaExterna(void)
{
	cli(); // Evita que o teclado trave após chamar rotina de memória
	MCUCR |= (1 << SRE);
	char *pMem;
	char nJ;
	unsigned short nI, start;
	unsigned long size;
	//size = 0x7FF8; // apresentou erro
	//size = 0x7B01; // mínimo a apresentar erro
	size = 0x7B00; // = a 31488 * 8 = 251904 bytes - 256000 = 4096 = ?
	start = 0x0500;
	// write
	pMem = (char *)start;
	for(nI = 0, nJ = 0; nI < size; nI ++, nJ ++)
	{
		*(pMem) = nJ;
		pMem ++;
	}
	// read and test
	pMem = (char *)start;
	for(nI = 0, nJ = 0; nI < size; nI ++, nJ ++)
	{
		// simula erro no último endereço
		if (nI == 0x7AFF)
		{
			//nJ = 0xAA;
		}
		//
		if(*(pMem) != nJ)
		{
			MCUCR &= ~(1 << SRE);
			return FALSE;
		}
		pMem ++;
	}
	MCUCR &= ~(1 << SRE);
	sei(); // Evita que o teclado trave após chamar rotina de memória
	return TRUE;
}
//
void escreveNaMemoriaExterna (unsigned int endereco, unsigned int valor)
{
	cli(); // Evita que o teclado trave após chamar rotina de memória
	MCUCR |= (1 << SRE);
	char *pMem;
	pMem = (char *)endereco;
	*(pMem) = (char)valor;
	MCUCR &= ~(1 << SRE);
	sei(); // Evita que o teclado trave após chamar rotina de memória
}
//
unsigned char leDaMemoriaExterna (unsigned int endereco)
{
	cli(); // Evita que o teclado trave após chamar rotina de memória
	MCUCR |= (1 << SRE);
	char *pMem;
	pMem = (char *)endereco;
	unsigned char valor = *(pMem);
	MCUCR &= ~(1 << SRE);
	sei(); // Evita que o teclado trave após chamar rotina de memória
	return valor;
}
//
void spiInit(void)
{
	DDRB |= (1<<PINB6);
	SPCR |= (1<<SPE);
	SPDR = 0xFF;
}
//
char spiReceive(void)
{
	//SPDR = 0xFF;
	while (!(SPSR & (1<<SPIF)))
	;
	return SPDR;
}
//
void beep(unsigned int duracao)
{
	for (int a = 0; a <= duracao; a++)
	{
		PORTD |= (1 << PIND7);
		_delay_us(50);
		PORTD &=~(1 << PIND7);
		_delay_us(50);
	}
}
//
void MC14499(unsigned char dig_4, unsigned char dig32, unsigned char dig1p)
{
	//          xxxxdddd    33332222    1111xxxx
	//MC14499(0bxxxxVdDA, 0b33332222, 0b1111xxxx);
	PORTB &=~(1 << PINB6); // pino 14, enable, escrita em nível zero
	for (int a = 19; a >= 0; a--) // conta até alcançar os 20 bits de dados do MC14499DW
	{
		PORTB |= (1 << PINB7); // pino 15, clock, rampa de subida
		// pino 5, dado, if(s) chaveiam linha de dados de acordo com os bits das variáveis de entrada
		// não usado, não importa, é transmito apenas por questão de compatibilidade
		if (a ==  0) {if (0b00000001 & dig1p) PORTB |= (1 << PINB5); else PORTB &=~(1 << PINB5);}
		if (a ==  1) {if (0b00000010 & dig1p) PORTB |= (1 << PINB5); else PORTB &=~(1 << PINB5);}
		if (a ==  2) {if (0b00000100 & dig1p) PORTB |= (1 << PINB5); else PORTB &=~(1 << PINB5);}
		if (a ==  3) {if (0b00001000 & dig1p) PORTB |= (1 << PINB5); else PORTB &=~(1 << PINB5);}
		// dígito 1 vai primeiro, no display é o que fica mais à esquerda
		if (a ==  4) {if (0b00010000 & dig1p) PORTB |= (1 << PINB5); else PORTB &=~(1 << PINB5);}
		if (a ==  5) {if (0b00100000 & dig1p) PORTB |= (1 << PINB5); else PORTB &=~(1 << PINB5);}
		if (a ==  6) {if (0b01000000 & dig1p) PORTB |= (1 << PINB5); else PORTB &=~(1 << PINB5);}
		if (a ==  7) {if (0b10000000 & dig1p) PORTB |= (1 << PINB5); else PORTB &=~(1 << PINB5);}
		// dígito 2, fica no meio
		if (a ==  8) {if (0b00000001 & dig32) PORTB |= (1 << PINB5); else PORTB &=~(1 << PINB5);}
		if (a ==  9) {if (0b00000010 & dig32) PORTB |= (1 << PINB5); else PORTB &=~(1 << PINB5);}
		if (a == 10) {if (0b00000100 & dig32) PORTB |= (1 << PINB5); else PORTB &=~(1 << PINB5);}
		if (a == 11) {if (0b00001000 & dig32) PORTB |= (1 << PINB5); else PORTB &=~(1 << PINB5);}
		// dígito 3, vai por último, é o da direita
		if (a == 12) {if (0b00010000 & dig32) PORTB |= (1 << PINB5); else PORTB &=~(1 << PINB5);}
		if (a == 13) {if (0b00100000 & dig32) PORTB |= (1 << PINB5); else PORTB &=~(1 << PINB5);}
		if (a == 14) {if (0b01000000 & dig32) PORTB |= (1 << PINB5); else PORTB &=~(1 << PINB5);}
		if (a == 15) {if (0b10000000 & dig32) PORTB |= (1 << PINB5); else PORTB &=~(1 << PINB5);}
		// pontos decimais, dois no display, dois nos leds
		if (a == 16) {if (0b00000001 & dig_4) PORTB |= (1 << PINB5); else PORTB &=~(1 << PINB5);}
		if (a == 17) {if (0b00000010 & dig_4) PORTB |= (1 << PINB5); else PORTB &=~(1 << PINB5);}
		if (a == 18) {if (0b00000100 & dig_4) PORTB |= (1 << PINB5); else PORTB &=~(1 << PINB5);}
		if (a == 19) {if (0b00001000 & dig_4) PORTB |= (1 << PINB5); else PORTB &=~(1 << PINB5);}
		// aguarda
		_delay_us(6);
		PORTB &=~(1 << PINB7); // pino 15, clock, rampa de descida
		// aguarda novamente, a cada contagem do "for", 1 bit é transmitido em um pulso de clock
		_delay_us(6);
	}
	PORTB |= (1 << PINB6); // pino 14, enable, desabilita escrita em nível um
}
//
void setaSaida(unsigned char endereco, unsigned char dado)
{
	resetaBarramento();
	PORTC = endereco;
	OUTPUT;
	if (!dado)
	{
		_DATA;
	}
	else
	{
		DATA;
	}
	_OUTPUT;
	_DATA;
}
//
void limpaSaida(void)
{
	resetaBarramento();
	OUTPUT;
	_DATA;
	for (int i = 0; i < 256; i++)
	{
		PORTC = i;
	}
	_OUTPUT;
	_DATA;
}
//
BOOL verificaEntrada(unsigned char endereco)
{
	resetaBarramento();
	PORTC = endereco;
	INPUT;
	if (!(PINB & 0b10))
	{
		_INPUT;
		return TRUE;
	}
	else
	{
		_INPUT;
		return FALSE;
	}
}
//
void resetaBarramento(void)
{
	// reseta padrão para habilitar AND (74HC08) do LCD
	// necessário para entrar e sair da memória
	// evita caracteres estranhos no LCD
	// usar antes de escrever no LCD e depois de ler/escrever na memória
	PORTC |= (1 << PINC7);
	PORTD |= (1 << PIND6) | (1 << PIND7);
	// desabilita E/S
	_INPUT;
	_OUTPUT;
	_DATA;
	lcd_init();
}
//
void USART0_Init(unsigned int baud)
{
	/* Set baud rate */
	baud = (F_CPU / 2) / 16 / (baud - 1);
	UBRR0H = (unsigned char) (baud >> 8);
	UBRR0L = (unsigned char) baud;
	/* Enable receiver and transmitter */
	UCSR0B = (1 << RXEN0) | ( 1 << TXEN0);
	/* Set frame format 8 data, none, 1 stop bit */
	UCSR0C = (1 << URSEL0) | (1 << UCSZ01) | (1 << UCSZ00);
	// Interrupção
	UCSR0B |= (1 << RXCIE0);
}
//
void configura(void)
{
	// Desabilita a interrupção global
	cli();
	DDRA  = 0b11111111;
	DDRB  = 0b11101001;
	DDRC  = 0b11111111;
	DDRD  = 0b11100110;
	DDRE =  0b11111111;
	// divisor de clock, deve ficar abaixo de 8MHz para o LCD funcionar além de economizar 1 pino do uC, pois RW está aterrado
	// cristal = 11059200 / 2 = 5529600
	clock_prescale_set(clock_div_2);
	//
	// Configura heart beat por hardware, assim o led pisca sempre
	// a cada segundo, indicando que o mcu está trabalhando
	// págs. 112, 118-119, 128-131
	// Ajusta timer 1 para modo CTC, pág. 131
	TCCR1B |= (1 << WGM12);
	// Habilita timer 1 para alternar quando OCR1A
	// alcançar limite predefinido, pag. 128
	TCCR1A |= (1 << COM1A0);
	// Define o limite, prescale de 256,
	// equivale a 1/2s, a 11.0592MHz, pág. 133
	OCR1A = 5400;
	// Inicia timer 1, prescale 256, pág. 132
	TCCR1B |= (1 << CS12);
	//
	USART0_Init(9600);
	// INT1 rampa de descida
	MCUCR |= (1 << ISC11);
	// Habilita INT1
	GICR= (1 << INT1);
	// Reabilita a interrupção global
	sei();
}
//
void enviaUart0(unsigned char byte)
{
	PORTD |= (1 << PIND5);
	_delay_loop_1(2);
	while (!(UCSR0A & (1 << UDRE0)))
	;
	UDR0 = byte;
	_delay_us(2292);
	PORTD &= ~(1 << PIND5);
}
//
void limiteExcedido(void)
{
	resetaBarramento();
	lcd_clrscr();
	lcd_puts("End of line");
	_delay_ms(500);
	lcd_clrscr();
}
//
void foraDeRange(void)
{
	resetaBarramento();
	lcd_clrscr();
	lcd_puts("Range exceeded");
	_delay_ms(500);
	lcd_clrscr();
}
//
BOOL erroDeSintaxe(unsigned char posicao)
{
	// Coloca o raractere ^ na segunda linha embaixo da posição do primeiro caractere não reconhecido na sequencia
	lcd_gotoxy(posicao,1);
	lcd_puts("^");
	lcd_gotoxy(indice,0);
	return FALSE;
}
//
BOOL trataEntradaUsuario(void)
{
	// Enquanto trata a entrada do usuário evita que ocorra outra interrupção
	cli();
	if (meuBuffer[0] == 0x4F) // O
	{
		if (meuBuffer[1] == 0x55) // U
		{
			if (meuBuffer[2] == 0x54) // T
			{
				if (meuBuffer[3] != SPACE)
				{
					// Pula o número da saída por enquanto, posiçãoes [3] e [4]
					if (meuBuffer[5] == SPACE)
					{
						if (meuBuffer[6] == 0x4F) // O
						{
							if (meuBuffer[7] == 0x4E) // N
							{
								resposta = TRUE;
								estado = ON;
							}
							else
							{
								if (meuBuffer[7] == 0x46) // F
								{
									if (meuBuffer[8] == 0x46) // F
									{
										resposta = TRUE;
										estado = OFF;
									} else return erroDeSintaxe(8);
								} else return erroDeSintaxe(7);
							} // Não é necessário chamar erroDeSintaxe(char) aqui pois pode haver um F(f) na sequencia
						} else return erroDeSintaxe(6);
					} else return erroDeSintaxe(5);
				} // Não chama erroDeSintaxe(char) aqui (espaço) pois pode haver mais caracteres a serem verificados
				else // Espaço de out = out all on/off
				{
					if (meuBuffer[4] == 0x41) // A
					{
						if (meuBuffer[5] == 0x4C) // L
						{
							if (meuBuffer[6] == 0x4C) // L
							{
								// Todas as saídas, on/off
								if (meuBuffer[7] == SPACE)
								{
									if (meuBuffer[8] == 0x4F) // O
									{
										if (meuBuffer[9] == 0x4E) // N
										{
											resetaBarramento();
											// liga tudo
											for (int i = 0; i < 255; i++)
											{
												PORTC = i;
												OUTPUT;
												DATA;
											}
											// volta condição original
											_OUTPUT;
											_DATA;
											return TRUE;
										}
										else
										{
											if (meuBuffer[9] == 0x46) // F
											{
												if (meuBuffer[10] == 0x46) // F
												{
													resetaBarramento();
													// desliga tudo
													for (int i = 0; i < 255; i++)
													{
														PORTC = i;
														OUTPUT;
														_DATA;
													}
													// volta condição original
													_OUTPUT;
													return TRUE;
												} else return erroDeSintaxe(10);
											} else return erroDeSintaxe(9);
										} // Apenas retorna erroDeSintaxe(char) a partir do primeiro F(f)
									} else return erroDeSintaxe(8);
								} else return erroDeSintaxe(7);
							} else return erroDeSintaxe(6);
						}  else return erroDeSintaxe(5);
					} else return erroDeSintaxe(4);
				} // Como qualquer caractere é diferente de espaço não é necessário chamar erroDeSintaxe(char)
			} else return erroDeSintaxe(2);
			// Verifica o número da saída
			if (resposta == TRUE)
			{
				resposta = FALSE;
				unidade = ((int) meuBuffer[4] - 0x30);
				dezena = ((int) meuBuffer[3] - 0x30);
				total = (dezena * 10) + unidade;
				switch (total)
				{
					case 1:
					{
						setaSaida(OUT1, estado);
						break;
					}
					case 2:
					{
						setaSaida(OUT2, estado);
						break;
					}
					case 3:
					{
						setaSaida(OUT3, estado);
						break;
					}
					case 4:
					{
						setaSaida(OUT4, estado);
						break;
					}
					case 5:
					{
						setaSaida(OUT5, estado);
						break;
					}
					case 6:
					{
						setaSaida(OUT6, estado);
						break;
					}
					case 7:
					{
						setaSaida(OUT7, estado);
						break;
					}
					case 8:
					{
						setaSaida(OUT8, estado);
						break;
					}
					case 9:
					{
						setaSaida(OUT9, estado);
						break;
					}
					case 10:
					{
						setaSaida(OUT10, estado);
						break;
					}
					case 11:
					{
						setaSaida(OUT11, estado);
						break;
					}
					case 12:
					{
						setaSaida(OUT12, estado);
						break;
					}
					case 13:
					{
						setaSaida(OUT13, estado);
						break;
					}
					case 14:
					{
						setaSaida(OUT14, estado);
						break;
					}
					case 15:
					{
						setaSaida(OUT15, estado);
						break;
					}
					case 16:
					{
						setaSaida(OUT16, estado);
						break;
					}
					case 17:
					{
						setaSaida(OUT17, estado);
						break;
					}
					case 18:
					{
						setaSaida(OUT18, estado);
						break;
					}
					case 19:
					{
						setaSaida(OUT19, estado);
						break;
					}
					case 20:
					{
						setaSaida(OUT20, estado);
						break;
					}
					default:
					{
						foraDeRange();
						break;
					}
				} // Fim switch (total)
			} // Fim if (resposta == TRUE)
		} else return erroDeSintaxe(1);
	} // Fim OUTPUT
	else if (meuBuffer[0] == 0x49) // I
	{
		if (meuBuffer[1] == 0x4E) // N
		{
			unidade = ((int) meuBuffer[3] - 0x30);
			dezena = ((int) meuBuffer[2] - 0x30);
			total = (dezena * 10) + unidade;
			switch (total)
			{
				case 1:
				{
					resposta = verificaEntrada(IN1);
					break;
				}
				case 2:
				{
					resposta = verificaEntrada(IN2);
					break;
				}
				case 3:
				{
					resposta = verificaEntrada(IN3);
					break;
				}
				case 4:
				{
					resposta = verificaEntrada(IN4);
					break;
				}
				case 5:
				{
					resposta = verificaEntrada(IN5);
					break;
				}
				case 6:
				{
					resposta = verificaEntrada(IN6);
					break;
				}
				case 7:
				{
					resposta = verificaEntrada(IN7);
					break;
				}
				case 8:
				{
					resposta = verificaEntrada(IN8);
					break;
				}
				case 9:
				{
					resposta = verificaEntrada(IN9);
					break;
				}
				case 10:
				{
					resposta = verificaEntrada(IN10);
					break;
				}
				case 11:
				{
					resposta = verificaEntrada(IN11);
					break;
				}
				case 12:
				{
					resposta = verificaEntrada(IN12);
					break;
				}
				case 13:
				{
					resposta = verificaEntrada(IN13);
					break;
				}
				case 14:
				{
					resposta = verificaEntrada(IN14);
					break;
				}
				case 15:
				{
					resposta = verificaEntrada(IN15);
					break;
				}
				case 16:
				{
					resposta = verificaEntrada(IN16);
					break;
				}
				case 17:
				{
					resposta = verificaEntrada(IN17);
					break;
				}
				case 18:
				{
					resposta = verificaEntrada(IN18);
					break;
				}
				case 19:
				{
					resposta = verificaEntrada(IN19);
					break;
				}
				case 20:
				{
					resposta = verificaEntrada(IN20);
					break;
				}
				case 21:
				{
					resposta = verificaEntrada(IN21);
					break;
				}
				case 22:
				{
					resposta = verificaEntrada(IN22);
					break;
				}
				case 23:
				{
					resposta = verificaEntrada(IN23);
					break;
				}
				case 24:
				{
					resposta = verificaEntrada(IN24);
					break;
				}
				case 25:
				{
					resposta = verificaEntrada(IN25);
					break;
				}
				case 26:
				{
					resposta = verificaEntrada(IN26);
					break;
				}
				case 27:
				{
					resposta = verificaEntrada(IN27);
					break;
				}
				case 28:
				{
					resposta = verificaEntrada(IN28);
					break;
				}
				case 29:
				{
					resposta = verificaEntrada(IN29);
					break;
				}
				case 30:
				{
					resposta = verificaEntrada(IN30);
					break;
				}
				case 31:
				{
					resposta = verificaEntrada(IN31);
					break;
				}
				case 32:
				{
					resposta = verificaEntrada(IN32);
					break;
				}
				case 33:
				{
					resposta = verificaEntrada(IN33);
					break;
				}
				case 34:
				{
					resposta = verificaEntrada(IN34);
					break;
				}
				case 35:
				{
					resposta = verificaEntrada(IN35);
					break;
				}
				case 36:
				{
					resposta = verificaEntrada(IN36);
					break;
				}
				case 37:
				{
					resposta = verificaEntrada(IN37);
					break;
				}
				case 38:
				{
					resposta = verificaEntrada(IN38);
					break;
				}
				case 39:
				{
					resposta = verificaEntrada(IN39);
					break;
				}
				case 40:
				{
					resposta = verificaEntrada(IN40);
					break;
				}
				default:
				{
					resposta = ESC;
					foraDeRange();
					break;
				}
			} // Fim switch (total)
			if (resposta == TRUE)
			{
				resetaBarramento();
				lcd_puts("ON");
				_delay_ms(500);
				lcd_clrscr();
			}
			else if (resposta == FALSE)
			{
				resetaBarramento();
				lcd_puts("OFF");
				_delay_ms(500);
				lcd_clrscr();
			}
		} else return erroDeSintaxe(1);
	} // Fim INPUT
	else if (meuBuffer[0] == 0x4D) // M
	{
		if (meuBuffer[1] == 0x45) // E
		{
			if (meuBuffer[2] == 0x4D) // M
			{
				if (meuBuffer[3] == SPACE)
				{
					if (meuBuffer[4] == 0x54) // T
					{
						if (meuBuffer[5] == 0x53) // S
						{
							if (meuBuffer[6] == 0x54) // T
							{
								resetaBarramento();
								lcd_clrscr();
								lcd_puts("Test 256k extram");
								lcd_gotoxy(0,1);
								lcd_puts("0x0500 -> 0x7B00");
								_delay_ms(1000);
								resposta = testaMemoriaExterna();
								resetaBarramento();
								lcd_clrscr();
								if (resposta == TRUE)
								{
									lcd_puts("Extern SRAM OK");
								}
								else if (resposta == FALSE)
								{
									lcd_puts("Extern SRAM NOK");
								}
								_delay_ms(1000);
								lcd_clrscr();
							} else return erroDeSintaxe(6);
						} else return erroDeSintaxe(5);
					} // Não verificar se é T(t) aqui pois pode ter sido digitado apenas valores em HEX
					else // Se houver dado escreve na memória no endereço especificado
					{
						char strEndereco[5] = {0x00};
						for (int i = 0; i < 4; i++)
						{
							// O endereço está na posição 4 a 7
							strEndereco[i] = meuBuffer[i + 4];
						}
						unsigned int endereco;
						if ((sscanf(strEndereco, "%x", &endereco)) != 1)
						{
							resetaBarramento();
							lcd_clrscr();
							lcd_puts("Address error");
							_delay_ms(500);
							lcd_clrscr();
							return TRUE; // Evita que endereço não existente apareça no LCD
						}
						// Verifica se está dentro do endereço de memória externa para este MCU
						if ((endereco < 0x0500) || (endereco > 0x7B00))
						{
							foraDeRange();
						}
						// Verifica se houve dado digitado
						else if (indice > 8) // posição do espaço depois do endereço
						{
							char strValorParaEscrever[2] = {0x00};
							// 01234567891
							//           0
							// mem 7B00 FF
							// O dado está na posição 9 a 10
							strValorParaEscrever[0] = meuBuffer[9];
							strValorParaEscrever[1] = meuBuffer[10];
							unsigned int valorParaEscrever;
							if ((sscanf(strValorParaEscrever, "%x", &valorParaEscrever)) != 1)
							{
								resetaBarramento();
								lcd_clrscr();
								lcd_puts("Parameter error");
								_delay_ms(500);
								lcd_clrscr();
							}
							else
							{
								// OK, pode escrever
								escreveNaMemoriaExterna(endereco, valorParaEscrever);
							}
						}
						else // Não foi digitado nada depois do endereço, então leia para a variável valor e exiba no LCD
						{
							unsigned char valorLido = leDaMemoriaExterna(endereco);
							//
							//comandoPS2 = valorLido; // Envia comando para o teclado PS2. Necessário setar tecla na ISR
							//
							// 5 (cinco) é o tamanho mínimo para evitar buffer overflow quando converter para binário.
							// 9 (nove) 8 bits mais 1 para o caractere terminador. Só por precaução.
							// Esse cuidado evita, por exemplo, que o teclado pare de funcionar momentaneamente após chamar esta utoa.
							char strValorLido[9] = {0x00};
							resetaBarramento();
							utoa(valorLido, strValorLido, 16);
							lcd_clrscr();
							lcd_puts(strValorLido);
							lcd_puts(" hex | ");
							utoa(valorLido, strValorLido, 10);
							lcd_puts(strValorLido);
							lcd_puts(" dec");
							utoa(valorLido, strValorLido, 2);
							lcd_gotoxy(0,1);
							lcd_puts(strValorLido);
							lcd_puts(" bin");
							_delay_ms(1000);
							lcd_clrscr();
						}
					}  // Fim else Se houver dado escreve na memória no endereço especificado
				} else return erroDeSintaxe(3);
			} else return erroDeSintaxe(2);
		} else return erroDeSintaxe(1);
	} // Fim SRAM externa
	else if (meuBuffer[0] == 0x45) // E
	{
		if (meuBuffer[1] == 0x45) // E
		{
			if (meuBuffer[2] == 0x50) // P
			{
				if (meuBuffer[3] == 0x52) // R
				{
					if (meuBuffer[4] == 0x4F) // O
					{
						if (meuBuffer[5] == 0x4D) // M
						{
							if (meuBuffer[6] == SPACE)
							{
								if (meuBuffer[7] == 0x43) // C
								{
									if (meuBuffer[8] == 0x4C) // L
									{
										if (meuBuffer[9] == 0x52) // R
										{
											resetaBarramento();
											lcd_clrscr();
											lcd_puts("Clearing EEPROM");
											lcd_gotoxy(0,1);
											lcd_puts("0x00 -> 0x200");
											_delay_ms(1000);
											for (int i = 0; i <= 0x200; i++)
											{
												eepromWrite(i, 0x00);
											}
										} else return erroDeSintaxe(9);
									} else return erroDeSintaxe(8);
								} // Não verificar se é C(c) aqui pois pode ter sido digitado apenas valores em HEX, ou outro comando a verificar
								else // Será que é SHOW contents?
								if (meuBuffer[7] == 0x53) // S
								{
									if (meuBuffer[8] == 0x48) // H
									{
										if (meuBuffer[9] == 0x4F) // O
										{
											if (meuBuffer[10] == 0x57) // W
											{
												mostraConteudoEEPROM();
											} else return erroDeSintaxe(10);
										} else return erroDeSintaxe(9);
									} else return erroDeSintaxe(8);
								} // Não verificar se é C(c) aqui pois pode ter sido digitado apenas valores em HEX, ou outro comando a verificar
								else // Se houver dado escreve na memória no endereço especificado
								{
									char strEndereco[4] = {0x00};
									for (int i = 0; i < 3; i++)
									{
										// O endereço está na posição 7 a 9
										strEndereco[i] = meuBuffer[i + 7];
									}
									unsigned int endereco;
									if ((sscanf(strEndereco, "%x", &endereco)) != 1)
									{
										resetaBarramento();
										lcd_clrscr();
										lcd_puts("Address error");
										_delay_ms(500);
										lcd_clrscr();
										return TRUE; // Evita que endereço não existente apareça no LCD
									}
									// Verifica se está dentro do endereço de memória EEPROM para este MCU
									if ((endereco < 0x00) || (endereco > 0x200))
									{
										foraDeRange();
									}
									// Verifica se houve dado digitado
									else if (indice > 10) // posição do espaço depois do endereço
									{
										char strValorParaEscrever[2] = {0x00};
										// 01234567891
										//           0
										// mem 7B00 FF
										// O dado está na posição 11 a 12
										strValorParaEscrever[0] = meuBuffer[11];
										strValorParaEscrever[1] = meuBuffer[12];
										unsigned int valorParaEscrever;
										if ((sscanf(strValorParaEscrever, "%x", &valorParaEscrever)) != 1)
										{
											resetaBarramento();
											lcd_clrscr();
											lcd_puts("Parameter error");
											_delay_ms(500);
											lcd_clrscr();
										}
										else
										{
											// OK, pode escrever
											eepromWrite(endereco, valorParaEscrever);
										}
									}
									else // Não foi digitado nada depois do endereço, então leia para a variável valor e exiba no LCD
									{
										unsigned char valorLido = eepromRead(endereco);
										// 5 (cinco) é o tamanho mínimo para evitar buffer overflow quando converter para binário.
										// 9 (nove) 8 bits mais 1 para o caractere terminador. Só por precaução.
										// Esse cuidado evita, por exemplo, que o teclado pare de funcionar momentaneamente após chamar esta utoa.
										char strValorLido[9] = {0x00};
										resetaBarramento();
										utoa(valorLido, strValorLido, 16);
										lcd_clrscr();
										lcd_puts(strValorLido);
										lcd_puts(" hex | ");
										utoa(valorLido, strValorLido, 10);
										lcd_puts(strValorLido);
										lcd_puts(" dec");
										utoa(valorLido, strValorLido, 2);
										lcd_gotoxy(0,1);
										lcd_puts(strValorLido);
										lcd_puts(" bin");
										_delay_ms(1000);
										lcd_clrscr();
									}
								}  // Fim else Se houver dado escreve na memória no endereço especificado
							} else return erroDeSintaxe(6);
						} else return erroDeSintaxe(5);
					} else return erroDeSintaxe(4);
				} else return erroDeSintaxe(3);
			} else return erroDeSintaxe(2);
		} else return erroDeSintaxe(1);
	} // Fim EEPROM
	else if (meuBuffer[0] == 0x3F) // ?
	{
		resetaBarramento();
		lcd_clrscr();
		lcd_puts("?");
		lcd_gotoxy(0,1);
		lcd_puts("Esta ajuda");
		_delay_ms(1000);
		lcd_clrscr();
		lcd_puts("MAIUSCULA ou");
		lcd_gotoxy(0,1);
		lcd_puts("minuscula");
		_delay_ms(1000);
// 		lcd_clrscr();
// 		lcd_puts("RESET");
// 		lcd_gotoxy(0,1);
// 		lcd_puts("Reinicia KitAVR");
// 		_delay_ms(1000);
		lcd_clrscr();
		lcd_puts("debug on/OFF");
		lcd_gotoxy(0,1);
		lcd_puts("Mostra scan code");
		_delay_ms(1000);
		lcd_clrscr();
		lcd_puts("OUTxx ON/OFF");
		lcd_gotoxy(0,1);
		lcd_puts("min 01 max 20");
		_delay_ms(1000);
		lcd_clrscr();
		lcd_puts("INxx");
		lcd_gotoxy(0,1);
		lcd_puts("min 01 max 40");
		_delay_ms(1000);
		lcd_clrscr();
		lcd_puts("Digite os nume-");
		lcd_gotoxy(0,1);
		lcd_puts("ros em xx");
		_delay_ms(1000);
		lcd_clrscr();
		lcd_puts("out all on/off");
		lcd_gotoxy(0,1);
		lcd_puts("Todas as saidas");
		_delay_ms(1000);
		lcd_clrscr();
		lcd_puts("MEM TST");
		lcd_gotoxy(0,1);
		lcd_puts("Testa extram");
		_delay_ms(1000);
		lcd_clrscr();
		lcd_puts("MEM HHHH HH");
		lcd_gotoxy(0,1);
		lcd_puts("WR  ADR  DATA");
		_delay_ms(1000);
		lcd_clrscr();
		lcd_puts("MEM HHHH");
		lcd_gotoxy(0,1);
		lcd_puts("RD  ADR");
		_delay_ms(1000);
		lcd_clrscr();
		lcd_puts("EEPROM SHOW");
		lcd_gotoxy(0,1);
		lcd_puts("Mostra conteudo");
		_delay_ms(1000);
		lcd_clrscr();
		lcd_puts("EEPROM clr");
		lcd_gotoxy(0,1);
		lcd_puts("Limpa conteudo");
		_delay_ms(1000);
		lcd_clrscr();
		lcd_puts("EEPROM HHH HH");
		lcd_gotoxy(0,1);
		lcd_puts("WR  ADR  DATA");
		_delay_ms(1000);
		lcd_clrscr();
		lcd_puts("EEPROM 1FF HH");
		lcd_gotoxy(0,1);
		lcd_puts("MSB contador");
		_delay_ms(1000);
		lcd_clrscr();
		lcd_puts("EEPROM 200 HH");
		lcd_gotoxy(0,1);
		lcd_puts("LSB contador");
		_delay_ms(1000);
		lcd_clrscr();
		lcd_puts("Digite os valo-");
		lcd_gotoxy(0,1);
		lcd_puts("res em HEX");
		_delay_ms(1000);
		lcd_clrscr();
		lcd_puts("blink on/off");
		lcd_gotoxy(0,1);
		lcd_puts("Pisca as saidas");
		_delay_ms(1000);
	} // Fim ajuda
// 	else if (meuBuffer[0] == 0x52) // R
// 	{
// 		if (meuBuffer[1] == 0x45) // E
// 		{
// 			if (meuBuffer[2] == 0x53) // S
// 			{
// 				if (meuBuffer[3] == 0x45) // E
// 				{
// 					if (meuBuffer[4] == 0x54) // T
// 					{
// 						softwareReset = TRUE; // Equivale a CTRL + ALT + DEL
// 					} else return erroDeSintaxe(4);
// 				} else return erroDeSintaxe(3);
// 			} else return erroDeSintaxe(2);
// 		} else return erroDeSintaxe(1);
// 	} // Fim reset
	else if (meuBuffer[0] == 0x42) // B
	{
		if (meuBuffer[1] == 0x4C) // L
		{
			if (meuBuffer[2] == 0x49) // I
			{
				if (meuBuffer[3] == 0x4E) // N
				{
					if (meuBuffer[4] == 0x4B) // K
					{
						if (meuBuffer[5] == SPACE)
						{
							if (meuBuffer[6] == 0x4F) // O
							{
								if (meuBuffer[7] == 0x4E) // N
								{
									// Ativa saídas piscantes na velocidade do cursor
									blink = TRUE;
								}
								else
								{
									if (meuBuffer[7] == 0x46) // F
									{
										if (meuBuffer[8] == 0x46) // F
										{
											// Desativa saídas piscantes na velocidade do cursor
											blink = FALSE;
										} else return erroDeSintaxe(8);
									} else return erroDeSintaxe(7);
								} // Apenas retorna erroDeSintaxe(char) a partir do primeiro F(f)
							} else return erroDeSintaxe(6);
						} else return erroDeSintaxe(5);
					} else return erroDeSintaxe(4);
				} else return erroDeSintaxe(3);
			} else return erroDeSintaxe(2);
		} else return erroDeSintaxe(1);
	} // Fim blink
	else if (meuBuffer[0] == 0x44) // D
	{
		if (meuBuffer[1] == 0x45) // E
		{
			if (meuBuffer[2] == 0x42) // B
			{
				if (meuBuffer[3] == 0x55) // U
				{
					if (meuBuffer[4] == 0x47) // G
					{
						if (meuBuffer[5] == SPACE)
						{
							if (meuBuffer[6] == 0x4F) // O
							{
								if (meuBuffer[7] == 0x4E) // N
								{
									debug = TRUE;
								}
								else
								{
									if (meuBuffer[7] == 0x46) // F
									{
										if (meuBuffer[8] == 0x46) // F
										{
											debug = FALSE;
										} else return erroDeSintaxe(8);
									} else return erroDeSintaxe(7);
								} // Apenas retorna erroDeSintaxe(char) a partir do primeiro F(f)
							} else return erroDeSintaxe(6);
						} else return erroDeSintaxe(5);
					} else return erroDeSintaxe(4);
				} else return erroDeSintaxe(3);
			} else return erroDeSintaxe(2);
		} else return erroDeSintaxe(1);
	} // Fim debug
	// Continuar com próxima cadeia de if else
	//else if (meuBuffer[0] == 0xHH)hh)) // Caractere
	//{
	// (..)
	//}
	else
	{
		return erroDeSintaxe(0);
	}
	// Retorna sempre TRUE a menos que dentro da função outro if retorne FALSE
	return TRUE;
}
//
ISR(USART0_RXC_vect)
{
	// Desabilita a interrupção global
	cli();
	// Espera até receber o byte completo
	while (!(UCSR0A & (1 << RXC0)))
	;
	meuBuffer[indice] = UDR0;
	if (meuBuffer[indice] == ENTER)
	{
		chave = ENTER;
	}
	else
	{
		// Exceto estes, qualquer caractere deve ser escrito no LCD e deve ser incrementado o indice
		if ((meuBuffer[indice] != BACKSPACE) && (meuBuffer[indice] != ENTER) && (meuBuffer[indice] != ESC) &&
			(meuBuffer[indice] != CTRL) && (meuBuffer[indice] != ALT) && (meuBuffer[indice] != DEL) &&
			(meuBuffer[indice] != INS) && (meuBuffer[indice] != HOME) && (meuBuffer[indice] != PGUP) &&
			(meuBuffer[indice] != END) && (meuBuffer[indice] != PGDN) && (meuBuffer[indice] != L_WINDOWS) &&
			(meuBuffer[indice] != R_WINDOWS) && (meuBuffer[indice] != U_ARROW) && (meuBuffer[indice] != D_ARROW) &&
			(meuBuffer[indice] != L_ARROW) && (meuBuffer[indice] != R_ARROW) && (indice < 16))
		{
			lcd_putc(meuBuffer[indice++]);
		}
		// Se for backspace, retorne o cursor um espaço à esquerda. Em main(), o caractere atual será apagado
		else
		{
			// Limite esquerdo do LCD
			if (indice > 0)
			{
				indice--;
			}
		}
	}
	// Reabilita a interrupção global
	sei();
}
