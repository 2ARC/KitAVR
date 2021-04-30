#ifndef EXT_KEYS_H
#define EXT_KEYS_H

//extended keys
// A sequencia 0xFn serve apenas para confirma��o do recebimento do caractere pelo MCU no LCD
#define HOME		0xF0
#define END			0xF1
#define DEL			0xF1
#define INS			0xF3
#define PGDN		0xF4
#define PGUP		0xF5
#define U_ARROW		0xF6
#define D_ARROW		0xF7
#define L_ARROW		0xF8
#define R_ARROW		0xF9
#define DIV			0xFA
#define CTRL		0xFB
#define ALT			0xFC
#define L_WINDOWS	0xFD
#define R_WINDOWS	0xFE
// S�o pr�-processadas em main()
#define	ACK			0xFA
#define	ESC			0x1B
#define	ENTER		0x0D
#define SPACE		0x20
#define	BACKSPACE	0x08
#define NUM_LOCK	0x77 // scancode
#define CAPS_LOCK	0x58 // scancode
#define SCROLL_LOCK	0x7E // scancode

#endif
