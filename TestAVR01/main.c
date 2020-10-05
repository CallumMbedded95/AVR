//********** USI_UART_Tx functions **********//
// Author: Callum McNeish

#include <avr/io.h>
#include <util/delay.h>
#include "USI.h" 

// LED STUFF
#define LED_DDR 	DDRB
#define LED_PORT 	PORTB
#define LED_PINS	PINB
#define LED_01		3
#define LED_02		4

 
int main (void)
{
	initialiseTx();

 	while (1) {
 		transmitBytes(0b01101000);

 		_delay_ms(500);

 		transmitBytes('e');

 		_delay_ms(500);

 		transmitBytes('l');

 		_delay_ms(500);

 		transmitBytes('l');

 		_delay_ms(500);

 		transmitBytes('o');

 		_delay_ms(500);
	// set PB3 low
	// // Try PORTB |= (1 << LED01) || (1 << LED02) ,  etc
 //    PORTB = (1 << LED_01); 
 //    _delay_ms(150);

 //    PORTB = 0x0;
 //    _delay_ms(20);

 //    PORTB = (1 << LED_02);  
 //    _delay_ms(50);

 //    PORTB |= (1 << LED_01);
 //    _delay_ms(5000);

 //    PORTB = 0x00;
 //    _delay_ms(1000);
  	}	
 
  	return 0;
}