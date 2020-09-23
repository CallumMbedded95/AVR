// main.c
// 
// A simple blinky program for ATtiny85
// Connect red LED at pin 2 (PB3)
//
// electronut.in

#include <avr/io.h>
#include <util/delay.h>
 
#define LED_DDR 	DDRB
#define LED_PORT 	PORTB
#define LED_PINS	PINB
#define LED_01		3
#define LED_02		4
 
int main (void)
{
  // set PB3 to be output
	//LED_DDR = 0b00011000; // Binary format
	LED_DDR = 0x3f; // Hex format = 0b00111111
	// LED_DDR = 0b00000000;
	// LED_DDR = (1 < LED_01);
	// LED_DDR |= (1 < LED_02);
  while (1) {
	// set PB3 low
    PORTB = (1 << LED_01); 
    _delay_ms(150);

    PORTB = 0x0;
    _delay_ms(20);

    PORTB = (1 << LED_02);  
    _delay_ms(50);

    PORTB |= (1 << LED_01);
    _delay_ms(100);

    PORTB = 0x00;
    _delay_ms(1000);
  }
 
  return 0;
}
