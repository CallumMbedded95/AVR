// main.c
// 
// A simple blinky program for ATtiny85
// Connect red LED at pin 2 (PB3)
//
// electronut.in

#include <avr/io.h>
#include <util/delay.h>
 
 
int main (void)
{
  // set PB3 to be output
	DDRB = 0b00001000;
  while (1) {
    
    // flash# 1:
    // set PB3 high
    PORTB = 0b00001000; 
    _delay_ms(20);
    // set PB3 low
    PORTB = 0b00000000;
    _delay_ms(20);

    // flash# 2:
    // set PB3 high
    PORTB = 0b00001000; 
    _delay_ms(200);
    // set PB3 low
    PORTB = 0b00000000;
    _delay_ms(200);
  }
 
  return 1;
}
