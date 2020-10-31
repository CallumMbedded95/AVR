//********** USI_UART_Tx functions **********//
// Author: Callum McNeish

#include <avr/io.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "UART.h" 
 
int main (void)
{
	flushBuffers(); // reset buffers and initialise receiver
	initialiseRx();
	
 	while (1) {
 		// transmitBytes('a');
 		// _delay_ms(50);
 		// transmitBytes('b');
 		// _delay_ms(50);
 		// transmitBytes('c');
 		// _delay_ms(50);
 		// transmitBytes('d');
 		// _delay_ms(1500);
 		while (DataInReceiveBuffer()) {
 			transmitBytes(Bit_Reverse(ReturnReceiveBufferTail())); // just prints out received
 			_delay_ms(20);
 		}
 		//_SLEEP(); // wait for pin change interrupt
  	}	
 
  	return 0;
}