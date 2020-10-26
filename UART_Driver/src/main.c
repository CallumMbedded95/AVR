//********** USI_UART_Tx functions **********//
// Author: Callum McNeish

#include <avr/io.h>
#include <util/delay.h>
#include "UART.h" 
 
int main (void)
{
	flushBuffers();
	//initialiseTx();
	initialiseRx();
	
 	while (1) {
 		if (returnTestB_()) {
 			initialiseTx();
 			transmitBytes(Bit_Reverse(returnTestB_()));
 			//transmitBytes('j');
			_delay_ms(500);
 		}
 		 //transmitBytes('e');

 		 //_delay_ms(500);

 		// transmitBytes('c');

 		// _delay_ms(500);

 		// transmitBytes('k');

 		// _delay_ms(500);

 		// transmitBytes('o');

 		// _delay_ms(500);
  	}	
 
  	return 0;
}