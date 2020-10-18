//********** USI_UART_Tx functions **********//
// Author: Callum McNeish

#include <avr/io.h>
#include <util/delay.h>
#include "USI_Tx.h" 
 
int main (void)
{
	flushBuffers();
	//initialiseTx();
	initialiseRx();
	
 	while (1) {
 		// if (returnTestB()) {
 		// 	initialiseTx();
 		// 	transmitBytes('e');
			// _delay_ms(500);
 		// }
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