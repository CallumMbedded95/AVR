//********** USI_UART_Tx functions **********//
// Author: Callum McNeish

#include <avr/io.h>
#include <util/delay.h>
#include "UART.h" 
 
int main (void)
{
	flushBuffers();
	//initialiseTx(); // Used for pure Tx solution
	initialiseRx();
	DDRB |= 1<<PB3;
 	while (1) {
 		// if (ReturnReceiveBufferTail()) { // yes this is bad, we fix later *****
 		// 	initialiseTx();
 		// 	transmitBytes(Bit_Reverse(ReturnReceiveBufferTail())); // just prints out received
			// _delay_ms(500);
 		// }
 		if (DataInReceiveBuffer()) {
 			initialiseTx();
 			transmitBytes(Bit_Reverse(ReturnReceiveBufferTail())); // just prints out received
			while (!isIdle());
			initialiseRx();
 		}
  	}	
 
  	return 0;
}