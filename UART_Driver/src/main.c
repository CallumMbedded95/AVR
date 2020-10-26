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
	
 	while (1) {
 		if (ReturnReceiveBufferTail()) { // yes this is bad, we fix later *****
 			initialiseTx();
 			transmitBytes(Bit_Reverse(ReturnReceiveBufferTail())); // just prints out received byte
 			//transmitBytes('j'); // Toggle for transmit test
			_delay_ms(500);
 		}
  	}	
 
  	return 0;
}