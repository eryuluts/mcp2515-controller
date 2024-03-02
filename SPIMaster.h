/*
 * SPI.h
 *
 * Created: 21.05.2020 18:02:13
 *  Author: Eray Ulutas
 */ 


#ifndef SPI_H_
#define SPI_H_
#include <avr/io.h>
#include <stdbool.h>


#define DDR_SPI		DDRB					// Data direction register for port with SPI
#define PORT_SPI	PORTB					// Port with SPI
#define PIN_MOSI	PB3						// MOSI pin on the PORTB_SPI
#define PIN_MISO	PB4						// MISO pin on the PORTB_SPI
#define PIN_SCK		PB5						// SCK pin on the PORTB_SPI
#define PIN_SS		PB2						// SS pin on the PORTB_SPI


/*
	We need a SPI Master library to interface with mcp2515's spi
	interface.
*/

void spiMasterInit();
unsigned char spiMasterTx(unsigned char data);	// SPI always makes a duplex transmission.
#define SPIMasterBegin	PORT_SPI &= ~(1 << PIN_SS);
#define SPIMasterEnd	PORT_SPI |= (1 << PIN_SS);

#endif /* SPI_H_ */