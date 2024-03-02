/*
 * SPIMaster.c
 *
 * Created: 21.05.2020 19:30:28
 *  Author: Eray Ulutas
 */ 

#include "SPIMaster.h"
#include <avr/io.h>
#include <stdbool.h>

void spiMasterInit() {
	/* Set MOSI and SCK output, all others input */
	DDR_SPI = (1 << PIN_MOSI) | (1 << PIN_SCK) | (1 << PIN_SS);
	PORT_SPI |= (1 << PIN_SS);	// deactivate slave.
	
	/*
		Enable SPI, Master, set clock rate fck/4, mode 0,0 (0 polarity, rising edge)
		fck/4 is fastest recommended transmission speed (ref. atmega8 ds.)
	*/
	SPCR = (1<<SPE) | (1<<MSTR);
}

unsigned char spiMasterTx(unsigned char data) {
	SPDR = data;
	/*
		Wait till transmission is complete.
		Since our transmission fast enough(fck/4).
		I think polling is the best choice.
	*/
	while(!(SPSR & (1 << SPIF)));
	
	return SPDR;	// SPDR register exchanged with slave.
}
