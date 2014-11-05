/*
The software part of DSP_INS is free software; you can redistribute it and/or modify 
it under the terms of the GNU General Public License version 3 as published by the 
Free Software Foundation. The hardware PCB design files (Protel DXP 2004) are also 
provided.

	By VrilleUp (vrilleup.pu@gmail.com), 2008.08.14
*/

/*
	McBSP config to SPI mode
*/

#ifndef MCBSP_SPI_H
#define MCBSP_SPI_H

extern void init_mcbspa_spi(void);
extern void init_mcbspb_spi(void);
extern Uint16 mcbspa_spi_send8bit(Uint16 data1);
extern Uint16 mcbspb_spi_send8bit(Uint16 data1);

#endif
