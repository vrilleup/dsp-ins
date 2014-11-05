/*
The software part of DSP_INS is free software; you can redistribute it and/or modify 
it under the terms of the GNU General Public License version 3 as published by the 
Free Software Foundation. The hardware PCB design files (Protel DXP 2004) are also 
provided.

	By VrilleUp (vrilleup.pu@gmail.com), 2008.08.14
*/

/*
	AT45DB642D.h
	AT45DB642D resources
*/
#ifndef __AT45_H
#define __AT45_H

//Status Register Format for AT45DB642D
//7			6		5	4	3	2	1	0
//RDY/BUSY	COMP   	1	1	1	1	X	1(1024)
#define FLASH_STAT  0xBD  //Flash status
#define PAGE_SIZE   1024   //page size
#define FLASH_PAGES 8191  //pages
#define FLASH_BLOCKS   1024  //blocks
#define FLASH_BYTE   8387584  //Flash bytes

#endif

