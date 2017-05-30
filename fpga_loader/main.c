/*
 * main.cpp
 *
 *  Created on: 30.09.2010
 *      Author: leen
 */
//-----------------------------------------------------------------------------
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <wchar.h>
#include "at91gpio.h"
//-----------------------------------------------------------------------------
#define DONE (1<<29)
#define CCLK (1<<10)
#define DIN  (1<<11)
#define PROG (1<<30)
//-----------------------------------------------------------------------------
int main (int argc, char ** argv)
{
	AT91S_PIO *pio;
	AT91S_PIO *pioM;
	AT91S_PIO *pioM1;
	int file;
	uint8_t *buf, byte, fl;
	uint32_t size, len, i, pdsr;
	int j;
	//

//	pioM = pio_map(PIOM_BASE); //set config pins M0=M1=1
//	pioM1 = pio_map(PIOM1_BASE); //additional set INIT_B to high for M0,M1 sample

	pio = pio_map(PIOC_BASE);
	printf("pio mapped\n");

//	pioM->PIO_PER = PROG;
//	pioM->PIO_PER = PROG;
//	pioM1->PIO_PER = PROG;

	pio->PIO_MDER = PROG;
	pio->PIO_OER = CCLK | DIN | PROG;
	pio->PIO_ODR = DONE;
	pio->PIO_PER = DONE | CCLK | DIN | PROG;
	printf("pins preset done\n");

	pio->PIO_SODR = CCLK | DIN | PROG;
	pio->PIO_CODR = PROG;
	usleep(10);
	pio->PIO_SODR = PROG;
	printf("prog pulse done, wait for init\n");
	usleep(3000);

	file = open(argv[1], O_RDONLY);
	if (file < 0)
	{
		perror("can't open input file");
		return 1;
	}
	size = lseek(file, 0, SEEK_END);
	printf("input file size %lu\n", size);
	lseek(file, 0, SEEK_SET);

	buf = NULL;
	buf = (uint8_t *) malloc(size);
	if (!buf)
	{
		perror("buffer allocation error");
		close(file);
		return 1;
	}
	printf("buffer allocated\n");
	len = read(file, buf, size);
	if (len != size)
	{
		perror("cannot read input file\n");
		close(file);
		free(buf);
		return 1;
	}
	close(file);
	printf("file read and closed\n");

	printf("start main loop\n");
	for (i = 0; i < len; i++)
	{
		byte = buf[i];
		for (j = 7; j >= 0; j--)
		{
			fl = 1<<j;
			fl &= byte;
			if (fl) pio->PIO_SODR = DIN;
			else pio->PIO_CODR = DIN;
			pio->PIO_SODR = CCLK;
			pio->PIO_CODR = CCLK;
		}
		j = i/1024;
		if (j*1024 == i)
			printf("loaded %d kB\r", j);
	}
	printf("\nmain loop finished\n");

	pio->PIO_SODR = DIN;
	i = 0;
	pdsr = pio->PIO_PDSR & DONE;
	while(!pdsr)
	{
		pio->PIO_SODR = CCLK;
		pio->PIO_CODR = CCLK;
		pdsr = pio->PIO_PDSR & DONE;
		if (i++ > 8*1024)
		{
			printf("error while start device, timeout\n");
			break;
		}
	}
	if (pdsr)
		printf("FPGA started\n");
	for (i = 0; i<6; i++)
	{
		pio->PIO_SODR = CCLK;
		pio->PIO_CODR = CCLK;
	}
	printf("start sequence completed\n");
	free(buf);
	printf("buffer freed\n");

//	pio->PIO_CODR = CCLK | DIN | PROG;

//	pio->PIO_ODR = DONE | CCLK | DIN | PROG;
//	pio->PIO_PER = DONE | CCLK | DIN | PROG;
//	printf("pins reset done\n");
	return 0;
}
//-----------------------------------------------------------------------------
