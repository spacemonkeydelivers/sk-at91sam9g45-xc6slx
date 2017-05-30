/*
 *  ecb_at91.h - AT91RM9200 Programmer
 *  This programmer uses AT91' GPIO lines
 *
 *  2006 by Carlos Camargo
 *  2007.May.10 Andres Calderon, generalized
 */

#ifndef ECB_AT91_H
#define ECB_AT91_H


#define MAP_SIZE 4096Ul
#define MAP_MASK (MAP_SIZE - 1)

#define PIOA_BASE 0xFFFFF200
#define PIOB_BASE 0xFFFFF400
#define PIOC_BASE 0xFFFFF600
#define PIOD_BASE 0xFFFFF800

#define PIOM_BASE 0x10000088
#define PIOM1_BASE (0x10000088 | 1<<19)

typedef /*volatile */unsigned int AT91_REG;
/* Hardware register definition */

typedef struct _AT91S_PIO {
    AT91_REG PIO_PER;           /* PIO Enable Register */
    AT91_REG PIO_PDR;           /* PIO Disable Register */
    AT91_REG PIO_PSR;           /* PIO Status Register */
    AT91_REG Reserved0[1];
    AT91_REG PIO_OER;           /* Output Enable Register */
    AT91_REG PIO_ODR;           /* Output Disable Registerr */
    AT91_REG PIO_OSR;           /* Output Status Register */
    AT91_REG Reserved1[1];
    AT91_REG PIO_IFER;          /* Input Filter Enable Register */
    AT91_REG PIO_IFDR;          /* Input Filter Disable Register */
    AT91_REG PIO_IFSR;          /* Input Filter Status Register */
    AT91_REG Reserved2[1];
    AT91_REG PIO_SODR;          /* Set Output Data Register */
    AT91_REG PIO_CODR;          /* Clear Output Data Register */
    AT91_REG PIO_ODSR;          /* Output Data Status Register */
    AT91_REG PIO_PDSR;          /* Pin Data Status Register */
    AT91_REG PIO_IER;           /* Interrupt Enable Register */
    AT91_REG PIO_IDR;           /* Interrupt Disable Register */
    AT91_REG PIO_IMR;           /* Interrupt Mask Register */
    AT91_REG PIO_ISR;           /* Interrupt Status Register */
    AT91_REG PIO_MDER;          /* Multi-driver Enable Register */
    AT91_REG PIO_MDDR;          /* Multi-driver Disable Register */
    AT91_REG PIO_MDSR;          /* Multi-driver Status Register */
    AT91_REG Reserved3[1];
    AT91_REG PIO_PPUDR;         /* Pull-up Disable Register */
    AT91_REG PIO_PPUER;         /* Pull-up Enable Register */
    AT91_REG PIO_PPUSR;         /* Pad Pull-up Status Register */
    AT91_REG Reserved4[1];
    AT91_REG PIO_ASR;           /* Select A Register */
    AT91_REG PIO_BSR;           /* Select B Register */
    AT91_REG PIO_ABSR;          /* AB Select Status Register */
    AT91_REG Reserved5[9];
    AT91_REG PIO_OWER;          /* Output Write Enable Register */
    AT91_REG PIO_OWDR;          /* Output Write Disable Register */
    AT91_REG PIO_OWSR;          /* Output Write Status Register */
} AT91S_PIO, *AT91PS_PIO;

void pio_out(AT91S_PIO * pio, int mask, int val);

int pio_in(AT91S_PIO * pio, int mask);

AT91S_PIO *pio_map(unsigned int piobase);

void pio_enable(AT91S_PIO * pio, int mask);

void pio_output_enable(AT91S_PIO * pio, int mask);

void pio_input_enable(AT91S_PIO * pio, int mask);

void pio_disable_irq(AT91S_PIO * pio, int mask);

void pio_disable_multiple_driver(AT91S_PIO * pio, int mask);

void pio_disable_pull_ups(AT91S_PIO * pio, int mask);

void pio_synchronous_data_output(AT91S_PIO * pio, int mask);

#endif
