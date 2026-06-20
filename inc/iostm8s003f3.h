#ifndef IOSTM8S003F3_H
#define IOSTM8S003F3_H

#include <stdint.h>

/* 8-bit bitfield overlay for bit-level register access */
typedef volatile struct {
  uint8_t b0:1, b1:1, b2:1, b3:1, b4:1, b5:1, b6:1, b7:1;
} __bits8_t;

/* ============== PORT C ============== */
volatile uint8_t __at(0x500A) _PC_ODR;
volatile uint8_t __at(0x500B) _PC_IDR;
volatile uint8_t __at(0x500C) _PC_DDR;
volatile uint8_t __at(0x500D) _PC_CR1;
volatile uint8_t __at(0x500E) _PC_CR2;

volatile __bits8_t __at(0x500B) _PC_IDR_bits;
volatile __bits8_t __at(0x500C) _PC_DDR_bits;
volatile __bits8_t __at(0x500D) _PC_CR1_bits;
volatile __bits8_t __at(0x500E) _PC_CR2_bits;

#define PC_IDR       _PC_IDR
#define PC_IDR_IDR4  _PC_IDR_bits.b4
#define PC_DDR       _PC_DDR
#define PC_DDR_DDR4  _PC_DDR_bits.b4
#define PC_CR1       _PC_CR1
#define PC_CR1_C14   _PC_CR1_bits.b4
#define PC_CR2       _PC_CR2
#define PC_CR2_C24   _PC_CR2_bits.b4

/* ============== PORT D ============== */
volatile uint8_t __at(0x500F) _PD_ODR;
volatile uint8_t __at(0x5010) _PD_IDR;
volatile uint8_t __at(0x5011) _PD_DDR;
volatile uint8_t __at(0x5012) _PD_CR1;
volatile uint8_t __at(0x5013) _PD_CR2;

volatile __bits8_t __at(0x500F) _PD_ODR_bits;
volatile __bits8_t __at(0x5011) _PD_DDR_bits;
volatile __bits8_t __at(0x5012) _PD_CR1_bits;
volatile __bits8_t __at(0x5013) _PD_CR2_bits;

#define PD_ODR       _PD_ODR
#define PD_ODR_ODR2  _PD_ODR_bits.b2
#define PD_ODR_ODR3  _PD_ODR_bits.b3
#define PD_DDR_DDR2  _PD_DDR_bits.b2
#define PD_DDR_DDR3  _PD_DDR_bits.b3
#define PD_CR1_C12   _PD_CR1_bits.b2
#define PD_CR1_C13   _PD_CR1_bits.b3
#define PD_CR2_C22   _PD_CR2_bits.b2
#define PD_CR2_C23   _PD_CR2_bits.b3

/* ============== SPI ============== */
volatile uint8_t __at(0x5200) _SPI_CR1;
volatile __bits8_t __at(0x5200) _SPI_CR1_bits;
volatile uint8_t __at(0x5203) _SPI_SR;
volatile __bits8_t __at(0x5203) _SPI_SR_bits;
volatile uint8_t __at(0x5204) _SPI_DR;

#define SPI_CR1      _SPI_CR1
#define SPI_CR1_SPE  _SPI_CR1_bits.b6
#define SPI_CR1_MSTR _SPI_CR1_bits.b2
#define SPI_SR       _SPI_SR
#define SPI_SR_TXE   _SPI_SR_bits.b1
#define SPI_SR_RXNE  _SPI_SR_bits.b0
#define SPI_DR       _SPI_DR

/* ============== UART1 ============== */
volatile uint8_t __at(0x5230) _UART1_SR;
volatile __bits8_t __at(0x5230) _UART1_SR_bits;
volatile uint8_t __at(0x5231) _UART1_DR;
volatile uint8_t __at(0x5232) _UART1_BRR1;
volatile uint8_t __at(0x5233) _UART1_BRR2;
volatile uint8_t __at(0x5235) _UART1_CR2;
volatile __bits8_t __at(0x5235) _UART1_CR2_bits;

#define UART1_SR      _UART1_SR
#define UART1_SR_TXE  _UART1_SR_bits.b7
#define UART1_SR_TC   _UART1_SR_bits.b6
#define UART1_DR      _UART1_DR
#define UART1_BRR1    _UART1_BRR1
#define UART1_BRR2    _UART1_BRR2
#define UART1_CR2     _UART1_CR2
#define UART1_CR2_TEN _UART1_CR2_bits.b3

/* ============== TIM2 ============== */
volatile uint8_t __at(0x5300) _TIM2_CR1;
volatile __bits8_t __at(0x5300) _TIM2_CR1_bits;
volatile uint8_t __at(0x5307) _TIM2_PSCR;
volatile uint8_t __at(0x530B) _TIM2_ARRH;
volatile uint8_t __at(0x530C) _TIM2_ARRL;

#define TIM2_CR1     _TIM2_CR1
#define TIM2_CR1_CEN _TIM2_CR1_bits.b0
#define TIM2_CR1_OPM _TIM2_CR1_bits.b3
#define TIM2_PSCR    _TIM2_PSCR
#define TIM2_ARRH    _TIM2_ARRH
#define TIM2_ARRL    _TIM2_ARRL

/* ============== IWDG ============== */
volatile uint8_t __at(0x50E0) _IWDG_KR;
volatile uint8_t __at(0x50E1) _IWDG_PR;
volatile uint8_t __at(0x50E2) _IWDG_RLR;

#define IWDG_KR  _IWDG_KR
#define IWDG_PR  _IWDG_PR
#define IWDG_RLR _IWDG_RLR

/* ============== RST ============== */
volatile uint8_t __at(0x50B3) _RST_SR;
volatile __bits8_t __at(0x50B3) _RST_SR_bits;

#define RST_SR         _RST_SR
#define RST_SR_EMCF    _RST_SR_bits.b0
#define RST_SR_SWIMF   _RST_SR_bits.b1
#define RST_SR_ILLOPF  _RST_SR_bits.b2
#define RST_SR_IWDGF   _RST_SR_bits.b3
#define RST_SR_WWDGF   _RST_SR_bits.b4

/* ============== CLK ============== */
volatile uint8_t __at(0x50C0) _CLK_CKDIVR;
volatile uint8_t __at(0x50C7) _CLK_PCKENR1;
volatile uint8_t __at(0x50C8) _CLK_PCKENR2;

#define CLK_CKDIVR   _CLK_CKDIVR
#define CLK_PCKENR1  _CLK_PCKENR1
#define CLK_PCKENR2  _CLK_PCKENR2

#endif