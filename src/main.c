#include <stdint.h>
#include <stdio.h>
#include "iostm8s003f3.h"
#include "pindef.h"

#define RF_CHANNEL              99

#define USE_IRQ                 1

#define CSN_LOW()               PD_ODR_ODR2 = 0;
#define CSN_HIGH()              PD_ODR_ODR2 = 1;
                                
#define CE_LOW()                PD_ODR_ODR3 = 0;
#define CE_HIGH()               PD_ODR_ODR3 = 1;
                                
#define RUN_CPU_FAST()          CLK_CKDIVR = 0
#define RUN_CPU_NORMAL()        CLK_CKDIVR = CKDIVR_HSIDIV_8 | CKDIVR_CPUDIV_1
#define RUN_CPU_SLOW()          CLK_CKDIVR = CKDIVR_HSIDIV_8 | CKDIVR_CPUDIV_128

#define NRF24L01_DETECTED       1
#define NRF24L01_NOT_DETECTED   0

// Peripheral clock gating register 1 (CLK_PCKENR1) bits
#define CLK_I2C                 (1 << 0)
#define CLK_SPI                 (1 << 1)
#define CLK_UART2               (1 << 2)
#define CLK_UART1               (1 << 3)
#define CLK_TIM4                (1 << 4)
#define CLK_TIM2                (1 << 5)
#define CLK_TIM3                (1 << 6)
#define CLK_TIM1                (1 << 7)
// Peripheral clock gating register 2 (CLK_PCKENR2) bits
#define CLK_AWU                 (1 << 2)
#define CLK_ADC                 (1 << 3)

#define PCLK_DISABLE            0

#define CLOCK_ENABLE(PERIPHERIAL)   CLK_PCKENR1 |= CLK_##PERIPHERIAL
#define CLOCK_DISABLE(PERIPHERIAL)  CLK_PCKENR1 &= ~CLK_##PERIPHERIAL

#define ENABLE_PCLOCK() CLK_PCKENR1 = (    \
          0 * CLK_I2C   |                  \
          1 * CLK_SPI   |                  \
          0 * CLK_UART2 |                  \
          1 * CLK_UART1 |                  \
          0 * CLK_TIM4  |                  \
          1 * CLK_TIM2  |                  \
          0 * CLK_TIM3  |                  \
          1 * CLK_TIM1                     \
        );                                 \
        CLK_PCKENR2 = PCLK_DISABLE

#define CKDIVR_HSIDIV_1         0    
#define CKDIVR_HSIDIV_2         (1 << 3)
#define CKDIVR_HSIDIV_4         (2 << 3)
#define CKDIVR_HSIDIV_8         (3 << 3)
    
#define CKDIVR_CPUDIV_1         0
#define CKDIVR_CPUDIV_2         1
#define CKDIVR_CPUDIV_4         2
#define CKDIVR_CPUDIV_8         3
#define CKDIVR_CPUDIV_16        4
#define CKDIVR_CPUDIV_32        5
#define CKDIVR_CPUDIV_64        6
#define CKDIVR_CPUDIV_128       7

#define NRF24_CONFIG_REG        0x00
#define NRF24_RF_CH_REG         0x05
#define NRF24_RF_SETUP_REG      0x06
#define NRF24_STATUS_REG        0x07
#define NRF24_TX_ADDR_REG       0x10
#define NRF24_FIFO_STATUS_REG   0x17
#define NRF24_RX_ADDR_P0_REG    0x0A
#define NRF24_DYNPD_REG         0x1C
#define NRF24_FEATURE_REG       0x1D 
          
// 00: CONFIG register bits from 0 t0 6, bit 7 must be set to 0
// ============================================================
#define NRF24_CONFIG_PRIM_RX    (1 << 0)
#define NRF24_CONFIG_PWR_UP     (1 << 1)
#define NRF24_CONFIG_CRCO       (1 << 2)
#define NRF24_CONFIG_EN_CRC     (1 << 3)
#define NRF24_CONFIG_MAX_RT     (1 << 4)
#define NRF24_CONFIG_TX_DS      (1 << 5)
#define NRF24_CONFIG_RX_DR      (1 << 6)

// 07: STATUS -- Status Register (In parallel to the SPI command
//               word applied on the MOSI pin, the STATUS register
//               is shifted serially out on the MISO pin)   (bits from 0 to 6)
// ===============================================================
#define NRF24_STATUS_MAX_RT     NRF24_CONFIG_MAX_RT
#define NRF24_STATUS_TX_DS      NRF24_CONFIG_TX_DS
#define NRF24_STATUS_RX_DR      NRF24_CONFIG_RX_DR

// 06: RF_SETUP -- RF Setup Register (bits from 0 to 6)
// ====================================================
#define NRF24_RF_SETUP_RF_PWR_0         (1 << 0)
#define NRF24_RF_SETUP_RF_PWR_1         (1 << 1)
#define NRF24_RF_SETUP_RF_DR_HIGH       (1 << 3)
#define NRF24_RF_SETUP_RF_DR_LOW        (1 << 5)
          
#define NRF24_FIFO_STATUS_RX_EMPTY      (1 << 0)
#define NRF24_FIFO_STATUS_TX_FULL       (1 << 5)
#define NRF24_FIFO_STATUS_TX_EMPTY      (1 << 4)
        
#define NRF24_FEATURE_EN_DYN_ACK        (1 << 0)
#define NRF24_FEATURE_EN_DPL            (1 << 2)

          
#define NRF24_R_REGISTER        0x00
#define NRF24_W_REGISTER        0x20
#define NRF24_REGISTER_MASK     0x1F
#define NRF24_R_RX_PL_WID       0x60
#define NRF24_R_RX_PAYLOAD      0x61
#define NRF24_W_TX_PAYLOAD      0xA0
          
#define NRF24_FLUSH_RX          0xE2
#define NRF24_FLUSH_TX          0xE1

#define NRF24_NOP               0xFF          
          
#define OPEN_UART()              CLOCK_ENABLE(UART1)         
#define CLOSE_UART()             while (UART1_SR_TC == 0); CLOCK_DISABLE(UART1)

#define UART_PUTC(C)	                                        \
  while (UART1_SR_TXE == 0);                                    \
  UART1_DR = C

uint8_t static inline uart_puts(char *s) {
  while (*s != 0) {
    UART_PUTC(*s++);
  }
  return !0;
}

#define uprintf(...) for(char _b[100]; snprintf(_b, sizeof(_b), __VA_ARGS__), uart_puts(_b), 0;){}

void static inline initUART(void) {
  
  //  Div= 16MHz/115200 = 0x8B -> Brr2= 0xB, Brr1= 0x8
  //  UART1_BRR2=0x0B;
  //  UART1_BRR1=0x08;
  
  UART1_BRR2=0x01;
  UART1_BRR1=0x01;
  UART1_CR2_TEN=1;    // Enable transmitter
  // UART1_CR2_REN=1;   // Enable receiver
}

void static inline print_hex(uint8_t c) {
  if (c < 10) {
    c += '0';
  } else {
    c += ('A' - 10);
  }
  while (UART1_SR_TXE == 0);
  UART1_DR = c;
}

void static inline print_str(char *c) {
  while (*c != 0) {
    while (UART1_SR_TXE == 0);
    UART1_DR = *c++;
  };
}

//void static inline print_char(char c) {
//  while (UART1_SR_TXE == 0);
//  UART1_DR = c;
//}

void static inline printHex(uint8_t buf[], uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    print_hex(buf[i] >> 4);
    print_hex(buf[i] & 0x0F);
  };
}

void static inline initGPIO(void) {

  // IRQ pin
  PIN_MODE(C, 4, INPUT + FLOAT);
  
  // CSN pin, default is high
  PIN_WRITE(D, 2, HIGH);
  PIN_MODE(D, 2, OUTPUT + PUSH_PULL + LOW_SPEED);
  
  // CE pin, default is low
  PIN_WRITE(D, 3, LOW);
  PIN_MODE(D, 3, OUTPUT + PUSH_PULL + LOW_SPEED);
}

// Initialize TIM2 to get overflow about ervery 15 sec. (fMASTER / 2^12 / 7325)
void static inline initTIM2(void) {
  TIM2_PSCR = 0x0C;   //  Prescaler = 2^12.
  TIM2_ARRH = 0x1C;   //  High byte of 7325
  TIM2_ARRL = 0x9D;   //  Low byte of 7325
  TIM2_CR1_OPM = 1;   //  One Pulse Mode
}

// Initialize SPI as master.
void static inline initSPI() {
  SPI_CR1_BR = 0;     //  fmaster / 2 (1,000 000 baud).
  SPI_CR1_MSTR = 1;   //  Master device.
  SPI_CR1_SPE = 1;    //  Enable SPI.
}

// Bidirectional SPI transfer function
uint8_t static inline spi(uint8_t tx) {
  while (SPI_SR_TXE != 1);
  SPI_DR = tx;
  while (SPI_SR_RXNE != 1);
  return SPI_DR;
}

// Send command to NRF24L01 module
uint8_t static inline nrf_write_cmd(uint8_t cmd) {
  CSN_LOW();
  uint8_t s = spi(cmd);
  CSN_HIGH();
  return s;
}

// Read NRF24L01's register
uint8_t static inline nrf_read_register(uint8_t regNo) {
  CSN_LOW();
  spi(NRF24_R_REGISTER | (NRF24_REGISTER_MASK & regNo));
  uint8_t r = spi(NRF24_NOP);
  CSN_HIGH();
  return r;
}

// Write a value to NRF24L01's register
uint8_t static inline nrf_write_register(uint8_t regNo, uint8_t regVal) {
  CSN_LOW();
  uint8_t s = spi(NRF24_W_REGISTER | (NRF24_REGISTER_MASK & regNo));
  spi(regVal);
  CSN_HIGH();
  return s;
}

// 
uint8_t static inline nrf_init(uint8_t channel) {

  nrf_write_register(0x03, 1);       //  REG 0x03 - RX/TX Address width
                                     //  0 = Illegal
                                     //  1 = 3 bytes
                                     //  2 = 4 bytes
                                     //  3 = 5 bytes
  // nrf_write_register(EN_AA, 0);
  nrf_write_register(0x05, channel); // REG 0x05 -- radio channel no
/*  
  nrf_write_register(0x06,           // REG 0x06 -- data rate and pwr lvl
      //  Set RF output power in TX mode
      //  0x00 = -18dBm
      //  0x02 = -12dBm
      //  0x04 = -6dBm
      //  0x06 =  0dBm
                     
      0x06                       |
                     
      //  Select between the high speed data rates.
      //  0x00 – 1Mbps
      //  0x08 – 2Mbps
      //  0x20 – 250kbps
        
      0x08
  );                     
*/  
  nrf_write_register(NRF24_FEATURE_REG,    // set payload parameters
    1 * NRF24_FEATURE_EN_DYN_ACK  |        // enable W_TX_PAYLOAD_NOACK command
    1 * NRF24_FEATURE_EN_DPL               // enable dynamic payload
  );
  
  return nrf_write_register(NRF24_DYNPD_REG,  1); // enable dynamic payload for pipe 0

}

uint8_t static inline nrf_start_receiving(void) {
  uint8_t nrf_status;

  CE_LOW();

  nrf_write_register(NRF24_STATUS_REG,     // clear status bits
    1 * NRF24_STATUS_MAX_RT       |
    1 * NRF24_STATUS_TX_DS        |
    1 * NRF24_STATUS_RX_DR
  );

  //nrf_write_register(RX_PW_P0, 8);
  
  nrf_write_cmd(NRF24_FLUSH_RX);       
  nrf_write_cmd(NRF24_FLUSH_TX);       

  nrf_status = nrf_write_register(NRF24_CONFIG_REG,     // power up receiver
    1 * NRF24_CONFIG_PRIM_RX      |  // select RX mode
    1 * NRF24_CONFIG_PWR_UP       |  // turn power on
    1 * NRF24_CONFIG_EN_CRC       |  // enable CRC
    1 * NRF24_CONFIG_CRCO         |  // CRC encoding scheme: '0' - 1 byte, '1' – 2 bytes
    1 * NRF24_CONFIG_MAX_RT       |  // MAX_RT interrupt not reflected on the IRQ pin
    1 * NRF24_CONFIG_TX_DS        |  // TX_DS interrupt not reflected on the IRQ pin
    0 * NRF24_CONFIG_RX_DR           // Reflect RX_DR as active low interrupt on the IRQ pin
  );
  CE_HIGH();  // start receiving
  return nrf_status;
}

//uint8_t static inline nrf_stop_receiving(void) {
//  CE_LOW();
//  uint8_t rx_status = nrf_write_register(CONFIG,     // power down receiver
//    0 * NRF24_CONFIG_PRIM_RX      |  // select TX mode
//    0 * NRF24_CONFIG_PWR_UP       |  // turn power off
//    1 * NRF24_CONFIG_EN_CRC       |  // enable CRC. Forced high if one of the bits in the
//                                     // EN_AA is high
//    1 * NRF24_CONFIG_CRCO         |  // CRC encoding scheme: '0' - 1 byte, '1' – 2 bytes
//    1 * NRF24_CONFIG_MAX_RT       |  // MAX_RT interrupt not reflected on the IRQ pin
//    1 * NRF24_CONFIG_TX_DS        |  // TX_DS interrupt not reflected on the IRQ pin
//    1 * NRF24_CONFIG_RX_DR           // RX_DR interrupt not reflected on the IRQ pin
//  );
//
//  nrf_write_cmd(FLUSH_RX);       
//
//  return rx_status;
//}

uint8_t static inline nrf_get_payload_size(void) {
  CSN_LOW();
  spi(NRF24_R_RX_PL_WID);  // 0x60 = "Read RX Payload Width" command
  uint8_t psize = spi(NRF24_NOP);
  CSN_HIGH();
  return psize;
} 

void static inline nrf_get_payload(uint8_t buf[], uint8_t len) {
  
  // The RX_DR IRQ is asserted by a new packet arrival event. 
  // The procedure for handling this interrupt should be: 
  //    1) read payload through SPI, 
  //    2) clear RX_DR IRQ, 
  //    3) read FIFO_STATUS to check if there are more
  //       payloads available in RX FIFO, 
  //    4) if there are more data in RX FIFO, repeat from step 1).  
  
  CSN_LOW();

  spi(NRF24_R_RX_PAYLOAD);  // 0x61 = "Read RX Payload" CMD

  for (int i = 0; i < len; i++) {
   buf[i] = spi(NRF24_NOP);
  }

  CSN_HIGH();
}

void static inline initWatchdog(void) {
  // Watchdog timeout period (LSI clock frequency = 128 kHz)
  //=========================================================
  //    Prescaler  |  PR[2:0] |          Timeout       
  //     divider   |   bits   | RL[7:0]= 0x00 RL[7:0]= 0xFF
  //===============+==========+==============================
  //       /4           0         62.5 µs       15.90 ms
  //       /8           1          125 µs       31.90 ms
  //      /16           2          250 µs       63.70 ms
  //      /32           3          500 µs         127 ms
  //      /64           4         1.00 ms         255 ms
  //     /128           5         2.00 ms         510 ms
  //     /256           6         4.00 ms         1.02 s
  //=========================================================  
  //
  // 		*** IWDOG INITIALIZATION ***
  //
  IWDG_KR  = 0xCC;  // enable and start the wdog counter at first!
  IWDG_KR  = 0x55;  // unlock wdog configuration registers
  IWDG_PR  = 0x03;  // set prescaler value
  IWDG_RLR = 0x3F;  // set timeout value
  IWDG_KR  = 0xAA;  // lock wdog registers & reload the wdog counter  
}

void static inline printResetStatus(void) {
  if (RST_SR != 0) {
    print_str("\nReset source:");
    
    if (RST_SR_WWDGF == 1) {
      RST_SR_WWDGF = 1;    // reset Window Watchdog bit
      print_str(" WWDG");
    }
  
    if (RST_SR_IWDGF == 1) {
      RST_SR_IWDGF = 1;    // reset Independent Watchdog bit
      print_str(" IWDG");
    }

    if (RST_SR_ILLOPF == 1) {
      RST_SR_ILLOPF = 1;   // reset Illegal OpCode bit
      print_str(" ILLOP");
    }
  
    if (RST_SR_SWIMF == 1) {
      RST_SR_SWIMF = 1;    // reset SWIM bit
      print_str(" SWIM");
    }

    if (RST_SR_EMCF == 1) {
      RST_SR_EMCF = 1;     // reset EMC bit
      print_str(" EMC");
    }
    print_str("\n");
  }
}

uint8_t static inline nrf_detect(void) {
  
  nrf_write_cmd(NRF24_FLUSH_TX);
  
  uint8_t fifo_status = nrf_read_register(NRF24_FIFO_STATUS_REG);
  if ((fifo_status & NRF24_FIFO_STATUS_TX_EMPTY) != NRF24_FIFO_STATUS_TX_EMPTY) {
    return NRF24L01_NOT_DETECTED;
  }
  if ((fifo_status & NRF24_FIFO_STATUS_TX_FULL) == NRF24_FIFO_STATUS_TX_FULL) {
    return NRF24L01_NOT_DETECTED;
  }

  // send dummy 1-byte payload to the 1st FIFO buffer
  CSN_LOW();
  spi(NRF24_W_TX_PAYLOAD);
  spi(0);     
  CSN_HIGH();
  
  fifo_status = nrf_read_register(NRF24_FIFO_STATUS_REG);
  if ((fifo_status & NRF24_FIFO_STATUS_TX_EMPTY) == NRF24_FIFO_STATUS_TX_EMPTY) {
    return NRF24L01_NOT_DETECTED;
  }

  if ((fifo_status & NRF24_FIFO_STATUS_TX_FULL) == NRF24_FIFO_STATUS_TX_FULL) {
    return NRF24L01_NOT_DETECTED;
  }
  
  // send dummy 1-byte payload to the 2nd FIFO buffer
  CSN_LOW();
  spi(NRF24_W_TX_PAYLOAD);
  spi(0); 
  CSN_HIGH();
  
  fifo_status = nrf_read_register(NRF24_FIFO_STATUS_REG);
  if ((fifo_status & NRF24_FIFO_STATUS_TX_EMPTY) == NRF24_FIFO_STATUS_TX_EMPTY) {
    return NRF24L01_NOT_DETECTED;
  }

  if ((fifo_status & NRF24_FIFO_STATUS_TX_FULL) == NRF24_FIFO_STATUS_TX_FULL) {
    return NRF24L01_NOT_DETECTED;
  }

  // send dummy 1-byte payload to the 3rd FIFO buffer
  CSN_LOW();
  spi(NRF24_W_TX_PAYLOAD);
  spi(0);
  CSN_HIGH();

  fifo_status = nrf_read_register(NRF24_FIFO_STATUS_REG);
  if ((fifo_status & NRF24_FIFO_STATUS_TX_EMPTY) == NRF24_FIFO_STATUS_TX_EMPTY) {
    return NRF24L01_NOT_DETECTED;
  }

  if ((fifo_status & NRF24_FIFO_STATUS_TX_FULL) != NRF24_FIFO_STATUS_TX_FULL) {
    return NRF24L01_NOT_DETECTED;
  }

  nrf_write_cmd(NRF24_FLUSH_TX);
  
  fifo_status = nrf_read_register(NRF24_FIFO_STATUS_REG);
  if ((fifo_status & NRF24_FIFO_STATUS_TX_EMPTY) != NRF24_FIFO_STATUS_TX_EMPTY) {
    return NRF24L01_NOT_DETECTED;
  }

  if ((fifo_status & NRF24_FIFO_STATUS_TX_FULL) == NRF24_FIFO_STATUS_TX_FULL) {
    return NRF24L01_NOT_DETECTED;
  }
  
  return NRF24L01_DETECTED;
}  

void swap_endianess(uint8_t src[], uint8_t dst[]) {
  dst[0] = src[3];
  dst[1] = src[2];
  dst[2] = src[1];
  dst[3] = src[0];
}

typedef struct __attribute__((packed)) {
  uint32_t p : 20;
  int32_t  t : 14;
  uint32_t h : 14;
} pld_t;

int main(void) {

  uint16_t v_bat;
  int32_t temp;
  uint32_t press, hum;
  uint8_t payload_buf[32];

  CLK_CKDIVR = 0;
  IWDG_KR= 0xAA;	   // wdog refresh
  
  ENABLE_PCLOCK();
  
  initWatchdog();
  initGPIO();
  initSPI();
  initUART();
  initTIM2();

  RUN_CPU_NORMAL();
  
  uprintf("NRF24L01 Receiver started.\n");
  printResetStatus();
  
  while (nrf_detect() == NRF24L01_NOT_DETECTED) {
    print_str("No NRF24L01 detected. Wait 15 sec...\n");
    TIM2_CR1_CEN = 1;           // run timer 2
    while (TIM2_CR1_CEN != 0) { // while until TIM2 overflow
      IWDG_KR= 0xAA;		// wdog refresh
    }
  }
  
  CLOCK_DISABLE(TIM2);      // stop TIM2 clocking

  nrf_init(RF_CHANNEL);

  uint8_t c1 = nrf_read_register(NRF24_RF_SETUP_REG); // get RF settings
  uint8_t c2 = nrf_read_register(NRF24_CONFIG_REG);   // get config
  uprintf("NRF24L01: CH = %u, PWR = %ddBm, BR = %s, CRC = %s\n", 
    nrf_read_register(NRF24_RF_CH_REG),
    (c1 & (NRF24_RF_SETUP_RF_PWR_0  | NRF24_RF_SETUP_RF_PWR_1)) * 6 - 18,
    ((c1 & NRF24_RF_SETUP_RF_DR_LOW) == NRF24_RF_SETUP_RF_DR_LOW) ? "256k" : (((c1 & NRF24_RF_SETUP_RF_DR_HIGH) == NRF24_RF_SETUP_RF_DR_HIGH) ? "2M" : "1M"),
    ((c2 & NRF24_CONFIG_EN_CRC) == 0) ? "NO" : ((c2 & NRF24_CONFIG_CRCO) == NRF24_CONFIG_CRCO) ? "16" : "8"
  )
    
  uprintf("RX ADDR: ");
    
  CSN_LOW();
  spi(NRF24_R_REGISTER | (NRF24_REGISTER_MASK & NRF24_RX_ADDR_P0_REG));
  for (uint8_t i = 0; i < 5; i++) {
    uint8_t b = spi(NRF24_NOP);
    print_hex(b >> 4);
    print_hex(b & 0x0F);
  }
  CSN_HIGH();

  uprintf("\nTX ADDR: ");  

  CSN_LOW();
  spi(NRF24_R_REGISTER | (NRF24_REGISTER_MASK & NRF24_TX_ADDR_REG));
  for (uint8_t i = 0; i < 5; i++) {
    uint8_t b = spi(NRF24_NOP);
    print_hex(b >> 4);
    print_hex(b & 0x0F);
  }
  CSN_HIGH();
  
  uprintf("\n");  
  
  CLOSE_UART();
  
  nrf_start_receiving();
  
  while (1) {              // ================= main loop ===============
    
    while(PC_IDR_IDR4 != 0) {           // while IRQ line stands high
      IWDG_KR= 0xAA;	                // wdog refresh
    }
    CLOCK_ENABLE(SPI);                  // start SPI clocking
    
    if ((nrf_read_register(NRF24_FIFO_STATUS_REG) & NRF24_FIFO_STATUS_RX_EMPTY) == NRF24_FIFO_STATUS_RX_EMPTY) {
      nrf_write_register(NRF24_STATUS_REG, 0x70); // clear status bits
      CLOCK_DISABLE(SPI);               // stop SPI clocking
      continue;
    }
      
    nrf_write_register(NRF24_STATUS_REG, 0x40); // clear RX_DR bit, release IRQ line
    
    uint8_t pSize = nrf_get_payload_size();
    if (pSize > 32) {
      nrf_write_register(NRF24_STATUS_REG, 0x70); // clear status bits
      nrf_write_cmd(0xE2);              // 0xE2 = clear RX buffer
      CLOCK_DISABLE(SPI);               // stop SPI clocking
      continue;
    }
    
    nrf_get_payload(payload_buf, pSize);
    
    CLOCK_DISABLE(SPI);                 // stop SPI clocking
    
    swap_endianess((uint8_t *)&payload_buf[0], (uint8_t *)&press);
    swap_endianess((uint8_t *)&payload_buf[4], (uint8_t *)&temp);
    swap_endianess((uint8_t *)&payload_buf[8], (uint8_t *)&hum);
    v_bat = payload_buf[13] << 8 | payload_buf[12];
    IWDG_KR= 0xAA;	              // wdog refresh
    
    OPEN_UART();
      printHex(payload_buf, pSize);
      hum /= 10;
      uprintf(":  %ld.%02ldC,\t %ld.%02ld Pa / %ld.%02ld mmHg,\t %ld.%02ld%%, V_BAT = %u.%03u\n", temp / 100, temp % 100, press / 100, press % 100, press / 13332, press % 13332 * 100 / 13332, hum / 100, hum % 100, v_bat / 1000, v_bat % 1000);
    CLOSE_UART();              // stop clocking UART1
  }
}

