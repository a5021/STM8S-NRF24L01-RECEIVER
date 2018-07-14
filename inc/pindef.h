#ifndef __PINDEF_INCLUDED
#define __PINDEF_INCLUDED
//
//  definitions to configure a pin in input mode
//

#define INPUT           0
#define FLOAT           0
#define PULL_UP         (1 << 1)
#define NO_INT          0
#define INT             (1 << 0)

  // most common pin settings for input mode
#define INPUT_MODE      PULL_UP

//
//  definitions to configure a pin in output mode
//

#define OUTPUT          (1 << 2)
#define OPEN_DRAIN      0
#define PUSH_PULL       (1 << 1)
#define LOW_SPEED       0
#define HIGH_SPEED      (1 << 0)

  // most common pin settings for output mode
#define OUTPUT_MODE     OUTPUT + PUSH_PULL + HIGH_SPEED

//
//   pin state
//
  
#define LOW             0
#define HIGH            1

  // macro to configure a pin
#define PIN_MODE(PORT, PIN, MODE)\
          P##PORT##_DDR_DDR##PIN = (MODE) >> 2;\
          P##PORT##_CR1_C1##PIN = ((MODE) >> 1) & 1;\
          P##PORT##_CR2_C2##PIN = (MODE) & 1

  // macro to set a pin
#define PIN_WRITE(PORT, PIN, VALUE)\
          P##PORT##_ODR_ODR##PIN = VALUE
                                      
  // macro to get state of a pin
#define PIN_READ(PORT, PIN)\
          P##PORT##_IDR_IDR##PIN

#endif