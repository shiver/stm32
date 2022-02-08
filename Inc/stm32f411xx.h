#ifndef STM32F411XX_H_
#define STM32F411XX_H_

#include <assert.h>
#include <stdbool.h>

#define DISABLED 0
#define ENABLED  1

/* Memory base addresses */

#define FLASH_BASE_ADDR    0x08000000U
#define ROM_BASE_ADDR      0x1FFF0000
#define SRAM1_BASE_ADDR    0x20000000
#define SRAM_BASE_ADDR     SRAM1_BASE_ADDR

/* Peripheral bus base addresses */
#define PERIPH_BASE_ADDR       0x40000000U
#define APB1_PERIPH_BASE_ADDR  (PERIPH_BASE_ADDR)
#define APB2_PERIPH_BASE_ADDR  (PERIPH_BASE_ADDR + 0x10000U)
#define AHB1_PERIPH_BASE_ADDR  (PERIPH_BASE_ADDR + 0x20000U)
#define AHB2_PERIPH_BASE_ADDR  0x50000000U

/* Peripheral base addresses */

#define RCC_BASE_ADDR      0x40023800U
#define RCC_CR_ADDR        (RCC_BASE_ADDR + 0x00U)
#define RCC_CFGR_REG_ADDR  (RCC_BASE_ADDR + 0x08U)

#define RCC_AHB1ENR_OFFSET 0x30
#define RCC_AHB1ENR_ADDR   (RCC_BASE_ADDR + RCC_AHB1ENR_OFFSET)

#define EXTI_BASE_ADDR   (APB2_PERIPH_BASE_ADDR + 0x3C00U)
#define SYSCFG_BASE_ADDR (APB2_PERIPH_BASE_ADDR + 0x3800U)

#define GPIO_BASE_ADDR   0x40020000UL
#define GPIO_PORT_A_ADDR (AHB1_PERIPH_BASE_ADDR)
#define GPIO_PORT_B_ADDR (AHB1_PERIPH_BASE_ADDR + 0x0400U)
#define GPIO_PORT_C_ADDR (AHB1_PERIPH_BASE_ADDR + 0x0800U)
#define GPIO_PORT_D_ADDR (AHB1_PERIPH_BASE_ADDR + 0x0C00U)
#define GPIO_PORT_E_ADDR (AHB1_PERIPH_BASE_ADDR + 0x1000U)
#define GPIO_PORT_H_ADDR (AHB1_PERIPH_BASE_ADDR + 0x1C00U)

#define I2C1_BASE_ADDR (APB1_PERIPH_BASE_ADDR + 0x5400U)
#define I2C2_BASE_ADDR (APB1_PERIPH_BASE_ADDR + 0x5800U)
#define I2C3_BASE_ADDR (APB1_PERIPH_BASE_ADDR + 0x5C00U)

#define SPI1_BASE_ADDR (APB2_PERIPH_BASE_ADDR + 0x3000U)
#define SPI2_BASE_ADDR (APB1_PERIPH_BASE_ADDR + 0x3800U)
#define SPI3_BASE_ADDR (APB1_PERIPH_BASE_ADDR + 0x3C00U)
#define SPI4_BASE_ADDR (APB2_PERIPH_BASE_ADDR + 0x3400U)
#define SPI5_BASE_ADDR (APB2_PERIPH_BASE_ADDR + 0x5000U)

#define USART1_BASE_ADDR (APB2_PERIPH_BASE_ADDR + 0x1000U)
#define USART2_BASE_ADDR (APB1_PERIPH_BASE_ADDR + 0x4400U)
#define USART6_BASE_ADDR (APB2_PERIPH_BASE_ADDR + 0x1400U)

#define GPIO_MODER_OFFSET   0x00
#define GPIO_OTYPER_OFFSET  0x04
#define GPIO_OSPEEDR_OFFSET 0x08
#define GPIO_PUPDR_OFFSET   0x0C
#define GPIO_IDR_OFFSET     0x10
#define GPIO_ODR_OFFSET     0x14
#define GPIO_BSRR_OFFSET    0x18
#define GPIO_LCKR_OFFSET    0x1C
#define GPIO_AFRL_OFFSET    0x20
#define GPIO_AFRH_OFFSET    0x24

#define GPIO_MODER_ADDR   (GPIO_BASE_ADDR + GPIO_MODER_OFFSET)
#define GPIO_OTYPER_ADDR  (GPIO_BASE_ADDR + GPIO_OTYPER_OFFSET)
#define GPIO_OSPEEDR_ADDR (GPIO_BASE_ADDR + GPIO_OSPEEDR_OFFSET)
#define GPIO_PUPDR_ADDR   (GPIO_BASE_ADDR + GPIO_PUPDR_OFFSET)
#define GPIO_IDR_ADDR     (GPIO_BASE_ADDR + GPIO_IDR_OFFSET)
#define GPIO_ODR_ADDR     (GPIO_BASE_ADDR + GPIO_ODR_OFFSET)
#define GPIO_BSRR_ADDR    (GPIO_BASE_ADDR + GPIO_BSRR_OFFSET)
#define GPIO_LCKR_ADDR    (GPIO_BASE_ADDR + GPIO_LCKR_OFFSET)
#define GPIO_AFRL_ADDR    (GPIO_BASE_ADDR + GPIO_AFRL_OFFSET)
#define GPIO_AFRH_ADDR    (GPIO_BASE_ADDR + GPIO_AFRH_OFFSET)

enum GPIO_PORT {
  GPIO_PORT_A = 1,
  GPIO_PORT_B = 2,
  GPIO_PORT_C = 4,
  GPIO_PORT_D = 8,
  GPIO_PORT_E = 16,
  GPIO_PORT_H = 32,
};

enum GPIO_PORT_MODE {
  GPIO_MODE_INPUT = 0,
  GPIO_MODE_OUTPUT = 1,
  GPIO_MODE_AF = 2,
  GPIO_MODE_ANALOG = 3,
};

enum GPIO_PORT_OTYPE {
  GPIO_OUTPUT_TYPE_PUSH_PULL = 0, GPIO_OUTPUT_TYPE_OPEN_DRAIN = 1,
};

enum GPIO_PORT_OSPEED {
  GPIO_SPEED_LOW = 0,
  GPIO_SPEED_MEDIUM = 1,
  GPIO_SPEED_FAST = 2,
  GPIO_SPEED_HIGH = 3,
};

enum GPIO_PORT_PUPD {
  GPIO_PUPD_NONE = 0, GPIO_PUPD_PULL_UP = 1, GPIO_PUPD_PULL_DOWN = 2,
};

enum GPIO_PORT_AF {
  GPIO_AF0 = 0,
  GPIO_AF1 = 1,
  GPIO_AF2 = 2,
  GPIO_AF3 = 3,
  GPIO_AF4 = 4,
  GPIO_AF5 = 5,
  GPIO_AF6 = 6,
  GPIO_AF7 = 7,
  GPIO_AF8 = 8,
  GPIO_AF9 = 9,
  GPIO_AF10 = 10,
  GPIO_AF11 = 11,
  GPIO_AF12 = 12,
  GPIO_AF13 = 13,
  GPIO_AF14 = 14,
  GPIO_AF15 = 15,
};

typedef struct {
  uint32_t GPIOA :1;
  uint32_t GPIOB :1;
  uint32_t GPIOC :1;
  uint32_t GPIOD :1;
  uint32_t GPIOE :1;
  uint32_t reserved1 :2;
  uint32_t GPIOH :1;
  uint32_t reserved2 :4;
  uint32_t CRC :1;
  uint32_t reserved3 :8;
  uint32_t DMA1 :1;
  uint32_t DMA2 :1;
  uint32_t reserved4 :9;
} RCC_AHB1ENR_t;

typedef struct {
  uint32_t pin0 :2;
  uint32_t pin1 :2;
  uint32_t pin2 :2;
  uint32_t pin3 :2;
  uint32_t pin4 :2;
  uint32_t pin5 :2;
  uint32_t pin6 :2;
  uint32_t pin7 :2;
  uint32_t pin8 :2;
  uint32_t pin9 :2;
  uint32_t pin10 :2;
  uint32_t pin11 :2;
  uint32_t pin12 :2;
  uint32_t pin13 :2;
  uint32_t pin14 :2;
  uint32_t pin15 :2;
} GPIO_MODER_t;

typedef struct {
  uint32_t volatile MODER;   // GPIO port mode register
  uint32_t volatile OTYPER;  // GPIO port output type register
  uint32_t volatile OSPEEDR; // GPIO port output speed register
  uint32_t volatile PUPDR;   // GPIO port pull-up/pull-down register
  uint32_t volatile IDR;     // GPIO port input data register
  uint32_t volatile ODR;     // GPIO port output data register
  uint32_t volatile BSRR;    // GPIO port bit set/reset register
  uint32_t volatile LCKR;    // GPIO port configuration lock register
  uint32_t volatile AFR[2];  // AFR[0] = GPIO alternate function low register,
                             // AFR[1] = GPIO alternate function high register
} GPIO_Registers_t;

typedef struct {
  uint32_t volatile CR;         // RCC clock control register
  uint32_t volatile PLLCFGR;    // RCC PLL configuration register
  uint32_t volatile CFGR;       // RCC clock configuration register
  uint32_t volatile CIR;        // RCC clock interrupt register
  uint32_t volatile AHB1RSTR;   // RCC AHB1 peripheral reset register
  uint32_t volatile AHB2RSTR;   // RCC AHB2 peripheral reset register
  uint32_t reserved1[2];
  uint32_t volatile APB1RSTR;   // RCC APB1 peripheral reset register
  uint32_t volatile APB2RSTR;   // RCC APB2 peripheral reset register
  uint32_t reserved2[2];
  uint32_t volatile AHB1ENR;    // RCC AHB1 peripheral clock enable register
  uint32_t volatile AHB2ENR;    // RCC AHB2 peripheral clock enable register
  uint32_t reserved3[2];
  uint32_t volatile APB1ENR;    // RCC APB1 peripheral clock enable register
  uint32_t volatile APB2ENR;    // RCC APB2 peripheral clock enable register
  uint32_t reserved4[2];
  uint32_t volatile AHB1LPENR; // RCC AHB1 peripheral clock enable in low power mode register
  uint32_t volatile AHB2LPENR; // RCC AHB2 peripheral clock enable in low power mode register
  uint32_t reserved5[2];
  uint32_t volatile APB1LPENR; // RCC APB1 peripheral clock enable in low power mode register
  uint32_t volatile APB2LPENR; // RCC APB2 peripheral clock enabled in low power mode register
  uint32_t reserved6[2];
  uint32_t volatile BDCR;       // RCC Backup domain control register
  uint32_t volatile CSR;        // RCC clock control & status register
  uint32_t reserved7[2];
  uint32_t volatile SSCGR;      // RCC spread spectrum clock generation register
  uint32_t volatile PLLI2SCFGR; // RCC PLLI2S configuration register
  uint32_t reserved8;
  uint32_t volatile DCKCFGR;    // RCC Dedicated Clocks Configuration Register
} RCC_Registers_t;

typedef struct {
  uint8_t number;
  uint8_t mode;
  uint8_t speed;
  uint8_t pullUpPullDown;
  uint8_t outputType;
  uint8_t altFunction;
} GPIO_PinConfig_t;

typedef struct {
  GPIO_Registers_t *port;      // Base address of the GPIO port
  GPIO_PinConfig_t pinConfig;
} GPIO_Handle_t;

/* Globals */

#define GPIOA ((GPIO_Registers_t *)GPIO_PORT_A_ADDR)
#define GPIOB ((GPIO_Registers_t *)GPIO_PORT_B_ADDR)
#define GPIOC ((GPIO_Registers_t *)GPIO_PORT_C_ADDR)
#define GPIOD ((GPIO_Registers_t *)GPIO_PORT_D_ADDR)
#define GPIOE ((GPIO_Registers_t *)GPIO_PORT_E_ADDR)
#define GPIOH ((GPIO_Registers_t *)GPIO_PORT_H_ADDR)
#define RCC ((RCC_Registers_t*)RCC_BASE_ADDR)

RCC_Registers_t *pRCC = (RCC_Registers_t*) RCC_BASE_ADDR;

uint32_t *const g_pRccAHB1ENR = (uint32_t*) RCC_AHB1ENR_ADDR;
RCC_AHB1ENR_t *const g_RccAHB1ENR = (RCC_AHB1ENR_t*) RCC_AHB1ENR_ADDR;
uint32_t *const g_GPIOMODERAddr = (uint32_t*) GPIO_MODER_ADDR;
GPIO_MODER_t *const g_GPIOMODER = (GPIO_MODER_t*) GPIO_MODER_ADDR;

/* Prototypes */

void gpio_init(GPIO_Handle_t *handle);
void gpio_deinit(GPIO_Handle_t *handle);
void gpio_setIRQConfig(void);
void gpio_readPin(void);
void gpio_readPort(void);
void gpio_writePin(GPIO_Handle_t *handle, uint8_t value);
void gpio_writePort(void);

/* Implementations */

void gpio_init(GPIO_Handle_t *handle) {
  GPIO_Registers_t *port = handle->port;

  // Enable the RCC clock for the GPIO peripheral
  if (port == GPIOA) {
    RCC->AHB1ENR |= (1 << 0);
  } else if (port == GPIOB) {
    RCC->AHB1ENR |= (1 << 1);
  } else if (port == GPIOC) {
    RCC->AHB1ENR |= (1 << 2);
  } else if (port == GPIOD) {
    RCC->AHB1ENR |= (1 << 3);
  } else if (port == GPIOE) {
    RCC->AHB1ENR |= (1 << 4);
  } else if (port == GPIOH) {
    RCC->AHB1ENR |= (1 << 7);
  }

  // Set the output type for the pin
  port->OTYPER |= (handle->pinConfig.outputType << handle->pinConfig.number);

  // Set the GPIO mode for the corresponding pin
  uint8_t pos = (handle->pinConfig.number * 2);
  if (handle->port == GPIOA) {
    handle->port->MODER &= ~(0x3 << pos);
    handle->port->MODER |= (handle->pinConfig.mode << pos);

    handle->port->OSPEEDR &= ~(0x3 << pos);
    handle->port->OSPEEDR |= (handle->pinConfig.speed << pos);

    handle->port->PUPDR &= ~(0x3 << pos);
    handle->port->PUPDR |= (handle->pinConfig.pullUpPullDown << pos);
  } else if (handle->port == GPIOB) {
    handle->port->MODER &= ~(0x3 << pos);
    handle->port->MODER |= (handle->pinConfig.mode << pos);

    handle->port->OSPEEDR &= ~(0x3 << pos);
    handle->port->OSPEEDR |= (handle->pinConfig.speed << pos);

    handle->port->PUPDR &= ~(0x3 << pos);
    handle->port->PUPDR |= (handle->pinConfig.pullUpPullDown << pos);
  } else if (handle->port == GPIOC) {
    handle->port->MODER &= ~(0x3 << pos);
    handle->port->MODER |= (handle->pinConfig.mode << pos);

    handle->port->OSPEEDR &= ~(0x3 << pos);
    handle->port->OSPEEDR |= (handle->pinConfig.speed << pos);

    handle->port->PUPDR &= ~(0x3 << pos);
    handle->port->PUPDR |= (handle->pinConfig.pullUpPullDown << pos);
  } else if (handle->port == GPIOD) {
    handle->port->MODER &= ~(0x3 << pos);
    handle->port->MODER |= (handle->pinConfig.mode << pos);

    handle->port->OSPEEDR &= ~(0x3 << pos);
    handle->port->OSPEEDR |= (handle->pinConfig.speed << pos);

    handle->port->PUPDR &= ~(0x3 << pos);
    handle->port->PUPDR |= (handle->pinConfig.pullUpPullDown << pos);
  } else if (handle->port == GPIOE) {
    handle->port->MODER &= ~(0x3 << pos);
    handle->port->MODER |= (handle->pinConfig.mode << pos);

    handle->port->OSPEEDR &= ~(0x3 << pos);
    handle->port->OSPEEDR |= (handle->pinConfig.speed << pos);

    handle->port->PUPDR &= ~(0x3 << pos);
    handle->port->PUPDR |= (handle->pinConfig.pullUpPullDown << pos);
  } else if (handle->port == GPIOH) {
    // STM32F411xC/E only supports pin 0 and 1 on port H
    assert(handle->pinConfig.number < 2);

    handle->port->MODER &= ~(0x3 << pos);
    handle->port->MODER |= (handle->pinConfig.mode << pos);

    handle->port->OSPEEDR &= ~(0x3 << pos);
    handle->port->OSPEEDR |= (handle->pinConfig.speed << pos);

    handle->port->PUPDR &= ~(0x3 << pos);
    handle->port->PUPDR |= (handle->pinConfig.pullUpPullDown << pos);
  }
}

void gpio_deinit(GPIO_Handle_t *handle) {
  if (handle->port == GPIOA) {
    pRCC->AHB1RSTR |= (1 << 0);
    pRCC->AHB1RSTR &= ~(1 << 0);
  } else if (handle->port == GPIOB) {
    pRCC->AHB1RSTR |= (1 << 1);
    pRCC->AHB1RSTR &= ~(1 << 1);
  } else if (handle->port == GPIOC) {
    pRCC->AHB1RSTR |= (1 << 2);
    pRCC->AHB1RSTR &= ~(1 << 2);
  } else if (handle->port == GPIOD) {
    pRCC->AHB1RSTR |= (1 << 3);
    pRCC->AHB1RSTR &= ~(1 << 3);
  } else if (handle->port == GPIOE) {
    pRCC->AHB1RSTR |= (1 << 4);
    pRCC->AHB1RSTR &= ~(1 << 4);
  } else if (handle->port == GPIOH) {
    pRCC->AHB1RSTR |= (1 << 7);
    pRCC->AHB1RSTR &= ~(1 << 7);
  }
}

void gpio_writePin(GPIO_Handle_t *handle, uint8_t value) {
  assert(value >= 0 && value <= 1);

  if (value == ENABLED) {
    handle->port->ODR |= (value << handle->pinConfig.number);
  } else if (value == DISABLED) {
    handle->port->ODR &= ~(1 << handle->pinConfig.number);
  }
}

/* Debug only */
#ifdef DEBUG

// Debug Exception and Monitor Control Register base address
#define DEMCR *((volatile uint32_t*) 0xE000EDFCU )

/* ITM register addresses */
#define ITM_STIMULUS_PORT0 *((volatile uint32_t*) 0xE0000000 )
#define ITM_TRACE_EN       *((volatile uint32_t*) 0xE0000E00 )

void debug_ITMSendChar(uint8_t ch) {
// Enable TRCENA
  DEMCR |= (1 << 24);

// Enable stimulus port 0
  ITM_TRACE_EN |= (1 << 0);

// Read FIFO status in bit [0]:
  while (!(ITM_STIMULUS_PORT0 & 1))
    ;

// Write to ITM stimulus port0
  ITM_STIMULUS_PORT0 = ch;
}

int _write(int file, char *ptr, int len) {
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++) {
    debug_ITMSendChar(*ptr++);
  }
  return len;
}

#endif

#endif
