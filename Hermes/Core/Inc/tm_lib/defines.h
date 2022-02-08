#define STM32F4xx

/* Defines for RCC settings for system */
/* I've added these defines in options for target in Keil uVision for each target different settings when using my examples */
#define RCC_OSCILLATORTYPE    RCC_OSCILLATORTYPE_HSE /*!< Used to select system oscillator type */
#define RCC_PLLM              8                      /*!< Used for PLL M parameter */
#define RCC_PLLN              144                    /*!< Used for PLL N parameter */
#define RCC_PLLP              2                      /*!< Used for PLL P parameter */
#define RCC_PLLQ              6                      /*!< Used for PLL Q parameter */
