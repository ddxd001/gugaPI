#ifndef HOST_TESTS_STUBS_TI_MSP_DL_CONFIG_H_
#define HOST_TESTS_STUBS_TI_MSP_DL_CONFIG_H_

/* Host tests only need the opaque GPIO register type used by ButtonConfig. */
typedef struct GPIO_Regs GPIO_Regs;
typedef struct I2C_Regs I2C_Regs;
typedef struct DMA_Regs DMA_Regs;
typedef struct MCAN_Regs MCAN_Regs;
typedef int IRQn_Type;

#endif /* HOST_TESTS_STUBS_TI_MSP_DL_CONFIG_H_ */
