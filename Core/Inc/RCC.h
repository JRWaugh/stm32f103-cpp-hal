/*
 * RCC2.h
 *
 *  Created on: 1 Jan 2021
 *      Author: krabb
 */

#ifndef SRC_RCC_H_
#define SRC_RCC_H_

#include "stm32f1xx.h"
#include "AFIO.h"

struct PLLInit {
	uint32_t PLLState;      // The new state of the PLL. This parameter can be a value of @ref RCC_PLL_Config
	uint32_t PLLSource;     // PLL entry clock source. This parameter must be a value of @ref RCC_PLL_Clock_Source
	uint32_t PLLMUL;        // Multiplication factor for PLL VCO input clock. This parameter must be a value of @ref RCCEx_PLL_Multiplication_Factor
};

struct ClkInit {
	uint32_t ClockType;             // The clock to be configured. This parameter can be a value of @ref RCC_System_Clock_Type

	uint32_t SYSCLKSource;          /*!< The clock source (SYSCLKS) used as system clock.
                                       This parameter can be a value of @ref RCC_System_Clock_Source */

	uint32_t AHBCLKDivider;         /*!< The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK).
                                       This parameter can be a value of @ref RCC_AHB_Clock_Source */

	uint32_t APB1CLKDivider;        /*!< The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */

	uint32_t APB2CLKDivider;        /*!< The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */
};

struct OscInit {
	uint32_t OscillatorType;       /*!< The oscillators to be configured.
                                       This parameter can be a value of @ref RCC_Oscillator_Type */

#if defined(STM32F105xC) || defined(STM32F107xC)
	uint32_t Prediv1Source;       /*!<  The Prediv1 source value. This parameter can be a value of @ref RCCEx_Prediv1_Source */
#endif /* STM32F105xC || STM32F107xC */

	uint32_t HSEState;              /*!< The new state of the HSE. This parameter can be a value of @ref RCC_HSE_Config */

	uint32_t HSEPredivValue;       /*!<  The Prediv1 factor value (named PREDIV1 or PLLXTPRE in RM) This parameter can be a value of @ref RCCEx_Prediv1_Factor */

	uint32_t LSEState;              /*!<  The new state of the LSE. This parameter can be a value of @ref RCC_LSE_Config */

	uint32_t HSIState;              /*!< The new state of the HSI.
                                       This parameter can be a value of @ref RCC_HSI_Config */

	uint32_t HSICalibrationValue;   /*!< The HSI calibration trimming value (default is RCC_HSICALIBRATION_DEFAULT).
                                       This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F */

	uint32_t LSIState;              /*!<  The new state of the LSI.
                                        This parameter can be a value of @ref RCC_LSI_Config */

	PLLInit PLL;         /*!< PLL structure parameters */

#if defined(STM32F105xC) || defined(STM32F107xC)
	RCC_PLL2InitTypeDef PLL2;         /*!< PLL2 structure parameters */
#endif /* STM32F105xC || STM32F107xC */
};

#define RCC_DBP_TIMEOUT_VALUE       100U    /* 100 ms */
#define CLOCKSWITCH_TIMEOUT_VALUE 	5000U   /* 5 s */
#define HSI_TIMEOUT_VALUE          	2U      /* 2 ms (minimum Tick + 1) */
#define LSI_TIMEOUT_VALUE          	2U      /* 2 ms (minimum Tick + 1) */
#define PLL_TIMEOUT_VALUE          	2U      /* 2 ms (minimum Tick + 1) */

#define RCC_OFFSET                	(RCC_BASE - PERIPH_BASE)
#define RCC_CR_OFFSET             	0x00U
#define RCC_CFGR_OFFSET           	0x04U
#define RCC_CIR_OFFSET            	0x08U
#define RCC_BDCR_OFFSET           	0x20U
#define RCC_CSR_OFFSET           	0x24U

#define RCC_CR_OFFSET_BB          	(RCC_OFFSET + RCC_CR_OFFSET)
#define RCC_CFGR_OFFSET_BB        	(RCC_OFFSET + RCC_CFGR_OFFSET)
#define RCC_CIR_OFFSET_BB         	(RCC_OFFSET + RCC_CIR_OFFSET)
#define RCC_BDCR_OFFSET_BB        	(RCC_OFFSET + RCC_BDCR_OFFSET)
#define RCC_CSR_OFFSET_BB         	(RCC_OFFSET + RCC_CSR_OFFSET)

/* --- CR Register ---*/
#define RCC_CR_HSION_BB           	(PERIPH_BB_BASE + RCC_CR_OFFSET_BB * 32U + RCC_CR_HSION_Pos * 4U)
#define RCC_CR_HSEON_BB           	(PERIPH_BB_BASE + RCC_CR_OFFSET_BB * 32U + RCC_CR_HSEON_Pos * 4U)
#define RCC_CR_CSSON_BB           	(PERIPH_BB_BASE + RCC_CR_OFFSET_BB * 32U + RCC_CR_CSSON_Pos * 4U)
#define RCC_CR_PLLON_BB           	(PERIPH_BB_BASE + RCC_CR_OFFSET_BB * 32U + RCC_CR_PLLON_Pos * 4U)

/* --- CSR Register ---*/
#define RCC_CSR_LSION_BB      		(PERIPH_BB_BASE + RCC_CSR_OFFSET_BB * 32U + RCC_CSR_LSION_Pos * 4U)
#define RCC_CSR_RMVF_BB          	(PERIPH_BB_BASE + RCC_CSR_OFFSET_BB * 32U + RCC_CSR_RMVF_Pos  * 4U)

/* --- BDCR Registers ---*/
#define RCC_BDCR_LSEON_BB          	(PERIPH_BB_BASE + RCC_BDCR_OFFSET_BB * 32U + RCC_BDCR_LSEON_Pos  * 4U)
#define RCC_BDCR_LSEBYP_BB         	(PERIPH_BB_BASE + RCC_BDCR_OFFSET_BB * 32U + RCC_BDCR_LSEBYP_Pos * 4U)
#define RCC_BDCR_RTCEN_BB          	(PERIPH_BB_BASE + RCC_BDCR_OFFSET_BB * 32U + RCC_BDCR_RTCEN_Pos  * 4U)
#define RCC_BDCR_BDRST_BB         	(PERIPH_BB_BASE + RCC_BDCR_OFFSET_BB * 32U + RCC_BDCR_BDRST_Pos  * 4U)

/* CR register byte 2 (Bits[23:16]) base address */
#define RCC_CR_BYTE2_ADDRESS      	(RCC_BASE + RCC_CR_OFFSET + 0x2U)

/* CIR register byte 1 (Bits[15:8]) base address */
#define RCC_CIR_BYTE1_ADDRESS     	(RCC_BASE + RCC_CIR_OFFSET + 0x1U)

/* CIR register byte 2 (Bits[23:16]) base address */
#define RCC_CIR_BYTE2_ADDRESS     	(RCC_BASE + RCC_CIR_OFFSET + 0x2U)

/* Defines used for Flags */
#define CR_REG_INDEX                0x01U
#define BDCR_REG_INDEX            	0x02U
#define CSR_REG_INDEX     			0x03U

#define RCC_FLAG_MASK            	0x1FU

#define RCC_OSCILLATORTYPE_NONE   	0x00U
#define RCC_OSCILLATORTYPE_HSE     	0x01U
#define RCC_OSCILLATORTYPE_HSI     	0x02U
#define RCC_OSCILLATORTYPE_LSE     	0x04U
#define RCC_OSCILLATORTYPE_LSI     	0x08U

#define RCC_HSE_BYPASS             	(RCC_CR_HSEBYP | RCC_CR_HSEON) /*!< External clock source for HSE clock */
#define RCC_LSE_BYPASS          	(RCC_BDCR_LSEBYP | RCC_BDCR_LSEON) /*!< External clock source for LSE clock */

#define RCC_HSICALIBRATION_DEFAULT 	0x10U         /* Default HSI calibration trimming value */

#define RCC_PLL_NONE             	0x00U  /*!< PLL is not configured */
#define RCC_PLL_OFF              	0x01U  /*!< PLL deactivation */
#define RCC_PLL_ON               	0x02U  /*!< PLL activation */

#define RCC_CLOCKTYPE_SYSCLK       	0x01U /*!< SYSCLK to configure */
#define RCC_CLOCKTYPE_HCLK       	0x02U /*!< HCLK to configure */
#define RCC_CLOCKTYPE_PCLK1       	0x04U /*!< PCLK1 to configure */
#define RCC_CLOCKTYPE_PCLK2       	0x08U /*!< PCLK2 to configure */

#define RCC_PLLSOURCE_HSI_DIV2      0x00U     /*!< HSI clock divided by 2 selected as PLL entry clock source */
#define RCC_RTCCLKSOURCE_NO_CLK   	0x00U                 /*!< No clock */
#define RCC_MCO1                  	0x00U
#define RCC_MCODIV_1             	0x00U

/* Flags in the CR register */
#define RCC_FLAG_HSIRDY          	((CR_REG_INDEX << 5U) | RCC_CR_HSIRDY_Pos) /*!< Internal High Speed clock ready flag */
#define RCC_FLAG_HSERDY            	((CR_REG_INDEX << 5U) | RCC_CR_HSERDY_Pos) /*!< External High Speed clock ready flag */
#define RCC_FLAG_PLLRDY           	((CR_REG_INDEX << 5U) | RCC_CR_PLLRDY_Pos) /*!< PLL clock ready flag */
#if defined(STM32F105xC) || defined(STM32F107xC)
#define RCC_FLAG_PLL2RDY           	((CR_REG_INDEX << 5U) | RCC_CR_PLL2RDY_Pos)
#define RCC_FLAG_PLLI2SRDY         	((CR_REG_INDEX << 5U) | RCC_CR_PLL3RDY_Pos)
#endif /* STM32F105xC || STM32F107xC*/

/* Flags in the CSR register */
#define RCC_FLAG_LSIRDY         	((CSR_REG_INDEX << 5U) | RCC_CSR_LSIRDY_Pos)   /*!< Internal Low Speed oscillator Ready */
#define RCC_FLAG_PINRST           	((CSR_REG_INDEX << 5U) | RCC_CSR_PINRSTF_Pos)  /*!< PIN reset flag */
#define RCC_FLAG_PORRST            	((CSR_REG_INDEX << 5U) | RCC_CSR_PORRSTF_Pos)  /*!< POR/PDR reset flag */
#define RCC_FLAG_SFTRST            	((CSR_REG_INDEX << 5U) | RCC_CSR_SFTRSTF_Pos)  /*!< Software Reset flag */
#define RCC_FLAG_IWDGRST          	((CSR_REG_INDEX << 5U) | RCC_CSR_IWDGRSTF_Pos) /*!< Independent Watchdog reset flag */
#define RCC_FLAG_WWDGRST           	((CSR_REG_INDEX << 5U) | RCC_CSR_WWDGRSTF_Pos) /*!< Window watchdog reset flag */
#define RCC_FLAG_LPWRRST           	((CSR_REG_INDEX << 5U) | RCC_CSR_LPWRRSTF_Pos) /*!< Low-Power reset flag */

/* Flags in the BDCR register */
#define RCC_FLAG_LSERDY           	((BDCR_REG_INDEX << 5U) | RCC_BDCR_LSERDY_Pos) /*!< External Low Speed oscillator Ready */

/* Exported constants --------------------------------------------------------*/
#define RCC_PERIPHCLK_RTC           0x01U
#define RCC_PERIPHCLK_ADC           0x02U
#if defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || defined(STM32F107xC)
#define RCC_PERIPHCLK_I2S2          0x04U
#define RCC_PERIPHCLK_I2S3          0x08U
#endif /* STM32F101xE || STM32F101xG || STM32F103xE || STM32F103xG || STM32F105xC || STM32F107xC */
#if defined(STM32F102x6) || defined(STM32F102xB) || defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || defined(STM32F107xC)
#define RCC_PERIPHCLK_USB          	0x10U
#endif /* STM32F102x6 || STM32F102xB || STM32F103x6 || STM32F103xB || STM32F103xE || STM32F103xG || STM32F105xC || STM32F107xC */

#if defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || defined(STM32F107xC)
#define RCC_I2S2CLKSOURCE_SYSCLK  	0x00U
#define RCC_I2S3CLKSOURCE_SYSCLK   	0x00U
#endif /* STM32F103xE || STM32F103xG || STM32F105xC || STM32F107xC */

#if defined(STM32F102x6) || defined(STM32F102xB) || defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG)
#define RCC_USBCLKSOURCE_PLL_DIV1_5 0x00U
#endif /* STM32F102x6 || STM32F102xB || STM32F103x6 || STM32F103xB || STM32F103xE || STM32F103xG */


#if defined(STM32F105xC) || defined(STM32F107xC)
#define RCC_USBCLKSOURCE_PLL_DIV3	0x00U
#endif /* STM32F105xC || STM32F107xC */

#define RCC_HSE_PREDIV_DIV1       	0x00U

#if defined(STM32F105xC) || defined(STM32F107xC)
#define RCC_PLL2_NONE             	0x00U
#define RCC_PLL2_OFF             	0x01U
#define RCC_PLL2_ON               	0x02U
#endif /* STM32F105xC || STM32F107xC */

extern struct RCC {
public:
	RCC() {
		HAL_Init();

		APB2ENR |= RCC_APB2ENR_AFIOEN;
		(void)(APB2ENR & RCC_APB2ENR_AFIOEN);
		APB1ENR |= RCC_APB1ENR_PWREN;
		(void)(APB1ENR & RCC_APB1ENR_PWREN);
		uint32_t tmpreg = afio.MAPR;
		tmpreg &= ~AFIO_MAPR_SWJ_CFG_Msk;
		tmpreg |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
		afio.MAPR = tmpreg;

		OscInit OscInitStruct{ 0 };
		OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
		OscInitStruct.HSEState = RCC_CR_HSEON;
		OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
		OscInitStruct.HSIState = RCC_CR_HSION;
		OscInitStruct.PLL.PLLState = RCC_PLL_ON;
		OscInitStruct.PLL.PLLSource = RCC_CFGR_PLLSRC;
		OscInitStruct.PLL.PLLMUL = RCC_CFGR_PLLMULL9;

		/*------------------------------- HSE Configuration ------------------------*/
		if ((OscInitStruct.OscillatorType & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE) {
			auto const& temp_register = ((RCC_FLAG_HSERDY >> 5U) == CR_REG_INDEX ? CR : (RCC_FLAG_HSERDY >> 5U) == BDCR_REG_INDEX ? BDCR : CSR) & (1U << (RCC_FLAG_HSERDY & RCC_FLAG_MASK));
			if ((CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSE || ((CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL && (CFGR & RCC_CFGR_PLLSRC) == RCC_CFGR_PLLSRC)) {
				if (temp_register != RESET && OscInitStruct.HSEState == 0)
					Error_Handler();
			} else {
				switch (OscInitStruct.HSEState) {
				case RCC_CR_HSEON:
					CR |= RCC_CR_HSEON;
					break;

				case RCC_HSE_BYPASS:
					CR |= RCC_CR_HSEBYP;
					CR |= RCC_CR_HSEON;
					break;

				default:
					CR &= ~(RCC_CR_HSEON);
					CR &= ~(RCC_CR_HSEBYP);
					break;
				}
				/* Check the HSE State */
				if (uint32_t const tickstart = HAL_GetTick(); OscInitStruct.HSEState != 0)
					while (temp_register == RESET && (HAL_GetTick() - tickstart) <= HSE_STARTUP_TIMEOUT);
				else
					while (temp_register != RESET && (HAL_GetTick() - tickstart) <= HSE_STARTUP_TIMEOUT);
			}
		}

		/*----------------------------- HSI Configuration --------------------------*/
		if ((OscInitStruct.OscillatorType & RCC_OSCILLATORTYPE_HSI) == RCC_OSCILLATORTYPE_HSI) {
			auto const& temp_register = ((RCC_FLAG_HSIRDY >> 5U) == CR_REG_INDEX ? CR : (RCC_FLAG_HSIRDY >> 5U) == BDCR_REG_INDEX ? BDCR : CSR) & (1U << (RCC_FLAG_HSIRDY & RCC_FLAG_MASK));
			/* Check if HSI is used as system clock or as PLL source when PLL is selected as system clock */
			if ((CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSI || ((CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL && (CFGR & RCC_CFGR_PLLSRC) == RCC_PLLSOURCE_HSI_DIV2)) {
				/* When HSI is used as system clock it will not disabled */
				if (temp_register != RESET && OscInitStruct.HSIState != RCC_CR_HSION)
					Error_Handler();
				else
					CR = (CR & ~(RCC_CR_HSITRIM)) | OscInitStruct.HSICalibrationValue << RCC_CR_HSITRIM_Pos;
			} else {
				if (uint32_t const tickstart = HAL_GetTick(); OscInitStruct.HSIState != 0) {
					*(__IO uint32_t *)RCC_CR_HSION_BB = ENABLE;
					//((uint32_t)(PERIPH_BB_BASE + (RCC_CR_OFFSET_BB * 32U) + (RCC_HSION_BIT_NUMBER * 4U)))
					while (temp_register == RESET && (HAL_GetTick() - tickstart) <= HSI_TIMEOUT_VALUE);
					CR = (CR & ~(RCC_CR_HSITRIM)) | OscInitStruct.HSICalibrationValue << RCC_CR_HSITRIM_Pos;
				} else {
					/* Disable the Internal High Speed oscillator (HSI). */
					*(__IO uint32_t *)RCC_CR_HSION_BB = DISABLE;
					while (temp_register != RESET && (HAL_GetTick() - tickstart) <= HSI_TIMEOUT_VALUE);
				}
			}
		}
		/*------------------------------ LSI Configuration -------------------------*/
		if ((OscInitStruct.OscillatorType & RCC_OSCILLATORTYPE_LSI) == RCC_OSCILLATORTYPE_LSI) {
			auto const& temp_register = ((RCC_FLAG_LSIRDY >> 5U) == CR_REG_INDEX ? CR : (RCC_FLAG_LSIRDY >> 5U) == BDCR_REG_INDEX ? BDCR : CSR) & (1U << (RCC_FLAG_LSIRDY & RCC_FLAG_MASK));
			if (uint32_t const tickstart = HAL_GetTick(); OscInitStruct.LSIState != 0) {
				/* Enable the Internal Low Speed oscillator (LSI). */
				*(__IO uint32_t *) RCC_CSR_LSION_BB = ENABLE;
				/* Wait till LSI is ready */
				while (temp_register == RESET && (HAL_GetTick() - tickstart) <= LSI_TIMEOUT_VALUE)
					HAL_Delay(1);
			} else {
				*(__IO uint32_t *) RCC_CSR_LSION_BB = DISABLE;
				/* Wait till LSI is disabled */
				while (temp_register != RESET && (HAL_GetTick() - tickstart) <= LSI_TIMEOUT_VALUE);

			}
		}
		/*------------------------------ LSE Configuration -------------------------*/
		if ((OscInitStruct.OscillatorType & RCC_OSCILLATORTYPE_LSE) == RCC_OSCILLATORTYPE_LSE) {
			bool pwrclkchanged = false;
			if ((APB1ENR & RCC_APB1ENR_PWREN) == false) {
				APB1ENR |= RCC_APB1ENR_PWREN;
				__IO uint32_t tmpreg = APB1ENR & RCC_APB1ENR_PWREN;
				(void) tmpreg;
				pwrclkchanged = true;
			}

			if ((CR & PWR_CR_DBP) == false) {
				CR |= PWR_CR_DBP;
				for (uint32_t const tickstart = HAL_GetTick(); (CR & PWR_CR_DBP) == false && (HAL_GetTick() - tickstart) <= RCC_DBP_TIMEOUT_VALUE; );
			}

			/* Set the new LSE configuration -----------------------------------------*/
			switch (OscInitStruct.LSEState) {
			case RCC_BDCR_LSEON:
				BDCR |= RCC_BDCR_LSEON;
				break;

			case RCC_LSE_BYPASS:
				BDCR |= RCC_BDCR_LSEBYP;
				BDCR |= RCC_BDCR_LSEON;
				break;

			default:
				BDCR &= ~(RCC_BDCR_LSEON);
				BDCR &= ~(RCC_BDCR_LSEBYP);
				break;
			}

			auto const& temp_register = ((RCC_FLAG_LSERDY >> 5U) == CR_REG_INDEX ? CR : (RCC_FLAG_LSERDY >> 5U) == BDCR_REG_INDEX ? BDCR : CSR) & (1U << (RCC_FLAG_LSERDY & RCC_FLAG_MASK));
			if (uint32_t const tickstart = HAL_GetTick(); OscInitStruct.LSEState != 0)
				/* Wait till LSE is ready */
				while (temp_register == RESET && (HAL_GetTick() - tickstart) <= LSE_STARTUP_TIMEOUT);
			else
				/* Wait till LSE is disabled */
				while (temp_register != RESET && (HAL_GetTick() - tickstart) <= LSE_STARTUP_TIMEOUT);

			if (pwrclkchanged)
				APB1ENR &= ~(RCC_APB1ENR_PWREN);
		}


#ifdef RCC_CR_PLL2ON
		/*-------------------------------- PLL2 Configuration -----------------------*/
		if ((OscInitStruct.PLL2.PLL2State) != RCC_PLL2_NONE) {
			/* This bit can not be cleared if the PLL2 clock is used indirectly as system
		      clock (i.e. it is used as PLL clock entry that is used as system clock). */
			if ((__HAL_RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_HSE) && \
					(__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK) && \
					((READ_BIT(RCC->CFGR2, RCC_CFGR2_PREDIV1SRC)) == RCC_CFGR2_PREDIV1SRC_PLL2))
			{
				return HAL_ERROR;
			}
			else
			{
				if ((OscInitStruct.PLL2.PLL2State) == RCC_PLL2_ON)
				{
					/* Check the parameters */
					assert_param(IS_RCC_PLL2_MUL(OscInitStruct.PLL2.PLL2MUL));
					assert_param(IS_RCC_HSE_PREDIV2(OscInitStruct.PLL2.HSEPrediv2Value));

					/* Prediv2 can be written only when the PLLI2S is disabled. */
					/* Return an error only if new value is different from the programmed value */
					if (HAL_IS_BIT_SET(RCC->CR, RCC_CR_PLL3ON) && \
							(__HAL_RCC_HSE_GET_PREDIV2() != OscInitStruct.PLL2.HSEPrediv2Value))
					{
						return HAL_ERROR;
					}

					/* Disable the main PLL2. */
					__HAL_RCC_PLL2_DISABLE();

					/* Get Start Tick */
					tickstart = HAL_GetTick();

					/* Wait till PLL2 is disabled */
					while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLL2RDY) != RESET)
					{
						if ((HAL_GetTick() - tickstart) > PLL2_TIMEOUT_VALUE)
						{
							return HAL_TIMEOUT;
						}
					}

					/* Configure the HSE prediv2 factor --------------------------------*/
					__HAL_RCC_HSE_PREDIV2_CONFIG(OscInitStruct.PLL2.HSEPrediv2Value);

					/* Configure the main PLL2 multiplication factors. */
					__HAL_RCC_PLL2_CONFIG(OscInitStruct.PLL2.PLL2MUL);

					/* Enable the main PLL2. */
					__HAL_RCC_PLL2_ENABLE();

					/* Get Start Tick */
					tickstart = HAL_GetTick();

					/* Wait till PLL2 is ready */
					while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLL2RDY)  == RESET)
					{
						if ((HAL_GetTick() - tickstart) > PLL2_TIMEOUT_VALUE)
						{
							return HAL_TIMEOUT;
						}
					}
				}
				else
				{
					/* Set PREDIV1 source to HSE */
					CLEAR_BIT(RCC->CFGR2, RCC_CFGR2_PREDIV1SRC);

					/* Disable the main PLL2. */
					__HAL_RCC_PLL2_DISABLE();

					/* Get Start Tick */
					tickstart = HAL_GetTick();

					/* Wait till PLL2 is disabled */
					while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLL2RDY)  != RESET)
					{
						if ((HAL_GetTick() - tickstart) > PLL2_TIMEOUT_VALUE)
						{
							return HAL_TIMEOUT;
						}
					}
				}
			}
		}

#endif /* RCC_CR_PLL2ON */
		/*-------------------------------- PLL Configuration -----------------------*/
		if ((OscInitStruct.PLL.PLLState) != RCC_PLL_NONE) {
			if ((CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
				auto const& temp_register = ((RCC_FLAG_PLLRDY >> 5U) == CR_REG_INDEX ? CR : (RCC_FLAG_PLLRDY >> 5U) == BDCR_REG_INDEX ? BDCR : CSR) & (1U << (RCC_FLAG_PLLRDY & RCC_FLAG_MASK));
				if (OscInitStruct.PLL.PLLState == RCC_PLL_ON) {
					*reinterpret_cast<__IO uint32_t *>(RCC_CR_PLLON_BB) = 0;
					uint32_t tickstart = HAL_GetTick();
					while (temp_register != RESET && (HAL_GetTick() - tickstart) <= PLL_TIMEOUT_VALUE);

					if (OscInitStruct.PLL.PLLSource == RCC_CFGR_PLLSRC) {
#ifdef RCC_CFGR2_PREDIV1SRC
						CFGR2 |= OscInitStruct.Prediv1Source;
#endif
						/* Set PREDIV1 Value */
						CFGR = (CFGR & ~RCC_CFGR_PLLXTPRE) | OscInitStruct.HSEPredivValue;
					}

					CFGR = (CFGR & ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL)) | (OscInitStruct.PLL.PLLSource | OscInitStruct.PLL.PLLMUL);

					/* Enable the main PLL. */
					*reinterpret_cast<__IO uint32_t *>(RCC_CR_PLLON_BB) = 1;

					/* Get Start Tick */
					tickstart = HAL_GetTick();

					/* Wait till PLL is ready */
					while (temp_register == RESET && (HAL_GetTick() - tickstart) <= PLL_TIMEOUT_VALUE);
				} else {
					/* Disable the main PLL. */
					*reinterpret_cast<__IO uint32_t *>(RCC_CR_PLLON_BB) = 0;

					/* Get Start Tick */
					uint32_t tickstart = HAL_GetTick();

					/* Wait till PLL is disabled */
					while (temp_register != RESET && (HAL_GetTick() - tickstart) > PLL_TIMEOUT_VALUE);
				}
			} else {
				/* Check if there is a request to disable the PLL used as System clock source */
				if (OscInitStruct.PLL.PLLState == RCC_PLL_OFF)
					Error_Handler();
				else if ((CFGR & RCC_CFGR_PLLSRC) != OscInitStruct.PLL.PLLSource || (CFGR & RCC_CFGR_PLLMULL) != OscInitStruct.PLL.PLLMUL)
					Error_Handler();
			}
		}

		/** Initializes the CPU, AHB and APB buses clocks */

		ClkInit ClkInitStruct;
		ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
		ClkInitStruct.SYSCLKSource = RCC_CFGR_SW_PLL;
		ClkInitStruct.AHBCLKDivider = RCC_CFGR_HPRE_DIV1;
		ClkInitStruct.APB1CLKDivider = RCC_CFGR_PPRE1_DIV2;
		ClkInitStruct.APB2CLKDivider = RCC_CFGR_PPRE1_DIV1;

		/* To correctly read data from FLASH memory, the number of wait states (LATENCY) must be correctly programmed according to the frequency of the CPU clock (HCLK) of the device. */

#ifdef FLASH_ACR_LATENCY
		/* Increasing the number of wait states because of higher CPU frequency */
		if (FLASH_LATENCY_2 > __HAL_FLASH_GET_LATENCY()) {
			/* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
			__HAL_FLASH_SET_LATENCY(FLASH_LATENCY_2);

			/* Check that the new number of wait states is taken into account to access the Flash
		    memory by reading the FLASH_ACR register */
			if ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_LATENCY_2)
				Error_Handler();
		}
#endif
		/*-------------------------- HCLK Configuration --------------------------*/
		if ((ClkInitStruct.ClockType & RCC_CLOCKTYPE_HCLK) == RCC_CLOCKTYPE_HCLK) {
			/* Set the highest APBx dividers in order to ensure that we do not go through a non-spec phase whatever we decrease or increase HCLK. */
			if ((ClkInitStruct.ClockType & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
				CFGR = (CFGR & ~RCC_CFGR_PPRE1) | RCC_CFGR_PPRE1_DIV16;

			if ((ClkInitStruct.ClockType & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
				CFGR = (CFGR & ~RCC_CFGR_PPRE2) | (RCC_CFGR_PPRE1_DIV16 << 3);

			CFGR = (CFGR & ~RCC_CFGR_HPRE) | ClkInitStruct.AHBCLKDivider;
		}

		/*------------------------- SYSCLK Configuration ---------------------------*/
		if ((ClkInitStruct.ClockType & RCC_CLOCKTYPE_SYSCLK) == RCC_CLOCKTYPE_SYSCLK) {
			if (ClkInitStruct.SYSCLKSource == RCC_CFGR_SW_HSE) {
				if (auto const temp_register = ((RCC_FLAG_HSERDY >> 5U) == CR_REG_INDEX ? CR : (RCC_FLAG_HSERDY >> 5U) == BDCR_REG_INDEX ? BDCR : CSR) & (1U << (RCC_FLAG_HSERDY & RCC_FLAG_MASK)); temp_register == RESET)
					Error_Handler();

				SystemCoreClock = HSE_VALUE >> AHBPrescTable[(CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];
			} else if (ClkInitStruct.SYSCLKSource == RCC_CFGR_SW_PLL) {
				/* Check the PLL ready flag */
				if (auto const temp_register = ((RCC_FLAG_PLLRDY >> 5U) == CR_REG_INDEX ? CR : (RCC_FLAG_PLLRDY >> 5U) == BDCR_REG_INDEX ? BDCR : CSR) & (1U << (RCC_FLAG_PLLRDY & RCC_FLAG_MASK)); temp_register == RESET)
					Error_Handler();

				uint8_t const aPLLMULFactorTable[] =
#if defined(RCC_CFGR2_PREDIV1SRC)
				{0, 0, 4, 5, 6, 7, 8, 9, 0, 0, 0, 0, 0, 13};
#else
				{2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 16};
#endif
				uint8_t const aPredivFactorTable[] =
#if defined(RCC_CFGR2_PREDIV1) || defined(RCC_CFGR2_PREDIV1SRC)
						uint8_t const aPredivFactorTable[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
#else
				{1, 2};
#endif

				uint32_t pllclk = 0;
				uint32_t pllmul = aPLLMULFactorTable[(CFGR & RCC_CFGR_PLLMULL) >> RCC_CFGR_PLLMULL_Pos];
				if ((CFGR & RCC_CFGR_PLLSRC) != RCC_PLLSOURCE_HSI_DIV2) {
#ifdef RCC_CFGR2_PREDIV1SRC
					if (CFGR2 & RCC_CFGR2_PREDIV1SRC) {
						/* PLL2 selected as Prediv1 source */
						/* PLLCLK = PLL2CLK / PREDIV1 * PLLMUL with PLL2CLK = HSE/PREDIV2 * PLL2MUL */
						uint32_t const prediv2 = ((CFGR2 & RCC_CFGR2_PREDIV2) >> RCC_CFGR2_PREDIV2_Pos) + 1;
						uint32_t const pll2mul = ((CFGR2 & RCC_CFGR2_PLL2MUL) >> RCC_CFGR2_PLL2MUL_Pos) + 2;
						pllclk = (uint32_t)(((uint64_t)HSE_VALUE * (uint64_t)pll2mul * (uint64_t)pllmul) / ((uint64_t)prediv2 * (uint64_t)prediv));
					} else {
						/* HSE used as PLL clock source : PLLCLK = HSE/PREDIV1 * PLLMUL */
						pllclk = HSE_VALUE * pllmul / prediv;
					}

					/* If PLLMUL was set to 13 means that it was to cover the case PLLMUL 6.5 (avoid using float) */
					/* In this case need to divide pllclk by 2 */
					if (pllmul == aPLLMULFactorTable[RCC_CFGR_PLLMULL6_5 >> RCC_CFGR_PLLMULL_Pos])
					{
						pllclk = pllclk / 2;
					}
#else
					/* HSE used as PLL clock source : PLLCLK = HSE/PREDIV1 * PLLMUL */
					pllclk = HSE_VALUE * pllmul /
#ifdef RCC_CFGR2_PREDIV1
							aPredivFactorTable[(CFGR2 & RCC_CFGR2_PREDIV1) >> RCC_CFGR2_PREDIV1_Pos];
#else
					aPredivFactorTable[(CFGR & RCC_CFGR_PLLXTPRE) >> RCC_CFGR_PLLXTPRE_Pos];
#endif /*RCC_CFGR2_PREDIV1SRC*/
				} else {
					pllclk = HSI_VALUE / 2 * pllmul;
				}
				SystemCoreClock = pllclk >> AHBPrescTable[(CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];
#endif
				/* HSI is selected as System Clock Source */
			} else {
				/* Check the HSI ready flag */
				if (auto const& temp_register = ((RCC_FLAG_HSIRDY >> 5U) == CR_REG_INDEX ? CR : (RCC_FLAG_HSIRDY >> 5U) == BDCR_REG_INDEX ? BDCR : CSR) & (1U << (RCC_FLAG_HSIRDY & RCC_FLAG_MASK)); temp_register == RESET)
					Error_Handler();

				SystemCoreClock = HSI_VALUE >> AHBPrescTable[(CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];
			}

			CFGR = (CFGR & ~RCC_CFGR_SW) | ClkInitStruct.SYSCLKSource;

			/* Get Start Tick */
			uint32_t const tickstart = HAL_GetTick();

			while ((CFGR & RCC_CFGR_SWS) != (ClkInitStruct.SYSCLKSource << RCC_CFGR_SWS_Pos))
				if ((HAL_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE)
					Error_Handler();
		}

#ifdef FLASH_ACR_LATENCY
		/* Decreasing the number of wait states because of lower CPU frequency */
		if (FLASH_LATENCY_2 < (FLASH->ACR & FLASH_ACR_LATENCY)) {
			/* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
			FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_LATENCY_2;
			/* Check that the new number of wait states is taken into account to access the Flash
		    memory by reading the FLASH_ACR register */
			if ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_LATENCY_2)
				Error_Handler();
		}
#endif /* FLASH_ACR_LATENCY */

		/*-------------------------- PCLK1 Configuration ---------------------------*/
		if ((ClkInitStruct.ClockType & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
			CFGR = (CFGR & ~RCC_CFGR_PPRE1) | ClkInitStruct.APB1CLKDivider;

		/*-------------------------- PCLK2 Configuration ---------------------------*/
		if ((ClkInitStruct.ClockType & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
			CFGR = (CFGR & ~RCC_CFGR_PPRE2) | (ClkInitStruct.APB2CLKDivider << 3);

		/* Configure the source of time base considering new system clocks settings*/
		HAL_InitTick(uwTickPrio);
	}

	void* operator new(size_t);

	void Error_Handler(void) {
		__disable_irq();
		while (true);
	}

	uint32_t get_HCLK() {
		return SystemCoreClock;
	}

	uint32_t get_PCLK1() {
		return get_HCLK() >> APBPrescTable[(CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos];
	}

	uint32_t get_PCLK2() {
		return get_HCLK() >> APBPrescTable[(CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos];
	}

	__IO uint32_t CR;
	__IO uint32_t CFGR;
	__IO uint32_t CIR;
	__IO uint32_t APB2RSTR;
	__IO uint32_t APB1RSTR;
	__IO uint32_t AHBENR;
	__IO uint32_t APB2ENR;
	__IO uint32_t APB1ENR;
	__IO uint32_t BDCR;
	__IO uint32_t CSR;
}& RCC1;

#endif /* SRC_RCC_H_ */
