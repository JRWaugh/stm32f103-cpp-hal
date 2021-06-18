/*
 * I2C.h
 *
 *  Created on: 7 Jan 2021
 *      Author: krabb
 */

#ifndef SRC_I2C_H_
#define SRC_I2C_H_

#include "stm32f1xx.h"
#include "RCC.h"
#include <new>
#include <gsl/span>

#define I2C_DUTYCYCLE_2                 0x00000000U
#define I2C_DUTYCYCLE_16_9              I2C_CCR_DUTY
#define I2C_MIN_PCLK_FREQ(__PCLK__, __SPEED__)             (((__SPEED__) <= 100000U) ? ((__PCLK__) < I2C_MIN_PCLK_FREQ_STANDARD) : ((__PCLK__) < I2C_MIN_PCLK_FREQ_FAST))
#define I2C_CCR_CALCULATION(__PCLK__, __SPEED__, __COEFF__)     (((((__PCLK__) - 1U)/((__SPEED__) * (__COEFF__))) + 1U) & I2C_CCR_CCR)
#define I2C_FREQRANGE(__PCLK__)                            ((__PCLK__)/1000000U)
#define I2C_RISE_TIME(__FREQRANGE__, __SPEED__)            (((__SPEED__) <= 100000U) ? ((__FREQRANGE__) + 1U) : ((((__FREQRANGE__) * 300U) / 1000U) + 1U))
#define I2C_SPEED_STANDARD(__PCLK__, __SPEED__)            ((I2C_CCR_CALCULATION((__PCLK__), (__SPEED__), 2U) < 4U)? 4U:I2C_CCR_CALCULATION((__PCLK__), (__SPEED__), 2U))
#define I2C_SPEED_FAST(__PCLK__, __SPEED__, __DUTYCYCLE__) (((__DUTYCYCLE__) == I2C_DUTYCYCLE_2)? I2C_CCR_CALCULATION((__PCLK__), (__SPEED__), 3U) : (I2C_CCR_CALCULATION((__PCLK__), (__SPEED__), 25U) | I2C_DUTYCYCLE_16_9))
#define I2C_SPEED(__PCLK__, __SPEED__, __DUTYCYCLE__)      (((__SPEED__) <= 100000U)? (I2C_SPEED_STANDARD((__PCLK__), (__SPEED__))) : \
		((I2C_SPEED_FAST((__PCLK__), (__SPEED__), (__DUTYCYCLE__)) & I2C_CCR_CCR) == 0U)? 1U : \
				((I2C_SPEED_FAST((__PCLK__), (__SPEED__), (__DUTYCYCLE__))) | I2C_CCR_FS))

#define I2C_7BIT_ADD_WRITE(__ADDRESS__)                    ((uint8_t)((__ADDRESS__) & (uint8_t)(~I2C_OAR1_ADD0)))
#define I2C_7BIT_ADD_READ(__ADDRESS__)                     ((uint8_t)((__ADDRESS__) | I2C_OAR1_ADD0))

#define I2C_10BIT_ADDRESS(__ADDRESS__)                     ((uint8_t)((uint16_t)((__ADDRESS__) & (uint16_t)0x00FF)))
#define I2C_10BIT_HEADER_WRITE(__ADDRESS__)                ((uint8_t)((uint16_t)((uint16_t)(((uint16_t)((__ADDRESS__) & (uint16_t)0x0300)) >> 7) | (uint16_t)0x00F0)))
#define I2C_10BIT_HEADER_READ(__ADDRESS__)                 ((uint8_t)((uint16_t)((uint16_t)(((uint16_t)((__ADDRESS__) & (uint16_t)0x0300)) >> 7) | (uint16_t)(0x00F1))))

#define I2C_MEM_ADD_MSB(__ADDRESS__)                       ((uint8_t)((uint16_t)(((uint16_t)((__ADDRESS__) & (uint16_t)0xFF00)) >> 8)))
#define I2C_MEM_ADD_LSB(__ADDRESS__)                       ((uint8_t)((uint16_t)((__ADDRESS__) & (uint16_t)0x00FF)))

extern class I2C {
public:
	enum class AddressMode{ SevenBit = 0x00004000U, TenBit = SevenBit | I2C_OAR1_ADDMODE };
	I2C(uint32_t clock_speed, uint32_t duty_cycle, uint32_t address_1, AddressMode addr_mode, uint32_t address_2 = 0x0, bool gen_addr_mode = false, bool no_stretch_mode = false) {
		/* Disable the selected I2C peripheral */
		CR1 &= ~I2C_CR1_PE;

		/*Reset I2C*/
		CR1 |= I2C_CR1_SWRST;
		CR1 &= ~I2C_CR1_SWRST;

		/* Get PCLK1 frequency */
		uint32_t const pclk1 = RCC1.get_PCLK1();

		/* Check (and ignore!) the minimum allowed PCLK1 frequency */
		//auto const min_allowed = (clock_speed <= 100000U) ? (pclk1 < MIN_PCLK_FREQ_STANDARD) : (pclk1 < MIN_PCLK_FREQ_FAST);

		/* Calculate frequency range */
		uint32_t const freqrange = pclk1 / 1000000U;

		/*---------------------------- CR2 Configuration ----------------------*/
		/* Configure Frequency range */
		MODIFY_REG(CR2, I2C_CR2_FREQ, freqrange);

		/*---------------------------- TRISE Configuration --------------------*/
		/* Configure Rise Time */
		MODIFY_REG(TRISE, I2C_TRISE_TRISE, I2C_RISE_TIME(freqrange, clock_speed));

		/*---------------------------- CCR Configuration ----------------------*/
		/* Configure Speed */
		MODIFY_REG(CCR, (I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR), I2C_SPEED(pclk1, clock_speed, duty_cycle));

		/*---------------------------- CR1 Configuration ----------------------*/
		/* Configure Generalcall and NoStretch mode */
		MODIFY_REG(CR1, (I2C_CR1_ENGC | I2C_CR1_NOSTRETCH), (gen_addr_mode ? I2C_CR1_ENGC : 0U) | (no_stretch_mode ? I2C_CR1_NOSTRETCH : 0U));

		/*---------------------------- OAR1 Configuration ---------------------*/
		/* Configure Own Address1 and addressing mode */
		MODIFY_REG(OAR1, (I2C_OAR1_ADDMODE | I2C_OAR1_ADD8_9 | I2C_OAR1_ADD1_7 | I2C_OAR1_ADD0), ((uint32_t)addr_mode | address_1));

		/*---------------------------- OAR2 Configuration ---------------------*/
		/* Configure Dual mode and Own Address2 */
		if (address_2)
			MODIFY_REG(OAR2, (I2C_OAR2_ENDUAL | I2C_OAR2_ADD2), (I2C_OAR2_ENDUAL | address_2));

		/* Enable the selected I2C peripheral */
		CR1 |= I2C_CR1_PE;
	}

	enum NUMBER { I2C_1, I2C_2 };
	void* operator new(size_t, NUMBER number) {
		if (number == I2C_1) {
			RCC1.APB1ENR |= RCC_APB1ENR_I2C1EN;
			(void)(RCC1.APB1ENR & RCC_APB1ENR_I2C1EN);
			NVIC_EnableIRQ(I2C1_EV_IRQn);
			NVIC_EnableIRQ(I2C1_ER_IRQn);
			return reinterpret_cast<void *>(I2C1_BASE);
		} else {
			RCC1.APB1ENR |= RCC_APB1ENR_I2C2EN;
			(void)(RCC1.APB1ENR & RCC_APB1ENR_I2C2EN);
			NVIC_EnableIRQ(I2C2_EV_IRQn);
			NVIC_EnableIRQ(I2C2_ER_IRQn);
			return reinterpret_cast<void *>(I2C2_BASE);
		}
	}

	void generate_start_condition() {
		CR1 |= I2C_CR1_START;
	}

	void transmit(uint32_t address, gsl::span<char const> tx_buffer) noexcept {
		/* Wait until BUSY flag is reset */
		while (SR2 & I2C_FLAG_BUSY);

		/* Disable Pos */
		CR1 &= ~I2C_CR1_POS;

		/* Send Slave Address */
		generate_start_condition();

		/* Wait until SB flag is set */
		while (!(SR1 & I2C_FLAG_SB));

		if ((OAR1 & TenBit) == TenBit) {
			/* Send header of slave address */
			DR = (address & 0x300) >> 7 | 0xF0;
			/* Wait until ADD10 flag is set */
			while (!(SR1 & I2C_FLAG_ADD10)) {
				if (SR1 & I2C_FLAG_AF) {
					/* Generate Stop */
					CR1 |= I2C_CR1_STOP;
					/* Clear AF Flag */
					SR1 &= ~I2C_FLAG_AF;

					return;
				}
			}

			/* Send slave address */
			DR = address & 0xFF;
		} else {
			DR = address & ~I2C_OAR1_ADD0;
		}

		/* Wait until ADDR flag is set */
		while (!(SR1 & I2C_FLAG_ADDR));

		/* Clear ADDR flag */           \
		(void) SR1;
		(void) SR2;

	}

	__IO uint32_t CR1;
	__IO uint32_t CR2;
	__IO uint32_t OAR1;
	__IO uint32_t OAR2;
	__IO uint32_t DR;
	__IO uint32_t SR1;
	__IO uint32_t SR2;
	__IO uint32_t CCR;
	__IO uint32_t TRISE;

private:
	static constexpr auto MIN_PCLK_FREQ_STANDARD{ 2000000U };   // 2 MHz
	static constexpr auto MIN_PCLK_FREQ_FAST{ 4000000U };     	// 4 MHz
}& i2c;

#endif /* SRC_I2C_H_ */
