/*
 * stm32f1xxgpio.h
 *
 *  Created on: 1 Jan 2021
 *      Author: krabb
 */

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_

#include "stm32f1xx.h"
#include "AFIO.h"
#include "EXTI.h"
#include <new>

#define  GPIO_NUMBER           16U
/* Definitions for bit manipulation of CRL and CRH register */
#define  GPIO_CR_MODE_INPUT         0x0ul /*!< 00: Input mode (reset state)  */
#define  GPIO_CR_CNF_ANALOG         0x0ul /*!< 00: Analog mode  */
#define  GPIO_CR_CNF_INPUT_FLOATING 0x4ul /*!< 01: Floating input (reset state)  */
#define  GPIO_CR_CNF_INPUT_PU_PD    0x8ul /*!< 10: Input with pull-up / pull-down  */
#define  GPIO_CR_CNF_GP_OUTPUT_PP   0x0ul /*!< 00: General purpose output push-pull  */
#define  GPIO_CR_CNF_GP_OUTPUT_OD   0x4ul /*!< 01: General purpose output Open-drain  */
#define  GPIO_CR_CNF_AF_OUTPUT_PP   0x8ul /*!< 10: Alternate function output Push-pull  */
#define  GPIO_CR_CNF_AF_OUTPUT_OD   0xCul /*!< 11: Alternate function output Open-drain  */

extern struct GPIO {
public:
	enum Port{ A, B, C };
	void* operator new(size_t, Port port);

	__IO uint32_t CRL;
	__IO uint32_t CRH;
	__IO uint32_t IDR;
	__IO uint32_t ODR;
	__IO uint32_t BSRR;
	__IO uint32_t BRR;
	__IO uint32_t LCKR;
}& GPIOA, &GPIOB, & GPIOC;

struct Pin {
public:
	~Pin();

	void enable_event(EXTI::TRIGGER const trigger);

	bool read() {
		return port.IDR & (1UL << pin);
	}

	void toggle() {
		port.BSRR = (port.ODR & (1UL << pin)) << 16U | (~port.ODR & (1UL << pin));
	}

	void write(bool state) {
		port.BSRR = state ? (1UL << pin) << 16UL : (1UL << pin);
	}

	bool lock() {
		__IO uint32_t tmp = GPIO_LCKR_LCKK | (1UL << pin);
		/* Set LCKx bit(s): LCKK='1' + LCK[15-0] */
		port.LCKR = tmp;
		/* Reset LCKx bit(s): LCKK='0' + LCK[15-0] */
		port.LCKR = 1UL << pin;
		/* Set LCKx bit(s): LCKK='1' + LCK[15-0] */
		port.LCKR = tmp;

		/* Read LCKK register. This read is mandatory to complete key lock sequence */
		tmp = port.LCKR;

		/* read again in order to confirm lock is active */
		if (port.LCKR & GPIO_LCKR_LCKK)
			return false; // 0 if succeeded.
		else
			return true;
	}

	operator uint32_t() const {
		return pin;
	}

	uint32_t get_pin() const {
		return pin;
	}

	void enable_interrupt(EXTI::TRIGGER trigger, void (*callback)()) const {
		exti.enable_line(pin, trigger, EXTI::MODE::Interrupt, callback);
	}

	void disable_interrupt() const {
		exti.disable_line(pin);
	}

protected:
	Pin(GPIO& port, uint32_t const pin);
	GPIO& port;
	uint32_t const pin;
};

struct Output_Pin : public Pin {
	enum Mode { PP = 1, AF_PP = 2, OD = 4, AF_OD = 8 };
	enum Speed { FREQ_LOW = GPIO_CRL_MODE0_1, FREQ_MEDIUM = GPIO_CRL_MODE0_0, FREQ_HIGH = GPIO_CRL_MODE0 };

	Output_Pin(GPIO& port, uint32_t const pin, Mode const mode, Speed const speed) : Pin{ port, pin } {
		uint32_t config = 0;
		if (mode & Mode::PP)
			config = speed + GPIO_CR_CNF_GP_OUTPUT_PP;
		else if (mode & Mode::OD)
			config = speed + GPIO_CR_CNF_GP_OUTPUT_OD;
		else if (mode & Mode::AF_PP)
			config = speed + GPIO_CR_CNF_AF_OUTPUT_PP;
		else if (mode & Mode::AF_OD)
			config = speed + GPIO_CR_CNF_AF_OUTPUT_OD;

		__IO uint32_t& configregister = pin < 8 ? port.CRL : port.CRH;
		uint32_t const registeroffset = pin < 8 ? pin << 2 : (pin - 8) << 2;
		configregister = (configregister & ~((GPIO_CRL_MODE0 | GPIO_CRL_CNF0) << registeroffset)) | (config << registeroffset);
	}
};

struct Input_Pin : public Pin {
	enum Pull { None, Up, Down };

	Input_Pin(GPIO& port, uint32_t const pin, Pull const pull, bool analog = false) : Pin{ port, pin } {
		uint32_t config = 0;
		if (analog) {
			config = GPIO_CR_MODE_INPUT + GPIO_CR_CNF_ANALOG;
		} else {
			switch (pull) {
			case Pull::Up:
				port.BSRR = 1UL << pin;
				config = GPIO_CR_MODE_INPUT + GPIO_CR_CNF_INPUT_PU_PD;
				break;

			case Pull::Down:
				port.BRR = 1UL << pin;
				config = GPIO_CR_MODE_INPUT + GPIO_CR_CNF_INPUT_PU_PD;
				break;

			case Pull::None:
			default:
				config = GPIO_CR_MODE_INPUT + GPIO_CR_CNF_INPUT_FLOATING;
				break;
			}
		}

		__IO uint32_t& configregister = pin < 8 ? port.CRL : port.CRH;
		uint32_t const registeroffset = pin < 8 ? pin << 2 : (pin - 8) << 2;
		configregister = (configregister & ~((GPIO_CRL_MODE0 | GPIO_CRL_CNF0) << registeroffset)) | (config << registeroffset);
	}
};


#endif /* SRC_GPIO_H_ */
