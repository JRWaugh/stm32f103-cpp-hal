/*
 * EXTI.h
 *
 *  Created on: 2 Jan 2021
 *      Author: krabb
 */

#ifndef SRC_EXTI_H_
#define SRC_EXTI_H_

#include "stm32f1xx_hal.h"
#include "RCC.h"
#include "AFIO.h"

#if defined(EXTI_IMR_IM19)
#define EXTI_LINE_NB                        20UL
#elif defined(EXTI_IMR_IM18)
#define EXTI_LINE_NB                        19UL
#else /* EXTI_IMR_IM17 */
#define EXTI_LINE_NB                        18UL
#endif /* EXTI_IMR_IM19 */

extern struct EXTI {
	enum GPIO_LINE { LINE_0, LINE_1, LINE_2, LINE_3, LINE_4, LINE_5, LINE_6, LINE_7, LINE_8, LINE_9, LINE_10, LINE_11, LINE_12, LINE_13, LINE_14, LINE_15 };
	enum CONFIG_LINE { LINE_16, LINE_17
#if defined(EXTI_IMR_IM18)
		, LINE_18
#endif
#if defined(EXTI_IMR_IM19)
		, LINE_19
#endif
	};
	enum MODE { Interrupt, Event };
	enum TRIGGER { Falling = 1, Rising, Both };

	void* operator new(size_t);

	void enable_line(uint32_t line, TRIGGER trigger, MODE mode, void (*callback)());

	void disable_line(uint32_t line);

	bool get_pending(uint32_t line) {
		return PR & (1UL << line) ? true : false;
	}

	void clear_pending(uint32_t line) {
		PR = 1UL << line;
	}
	void generate_swi(uint32_t line) {
		SWIER = 1UL << line;
	}
	__IO uint32_t IMR;
	__IO uint32_t EMR;
	__IO uint32_t RTSR;
	__IO uint32_t FTSR;
	__IO uint32_t SWIER;
	__IO uint32_t PR;

private:
}& exti;


#endif /* SRC_EXTI_H_ */
