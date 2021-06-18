#include "EXTI.h"
#include <new>

using callback = void (*)();
callback lines[16] { nullptr };

extern "C" {
void EXTI0_IRQHandler() {
	if (lines[0]) {
		lines[0]();
	}
	exti.clear_pending(0);
}
void EXTI1_IRQHandler() {
	if (lines[1]) {
		lines[1]();
	}
	exti.clear_pending(1);
}
void EXTI2_IRQHandler() {
	if (lines[2]) {
		lines[2]();
	}
	exti.clear_pending(2);
}
void EXTI3_IRQHandler() {
	if (lines[3]) {
		lines[3]();
	}
	exti.clear_pending(3);
}
void EXTI4_IRQHandler() {
	if (lines[4]) {
		lines[4]();
	}
	exti.clear_pending(4);
}
void EXTI9_5_IRQHandler() {
	for (size_t i = 5UL; i <= 9; ++i) {
		if (exti.get_pending(i)) {
			if (lines[i]) {
				lines[i]();
			}
			exti.clear_pending(i);
		}
	}
}
void EXTI15_10_IRQHandler() {
	for (size_t i = 10UL; i <= 15; ++i) {
		if (exti.get_pending(i)) {
			if (lines[i]) {
				lines[i]();
			}
			exti.clear_pending(i);
		}
	}
}
}

void* EXTI::operator new(size_t) {
	RCC1.APB2ENR |= RCC_APB2ENR_AFIOEN;
	(void)(RCC1.APB2ENR & RCC_APB2ENR_AFIOEN);

	return reinterpret_cast<void *>(EXTI_BASE);
}

void EXTI::enable_line(uint32_t line, TRIGGER trigger, MODE mode, void (*callback)()) {
	afio.EXTICR[line >> 2] = afio.EXTICR[line >> 2] & ~(0xF << (4 * (line & 3))) | (2 << (4 * (line & 3)));

	if (mode == MODE::Interrupt) {
		IMR |= 1UL << line;
		EMR &= ~(1UL << line);
	} else if (mode == MODE::Event) {
		IMR &= ~(1UL << line);
		EMR |= 1UL << line;
	}

	if (trigger & EXTI::TRIGGER::Rising)
		RTSR |= 1UL << line;

	if (trigger & EXTI::TRIGGER::Falling)
		FTSR |= 1UL << line;

	lines[line] = callback;

	if (line < 5)
		NVIC_EnableIRQ(static_cast<IRQn_Type>(line + EXTI0_IRQn));
	else if (line < 10)
		NVIC_EnableIRQ(EXTI9_5_IRQn);
	else
		NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void EXTI::disable_line(uint32_t line) {
	afio.EXTICR[line >> 2] = afio.EXTICR[line >> 2] & (0xF << (4 * (line & 3)));

	IMR &= ~(1UL << line);
	EMR &= ~(1UL << line);
	RTSR &= ~(1UL << line);
	FTSR &= ~(1UL << line);

	lines[line] = nullptr;

	if (line < 5)
		NVIC_DisableIRQ(static_cast<IRQn_Type>(line + EXTI0_IRQn));
	else if (line < 10)
		NVIC_DisableIRQ(EXTI9_5_IRQn);
	else
		NVIC_DisableIRQ(EXTI15_10_IRQn);
}
