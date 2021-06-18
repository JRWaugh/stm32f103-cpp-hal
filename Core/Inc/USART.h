/*
 * USART.h
 *
 *  Created on: 4 Jan 2021
 *      Author: krabb
 */

#ifndef SRC_USART_H_
#define SRC_USART_H_

#include "stm32f1xx.h"
#include <new>
#include "RCC.h"
#include "GPIO.h"
#include <optional>
#include <gsl/span>
#include "circularbuffer.h"


enum HW_CONTROL { NONE, RTS = USART_CR3_RTSE, CTS = USART_CR3_CTSE, BOTH = USART_CR3_RTSE | USART_CR3_CTSE };

#define UART_OVERSAMPLING_16                    0x00000000U
#if defined(USART_CR1_OVER8)
#define UART_OVERSAMPLING_8                     ((uint32_t)USART_CR1_OVER8)
#endif /* USART_CR1_OVER8 */

#define USART_DIV(_PCLK_, _BAUD_)      (((_PCLK_)*25U)/(4U*(_BAUD_)))
#define USART_DIVMANT(_PCLK_, _BAUD_)  (USART_DIV((_PCLK_), (_BAUD_))/100U)
#define USART_DIVFRAQ(_PCLK_, _BAUD_)  ((((USART_DIV((_PCLK_), (_BAUD_)) - (USART_DIVMANT((_PCLK_), (_BAUD_)) * 100U)) * 16U) + 50U) / 100U)

#define UART_BRR_SAMPLING16(_PCLK_, _BAUD_)            (((USART_DIVMANT((_PCLK_), (_BAUD_)) << 4U) + \
		(USART_DIVFRAQ((_PCLK_), (_BAUD_)) & 0xF0U)) + \
		(USART_DIVFRAQ((_PCLK_), (_BAUD_)) & 0x0FU))

#define UART_DIV(_PCLK_, _BAUD_)             (((_PCLK_)*25U)/(2U*(_BAUD_)))
#define UART_DIVMANT(_PCLK_, _BAUD_)         (UART_DIV((_PCLK_), (_BAUD_))/100U)
#define UART_DIVFRAQ(_PCLK_, _BAUD_)         ((((UART_DIV((_PCLK_), (_BAUD_)) - (UART_DIVMANT((_PCLK_), (_BAUD_)) * 100U)) * 8U) + 50U) / 100U)

#define UART_BRR_SAMPLING8(_PCLK_, _BAUD_)             (((UART_DIVMANT((_PCLK_), (_BAUD_)) << 4U) + \
		((UART_DIVFRAQ((_PCLK_), (_BAUD_)) & 0xF8U) << 1U)) + \
		(UART_DIVFRAQ((_PCLK_), (_BAUD_)) & 0x07U))

#define USART_BRR(_PCLK_, _BAUD_)      (((USART_DIVMANT((_PCLK_), (_BAUD_)) << 4U) + \
		((USART_DIVFRAQ((_PCLK_), (_BAUD_)) & 0xF0U) << 1U)) + \
		(USART_DIVFRAQ((_PCLK_), (_BAUD_)) & 0x0FU))

struct Message {
	Message(gsl::span<char> buffer, void (*callback)(gsl::span<char>)) : buffer{ buffer }, callback{ callback } {}
	gsl::span<char> buffer;
	void (*callback)(gsl::span<char>);
};

struct Mailbox {
	friend class USART;
private:
	circularbuffer<char, 64> tx_buffer;
	circularbuffer<char, 64> rx_buffer;
	std::optional<Message> receive_pending;
};

extern class USART {
public:
	enum FLAG { TXE = USART_SR_TXE, TC = USART_SR_TC, RXNE = USART_SR_RXNE, IDLE = USART_SR_IDLE,
		ORE = USART_SR_ORE, NE = USART_SR_NE, FE = USART_SR_FE, PE = USART_SR_PE
	};

	enum NUMBER { USART_1 = USART1_BASE, USART_2 = USART2_BASE, USART_3 = USART3_BASE };

	USART(uint32_t BaudRate, uint32_t WordLength, uint32_t StopBits, uint32_t Parity, uint32_t Mode, uint32_t CLKPolarity, uint32_t CLKPhase, uint32_t CLKLastBit) {
		/* The LBCL, CPOL and CPHA bits have to be selected when both the transmitter and the
		     receiver are disabled (TE=RE=0) to ensure that the clock pulses function correctly.
		     At this point we also disable the peripheral (UE=0). */
		CR1 &= ~(USART_CR1_TE | USART_CR1_RE | USART_CR1_UE);

		/*---------------------------- USART CR2 Configuration ---------------------*/
		uint32_t tmpreg = CR2 & ~(USART_CR2_CPHA | USART_CR2_CPOL | USART_CR2_CLKEN | USART_CR2_LBCL | USART_CR2_STOP);
		tmpreg |= (USART_CLOCK_ENABLE | CLKPolarity | CLKPhase | CLKLastBit | StopBits);
		CR2 = tmpreg;

		/*-------------------------- USART CR1 Configuration -----------------------*/
		tmpreg = CR1 & ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS);
		tmpreg |= WordLength | Parity | Mode;
		CR1 = tmpreg;

		/*-------------------------- USART CR3 Configuration -----------------------*/
		CR3 &= ~(USART_CR3_RTSE | USART_CR3_CTSE);

		/*-------------------------- USART BRR Configuration -----------------------*/
		if(this == reinterpret_cast<USART*>(USART1_BASE))
			BRR = USART_BRR(RCC1.get_PCLK2(), BaudRate);
		else
			BRR = USART_BRR(RCC1.get_PCLK1(), BaudRate);

		/* In both USART and UART mode, the following bits must be kept cleared:
				     - LINEN bit in the USART_CR2 register
				     - HDSEL, SCEN and IREN bits in the USART_CR3 register */
		CR2 &= ~USART_CR2_LINEN;
		CR3 &= ~(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);

		/* Enable the Data Register Not Empty, Parity Error and Transmit Data Register Empty Interrupts */
		CR1 |= (USART_CR1_RXNEIE | USART_CR1_PEIE);
		/* Enable the Error Interrupt: (Frame error, noise error, overrun error) */
		CR3 |= USART_CR3_EIE;

		/* Enable the Peripheral */
		CR1 |= USART_CR1_UE;
	}

	void* operator new(size_t, NUMBER number) {
		switch (number) {
		case USART_1:
			RCC1.APB2ENR |= RCC_APB2ENR_USART1EN;
			(void)(RCC1.APB2ENR & RCC_APB2ENR_USART1EN);
			NVIC_EnableIRQ(USART1_IRQn);
			return reinterpret_cast<void*>(number);
		case USART_2:
			RCC1.APB1ENR |= RCC_APB1ENR_USART2EN;
			(void)(RCC1.APB1ENR & RCC_APB1ENR_USART2EN);
			return reinterpret_cast<void*>(number);
		case USART_3:
			RCC1.APB1ENR |= RCC_APB1ENR_USART3EN;
			(void)(RCC1.APB1ENR & RCC_APB1ENR_USART3EN);
			return reinterpret_cast<void*>(number);
		}
	}

	void transmit(gsl::span<char const>);
	void receive(gsl::span<char>);
	void receive_it(gsl::span<char>, void (*)(gsl::span<char>));

	void IRQHandler(Mailbox& mailbox) {
		uint32_t isrflags = SR;
		uint32_t cr1its = CR1;
		uint32_t cr3its = CR3;
		uint32_t errorcode = 0;
		// Receive Not Empty flag set
		if (isrflags & USART_SR_RXNE) {
			if ((CR1 & USART_WORDLENGTH_9B) == USART_WORDLENGTH_9B) {
				if ((CR1 & USART_PARITY_NONE) == USART_PARITY_NONE) {
					*reinterpret_cast<uint16_t*>(&mailbox.rx_buffer.back()) = DR & 0x1FF; // Bug
				} else {
					*reinterpret_cast<uint16_t*>(&mailbox.rx_buffer.back()) = DR & 0x0FF;
				}

				if (mailbox.tx_buffer.empty()) {
					/* Send dummy byte in order to generate the clock for the slave to send the next data */
					DR = 0x01FF;
				}
			} else {
				if ((CR1 & USART_PARITY_NONE) == USART_PARITY_NONE)
					mailbox.rx_buffer.push_back(DR & 0x0FF);
				else
					mailbox.rx_buffer.push_back(DR & 0x07F);

				if (mailbox.tx_buffer.empty()) {
					DR = 0x0FF;
				}
			}

			if (mailbox.receive_pending) {
				if (mailbox.receive_pending->buffer.size() <= mailbox.rx_buffer.size()) {
					mailbox.rx_buffer.pop_front_to_buffer(mailbox.receive_pending->buffer.data(), mailbox.receive_pending->buffer.size());
					mailbox.receive_pending->callback(mailbox.receive_pending->buffer);
					mailbox.receive_pending.reset();
				}
			}
		}

		// Transmit Empty flag set
		if (isrflags & USART_SR_TXE and mailbox.tx_buffer.size() > 0) {
			if ((CR1 & USART_WORDLENGTH_9B) == USART_WORDLENGTH_9B) {
				//DR = *reinterpret_cast<uint16_t const *>(&mailbox.tx_buffer[mailbox.transmitted++]) & 0x1FF;
				if ((CR1 & USART_PARITY_NONE) == USART_PARITY_NONE);
				//mailbox.transmitted++; // Bug
			} else {
				DR = mailbox.tx_buffer.front();
				mailbox.tx_buffer.pop_front();
			}

			if (mailbox.tx_buffer.empty())
				CR1 &= ~USART_CR1_TXEIE;
		}

		// Error handling. Can put callback stuff here later.
		if ((cr3its & USART_CR3_EIE) || (cr1its & (USART_CR1_RXNEIE | USART_CR1_PEIE))) {
			/* USART parity error interrupt occurred ----------------------------------*/
			if ((isrflags & USART_SR_PE) && (cr1its & USART_CR1_PEIE))
				errorcode |= HAL_USART_ERROR_PE;

			/* USART noise error interrupt occurred --------------------------------*/
			if ((isrflags & USART_SR_NE) && (cr3its & USART_CR3_EIE))
				errorcode |= HAL_USART_ERROR_NE;

			/* USART frame error interrupt occurred --------------------------------*/
			if ((isrflags & USART_SR_FE) && (cr3its & USART_CR3_EIE))
				errorcode |= HAL_USART_ERROR_FE;

			/* USART Over-Run interrupt occurred -----------------------------------*/
			if ((isrflags & USART_SR_ORE) && ((cr1its & USART_CR1_RXNEIE) || (cr3its & USART_CR3_EIE)))
				errorcode |= HAL_USART_ERROR_ORE;

			if (errorcode) {
				/* If Overrun error occurs, or if any error occurs in DMA mode reception, consider error as blocking */
				if ((errorcode & HAL_USART_ERROR_ORE) || (CR3 & USART_CR3_DMAR)) {
					/* Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
					//USART_EndRxTransfer(husart);
				}
			}
			return;
		}

	}

	__IO uint32_t SR;         /*!< USART Status register,                   Address offset: 0x00 */
	__IO uint32_t DR;         /*!< USART Data register,                     Address offset: 0x04 */
	__IO uint32_t BRR;        /*!< USART Baud rate register,                Address offset: 0x08 */
	__IO uint32_t CR1;        /*!< USART Control register 1,                Address offset: 0x0C */
	__IO uint32_t CR2;        /*!< USART Control register 2,                Address offset: 0x10 */
	__IO uint32_t CR3;        /*!< USART Control register 3,                Address offset: 0x14 */
	__IO uint32_t GTPR;       /*!< USART Guard time and prescaler register, Address offset: 0x18 */

private:
	Mailbox& get_mailbox() const noexcept;
}& usart;

extern class UART : public USART {
public:
	UART(uint32_t BaudRate, uint32_t WordLength, uint32_t StopBits, uint32_t Parity, uint32_t Mode,  HW_CONTROL hw_control, uint32_t OverSampling) :
		USART(BaudRate, WordLength, StopBits, Parity, Mode, 0, 0, 0) {
		CR1 &= ~USART_CR1_UE;

		/*-------------------------- UART CR2 Configuration -----------------------*/
		// In UART mode, CLKEN must be disabled.
		CR2 &= ~USART_CR2_CLKEN;

		/*-------------------------- UART CR1 Configuration -----------------------*/
#ifdef USART_CR1_OVER8
		CR1 &= ~USART_CR1_OVER8;
		CR1 |= OverSampling;
#endif
		/*-------------------------- UART CR3 Configuration -----------------------*/
		/* Configure the UART HFC: Set CTSE and RTSE bits according to huart->Init.HwFlowCtl value */
		CR3 |= hw_control;

		/*-------------------------- UART BRR Configuration -----------------------*/
		uint32_t pclk = 0;
		if (this == reinterpret_cast<USART*>(USART1_BASE))
			pclk = RCC1.get_PCLK2();
		else
			pclk = RCC1.get_PCLK1();
#ifdef USART_CR1_OVER8
		if (OverSampling == UART_OVERSAMPLING_8)
			BRR = UART_BRR_SAMPLING8(pclk, BaudRate);
		else
#endif /* USART_CR1_OVER8 */
			BRR = UART_BRR_SAMPLING16(pclk, BaudRate);

		CR1 |= USART_CR1_UE;
	}
}& uart;

#endif /* SRC_USART_H_ */
