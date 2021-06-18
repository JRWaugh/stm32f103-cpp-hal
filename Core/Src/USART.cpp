/*
 * USART.cpp
 *
 *  Created on: 4 Jan 2021
 *      Author: krabb
 */

#include "USART.h"

static Mailbox mailboxes[3];

extern "C" {
void USART1_IRQHandler() {
	uart.IRQHandler(mailboxes[0]);
}
void USART2_IRQHandler() {

}
void USART3_IRQHandler() {

}
}

void USART::transmit(gsl::span<char const> tx_buffer) {
	auto& mailbox = get_mailbox();
	for (auto const c : tx_buffer) {
		while (mailbox.tx_buffer.full()) {
			CR1 |= USART_CR1_TXEIE;
			__WFI();
		}
		mailbox.tx_buffer.push_back(c);
		CR1 |= USART_CR1_TXEIE;
	}
}

void USART::receive(gsl::span<char> rx_buffer) {
	auto& mailbox = get_mailbox();
	mailbox.receive_pending = std::nullopt;
	while (mailbox.rx_buffer.size() < rx_buffer.size())
		__WFI();
	mailbox.rx_buffer.pop_front_to_buffer(rx_buffer.data(), rx_buffer.size());
}

void USART::receive_it(gsl::span<char> rx_buffer, void (*on_receive_complete)(gsl::span<char>)) {
	auto& mailbox = get_mailbox();
	if (mailbox.rx_buffer.size() >= rx_buffer.size()) {
		mailbox.rx_buffer.pop_front_to_buffer(rx_buffer.data(), rx_buffer.size());
		on_receive_complete(rx_buffer);
	} else
		mailbox.receive_pending = { rx_buffer, on_receive_complete };
}

Mailbox& USART::get_mailbox() const noexcept {
	switch (reinterpret_cast<int>(this)) {
	case USART1_BASE:
		return mailboxes[0];
	case USART2_BASE:
		return mailboxes[1];
	case USART3_BASE:
		return mailboxes[2];
	}
}
