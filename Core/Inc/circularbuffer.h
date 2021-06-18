/*
 * circularbuffer.h
 *
 *  Created on: 9 Feb 2020
 *      Author: Joshua
 */
#ifndef CIRCULARBUFFER_H_
#define CIRCULARBUFFER_H_

#include <utility>
#include <cstring>
/* Usage example */
/* circularbuffer<uint8_t, 4> cb;  This constructs a circular buffer of type uint8_t with a length of 4.
 * for(std::size_t i = 0; i < 6; ++i)
 *     cb.push_back(i);
 * Array as the loop iterates:
 * { 0 }, { 0, 1 }, { 0, 1, 2 }, { 0, 1, 2, 3 }, { 1, 2, 3, 4 }, { 2, 3, 4, 5 }
 * cb.push_front(7); // { 7, 2, 3, 4 }
 */

template <typename T, std::size_t N>
class circularbuffer {
public:
	using value_type = T;
	using size_type = std::size_t;
	using difference_type = std::ptrdiff_t;
	using reference = T&;
	using const_reference = const T&;
	using pointer = T *;
	using const_pointer = T const *;

	template <typename Pointer>
	class iterator_type {
	public:
		using iterator_category = std::bidirectional_iterator_tag;

		iterator_type(Pointer buffer, Pointer pos, size_type remaining) : buffer{ buffer }, pos{ pos }, remaining{ remaining } {}
		constexpr reference operator*() const noexcept {
			return *pos;
		}

		constexpr Pointer operator->() const noexcept {
			return pos;
		}

		constexpr iterator_type& operator++() noexcept {
			if (++pos == buffer + N)
				pos = buffer;
			--remaining;
			return *this;
		}

		constexpr iterator_type& operator--() noexcept {
			if (pos == buffer)
				pos = buffer + N - 1;
			else
				--pos;
			++remaining; // A bit buggy!
			return *this;
		}

		constexpr iterator_type operator++(int) noexcept {
			iterator temp = *this;
			this->operator++();
			return temp;
		}

		constexpr iterator_type operator--(int) noexcept {
			iterator temp = *this;
			this->operator--();
			return temp;
		}

		constexpr bool operator==(const iterator_type& rhs) const noexcept {
			return buffer == rhs.buffer && pos == rhs.pos && remaining == rhs.remaining;
		}

		constexpr bool operator!=(const iterator_type& rhs) const noexcept {
			return !(operator==(rhs));
		}



	private:
		Pointer 	buffer = nullptr;
		Pointer		pos = 0;
		size_type	remaining = 0;
	};

	using iterator = iterator_type<pointer>;
	using const_iterator = iterator_type<const_pointer>;
	using reverse_iterator = std::reverse_iterator<iterator>;
	using const_reverse_iterator = std::reverse_iterator<const_iterator>;

	constexpr circularbuffer() : buffer{ 0 }, tail{ buffer }, size_{ 0 }  {}

	template <typename... Ts>
	circularbuffer(Ts const &... args) : buffer{ std::forward<T>(args)... }, tail{ buffer }, size_{ sizeof...(Ts) } {}

	/* Elements are indexed from oldest to newest starting from 0, going to (size - 1) */
	reference operator[](std::size_t const index) { return buffer[(tail - buffer + index) % N]; }
	const_reference operator[](std::size_t const index) const { return buffer[(tail - buffer + index) % N]; }

	/* Returns a reference to the oldest element in the buffer */
	reference front() { return *tail; }
	const_reference front() const { return *tail; }

	/* Returns a reference to the newest element in the buffer */
	reference back() { return operator[](size_ - 1); }
	const_reference back() const { return operator[](size_ - 1); }


	iterator begin() { return iterator(buffer, tail, size_); }
	const_iterator cbegin() { return const_iterator(buffer, tail, size_); }
	iterator end() { return iterator(buffer, &operator[](size_), 0); }
	const_iterator cend() { return const_iterator(buffer, &operator[](size_), 0); }

	/* Pushes new data into the back of the buffer and evicts the oldest element if the buffer is full */
	void push_back(value_type const value) {
		if (size_ < N) {
			operator[](size_++) = value;
		} else {
			// Replace the oldest datum.
			*tail = value;

			// Update tail. If it points to one-past-the-end of underlying buffer, point it to the beginning of the buffer.
			if (++tail == buffer + N)
				tail = buffer;
		}
	}

	/* Pushes new data into the front of the buffer and evicts what was the newest element if the buffer is full */
	void push_front(value_type const value) {
		if (--tail < buffer)
			tail = buffer + N - 1;

		*tail = value;

		if (size_ < N)
			++size_;
	}

	// It is the user's responsibility to ensure that pop_count does not exceed size_
	constexpr void pop_back(size_type pop_count = 1) {
		size_ -= pop_count;
	}

	constexpr void pop_front(size_type pop_count = 1) {
		size_ -= pop_count;
		tail = &operator[](pop_count);
	}

	void pop_front_to_buffer(pointer dst_buffer, size_type pop_count) {
		size_type const end_count = buffer + N - tail;
		if (pop_count < end_count) {
			std::memcpy(dst_buffer, tail, sizeof(T) * pop_count);
		} else {
			std::memcpy(dst_buffer, tail, end_count);
			std::memcpy(dst_buffer + end_count, buffer, sizeof(T) * (pop_count - end_count));
		}
		pop_front(pop_count);
	}

	constexpr size_type size() const noexcept {
		return size_;
	}

	constexpr size_type max_size() const noexcept {
		return N;
	}

	bool empty() const noexcept {
		return size_ == 0;
	}

	bool full() const noexcept {
		return size_ == N;
	}

	auto data() const noexcept {
		return buffer;
	}

	void clear() {
		tail = buffer;
		size_ = 0;
	}

private:
	value_type buffer[N];
	pointer tail; /* Points to the oldest datum, although that gets muddied if you use push_front() */
	size_type volatile size_;
};

#endif /* CIRCULARBUFFER_H_ */
