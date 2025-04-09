#ifndef HAL_MMIO_HPP
#define HAL_MMIO_HPP

namespace hal::mmio {

/**
 * set the value of mem to value, i.e. performs ``mem = value``.
 *
 * @tparam T the register type
 * @param mem the memory location, i.e. the register
 * @param value the value to set
 */
template <typename T, typename U> void set(volatile T &mem, U value) {
  mem = static_cast<T>(value);
}

/**
 * set a bit fields value, i.e. performs ``mem = (mem & ~mask) | (value
 * << shift)``.
 *
 * Example: setting the bits 6-9 on a register to a value of 5.
 * ```
 *  enum {
 *    REG_ADDRESS = 0xdeadbeef,
 *    FIELD = 0b1111 << 6,
 *    FIELD_POS = 6
 *  }
 *
 *  volatile uint32_t& reg = *reinterpret_cast<voaltile uint32_t*>(REG_ADDRESS);
 *  hal::mmio::set(reg, FIELD, 5, FIELD_POS);
 * ```
 *
 * @tparam T the register type
 * @tparam U the mask type
 * @tparam V the value type
 * @param mem the memory location, i.e. the register
 * @param mask the mask used to clear the relevant bits in mem. A 1 in the mask
 * means that a bit will be cleared.
 * @param value the value to write
 * @param shift how much value is shifted to the left
 */
template <typename T, typename U, typename V>
void set(volatile T &mem, U mask, V value, unsigned shift = 0) {
  mem = (mem & ~static_cast<T>(mask)) |
        (static_cast<T>(value) << static_cast<T>(shift));
}

/**
 * set bits to 1 in mem, i.e. performs ``mem |= bits``.
 *
 * @tparam T the register type
 * @tparam U the bits type
 * @param mem the memory location, i.e. the register
 * @param bits a mask of bits to set
 */
template <typename T, typename U> void set_bits(volatile T &mem, U bits) {
  mem |= static_cast<T>(bits);
}

/**
 * reset bits to 0 in mem, i.e. performs ``mem &= ~bits``.
 *
 * @tparam T the register type
 * @tparam U the bits type
 * @param mem the memory location, i.e. the register
 * @param bits a mask of bits to reset
 */
template <typename T, typename U> void reset_bits(volatile T &mem, U bits) {
  mem &= ~static_cast<T>(bits);
}

/**
 * read bits of mem.
 *
 * @tparam T the register type
 * @tparam U the bits type
 * @param mem the memory location, i.e. the register
 * @param bits a mask of bits to read
 * @return mem & bits
 */
template <typename T, typename U> T get(volatile T &mem, U bits) {
  return mem & static_cast<T>(bits);
}

/**
 * read bits of mem and right shift the result by shift bits.
 *
 *
 * Example: reading the bits 6-9 of a register.
 * ```
 *  enum {
 *    REG_ADDRESS = 0xdeadbeef,
 *    FIELD = 0b1111 << 6,
 *    FIELD_POS = 6
 *  }
 *
 *  volatile uint32_t& reg = *reinterpret_cast<voaltile uint32_t*>(REG_ADDRESS);
 *  uint32_t field = hal::mmio::get(reg, FIELD, FIELD_POS);
 *  // can now use field without needing any more shifts.
 * ```
 *
 * @tparam T the register type
 * @tparam U the mask type
 * @param mem the memory location, i.e. the register
 * @param mask a mask of bits to read
 * @param shift the amount to right shift the read result.
 * @return (mem & mask) >> shift
 */
template <typename T, typename U>
T get(volatile T &mem, U mask, unsigned shift) {
  return (mem & static_cast<T>(mask)) >> static_cast<T>(shift);
}
} // namespace hal::mmio
#endif
