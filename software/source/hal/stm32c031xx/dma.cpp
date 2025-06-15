#include "hal/dma.hpp"

namespace stm32c0::dma {

enum class RequestInput {
  dmamux_gen0_dma = 1,
  dmamux_gen1_dma = 2,
  dmamux_gen2_dma = 3,
  dmamux_gen3_dma = 4,
  adc1_dma = 5,
  i2c1_rx_dma = 10,
  i2c1_tx_dma = 11,
  i2c2_rx_dma = 12,
  i2c2_tx_dma = 13,
  spi2s1_rx_dma = 16,
  spi2s1_tx_dma = 17,
  spi2_rx_dma = 18,
  spi2_tx_dma = 19,
  tim1_ch1_dma = 20,
  tim1_ch2_dma = 21,
  tim1_ch3_dma = 22,
  tim1_ch4_dma = 23,
  tim1_trgi_com_dma = 24,
  tim1_up_dma = 25,
  tim2_ch1_dma = 26,
  tim2_ch2_dma = 27,
  tim2_ch3_dma = 28,
  tim2_ch4_dma = 29,
  tim2_trgi_dma = 30,
  tim2_up_dma = 31,
  tim3_ch1_dma = 32,
  tim3_ch2_dma = 33,
  tim3_ch3_dma = 34,
  tim3_ch4_dma = 35,
  tim3_trgi_dma = 36,
  tim3_up_dma = 37,
  tim15_ch1_dma = 40,
  tim15_ch2_dma = 41,
  tim15_trgi_com_dma = 42,
  tim15_up_dma = 43,
  tim16_ch1_dma = 44,
  tim16_trgi_com_dma = 45,
  tim16_up_dma = 46,
  tim17_ch1_dma = 47,
  tim17_trgi_com_dma = 48,
  tim17_up_dma = 49,
  usart1_rx_dma = 50,
  usart1_tx_dma = 51,
  usart2_rx_dma = 52,
  usart2_tx_dma = 53,
  usart3_rx_dma = 54,
  usart3_tx_dma = 55,
  usart4_rx_dma = 56,
  usart4_tx_dma = 57,
};

enum class TriggerInput {
  EXTI0 = 0,
  EXTI1 = 1,
  EXTI2 = 2,
  EXTI3 = 3,
  EXTI4 = 4,
  EXTI5 = 5,
  EXTI6 = 6,
  EXTI7 = 7,
  EXTI8 = 8,
  EXTI9 = 9,
  EXTI10 = 10,
  EXTI11 = 11,
  EXTI12 = 12,
  EXTI13 = 13,
  EXTI14 = 14,
  EXTI15 = 15,
  dmamux_evt0 = 16,
  dmamux_evt1 = 17,
  dmamux_evt2 = 18,
  dmamux_evt3 = 19,
  tim14_tgro = 22
};

enum class SyncInput {
  EXTI0 = 0,
  EXTI1 = 1,
  EXTI2 = 2,
  EXTI3 = 3,
  EXTI4 = 4,
  EXTI5 = 5,
  EXTI6 = 6,
  EXTI7 = 7,
  EXTI8 = 8,
  EXTI9 = 9,
  EXTI10 = 10,
  EXTI11 = 11,
  EXTI12 = 12,
  EXTI13 = 13,
  EXTI14 = 14,
  EXTI15 = 15,
  dmamux_evt0 = 16,
  dmamux_evt1 = 17,
  dmamux_evt2 = 18,
  dmamux_evt3 = 19,
  tim14_tgro = 21
};

#define DMAMUX reinterpret_cast<stm32c0::Mux *>(0x40020800u)
#define DMA1 reinterpret_cast<stm32c0::DMA *>(0x40020800u)

struct Mux {
  volatile std::uint32_t CxCR[4];
  std::uint32_t reserved0[26];
  volatile std::uint32_t CSR;
  volatile std::uint32_t CFR;
  std::uint32_t reserved1[29];
  volatile std::uint32_t RGxCR[4];
  std::uint32_t reserved2[44];
  volatile std::uint32_t RGSR;
  volatile std::uint32_t RGCFR;
  std::uint32_t reserved3[173];
};

struct Channel {
  volatile uint32_t CCR;   /*!< DMA channel x configuration register        */
  volatile uint32_t CNDTR; /*!< DMA channel x number of data register       */
  volatile uint32_t CPAR;  /*!< DMA channel x peripheral address register   */
  volatile uint32_t CMAR;  /*!< DMA channel x memory address register       */
};

enum IRQ_Flags : std::uint32_t {
  TEIF7 = 1u << 27u,
  HTIF7 = 1u << 26u,
  TCIF7 = 1u << 25u,
  GIF7 = 1u << 24u,
  TEIF6 = 1u << 23u,
  HTIF6 = 1u << 22u,
  TCIF6 = 1u << 21u,
  GIF6 = 1u << 20u,
  TEIF5 = 1u << 19u,
  HTIF5 = 1u << 18u,
  TCIF5 = 1u << 17u,
  GIF5 = 1u << 16u,
  TEIF4 = 1u << 15u,
  HTIF4 = 1u << 14u,
  TCIF4 = 1u << 13u,
  GIF4 = 1u << 12u,
  TEIF3 = 1u << 11u,
  HTIF3 = 1u << 10u,
  TCIF3 = 1u << 9u,
  GIF3 = 1u << 8u,
  TEIF2 = 1u << 7u,
  HTIF2 = 1u << 6u,
  TCIF2 = 1u << 5u,
  GIF2 = 1u << 4u,
  TEIF1 = 1u << 3u,
  HTIF1 = 1u << 2u,
  TCIF1 = 1u << 1u,
  GIF1 = 1u << 0u,
};

struct DMA {
  volatile uint32_t ISR;
  volatile uint32_t IFCR;
  Channel channel1;
  std::uint32_t reserved1;
  Channel channel2;
  std::uint32_t reserved2;
  Channel channel3;
  std::uint32_t reserved3;
  Channel channel4;
  std::uint32_t reserved4;
  Channel channel5;
  std::uint32_t reserved5;
  Channel channel6;
  std::uint32_t reserved6;
  Channel channel7;
};
} // namespace stm32c0::dma

namespace hal::dma {

Error initiate(const Transfer &transfer, Callback &callback) {
  using namespace stm32c0::dma;
  if (auto src_p = std::get_if<Peripheral>(&transfer.src)) {
    if (auto dest_p = std::get_if<Peripheral>(&transfer.dst)) {
      // peripheral to peripheral
      switch (*src_p) {
      default:
        return Error::invalid_param;
      }
    } else {
      // peripheral to memory
      Block dest_mem = std::get<Block>(transfer.dst);
      switch (*src_p) {
      default:
        return Error::invalid_param;
      }
    }
  } else {
    Block src_mem = std::get<Block>(transfer.src);
    if (auto dest_p = std::get_if<Peripheral>(&transfer.dst)) {
      // memory to peripheral
      switch (*dest_p) {
      default:
        return Error::invalid_param;
      }
    } else {
      // memory to memory
      Block dest_mem = std::get<Block>(transfer.dst);
    }
  }
}
} // namespace hal::dma

extern "C" {
void DMA1_Channel1_IRQHandler(void) {}
void DMA1_Channel2_3_IRQHandler(void) {}
void DMA1_Channel4_5_IRQHandler(void) {}
void DMAMUX1_IRQHandler(void) {}
}
