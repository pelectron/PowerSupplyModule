#ifndef HAL_PWR_STM32C031XX_HPP
#define HAL_PWR_STM32C031XX_HPP

#include <cstdint>
namespace stm32c031xx {

enum PWR_CR1 : std::uint32_t {
  PWR_CR1_FPD_SLP = 1u << 5u,
  PWR_CR1_FPD_STOP = 1u << 3u,
  PWR_CR1_LPMS = 0b111u,
  PWR_CR1_LPMS_POS = 0
};

enum PWR_CR2 : std::uint32_t {
  PWR_CR2_PVM_VDDIO2 = 0b11u << 8,
  PWR_CR2_PVM_VDDIO2_POS = 8
};

enum PWR_CR3 : std::uint32_t {
  PWR_CR3_EIWUL = 1u << 15u,
  PWR_CR3_APC = 1u << 10u,
  PWR_CR3_EWUP6 = 1u << 5u,
  PWR_CR3_EWUP5 = 1u << 4u,
  PWR_CR3_EWUP4 = 1u << 3u,
  PWR_CR3_EWUP3 = 1u << 2u,
  PWR_CR3_EWUP2 = 1u << 1u,
  PWR_CR3_EWUP1 = 1u << 0u,
};

enum PWR_CR4 : std::uint32_t {
  PWR_CR4_WP6 = 1u << 5u,
  PWR_CR4_WP5 = 1u << 4u,
  PWR_CR4_WP4 = 1u << 3u,
  PWR_CR4_WP3 = 1u << 2u,
  PWR_CR4_WP2 = 1u << 1u,
  PWR_CR4_WP1 = 1u << 0u,
};

enum PWR_SR1 : std::uint32_t {
  PWR_SR1_WUIF = 1u << 15u,
  PWR_SR1_SBF = 1u << 8u,
  PWR_SR1_WUF6 = 1u << 5u,
  PWR_SR1_WUF5 = 1u << 4u,
  PWR_SR1_WUF4 = 1u << 3u,
  PWR_SR1_WUF3 = 1u << 2u,
  PWR_SR1_WUF2 = 1u << 1u,
  PWR_SR1_WUF1 = 1u << 0u,
};

enum PWR_SR2 : std::uint32_t {
  PWR_SR2_PVM_VDDIO2_OUT = 1u << 13u,
  PWR_SR2_FLASH_RDY = 1u << 7u
};

enum PWR_SCR : std::uint32_t {
  PWR_SCR_CSBF = 1u << 8u,
  PWR_SCR_CWUF6 = 1u << 5u,
  PWR_SCR_CWUF5 = 1u << 4u,
  PWR_SCR_CWUF4 = 1u << 3u,
  PWR_SCR_CWUF3 = 1u << 2u,
  PWR_SCR_CWUF2 = 1u << 1u,
  PWR_SCR_CWUF1 = 1u << 0u,
};

enum PWR_PUCRA : std::uint32_t {
  PWR_PUCRA_PU15 = 1u << 15u,
  PWR_PUCRA_PU14 = 1u << 14u,
  PWR_PUCRA_PU13 = 1u << 13u,
  PWR_PUCRA_PU12 = 1u << 12u,
  PWR_PUCRA_PU11 = 1u << 11u,
  PWR_PUCRA_PU10 = 1u << 10u,
  PWR_PUCRA_PU9 = 1u << 9u,
  PWR_PUCRA_PU8 = 1u << 8u,
  PWR_PUCRA_PU7 = 1u << 7u,
  PWR_PUCRA_PU6 = 1u << 6u,
  PWR_PUCRA_PU5 = 1u << 5u,
  PWR_PUCRA_PU4 = 1u << 4u,
  PWR_PUCRA_PU3 = 1u << 3u,
  PWR_PUCRA_PU2 = 1u << 2u,
  PWR_PUCRA_PU1 = 1u << 1u,
  PWR_PUCRA_PU0 = 1u << 0u,
};

enum PWR_PDCRA : std::uint32_t {
  PWR_PDCRA_PD15 = 1u << 15u,
  PWR_PDCRA_PD14 = 1u << 14u,
  PWR_PDCRA_PD13 = 1u << 13u,
  PWR_PDCRA_PD12 = 1u << 12u,
  PWR_PDCRA_PD11 = 1u << 11u,
  PWR_PDCRA_PD10 = 1u << 10u,
  PWR_PDCRA_PD9 = 1u << 9u,
  PWR_PDCRA_PD8 = 1u << 8u,
  PWR_PDCRA_PD7 = 1u << 7u,
  PWR_PDCRA_PD6 = 1u << 6u,
  PWR_PDCRA_PD5 = 1u << 5u,
  PWR_PDCRA_PD4 = 1u << 4u,
  PWR_PDCRA_PD3 = 1u << 3u,
  PWR_PDCRA_PD2 = 1u << 2u,
  PWR_PDCRA_PD1 = 1u << 1u,
  PWR_PDCRA_PD0 = 1u << 0u,
};

enum PWR_PUCRB : std::uint32_t {
  PWR_PUCRB_PU15 = 1u << 15u,
  PWR_PUCRB_PU14 = 1u << 14u,
  PWR_PUCRB_PU13 = 1u << 13u,
  PWR_PUCRB_PU12 = 1u << 12u,
  PWR_PUCRB_PU11 = 1u << 11u,
  PWR_PUCRB_PU10 = 1u << 10u,
  PWR_PUCRB_PU9 = 1u << 9u,
  PWR_PUCRB_PU8 = 1u << 8u,
  PWR_PUCRB_PU7 = 1u << 7u,
  PWR_PUCRB_PU6 = 1u << 6u,
  PWR_PUCRB_PU5 = 1u << 5u,
  PWR_PUCRB_PU4 = 1u << 4u,
  PWR_PUCRB_PU3 = 1u << 3u,
  PWR_PUCRB_PU2 = 1u << 2u,
  PWR_PUCRB_PU1 = 1u << 1u,
  PWR_PUCRB_PU0 = 1u << 0u,
};

enum PWR_PDCRB : std::uint32_t {
  PWR_PDCRB_PD15 = 1u << 15u,
  PWR_PDCRB_PD14 = 1u << 14u,
  PWR_PDCRB_PD13 = 1u << 13u,
  PWR_PDCRB_PD12 = 1u << 12u,
  PWR_PDCRB_PD11 = 1u << 11u,
  PWR_PDCRB_PD10 = 1u << 10u,
  PWR_PDCRB_PD9 = 1u << 9u,
  PWR_PDCRB_PD8 = 1u << 8u,
  PWR_PDCRB_PD7 = 1u << 7u,
  PWR_PDCRB_PD6 = 1u << 6u,
  PWR_PDCRB_PD5 = 1u << 5u,
  PWR_PDCRB_PD4 = 1u << 4u,
  PWR_PDCRB_PD3 = 1u << 3u,
  PWR_PDCRB_PD2 = 1u << 2u,
  PWR_PDCRB_PD1 = 1u << 1u,
  PWR_PDCRB_PD0 = 1u << 0u,
};

enum PWR_PUCRC : std::uint32_t {
  PWR_PUCRC_PU15 = 1u << 15u,
  PWR_PUCRC_PU14 = 1u << 14u,
  PWR_PUCRC_PU13 = 1u << 13u,
  PWR_PUCRC_PU12 = 1u << 12u,
  PWR_PUCRC_PU11 = 1u << 11u,
  PWR_PUCRC_PU10 = 1u << 10u,
  PWR_PUCRC_PU9 = 1u << 9u,
  PWR_PUCRC_PU8 = 1u << 8u,
  PWR_PUCRC_PU7 = 1u << 7u,
  PWR_PUCRC_PU6 = 1u << 6u,
  PWR_PUCRC_PU5 = 1u << 5u,
  PWR_PUCRC_PU4 = 1u << 4u,
  PWR_PUCRC_PU3 = 1u << 3u,
  PWR_PUCRC_PU2 = 1u << 2u,
  PWR_PUCRC_PU1 = 1u << 1u,
  PWR_PUCRC_PU0 = 1u << 0u,
};

enum PWR_PDCRC : std::uint32_t {
  PWR_PDCRC_PD15 = 1u << 15u,
  PWR_PDCRC_PD14 = 1u << 14u,
  PWR_PDCRC_PD13 = 1u << 13u,
  PWR_PDCRC_PD12 = 1u << 12u,
  PWR_PDCRC_PD11 = 1u << 11u,
  PWR_PDCRC_PD10 = 1u << 10u,
  PWR_PDCRC_PD9 = 1u << 9u,
  PWR_PDCRC_PD8 = 1u << 8u,
  PWR_PDCRC_PD7 = 1u << 7u,
  PWR_PDCRC_PD6 = 1u << 6u,
  PWR_PDCRC_PD5 = 1u << 5u,
  PWR_PDCRC_PD4 = 1u << 4u,
  PWR_PDCRC_PD3 = 1u << 3u,
  PWR_PDCRC_PD2 = 1u << 2u,
  PWR_PDCRC_PD1 = 1u << 1u,
  PWR_PDCRC_PD0 = 1u << 0u,
};

enum PWR_PUCRD : std::uint32_t {
  PWR_PUCRD_PU15 = 1u << 15u,
  PWR_PUCRD_PU14 = 1u << 14u,
  PWR_PUCRD_PU13 = 1u << 13u,
  PWR_PUCRD_PU12 = 1u << 12u,
  PWR_PUCRD_PU11 = 1u << 11u,
  PWR_PUCRD_PU10 = 1u << 10u,
  PWR_PUCRD_PU9 = 1u << 9u,
  PWR_PUCRD_PU8 = 1u << 8u,
  PWR_PUCRD_PU7 = 1u << 7u,
  PWR_PUCRD_PU6 = 1u << 6u,
  PWR_PUCRD_PU5 = 1u << 5u,
  PWR_PUCRD_PU4 = 1u << 4u,
  PWR_PUCRD_PU3 = 1u << 3u,
  PWR_PUCRD_PU2 = 1u << 2u,
  PWR_PUCRD_PU1 = 1u << 1u,
  PWR_PUCRD_PU0 = 1u << 0u,
};

enum PWR_PDCRD : std::uint32_t {
  PWR_PDCRD_PD15 = 1u << 15u,
  PWR_PDCRD_PD14 = 1u << 14u,
  PWR_PDCRD_PD13 = 1u << 13u,
  PWR_PDCRD_PD12 = 1u << 12u,
  PWR_PDCRD_PD11 = 1u << 11u,
  PWR_PDCRD_PD10 = 1u << 10u,
  PWR_PDCRD_PD9 = 1u << 9u,
  PWR_PDCRD_PD8 = 1u << 8u,
  PWR_PDCRD_PD7 = 1u << 7u,
  PWR_PDCRD_PD6 = 1u << 6u,
  PWR_PDCRD_PD5 = 1u << 5u,
  PWR_PDCRD_PD4 = 1u << 4u,
  PWR_PDCRD_PD3 = 1u << 3u,
  PWR_PDCRD_PD2 = 1u << 2u,
  PWR_PDCRD_PD1 = 1u << 1u,
  PWR_PDCRD_PD0 = 1u << 0u,
};

enum PWR_PUCRF : std::uint32_t {
  PWR_PUCRF_PU15 = 1u << 15u,
  PWR_PUCRF_PU14 = 1u << 14u,
  PWR_PUCRF_PU13 = 1u << 13u,
  PWR_PUCRF_PU12 = 1u << 12u,
  PWR_PUCRF_PU11 = 1u << 11u,
  PWR_PUCRF_PU10 = 1u << 10u,
  PWR_PUCRF_PU9 = 1u << 9u,
  PWR_PUCRF_PU8 = 1u << 8u,
  PWR_PUCRF_PU7 = 1u << 7u,
  PWR_PUCRF_PU6 = 1u << 6u,
  PWR_PUCRF_PU5 = 1u << 5u,
  PWR_PUCRF_PU4 = 1u << 4u,
  PWR_PUCRF_PU3 = 1u << 3u,
  PWR_PUCRF_PU2 = 1u << 2u,
  PWR_PUCRF_PU1 = 1u << 1u,
  PWR_PUCRF_PU0 = 1u << 0u,
};

enum PWR_PDCRF : std::uint32_t {
  PWR_PDCRF_PD15 = 1u << 15u,
  PWR_PDCRF_PD14 = 1u << 14u,
  PWR_PDCRF_PD13 = 1u << 13u,
  PWR_PDCRF_PD12 = 1u << 12u,
  PWR_PDCRF_PD11 = 1u << 11u,
  PWR_PDCRF_PD10 = 1u << 10u,
  PWR_PDCRF_PD9 = 1u << 9u,
  PWR_PDCRF_PD8 = 1u << 8u,
  PWR_PDCRF_PD7 = 1u << 7u,
  PWR_PDCRF_PD6 = 1u << 6u,
  PWR_PDCRF_PD5 = 1u << 5u,
  PWR_PDCRF_PD4 = 1u << 4u,
  PWR_PDCRF_PD3 = 1u << 3u,
  PWR_PDCRF_PD2 = 1u << 2u,
  PWR_PDCRF_PD1 = 1u << 1u,
  PWR_PDCRF_PD0 = 1u << 0u,
};

enum PWR_BKPR : std::uint32_t {
  BKP = 0xFFFFu,
  BKP_POS = 0,
};

} // namespace stm32c031xx
#endif
