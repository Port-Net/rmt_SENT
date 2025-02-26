#pragma once

#include <Arduino.h>

#if __has_include("driver/rmt_common.h")
//#ifdef HAS_RMT_INCLUDES
#include <driver/rmt_common.h>
#include <driver/rmt_rx.h>
#else
typedef enum {
  // For CPU domain
  SOC_MOD_CLK_CPU = 1,                       /*!< CPU_CLK can be sourced from XTAL, PLL, or RC_FAST by configuring soc_cpu_clk_src_t */
  // For RTC domain
  SOC_MOD_CLK_RTC_FAST,                      /*!< RTC_FAST_CLK can be sourced from XTAL_D2 or RC_FAST by configuring soc_rtc_fast_clk_src_t */
  SOC_MOD_CLK_RTC_SLOW,                      /*!< RTC_SLOW_CLK can be sourced from RC_SLOW, XTAL32K, or RC_FAST_D256 by configuring soc_rtc_slow_clk_src_t */
  // For digital domain: peripherals, WIFI, BLE
  SOC_MOD_CLK_APB,                           /*!< APB_CLK is highly dependent on the CPU_CLK source */
  SOC_MOD_CLK_PLL_F80M,                      /*!< PLL_F80M_CLK is derived from PLL, and has a fixed frequency of 80MHz */
  SOC_MOD_CLK_PLL_F160M,                     /*!< PLL_F160M_CLK is derived from PLL, and has a fixed frequency of 160MHz */
  SOC_MOD_CLK_PLL_D2,                        /*!< PLL_D2_CLK is derived from PLL, it has a fixed divider of 2 */
  SOC_MOD_CLK_XTAL32K,                       /*!< XTAL32K_CLK comes from the external 32kHz crystal, passing a clock gating to the peripherals */
  SOC_MOD_CLK_RC_FAST,                       /*!< RC_FAST_CLK comes from the internal 20MHz rc oscillator, passing a clock gating to the peripherals */
  SOC_MOD_CLK_RC_FAST_D256,                  /*!< RC_FAST_D256_CLK comes from the internal 20MHz rc oscillator, divided by 256, and passing a clock gating to the peripherals */
  SOC_MOD_CLK_XTAL,                          /*!< XTAL_CLK comes from the external 40MHz crystal */
  SOC_MOD_CLK_TEMP_SENSOR,                   /*!< TEMP_SENSOR_CLK comes directly from the internal 20MHz rc oscillator */
  SOC_MOD_CLK_INVALID,                       /*!< Indication of the end of the available module clock sources */
} soc_module_clk_t;

typedef enum {
  RMT_CLK_SRC_APB = SOC_MOD_CLK_APB,         /*!< Select APB as the source clock */
  RMT_CLK_SRC_RC_FAST = SOC_MOD_CLK_RC_FAST, /*!< Select RC_FAST as the source clock */
  RMT_CLK_SRC_XTAL = SOC_MOD_CLK_XTAL,       /*!< Select XTAL as the source clock */
  RMT_CLK_SRC_DEFAULT = SOC_MOD_CLK_APB,     /*!< Select APB as the default choice */
} soc_periph_rmt_clk_src_t;

typedef enum {
  RMT_MEM_NUM_BLOCKS_1 = 1,
  RMT_MEM_NUM_BLOCKS_2 = 2,
#if SOC_RMT_TX_CANDIDATES_PER_GROUP > 2
  RMT_MEM_NUM_BLOCKS_3 = 3,
  RMT_MEM_NUM_BLOCKS_4 = 4,
#if SOC_RMT_TX_CANDIDATES_PER_GROUP > 4
  RMT_MEM_NUM_BLOCKS_5 = 5,
  RMT_MEM_NUM_BLOCKS_6 = 6,
  RMT_MEM_NUM_BLOCKS_7 = 7,
  RMT_MEM_NUM_BLOCKS_8 = 8,
#endif
#endif
} rmt_reserve_memsize__t;

#define RMT_SYMBOLS_PER_CHANNEL_BLOCK SOC_RMT_MEM_WORDS_PER_CHANNEL

typedef soc_periph_rmt_clk_src_t rmt_clock_source_t;

typedef union {
  struct {
      uint16_t duration0 : 15; /*!< Duration of level0 */
      uint16_t level0 : 1;     /*!< Level of the first part */
      uint16_t duration1 : 15; /*!< Duration of level1 */
      uint16_t level1 : 1;     /*!< Level of the second part */
  };
  uint32_t val; /*!< Equivalent unsigned value for the RMT symbol */
} rmt_symbol_word_t;

typedef struct {
  rmt_symbol_word_t *received_symbols; /*!< Point to the received RMT symbols */
  size_t num_symbols;                  /*!< The number of received RMT symbols */
} rmt_rx_done_event_data_t;

typedef struct rmt_channel_t *rmt_channel_handle_t;

typedef struct {
  gpio_num_t gpio_num;        /*!< GPIO number used by RMT RX channel. Set to -1 if unused */
  rmt_clock_source_t clk_src; /*!< Clock source of RMT RX channel, channels in the same group must use the same clock source */
  uint32_t resolution_hz;     /*!< Channel clock resolution, in Hz */
  size_t mem_block_symbols;   /*!< Size of memory block, in number of `rmt_symbol_word_t`, must be an even.
                                   In the DMA mode, this field controls the DMA buffer size, it can be set to a large value (e.g. 1024);
                                   In the normal mode, this field controls the number of RMT memory block that will be used by the channel. */
  struct {
      uint32_t invert_in: 1;    /*!< Whether to invert the incoming RMT channel signal */
      uint32_t with_dma: 1;     /*!< If set, the driver will allocate an RMT channel with DMA capability */
      uint32_t io_loop_back: 1; /*!< For debug/test, the signal output from the GPIO will be fed to the input path as well */
  } flags;                      /*!< RX channel config flags */
  int intr_priority;            /*!< RMT interrupt priority,
                                     if set to 0, the driver will try to allocate an interrupt with a relative low priority (1,2,3) */
} rmt_rx_channel_config_t;

typedef struct {
  uint32_t signal_range_min_ns; /*!< A pulse whose width is smaller than this threshold will be treated as glitch and ignored */
  uint32_t signal_range_max_ns; /*!< RMT will stop receiving if one symbol level has kept more than `signal_range_max_ns` */
} rmt_receive_config_t;

typedef bool (*rmt_rx_done_callback_t)(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_ctx);

typedef struct {
  rmt_rx_done_callback_t on_recv_done; /*!< Event callback, invoked when one RMT channel receiving transaction completes */
} rmt_rx_event_callbacks_t;

esp_err_t rmt_new_rx_channel(const rmt_rx_channel_config_t *config, rmt_channel_handle_t *ret_chan);
esp_err_t rmt_rx_register_event_callbacks(rmt_channel_handle_t rx_channel, const rmt_rx_event_callbacks_t *cbs, void *user_data);
esp_err_t rmt_enable(rmt_channel_handle_t channel);
esp_err_t rmt_receive(rmt_channel_handle_t rx_channel, void *buffer, size_t buffer_size, const rmt_receive_config_t *config);

#endif