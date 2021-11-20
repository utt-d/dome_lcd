#ifndef MCP23S17_H
#define MCP23S17_H

// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/FreeRTOSConfig.h"
// #include "freertos/semphr.h"
// #include "freertos/portmacro.h"
// #include "sdkconfig.h"
// #include "esp_log.h"
// #include "driver/spi_master.h"
// #include "driver/gpio.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <unistd.h>
#include "esp_log.h"
#include <sys/param.h>
#include "sdkconfig.h"
// Workaround: The driver depends on some data in the flash and cannot be placed to DRAM easily for
// now. Using the version in LL instead.
#define gpio_set_level  gpio_set_level_patch
#include "hal/gpio_ll.h"
/*
#include "driver/gpio.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/spi_reg.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "esp_heap_caps.h"
*/

#  define MCP23S17_HOST    SPI2_HOST

#  define PIN_NUM_MISO GPIO_NUM_4
#  define PIN_NUM_MOSI GPIO_NUM_5
#  define PIN_NUM_CLK  GPIO_NUM_6
#  define PIN_NUM_CS1  GPIO_NUM_7
#  define PIN_NUM_CS2  GPIO_NUM_8


#define MCP23S17_BUSY_TIMEOUT_MS  5

#define MCP23S17_CLK_FREQ         (10*1000*1000)   //When powered by 3.3V, MCP23S17 max freq is 10MHz
#define MCP23S17_INPUT_DELAY_NS   ((1000*1000*1000/MCP23S17_CLK_FREQ)/2+45)


// https://github.com/h1romas4/esp32-genesis-player/blob/master/components/mcp23s17/src/mcp23s17.h
#define MCP23S17_IODIRA		0x00
#define MCP23S17_IPOLA 		0x02
#define MCP23S17_GPINTENA 	0x04
#define MCP23S17_DEFVALA 	0x06
#define MCP23S17_INTCONA 	0x08
#define MCP23S17_IOCONA 	0x0A
#define MCP23S17_GPPUA 		0x0C
#define MCP23S17_INTFA 		0x0E
#define MCP23S17_INTCAPA 	0x10
#define MCP23S17_GPIOA 		0x12
#define MCP23S17_OLATA 		0x14

#define MCP23S17_IODIRB 	0x01
#define MCP23S17_IPOLB 		0x03
#define MCP23S17_GPINTENB 	0x05
#define MCP23S17_DEFVALB 	0x07
#define MCP23S17_INTCONB 	0x09
#define MCP23S17_IOCONB 	0x0B
#define MCP23S17_GPPUB 		0x0D
#define MCP23S17_INTFB 		0x0F
#define MCP23S17_INTCAPB 	0x11
#define MCP23S17_GPIOB 		0x13
#define MCP23S17_OLATB 		0x15

// 0 1 0 0 A2 A1 A0 RW
#define MCP23S17_DEFAULT_ADDR 0x40
#define MCP23S17_WRITE_REGISTER 0x00
#define MCP23S17_READ_REGISTER 0x01

typedef enum {
   MCP23S17_ERR_OK      = 0x00,
   MCP23S17_ERR_CONFIG  = 0x01,
   MCP23S17_ERR_INSTALL = 0x02,
   MCP23S17_ERR_FAIL    = 0x03
} mcp23s17_err_t;

typedef enum {
	MCP23S17_IODIR		= 0x00,
	MCP23S17_IPOL		= 0x01,
	MCP23S17_GPINTEN	= 0x02,
	MCP23S17_DEFVAL	    = 0x03,
	MCP23S17_INTCON	    = 0x04,
	MCP23S17_IOCON		= 0x05,
	MCP23S17_GPPU		= 0x06,
	MCP23S17_INTF		= 0x07,
	MCP23S17_INTCAP	    = 0x08,
	MCP23S17_GPIO		= 0x09,
	MCP23S17_OLAT		= 0x0A
} mcp23s17_reg_t;

typedef enum {
	GPIOA = 0x00,
	GPIOB = 0x01
} mcp23s17_gpio_t;

/// Configurations of the spi_mcp23s17
typedef struct {
    spi_host_device_t host; ///< The SPI host used, set before calling `spi_eeprom_init()`
    gpio_num_t cs_io;       ///< CS gpio number, set before calling `spi_eeprom_init()`
    gpio_num_t miso_io;     ///< MISO gpio number, set before calling `spi_eeprom_init()`
    bool intr_used;         ///< Whether to use polling or interrupt when waiting for write to be done. Set before calling `spi_eeprom_init()`.
} mcp23s17_config_t;

/// Context (config and data) of the spi_mcp23s17
struct mcp23s17_context_t{
    mcp23s17_config_t cfg;        ///< Configuration by the caller.
    spi_device_handle_t spi;    ///< SPI device handle
    xSemaphoreHandle ready_sem; ///< Semaphore for ready signal
};

typedef struct mcp23s17_context_t mcp23s17_context_t;

typedef struct mcp23s17_context_t* mcp23s17_handle_t;

void mcp23s17_init(void);
static esp_err_t mcp23s17_simple_cmd(mcp23s17_context_t *ctx, uint16_t cmd);
static esp_err_t mcp23s17_wait_done(mcp23s17_context_t* ctx);
static void cs_high(spi_transaction_t* t);
static void cs_low(spi_transaction_t* t);
void ready_rising_isr(void* arg);
esp_err_t spi_mcp23s17_deinit(mcp23s17_context_t* ctx);
esp_err_t spi_mcp23s17_init(const mcp23s17_config_t *cfg, mcp23s17_context_t** out_ctx);
static inline esp_err_t gpio_set_level_patch(gpio_num_t gpio_num, uint32_t level);
mcp23s17_err_t mcp23s17_write_register(mcp23s17_context_t* ctx, uint8_t addr, mcp23s17_reg_t reg, mcp23s17_gpio_t group, uint8_t data);
mcp23s17_err_t mcp23s17_write_register_seq(mcp23s17_context_t* ctx, uint8_t addr, mcp23s17_reg_t reg, mcp23s17_gpio_t group, uint8_t *data, size_t size);

#endif