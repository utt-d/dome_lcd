#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <mcp23s17.h>

void app_main() {
	mcp23s17_init();
	// make all I/O's output
	mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_IODIR, GPIOA, 0x00);
	mcp23s17_write_register(2, MCP23S17_DEFAULT_ADDR, MCP23S17_IODIR, GPIOA, 0x00);
	mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_IODIR, GPIOB, 0x00);
	mcp23s17_write_register(2, MCP23S17_DEFAULT_ADDR, MCP23S17_IODIR, GPIOB, 0x00);
    // set byte mode and seq mode
	mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_IOCON, GPIOA, 0b00100000);
	mcp23s17_write_register(2, MCP23S17_DEFAULT_ADDR, MCP23S17_IOCON, GPIOA, 0b00100000);
    // all bit HIGH
    mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOA, 0xff);
    mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, 0xff);
    mcp23s17_write_register(2, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOA, 0xff);
    mcp23s17_write_register(2, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, 0xff);


}


 