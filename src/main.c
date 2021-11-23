#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <mcp23s17.h>
#include "data.h"

#define LCD_H_SIZE 196
#define LCD_V_SIZE 196
#define CLK_1HSYNC_LOW 11
#define CLK_1HSYNC_HIGH 418
#define CLK_DE_PRE_LOW 42
#define CLK_DE_HIGH 320
#define CLK_DUMMY_DE_HIGH 62

// LCD表示用のRGBのバッファ(RGB565形式)
static uint16_t* g_lcd_buf;


// void loadImage(void *image, int x, int y, int w, int h){

// }

void update1HSYNCWithoutDE(uint8_t c_data){
	 uint16_t c;

	// 強制的にRGBをLowにする
	mcp23s17_write_register(2, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOA, 0x00);
	mcp23s17_write_register(2, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, 0x00);
	mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOA, 0x00);

	for(c=0;c<CLK_1HSYNC_LOW;c++){
		// 強制的にCLKをHigh, HSYNCをLow, DEをLowにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, (c_data & 0x1B) | 0x01);
		// 強制的にCLKをLow, HSYNCをLow, DEをLowにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, (c_data & 0x1B) & 0x00);
	}
	
	for(c=0;c<CLK_1HSYNC_HIGH;c++){
		// 強制的にCLKをHigh, HSYNCをHigh, DEをLowにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, ((c_data & 0x1B) | 0x04) | 0x01);
		// 強制的にCLKをLow, HSYNCをHigh, DEをLowにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, ((c_data & 0x1B) | 0x04) & 0x00);
	}
}


void update1HSYNCWithDE(uint16_t g_lcd_h_data[LCD_H_SIZE], uint8_t c_data){
	uint16_t c;

	// 強制的にRGBをLowにする
	mcp23s17_write_register(2, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOA, 0x00);
	mcp23s17_write_register(2, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, 0x00);
	mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOA, 0x00);

	for(c=0;c<CLK_1HSYNC_LOW;c++){
		// 強制的にCLKをHigh, HSYNCをLow, DEをLowにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, (c_data & 0x1B) | 0x01);
		// 強制的にCLKをLow, HSYNCをLow, DEをLowにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, (c_data & 0x1B) & 0x00);
	}
	
	for(c=0;c<CLK_DE_PRE_LOW-CLK_1HSYNC_LOW;c++){
		// 強制的にCLKをHigh, HSYNCをHigh, DEをLowにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, ((c_data & 0x1B) | 0x04) | 0x01);
		// 強制的にCLKをLow, HSYNCをHigh, DEをLowにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, ((c_data & 0x1B) | 0x04) & 0x00);
	}
	
	for(c=0;c<CLK_DUMMY_DE_HIGH;c++){
		// 強制的にCLKをHigh, HSYNCをHigh, DEをHighにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, (c_data | 0x24) | 0x01);
		// 強制的にCLKをLow, HSYNCをHigh, DEをHighにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, (c_data | 0x24) & 0x00);
	}

	for(c=0;c<LCD_H_SIZE;c++){
		// RGBデータを出力する
		mcp23s17_write_register(2, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOA, (g_lcd_h_data[c] & 0xF800) >> 11);
		mcp23s17_write_register(2, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, (g_lcd_h_data[c] & 0x07E0) >> 5);
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOA, g_lcd_h_data[c] & 0x001F);
		// 強制的にCLKをHigh, HSYNCをHigh, DEをHighにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, (c_data | 0x24) | 0x01);
		// 強制的にCLKをLow, HSYNCをHigh, DEをHighにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, (c_data | 0x24) & 0x00);
	}

	// 強制的にRGBをLowにする
	mcp23s17_write_register(2, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOA, 0x00);
	mcp23s17_write_register(2, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, 0x00);
	mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOA, 0x00);

	for(c=0;c<CLK_DUMMY_DE_HIGH;c++){
		// 強制的にCLKをHigh, HSYNCをHigh, DEをHighにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, (c_data | 0x24) | 0x01);
		// 強制的にCLKをLow, HSYNCをHigh, DEをHighにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, (c_data | 0x24) & 0x00);
	}

	for(c=0;c<CLK_1HSYNC_LOW+CLK_1HSYNC_HIGH-CLK_DE_PRE_LOW-CLK_DE_HIGH;c++){
		// 強制的にCLKをHigh, HSYNCをHigh, DEをLowにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, ((c_data & 0x1B) | 0x04) | 0x01);
		// 強制的にCLKをLow, HSYNCをHigh, DEをLowにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, ((c_data & 0x1B) | 0x04) & 0x00);
	}
}


void update1HSYNCWithDEDummy(uint8_t c_data){
	uint16_t c;

	// 強制的にRGBをLowにする
	mcp23s17_write_register(2, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOA, 0x00);
	mcp23s17_write_register(2, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, 0x00);
	mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOA, 0x00);

	for(c=0;c<CLK_1HSYNC_LOW;c++){
		// 強制的にCLKをHigh, HSYNCをLow, DEをLowにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, (c_data & 0x1B) | 0x01);
		// 強制的にCLKをLow, HSYNCをLow, DEをLowにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, (c_data & 0x1B) & 0x00);
	}
	
	for(c=0;c<CLK_DE_PRE_LOW-CLK_1HSYNC_LOW;c++){
		// 強制的にCLKをHigh, HSYNCをHigh, DEをLowにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, ((c_data & 0x1B) | 0x04) | 0x01);
		// 強制的にCLKをLow, HSYNCをHigh, DEをLowにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, ((c_data & 0x1B) | 0x04) & 0x00);
	}
	
	for(c=0;c<CLK_DUMMY_DE_HIGH;c++){
		// 強制的にCLKをHigh, HSYNCをHigh, DEをHighにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, (c_data | 0x24) | 0x01);
		// 強制的にCLKをLow, HSYNCをHigh, DEをHighにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, (c_data | 0x24) & 0x00);
	}

	for(c=0;c<LCD_H_SIZE;c++){
		// 強制的にCLKをHigh, HSYNCをHigh, DEをHighにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, (c_data | 0x24) | 0x01);
		// 強制的にCLKをLow, HSYNCをHigh, DEをHighにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, (c_data | 0x24) & 0x00);
	}

	// 強制的にRGBをLowにする
	mcp23s17_write_register(2, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOA, 0x00);
	mcp23s17_write_register(2, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, 0x00);
	mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOA, 0x00);

	for(c=0;c<CLK_DUMMY_DE_HIGH;c++){
		// 強制的にCLKをHigh, HSYNCをHigh, DEをHighにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, (c_data | 0x24) | 0x01);
		// 強制的にCLKをLow, HSYNCをHigh, DEをHighにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, (c_data | 0x24) & 0x00);
	}

	for(c=0;c<CLK_1HSYNC_LOW+CLK_1HSYNC_HIGH-CLK_DE_PRE_LOW-CLK_DE_HIGH;c++){
		// 強制的にCLKをHigh, HSYNCをHigh, DEをLowにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, ((c_data & 0x1B) | 0x04) | 0x01);
		// 強制的にCLKをLow, HSYNCをHigh, DEをLowにする
		mcp23s17_write_register(1, MCP23S17_DEFAULT_ADDR, MCP23S17_GPIO, GPIOB, ((c_data & 0x1B) | 0x04) & 0x00);
	}
}

 void updateLCD(void){
	 uint16_t h,v;
	 
	// HSYNCパルス幅: Min 2CLK, Max 20us
	// VSYNCパルス幅: Min 4CLK, Max none
	// DEパルス幅: Typ 320CLK

	// 入力タイミング例より
	// 1HSYNCは11CLKの間Lowの後418CLKの間Highで1単位
	// 1行目の表示前
	// 1.VSYNCを3HSYNCの時間だけLowにする
	// 1.DE, RGB信号を6HSYNCの時間だけLowにする
	// 2.DE, RGB信号に22HSYNCの時間だけダミーデータを入力する
	// 1行目の表示開始
	// 3.HSYNCを11CLKの時間だけLowにし、その後429CLKの時間だけHighにする
	// 3.DE, RGB信号を42CLKの時間だけLowにする
	// 4.DE信号を320CLKの時間だけHighにする
	// 4.RGB信号に62CLKの時間だけダミーデータを入力する
	// 5.RGB信号に196CLKの時間でデータを入力する(左から1ドットずつに並ぶ?)
	// 6.RGB信号に62CLKの時間だけダミーデータを入力する
	// 1行目の表示終了
	// 2行目の表示開始
	// 7.1行ごとに3番目から6番目まで実行.196行目まで繰り返す
	// 8.DE, RGB信号に22HSYNCの時間だけダミーデータを入力する
	// 9.1に戻る

	// 1.VSYNCを3HSYNCの時間だけLowにする
	// 1.DE, RGB信号を6HSYNCの時間だけLowにする
	for(h=0;h<3;h++){
		update1HSYNCWithoutDE(0x00);
	}

	// 1.DE, RGB信号を6HSYNCの時間だけLowにする
	for(h=0;h<3;h++){
		update1HSYNCWithoutDE(0x02);
	}

	// 2.DE, RGB信号に22HSYNCの時間だけダミーデータを入力する
	for(h=0;h<22;h++){
		update1HSYNCWithDEDummy(0x02);
	}

	// 3.HSYNCを11CLKの時間だけLowにし、その後429CLKの時間だけHighにする
	// 3.DE, RGB信号を42CLKの時間だけLowにする
	// 4.DE信号を320CLKの時間だけHighにする
	// 4.RGB信号に62CLKの時間だけダミーデータを入力する
	// 5.RGB信号に196CLKの時間でデータを入力する(左から1ドットずつに並ぶ?)
	// 6.RGB信号に62CLKの時間だけダミーデータを入力する
	// 7.1行ごとに3番目から6番目まで実行.196行目まで繰り返す
	for(v=0;v<LCD_V_SIZE;v++){
		update1HSYNCWithDE(&(g_lcd_buf[v]), 0x02);
	}

	// 8.DE, RGB信号に22HSYNCの時間だけダミーデータを入力する
	for(h=0;h<22;h++){
		update1HSYNCWithDEDummy(0x02);
	}

 }



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

	g_lcd_buf = img;

	while(1){
		updateLCD();
	}


}