#include <mcp23s17.h>


// SPI通信用のバッファ
uint8_t *spi_buf;

static const char TAG[] = "mcp23s17";
mcp23s17_handle_t mcp23s17_1_handle;
mcp23s17_handle_t mcp23s17_2_handle;

// MCP23S17の初期化
void mcp23s17_init(void){

    // エラー識別用の変数
    esp_err_t ret;

    // ログに初期化中のメッセージを出力
    ESP_LOGI(TAG, "Initializing bus SPI%d...", MCP23S17_HOST+1);

    // SPIバスの設定用変数
    spi_bus_config_t buscfg={
        // MOSIのGPIOの番号を設定
        .mosi_io_num = PIN_NUM_MOSI,
        // MISOのGPIOの番号を設定
        .miso_io_num = PIN_NUM_MISO,
        // SCKのGPIOの番号を設定
        .sclk_io_num = PIN_NUM_CLK,
        // WP(ライトプロテクト)のGPIOの番号を設定(使用しない場合は-1に設定)
        .quadwp_io_num = -1,
        // HD(ホールド)のGPIOの番号を設定(使用しない場合は-1に設定)
        .quadhd_io_num = -1,
        // データ転送量の最大値をバイト単位で設定(0の場合は4094バイト)
        .max_transfer_sz = 32,
    };
    // SPIバスの初期化
    // 今回はDMAを使用するので、第三引数にSPI_DMA_CH_AUTOを指定
    // DMAを使用しない場合はSPI_DMA_DISABLEDを指定する
    ret = spi_bus_initialize(MCP23S17_HOST, &buscfg, SPI_DMA_CH_AUTO);

    // SPIバス初期化後のエラーを確認
    ESP_ERROR_CHECK(ret);

    // SPIバスのMCP23S17(1番目)に関連する設定
    mcp23s17_config_t mcp23s17_1_config = {
        // 使用するSPIのペリフェラルを設定
        .host = MCP23S17_HOST,
        // CSのGPIOの番号を設定 
        .cs_io = PIN_NUM_CS1,
        // MISOのGPIOの番号を設定
        .miso_io = PIN_NUM_MISO,
    };

    // SPIバスのMCP23S17(2番目)に関連する設定
    mcp23s17_config_t mcp23s17_2_config = {
        // 使用するSPIのペリフェラルを設定
        .host = MCP23S17_HOST,
        // CSのGPIOの番号を設定 
        .cs_io = PIN_NUM_CS2,
        // MISOのGPIOの番号を設定
        .miso_io = PIN_NUM_MISO,
    };
    // SPIの書き込み完了待ちにおいて、割り込み動作を有効にする
    mcp23s17_1_config.intr_used = true;
    mcp23s17_2_config.intr_used = true;
    gpio_install_isr_service(0);

    // SPIの初期化
    ESP_LOGI(TAG, "Initializing device...");
    ret = spi_mcp23s17_init(&mcp23s17_1_config, &mcp23s17_1_handle);
    ESP_ERROR_CHECK(ret);
    ret = spi_mcp23s17_init(&mcp23s17_2_config, &mcp23s17_2_handle);
    ESP_ERROR_CHECK(ret);

    // バッファの確保
    spi_buf = (uint8_t *)heap_caps_malloc(64, MALLOC_CAP_8BIT);

}


static inline esp_err_t gpio_set_level_patch(gpio_num_t gpio_num, uint32_t level)
{
    gpio_ll_set_level(&GPIO, gpio_num, level);
    return ESP_OK;
}


static esp_err_t mcp23s17_simple_cmd(mcp23s17_context_t *ctx, uint16_t cmd)
{
    spi_transaction_t t = {
        .cmd = cmd,
        .user = ctx
    };
    return spi_device_polling_transmit(ctx->spi, &t);
}

static esp_err_t mcp23s17_wait_done(mcp23s17_context_t* ctx)
{
    //have to keep cs low for 250ns
    usleep(1);
    //clear signal
    if (ctx->cfg.intr_used) {
        xSemaphoreTake(ctx->ready_sem, 0);
        gpio_set_level(ctx->cfg.cs_io, 1);
        gpio_intr_enable(ctx->cfg.miso_io);

        //Max processing time is 5ms, tick=1 may happen very soon, set to 2 at least
        uint32_t tick_to_wait = MAX(MCP23S17_BUSY_TIMEOUT_MS / portTICK_PERIOD_MS, 2);
        BaseType_t ret = xSemaphoreTake(ctx->ready_sem, tick_to_wait);
        gpio_intr_disable(ctx->cfg.miso_io);
        gpio_set_level(ctx->cfg.cs_io, 0);

        if (ret != pdTRUE) return ESP_ERR_TIMEOUT;
    } else {
        bool timeout = true;
        gpio_set_level(ctx->cfg.cs_io, 1);
        for (int i = 0; i < MCP23S17_BUSY_TIMEOUT_MS * 1000; i ++) {
            if (gpio_get_level(ctx->cfg.miso_io)) {
                timeout = false;
                break;
            }
            usleep(1);
        }
        gpio_set_level(ctx->cfg.cs_io, 0);
        if (timeout) return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static void cs_high(spi_transaction_t* t)
{
    ESP_EARLY_LOGV(TAG, "cs high %d.", ((mcp23s17_context_t*)t->user)->cfg.cs_io);
    gpio_set_level(((mcp23s17_context_t*)t->user)->cfg.cs_io, 1);
}

static void cs_low(spi_transaction_t* t)
{
    gpio_set_level(((mcp23s17_context_t*)t->user)->cfg.cs_io, 0);
    ESP_EARLY_LOGV(TAG, "cs low %d.", ((mcp23s17_context_t*)t->user)->cfg.cs_io);
}

void ready_rising_isr(void* arg)
{
    mcp23s17_context_t* ctx = (mcp23s17_context_t*)arg;
    xSemaphoreGive(ctx->ready_sem);
    ESP_EARLY_LOGV(TAG, "ready detected.");
}

esp_err_t spi_mcp23s17_deinit(mcp23s17_context_t* ctx)
{
    spi_bus_remove_device(ctx->spi);
    if (ctx->cfg.intr_used) {
        vSemaphoreDelete(ctx->ready_sem);
    }
    free(ctx);
    return ESP_OK;
}

// SPIの初期化
esp_err_t spi_mcp23s17_init(const mcp23s17_config_t *cfg, mcp23s17_context_t** out_ctx)
{
    // エラーの状態を示す変数を初期化
    esp_err_t err = ESP_OK;

    // SPI1を使用して、なおかつ
    // SPIの書き込み完了待ちにおいて、割り込み動作を有効にしようとした場合
    if (cfg->intr_used && cfg->host == SPI1_HOST) {
        // ログに、SPI1では割り込み動作が使用不可の旨のメッセージを出力
        ESP_LOGE(TAG, "interrupt cannot be used on SPI1 host.");
        // エラーの状態を示す変数として、引数が不正であることを示すエラーを返して終了
        return ESP_ERR_INVALID_ARG;
    }

    // mcp23s17の設定用の構造体のメモリを確保
    mcp23s17_context_t* ctx = (mcp23s17_context_t*)malloc(sizeof(mcp23s17_context_t));
    
    // メモリを確保できなければメモリがないことを示すエラーを返して終了
    if (!ctx) return ESP_ERR_NO_MEM;

    // mcp23s17の設定用の構造体にSPIの設定用の構造体を設定
    *ctx = (mcp23s17_context_t) {
        .cfg = *cfg,
    };

    // SPIの各接続デバイスの設定用構造体を宣言
    // 下記をもとに設定
    // https://github.com/h1romas4/esp32-genesis-player/blob/master/components/mcp23s17/src/mcp23s17.c
    spi_device_interface_config_t devcfg={
        // コマンドとアドレスのデータ長を0に設定
        // (設定を使用しない)
        .command_bits = 0,
        .address_bits = 0,

        // SPIのモードを0に設定
        // CPOL: 0, CPHA: 0
        .mode = 0,          //SPI mode 0

        // 通信時の周波数を設定
        .clock_speed_hz = MCP23S17_CLK_FREQ,

        // SCLK切り替えタイミングからスレーブ側のデータ送信が切り替わるタイミングまでの遅延時間をns単位で設定
        // MCP23S17ではSCLKがLowに落ちた後にMISOが次のデータに切り替わるまで最大45nsかかる(2.7～5.5V)らしいので
        // とりあえずクロック半周期分 + 45nsで設定しておく
        .input_delay_ns = MCP23S17_INPUT_DELAY_NS,

        /*
         * The timing requirements to read the busy signal from the EEPROM cannot be easily emulated
         * by SPI transactions. We need to control CS pin by SW to check the busy signal manually.
         */
        // 元にしたEEPROM用のSPIではCSをGPIOとして動作するように設定し、手動で切り替えていたが
        // 今回はGPIOに設定せず、CSを自動で切り替えるようにする

        // CSのGPIOの番号を設定
        .spics_io_num = cfg -> cs_io,

        // 特殊な設定
        // デフォルトではMSBが最初に送信される
        // MCP23S17も同じなのでそのままにしておく(LSBを最初にする場合はSPI_DEVICE_TXBIT_LSBFIRST, SPI_DEVICE_RXBIT_LSBFIRSTを設定する)
        // MOSIに送受信両方のデータを乗せる方式もあるらしい(SPI_DEVICE_3WIRE)
        // アクティブの間CSピンをHiにする場合はSPI_DEVICE_POSITIVE_CSを設定する(MCP23S17ではLowなので設定しない)
        // MCP23S17では送受信を同時に行わない(送信の後に受信)ので、SPI_DEVICE_HALFDUPLEXを設定する
        // CSピンからクロックを出力する設定もあるらしい(SPI_DEVICE_CLK_AS_CS)
        .flags = SPI_DEVICE_HALFDUPLEX,

        // キューのサイズを設定
        // とりあえず1にしておく
        .queue_size = 1,

        // 送信開始前に実行する関数を設定
        .pre_cb = cs_high,

        // 送信完了後に実行する関数を設定
        .post_cb = cs_low,
    };
    // SPIバスにデバイスを追加する
    err = spi_bus_add_device(ctx->cfg.host, &devcfg, &ctx->spi);
    // 失敗した場合
    if (err != ESP_OK) {
        // ctx->spiが存在すれば削除する
        if (ctx->spi) {
            spi_bus_remove_device(ctx->spi);
            ctx->spi = NULL;
        }
        // ctx->ready_semが存在すれば削除する
        if (ctx->ready_sem) {
            vSemaphoreDelete(ctx->ready_sem);
            ctx->ready_sem = NULL;
        }
        // ctxを削除(メモリ解放)する
        free(ctx);
        return err;
    }

    // CSピンをLowに設定
    gpio_set_level(ctx->cfg.cs_io, 0);

    // CSピンの設定
    gpio_config_t cs_cfg = {
        // CSピンのGPIOの番号にビットマスクを設定?(このgpio_config_tで設定するピンをここで指定する?)
        .pin_bit_mask = BIT64(ctx->cfg.cs_io),
        // CSピンを出力に設定
        .mode = GPIO_MODE_OUTPUT,
    };

    // CSピンの設定を適用する
    gpio_config(&cs_cfg);

    // 割り込み動作を使用する場合
    if (ctx->cfg.intr_used) {
        // セマフォにバイナリセマフォを作成する
        // バイナリセマフォは資源が使用可能かどうかを0か1かで設定・判別する変数らしい
        ctx->ready_sem = xSemaphoreCreateBinary();

        // セマフォの作成に失敗した場合
        if (ctx->ready_sem == NULL) {
            // メモリがない旨のエラーを返す
            err = ESP_ERR_NO_MEM;
            // ctx->spiが存在すれば削除する
            if (ctx->spi) {
                spi_bus_remove_device(ctx->spi);
                ctx->spi = NULL;
            }
            // ctx->ready_semが存在すれば削除する
            if (ctx->ready_sem) {
                vSemaphoreDelete(ctx->ready_sem);
                ctx->ready_sem = NULL;
            }
            // ctxを削除(メモリ解放)する
            free(ctx);
            return err;
        }

        // MISOピンの立ち上がりエッジでGPIOの割り込みが発生するように設定する
        gpio_set_intr_type(ctx->cfg.miso_io, GPIO_INTR_POSEDGE);
        
        // 割り込みハンドラ(Interrupt Service Routine Handler)を追加する
        err = gpio_isr_handler_add(ctx->cfg.miso_io, ready_rising_isr, ctx);
        // 失敗した場合
        if (err != ESP_OK) {
            // ctx->spiが存在すれば削除する
            if (ctx->spi) {
                spi_bus_remove_device(ctx->spi);
                ctx->spi = NULL;
            }
            // ctx->ready_semが存在すれば削除する
            if (ctx->ready_sem) {
                vSemaphoreDelete(ctx->ready_sem);
                ctx->ready_sem = NULL;
            }
            // ctxを削除(メモリ解放)する
            free(ctx);
            return err;
        }

        // MISOピンの割り込みを無効にしておく
        gpio_intr_disable(ctx->cfg.miso_io);
    }
    *out_ctx = ctx;
    return ESP_OK;

    
}



// 以下、下記よりコピーし、改変
// https://github.com/h1romas4/esp32-genesis-player/blob/master/components/mcp23s17/src/mcp23s17.c

// MCP23S17内のレジスタとグループを送信データに変換する関数
uint8_t mcp23s17_register(mcp23s17_reg_t reg, mcp23s17_gpio_t group)
{
   return (group == GPIOA)?(reg << 1): (reg << 1) | 1;
}

/**
 * spi_write_byte.
 *
 * @see https://github.com/espressif/esp32-nesemu/blob/master/components/nofrendo-esp32/spi_lcd.c
 */
mcp23s17_err_t mcp23s17_write_register(uint8_t ch, uint8_t addr, mcp23s17_reg_t reg, mcp23s17_gpio_t group, uint8_t data)
{
    esp_err_t err;
    uint8_t regpos = mcp23s17_register(reg, group);
    mcp23s17_context_t* ctx;

    switch (ch)
    {
    case 1:
        ctx = mcp23s17_1_handle;
        break;
    case 2:
        ctx = mcp23s17_2_handle;
        break;
    
    default:
        return;
        break;
    }

    err = spi_device_acquire_bus(ctx->spi, portMAX_DELAY);
    if (err != ESP_OK) return MCP23S17_ERR_FAIL;

    spi_transaction_t t = {
        // cmd, addrは使用しない
        .flags = SPI_TRANS_USE_TXDATA,
        // データ長は24bit
        // OPコード(スレーブアドレス+R/W)8bit + レジスタアドレス8bit + データ8bit
        .length = 24,
        .user = ctx,
        .tx_data = {addr, regpos, data, 0}
    };

    // 送信
    err = spi_device_polling_transmit(ctx->spi, &t);

    if (err == ESP_OK) {
        err = mcp23s17_wait_done(ctx);
    }

    spi_device_release_bus(ctx->spi);
    // return err;

    return MCP23S17_ERR_OK;
}

mcp23s17_err_t mcp23s17_write_register_seq(uint8_t ch, uint8_t addr, mcp23s17_reg_t reg, mcp23s17_gpio_t group, uint8_t *data, size_t size)
{
    // address + register + size <= 64 (4byte * 16)
    // assert(size <= 62);

    esp_err_t err;
    uint8_t regpos = mcp23s17_register(reg, group);
    mcp23s17_context_t* ctx;

    switch (ch)
    {
    case 1:
        ctx = mcp23s17_1_handle;
        break;
    case 2:
        ctx = mcp23s17_2_handle;
        break;
    
    default:
        return;
        break;
    }

    spi_buf[0] = addr;
    spi_buf[1] = regpos;
    for(uint8_t i = 0; i < size; i++) {
        spi_buf[i + 2] = data[i];
    }


    err = spi_device_acquire_bus(ctx->spi, portMAX_DELAY);
    if (err != ESP_OK) return MCP23S17_ERR_FAIL;

    spi_transaction_t t = {
        // cmd, addrは使用しない
        // データ長を設定
        // OPコード(スレーブアドレス+R/W)8bit + レジスタアドレス8bit + データ数 × 8bit
        .length = 16 + 8 * size,
        .user = ctx,
        .tx_buffer = spi_buf,
    };
    // 送信
    err = spi_device_polling_transmit(ctx->spi, &t);

    if (err == ESP_OK) {
        err = mcp23s17_wait_done(ctx);
    }

    spi_device_release_bus(ctx->spi);

    return MCP23S17_ERR_OK;
}