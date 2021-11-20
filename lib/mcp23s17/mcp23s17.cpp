#include <mcp23s17.h>

// for vscode c_cpp_extention
#ifdef __INTELLISENSE__
#include "build/include/sdkconfig.h"
#endif

// MCP23S17の初期化
void mcp23s17_init(void){

    // エラー識別用の変数
    esp_err_t ret;

    // ログに初期化中のメッセージを出力
    ESP_LOGI(TAG, "Initializing bus SPI%d...", MCP23S17_HOST+1);

    // SPIバスの設定用変数
    spi_bus_config_t buscfg={
        // MISOのGPIOの番号を設定
        .miso_io_num = PIN_NUM_MISO,
        // MOSIのGPIOの番号を設定
        .mosi_io_num = PIN_NUM_MOSI,
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
        // CSのGPIOの番号を設定 
        .cs_io = PIN_NUM_CS1,
        // 使用するSPIのペリフェラルを設定
        .host = MCP23S17_HOST,
        // MISOのGPIOの番号を設定
        .miso_io = PIN_NUM_MISO,
    };

    // SPIバスのMCP23S17(2番目)に関連する設定
    mcp23s17_config_t mcp23s17_2_config = {
        // CSのGPIOの番号を設定 
        .cs_io = PIN_NUM_CS2,
        // 使用するSPIのペリフェラルを設定
        .host = MCP23S17_HOST,
        // MISOのGPIOの番号を設定
        .miso_io = PIN_NUM_MISO,
    };
    // SPIの書き込み完了待ちにおいて、割り込み動作を有効にする
    mcp23s17_1_config.intr_used = true;
    mcp23s17_2_config.intr_used = true;
    gpio_install_isr_service(0);

    mcp23s17_handle_t mcp23s17_1_handle;
    mcp23s17_handle_t mcp23s17_2_handle;

    // SPIの初期化
    ESP_LOGI(TAG, "Initializing device...");
    ret = spi_mcp23s17_init(&mcp23s17_1_config, &mcp23s17_1_handle);
    ESP_ERROR_CHECK(ret);
    ret = spi_mcp23s17_init(&mcp23s17_2_config, &mcp23s17_2_handle);
    ESP_ERROR_CHECK(ret);

    // SPIの書き込み許可
    ret = spi_mcp23s17_write_enable(mcp23s17_handle);
    ESP_ERROR_CHECK(ret);

    const char test_str[] = "Hello World!";
    ESP_LOGI(TAG, "Write: %s", test_str);
    for (int i = 0; i < sizeof(test_str); i++) {
        // No need for this EEPROM to erase before write.
        ret = spi_mcp23s17_write(mcp23s17_handle, i, test_str[i]);
        ESP_ERROR_CHECK(ret);
    }

    uint8_t test_buf[32] = "";
    for (int i = 0; i < sizeof(test_str); i++) {
        ret = spi_mcp23s17_read(mcp23s17_handle, i, &test_buf[i]);
        ESP_ERROR_CHECK(ret);
    }
    ESP_LOGI(TAG, "Read: %s", test_buf);

    ESP_LOGI(TAG, "Example finished.");
}
static const char TAG[] = "mcp23s17";


// Workaround: The driver depends on some data in the flash and cannot be placed to DRAM easily for
// now. Using the version in LL instead.
#define gpio_set_level  gpio_set_level_patch
#include "hal/gpio_ll.h"
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
        uint32_t tick_to_wait = MAX(mcp23s17_BUSY_TIMEOUT_MS / portTICK_PERIOD_MS, 2);
        BaseType_t ret = xSemaphoreTake(ctx->ready_sem, tick_to_wait);
        gpio_intr_disable(ctx->cfg.miso_io);
        gpio_set_level(ctx->cfg.cs_io, 0);

        if (ret != pdTRUE) return ESP_ERR_TIMEOUT;
    } else {
        bool timeout = true;
        gpio_set_level(ctx->cfg.cs_io, 1);
        for (int i = 0; i < EEPROM_BUSY_TIMEOUT_MS * 1000; i ++) {
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

        // 通信時の周波数を設定
        .clock_speed_hz = MCP23S17_CLK_FREQ,

        // SPIのモードを0に設定
        // CPOL: 0, CPHA: 0
        .mode = 0,          //SPI mode 0
        /*
         * The timing requirements to read the busy signal from the EEPROM cannot be easily emulated
         * by SPI transactions. We need to control CS pin by SW to check the busy signal manually.
         */
        // 元にしたEEPROM用のSPIではCSをGPIOとして動作するように設定し、手動で切り替えていたが
        // 今回はGPIOに設定せず、CSを自動で切り替えるようにする

        // CSのGPIOの番号を設定
        .spics_io_num = cfg -> cs_io,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_POSITIVE_CS,
        .pre_cb = cs_high,
        .post_cb = cs_low,
        .input_delay_ns = MCP23S17_INPUT_DELAY_NS,  //the EEPROM output the data half a SPI clock behind.
    };
    //Attach the EEPROM to the SPI bus
    err = spi_bus_add_device(ctx->cfg.host, &devcfg, &ctx->spi);
    if  (err != ESP_OK) {
        goto cleanup;
    }

    gpio_set_level(ctx->cfg.cs_io, 0);
    gpio_config_t cs_cfg = {
        .pin_bit_mask = BIT64(ctx->cfg.cs_io),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cs_cfg);

    if (ctx->cfg.intr_used) {
        ctx->ready_sem = xSemaphoreCreateBinary();
        if (ctx->ready_sem == NULL) {
            err = ESP_ERR_NO_MEM;
            goto cleanup;
        }

        gpio_set_intr_type(ctx->cfg.miso_io, GPIO_INTR_POSEDGE);
        err = gpio_isr_handler_add(ctx->cfg.miso_io, ready_rising_isr, ctx);
        if (err != ESP_OK) {
            goto cleanup;
        }
        gpio_intr_disable(ctx->cfg.miso_io);
    }
    *out_ctx = ctx;
    return ESP_OK;

cleanup:
    if (ctx->spi) {
        spi_bus_remove_device(ctx->spi);
        ctx->spi = NULL;
    }
    if (ctx->ready_sem) {
        vSemaphoreDelete(ctx->ready_sem);
        ctx->ready_sem = NULL;
    }
    free(ctx);
    return err;
}

esp_err_t spi_mcp23s17_read(mcp23s17_context_t* ctx, uint8_t addr, uint8_t* out_data)
{
    spi_transaction_t t = {
        .cmd = CMD_READ | (addr & ADDR_MASK),
        .rxlength = 8,
        .flags = SPI_TRANS_USE_RXDATA,
        .user = ctx,
    };
    esp_err_t err = spi_device_polling_transmit(ctx->spi, &t);
    if (err!= ESP_OK) return err;

    *out_data = t.rx_data[0];
    return ESP_OK;
}

esp_err_t spi_mcp23s17_erase(mcp23s17_context_t* ctx, uint8_t addr)
{
    esp_err_t err;
    err = spi_device_acquire_bus(ctx->spi, portMAX_DELAY);
    if (err != ESP_OK) return err;

    err = mcp23s17_simple_cmd(ctx, CMD_ERASE | (addr & ADDR_MASK));

    if (err == ESP_OK) {
        err = mcp23s17_wait_done(ctx);
    }

    spi_device_release_bus(ctx->spi);
    return err;
}

esp_err_t spi_mcp23s17_write(mcp23s17_context_t* ctx, uint8_t addr, uint8_t data)
{
    esp_err_t err;
    err = spi_device_acquire_bus(ctx->spi, portMAX_DELAY);
    if (err != ESP_OK) return err;

    spi_transaction_t t = {
        .cmd = CMD_WRITE | (addr & ADDR_MASK),
        .length = 8,
        .flags = SPI_TRANS_USE_TXDATA,
        .tx_data = {data},
        .user = ctx,
    };
    err = spi_device_polling_transmit(ctx->spi, &t);

    if (err == ESP_OK) {
        err = mcp23s17_wait_done(ctx);
    }

    spi_device_release_bus(ctx->spi);
    return err;
}

esp_err_t spi_mcp23s17_write_enable(mcp23s17_context_t* ctx)
{
    return mcp23s17_simple_cmd(ctx, CMD_EWEN | ADD_EWEN);
}

esp_err_t spi_mcp23s17_write_disable(mcp23s17_context_t* ctx)
{
    return mcp23s17_simple_cmd(ctx, CMD_EWDS | ADD_EWDS);
}

esp_err_t spi_mcp23s17_erase_all(mcp23s17_context_t* ctx)
{
#if !CONFIG_EXAMPLE_5V_COMMANDS
    //not supported in 3.3V VCC
    ESP_LOGE(TAG, "erase all not supported by mcp23s17 under 3.3V VCC");
    return ESP_ERR_NOT_SUPPORTED;
#endif

    esp_err_t err;
    err = spi_device_acquire_bus(ctx->spi, portMAX_DELAY);
    if (err != ESP_OK) return err;

    err = mcp23s17_simple_cmd(ctx, CMD_ERAL | ADD_ERAL);

    if (err == ESP_OK) {
        err = mcp23s17_wait_done(ctx);
    }

    spi_device_release_bus(ctx->spi);
    return err;
}

esp_err_t spi_mcp23s17_write_all(mcp23s17_context_t* ctx, uint8_t data)
{
#if !CONFIG_EXAMPLE_5V_COMMANDS
    //not supported in 3.3V VCC
    ESP_LOGE(TAG, "write all not supported by EEPROM under 3.3V VCC");
    return ESP_ERR_NOT_SUPPORTED;
#endif

    esp_err_t err;
    err = spi_device_acquire_bus(ctx->spi, portMAX_DELAY);
    if (err != ESP_OK) return err;

    spi_transaction_t t = {
        .cmd = CMD_WRAL | ADD_WRAL,
        .length = 8,
        .flags = SPI_TRANS_USE_TXDATA,
        .tx_data = {data},
        .user = ctx,
    };
    err = spi_device_polling_transmit(ctx->spi, &t);

    if (err == ESP_OK) {
        err = mcp23s17_wait_done(ctx);
    }

    spi_device_release_bus(ctx->spi);
    return err;
}