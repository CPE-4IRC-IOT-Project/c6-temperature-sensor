#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/adc_types_legacy.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gatt.h"
#include "host/ble_hs_adv.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_store.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "c6-temp-adc";

#define TEMP_ADC_UNIT       ADC_UNIT_1
#define TEMP_ADC_CHANNEL    ADC_CHANNEL_4   // GPIO4 on ESP32-C6
#define TEMP_ADC_ATTEN      ADC_ATTEN_DB_12
#define TEMP_ADC_BITWIDTH   ADC_BITWIDTH_DEFAULT
#define SAMPLE_PERIOD_MS    1000

/*
 * Typical KY-028/KY-013 NTC parameters.
 * Adjust these values if your module uses different resistor/thermistor specs.
 */
#define NTC_SERIES_RESISTOR_OHM    10000.0f
#define NTC_NOMINAL_RESISTANCE_OHM 2700.0f
#define NTC_NOMINAL_TEMPERATURE_C  21.5f
#define NTC_BETA_COEFFICIENT       3950.0f
#define NTC_SUPPLY_VOLTAGE_MV      3300.0f
#define NTC_TO_GND                 1
#define ADC_SAMPLES_PER_READ       16
#define ADC_SAMPLE_INTERVAL_MS     5
#define VOLTAGE_EMA_ALPHA          0.05f
#define NTC_VALID_MIN_MV           80.0f
#define NTC_VALID_MAX_MV           3220.0f
#define BLE_TEMP_DEVICE_NAME       "C6-TEMP-SENSOR"
#define UART_TX_PORT               UART_NUM_1
#define UART_TX_PIN                16
#define UART_TX_BAUD               115200
#define UART_TX_RX_BUF_SIZE        256
#define UART_MIRROR_TO_P4          0

// Shared BLE profile with ESP32-P4 client.
static const ble_uuid128_t BLE_TEMP_SERVICE_UUID =
    BLE_UUID128_INIT(0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);
static const ble_uuid128_t BLE_TEMP_CHARACTERISTIC_UUID =
    BLE_UUID128_INIT(0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static uint8_t s_ble_addr_type = BLE_OWN_ADDR_PUBLIC;
static uint16_t s_ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_ble_temp_val_handle = 0;
static bool s_ble_notify_enabled = false;
static int16_t s_ble_temp_centi = 0;

void ble_store_config_init(void);

static bool adc_calibration_init(adc_unit_t unit,
                                 adc_channel_t channel,
                                 adc_atten_t atten,
                                 adc_cali_handle_t *out_handle)
{
    *out_handle = NULL;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = TEMP_ADC_BITWIDTH,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_config, out_handle) == ESP_OK) {
        return true;
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = TEMP_ADC_BITWIDTH,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_config, out_handle) == ESP_OK) {
        return true;
    }
#endif

    ESP_LOGW(TAG, "ADC calibration unavailable, only raw value will be reported");
    return false;
}

static float ntc_resistance_from_voltage(float voltage_mv)
{
    // Reject near-rail voltages: likely short/open/floating input, not a valid NTC reading.
    if (voltage_mv <= NTC_VALID_MIN_MV || voltage_mv >= NTC_VALID_MAX_MV) {
        return NAN;
    }

#if NTC_TO_GND
    return NTC_SERIES_RESISTOR_OHM * (voltage_mv / (NTC_SUPPLY_VOLTAGE_MV - voltage_mv));
#else
    return NTC_SERIES_RESISTOR_OHM * ((NTC_SUPPLY_VOLTAGE_MV - voltage_mv) / voltage_mv);
#endif
}

static float ntc_temperature_c_from_resistance(float ntc_res_ohm)
{
    if (!(ntc_res_ohm > 0.0f)) {
        return NAN;
    }

    const float t0_kelvin = NTC_NOMINAL_TEMPERATURE_C + 273.15f;
    const float inv_t = (1.0f / t0_kelvin) +
                        (logf(ntc_res_ohm / NTC_NOMINAL_RESISTANCE_OHM) / NTC_BETA_COEFFICIENT);
    return (1.0f / inv_t) - 273.15f;
}

static float estimate_voltage_mv_from_raw(int raw)
{
    const float adc_max = 4095.0f;
    return ((float)raw / adc_max) * NTC_SUPPLY_VOLTAGE_MV;
}

static esp_err_t raw_to_voltage_mv(int raw,
                                   bool cali_enabled,
                                   adc_cali_handle_t cali_handle,
                                   float *voltage_mv)
{
    if (cali_enabled) {
        int calibrated_mv = 0;
        esp_err_t err = adc_cali_raw_to_voltage(cali_handle, raw, &calibrated_mv);
        if (err != ESP_OK) {
            return err;
        }
        *voltage_mv = (float)calibrated_mv;
        return ESP_OK;
    }

    *voltage_mv = estimate_voltage_mv_from_raw(raw);
    return ESP_OK;
}

static int ble_temp_chr_access_cb(uint16_t conn_handle,
                                  uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt,
                                  void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint8_t payload[2] = {
        (uint8_t)(s_ble_temp_centi & 0xFF),
        (uint8_t)(((uint16_t)s_ble_temp_centi >> 8) & 0xFF)
    };
    int rc = os_mbuf_append(ctxt->om, payload, sizeof(payload));
    return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static const struct ble_gatt_chr_def ble_temp_characteristics[] = {
    {
        .uuid = &BLE_TEMP_CHARACTERISTIC_UUID.u,
        .access_cb = ble_temp_chr_access_cb,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &s_ble_temp_val_handle,
    },
    {0},
};

static const struct ble_gatt_svc_def ble_temp_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &BLE_TEMP_SERVICE_UUID.u,
        .characteristics = ble_temp_characteristics,
    },
    {0},
};

static void ble_temp_advertise(void);

static int ble_temp_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_ble_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "BLE connected; conn_handle=%u", (unsigned)s_ble_conn_handle);
        } else {
            ESP_LOGW(TAG, "BLE connect failed; status=%d", event->connect.status);
            ble_temp_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "BLE disconnected; reason=%d", event->disconnect.reason);
        s_ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_ble_notify_enabled = false;
        ble_temp_advertise();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_ble_temp_val_handle) {
            s_ble_notify_enabled = event->subscribe.cur_notify != 0;
            ESP_LOGI(TAG, "BLE notify=%d for temp characteristic", s_ble_notify_enabled ? 1 : 0);
        }
        return 0;

    default:
        return 0;
    }
}

static void ble_temp_advertise(void)
{
    struct ble_hs_adv_fields fields = {0};
    struct ble_hs_adv_fields rsp_fields = {0};
    struct ble_gap_adv_params adv_params = {0};

    /* Keep primary ADV payload small enough for legacy advertising (31 bytes). */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = (ble_uuid128_t *)&BLE_TEMP_SERVICE_UUID;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE adv set fields failed; rc=%d", rc);
        return;
    }

    /* Put device name in scan response to avoid overflowing advertising payload. */
    rsp_fields.name = (uint8_t *)BLE_TEMP_DEVICE_NAME;
    rsp_fields.name_len = strlen(BLE_TEMP_DEVICE_NAME);
    rsp_fields.name_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE adv rsp set fields failed; rc=%d", rc);
        return;
    }

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_ble_addr_type,
                           NULL,
                           BLE_HS_FOREVER,
                           &adv_params,
                           ble_temp_gap_event,
                           NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE adv start failed; rc=%d", rc);
    }
}

static void ble_temp_on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE host reset; reason=%d", reason);
}

static void ble_temp_on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_ble_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE infer addr type failed; rc=%d", rc);
        return;
    }

    ble_temp_advertise();
}

static void ble_temp_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static esp_err_t ble_temp_init(void)
{
    int rc = nimble_port_init();
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", rc);
        return ESP_FAIL;
    }

    ble_hs_cfg.reset_cb = ble_temp_on_reset;
    ble_hs_cfg.sync_cb = ble_temp_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set(BLE_TEMP_DEVICE_NAME);
    ble_store_config_init();

    rc = ble_gatts_count_cfg(ble_temp_services);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        return ESP_FAIL;
    }
    rc = ble_gatts_add_svcs(ble_temp_services);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        return ESP_FAIL;
    }

    nimble_port_freertos_init(ble_temp_host_task);
    ESP_LOGI(TAG, "BLE temperature service started");
    return ESP_OK;
}

static void ble_temp_publish(float temp_c)
{
    int16_t centi = (int16_t)lroundf(temp_c * 100.0f);
    s_ble_temp_centi = centi;

    if (s_ble_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_ble_notify_enabled) {
        return;
    }

    uint8_t payload[2] = {
        (uint8_t)(centi & 0xFF),
        (uint8_t)(((uint16_t)centi >> 8) & 0xFF)
    };
    struct os_mbuf *om = ble_hs_mbuf_from_flat(payload, sizeof(payload));
    if (om == NULL) {
        ESP_LOGW(TAG, "BLE notify skipped (no mbuf)");
        return;
    }

    int rc = ble_gatts_notify_custom(s_ble_conn_handle, s_ble_temp_val_handle, om);
    if (rc != 0 && rc != BLE_HS_ENOTCONN) {
        ESP_LOGW(TAG, "BLE notify failed; rc=%d", rc);
    }
}

#if UART_MIRROR_TO_P4
static void uart_temp_tx_init(void)
{
    const uart_config_t cfg = {
        .baud_rate = UART_TX_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // ESP-IDF UART driver requires a non-zero RX buffer length.
    ESP_ERROR_CHECK(uart_driver_install(UART_TX_PORT, UART_TX_RX_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_TX_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_TX_PORT,
                                 UART_TX_PIN,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART temperature TX ready on UART%d TX=%d baud=%d",
             (int)UART_TX_PORT, UART_TX_PIN, UART_TX_BAUD);
}

static void uart_temp_tx_publish(float temp_c)
{
    char line[48];
    int n = snprintf(line, sizeof(line), "TEMP,%.2f\r\n", (double)temp_c);
    if (n <= 0) {
        return;
    }

    int written = uart_write_bytes(UART_TX_PORT, line, n);
    if (written != n) {
        ESP_LOGW(TAG, "UART temp write short: %d/%d", written, n);
    }
}
#endif

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(ble_temp_init());

    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = TEMP_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = TEMP_ADC_ATTEN,
        .bitwidth = TEMP_ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, TEMP_ADC_CHANNEL, &channel_config));

    adc_cali_handle_t cali_handle = NULL;
    bool cali_enabled = adc_calibration_init(TEMP_ADC_UNIT, TEMP_ADC_CHANNEL, TEMP_ADC_ATTEN, &cali_handle);

    ESP_LOGI(TAG, "Reading NTC on GPIO4 (ADC1_CH4) every %" PRIu32 " ms", (uint32_t)SAMPLE_PERIOD_MS);

    bool ema_initialized = false;
    float voltage_ema_mv = 0.0f;

    while (1) {
        int min_raw = 4095;
        int max_raw = 0;
        int raw_sum = 0;
        int raw_valid_count = 0;

        for (int i = 0; i < ADC_SAMPLES_PER_READ; ++i) {
            int raw = 0;
            esp_err_t err = adc_oneshot_read(adc_handle, TEMP_ADC_CHANNEL, &raw);
            if (err == ESP_OK) {
                if (raw < min_raw) {
                    min_raw = raw;
                }
                if (raw > max_raw) {
                    max_raw = raw;
                }
                raw_sum += raw;
                raw_valid_count++;
            }
            vTaskDelay(pdMS_TO_TICKS(ADC_SAMPLE_INTERVAL_MS));
        }

        if (raw_valid_count < 3) {
            ESP_LOGE(TAG, "not enough valid ADC samples");
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
            continue;
        }

        int trimmed_sum = raw_sum - min_raw - max_raw;
        int trimmed_count = raw_valid_count - 2;
        int raw_filtered = trimmed_sum / trimmed_count;

        float voltage_instant_mv = 0.0f;
        esp_err_t conv_err = raw_to_voltage_mv(raw_filtered, cali_enabled, cali_handle, &voltage_instant_mv);
        if (conv_err != ESP_OK) {
            ESP_LOGW(TAG, "voltage calibration failed: %s, using raw estimate", esp_err_to_name(conv_err));
            voltage_instant_mv = estimate_voltage_mv_from_raw(raw_filtered);
        }

        if (!ema_initialized) {
            voltage_ema_mv = voltage_instant_mv;
            ema_initialized = true;
        } else {
            voltage_ema_mv = (VOLTAGE_EMA_ALPHA * voltage_instant_mv) +
                             ((1.0f - VOLTAGE_EMA_ALPHA) * voltage_ema_mv);
        }

        float ntc_res_ohm = ntc_resistance_from_voltage(voltage_ema_mv);
        float temp_c = ntc_temperature_c_from_resistance(ntc_res_ohm);
        if (isnan(temp_c)) {
            ESP_LOGW(TAG, "GPIO4 raw=%d v_inst=%.0f mV v_filt=%.0f mV temp=INVALID",
                     raw_filtered,
                     voltage_instant_mv,
                     voltage_ema_mv);
        } else {
            ESP_LOGI(TAG, "GPIO4 raw=%d v_inst=%.0f mV v_filt=%.0f mV ntc=%.0f ohm temp=%.2f C",
                     raw_filtered,
                     voltage_instant_mv,
                     voltage_ema_mv,
                     ntc_res_ohm,
                     temp_c);
            ble_temp_publish(temp_c);
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}
