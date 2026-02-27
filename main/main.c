#include <stdio.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/adc_types_legacy.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

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

void app_main(void)
{
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
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}
