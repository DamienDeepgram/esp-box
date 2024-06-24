/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "app_sr.h"
#include "esp_mn_speech_commands.h"
#include "esp_process_sdkconfig.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_models.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_iface.h"
#include "esp_mn_iface.h"
#include "model_path.h"
#include "bsp_board.h"
#include "app_audio.h"
#include "app_wifi.h"
#include <math.h>
#include "esp_dsp.h"
#include "esp_websocket_client.h"
#include "esp_tls.h"
#include "settings.h"

#define NO_DATA_TIMEOUT_SEC 30
static sys_param_t *sys_param = NULL;

static TimerHandle_t shutdown_signal_timer;
static SemaphoreHandle_t shutdown_sema;

esp_websocket_client_handle_t client;

static const char *TAG = "app_sr";

static esp_afe_sr_iface_t *afe_handle = NULL;
static srmodel_list_t *models = NULL;
static bool manul_detect_flag = false;

sr_data_t *g_sr_data = NULL;

#define I2S_CHANNEL_NUM      2

extern bool record_flag;
extern uint32_t record_total_len;

#define PI 3.14159265358979323846

#define N_SAMPLES 1024  // Adjust this if a different FFT size/resolution is required
int N = N_SAMPLES;
float sampleRate = 16000;  // Set the sample rate as per your configuration

// Working complex array
float y_cf[N_SAMPLES * 2];

void findPeaks(float* y_cf, int N, float sampleRate) {
    int peakCount = 0;
    float threshold = 0.1;  // Example threshold, adjust based on your application needs

    // Dynamically calculate a reasonable threshold based on average power
    float averagePower = 0;
    for (int i = 0; i < N / 2; i++) {
        averagePower += (y_cf[i * 2] * y_cf[i * 2] + y_cf[i * 2 + 1] * y_cf[i * 2 + 1]) / N;
    }
    averagePower /= (N / 2);
    threshold = averagePower * 2;  // Setting threshold to twice the average power

    // Find peaks - simple local maxima detection
    for (int i = 1; i < (N / 2) - 1; i++) { // skip the first and last index to avoid boundary issues
        float currentMagnitude = (y_cf[i * 2] * y_cf[i * 2] + y_cf[i * 2 + 1] * y_cf[i * 2 + 1]) / N;
        if (currentMagnitude > (y_cf[(i - 1) * 2] * y_cf[(i - 1) * 2] + y_cf[(i - 1) * 2 + 1] * y_cf[(i - 1) * 2 + 1]) / N &&
            currentMagnitude > (y_cf[(i + 1) * 2] * y_cf[(i + 1) * 2] + y_cf[(i + 1) * 2 + 1] * y_cf[(i + 1) * 2 + 1]) / N &&
            currentMagnitude > threshold) {
            float frequency = ((float)i / N) * sampleRate;
            float magnitudeDB = 10 * log10f(currentMagnitude);  // Convert magnitude to dB
            ESP_LOGI(TAG, "Peak %d: Frequency = %.2f Hz, Magnitude = %f dB", ++peakCount, frequency, magnitudeDB);
        }
    }
}

// websocket code START
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static void shutdown_signaler(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "No data received for %d seconds, signaling shutdown", NO_DATA_TIMEOUT_SEC);
    xSemaphoreGive(shutdown_sema);
}

static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "WEBSOCKET_EVENT_CONNECTED");
        break;
    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "WEBSOCKET_EVENT_DISCONNECTED");
        log_error_if_nonzero("HTTP status code",  data->error_handle.esp_ws_handshake_status_code);
        if (data->error_handle.error_type == WEBSOCKET_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", data->error_handle.esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", data->error_handle.esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  data->error_handle.esp_transport_sock_errno);
        }
        break;
    case WEBSOCKET_EVENT_DATA:
        ESP_LOGI(TAG, "WEBSOCKET_EVENT_DATA");
        ESP_LOGI(TAG, "Received opcode=%d", data->op_code);
        if (data->op_code == 0x2) { // Opcode 0x2 indicates binary data
            ESP_LOG_BUFFER_HEX("Received binary data", data->data_ptr, data->data_len);
        } else if (data->op_code == 0x08 && data->data_len == 2) {
            ESP_LOGW(TAG, "Received closed message with code=%d", 256 * data->data_ptr[0] + data->data_ptr[1]);
        } else {
            ESP_LOGW(TAG, "Received=%.*s\n\n", data->data_len, (char *)data->data_ptr);
        }

        // If received data contains json structure it succeed to parse
        // cJSON *root = cJSON_Parse(data->data_ptr);
        // if (root) {
        //     for (int i = 0 ; i < cJSON_GetArraySize(root) ; i++) {
        //         cJSON *elem = cJSON_GetArrayItem(root, i);
        //         cJSON *id = cJSON_GetObjectItem(elem, "id");
        //         cJSON *name = cJSON_GetObjectItem(elem, "name");
        //         ESP_LOGW(TAG, "Json={'id': '%s', 'name': '%s'}", id->valuestring, name->valuestring);
        //     }
        //     cJSON_Delete(root);
        // }

        // ESP_LOGW(TAG, "Total payload length=%d, data_len=%d, current payload offset=%d\r\n", data->payload_len, data->data_len, data->payload_offset);

        xTimerReset(shutdown_signal_timer, portMAX_DELAY);
        break;
    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGI(TAG, "WEBSOCKET_EVENT_ERROR");
        log_error_if_nonzero("HTTP status code",  data->error_handle.esp_ws_handshake_status_code);
        if (data->error_handle.error_type == WEBSOCKET_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", data->error_handle.esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", data->error_handle.esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  data->error_handle.esp_transport_sock_errno);
        }
        break;
    }
}

char* concat(const char *s1, const char *s2)
{
    char *result = malloc(strlen(s1) + strlen(s2) + 1); // +1 for the null-terminator
    // in real code you would check for errors in malloc here
    strcpy(result, s1);
    strcat(result, s2);
    return result;
}

static void websocket_start()
{
    const char *auth_prefix = "Authorization: Token ";
    const char *auth_header = concat(auth_prefix, sys_param->deepgram_key);
    const char *authorization_header = concat(auth_header, "\r\n");
    ESP_LOGI(TAG, "authorization_header=%s", authorization_header);

    // STS API
    const char *protocol = "wss://";
    const char *path = concat(protocol, sys_param->deepgram_url);
    const char *params = "listen?encoding=linear16&sample_rate=24000&channels=1&model=nova-2-drivethru";
    const char *uri = concat(path, params);
    extern const char cacert_start[] asm("_binary_api_deepgram_com_pem_start"); // CA cert of wss://echo.websocket.event, modify it if using another server
    esp_websocket_client_config_t websocket_cfg = {
        .uri = uri,
        .headers = authorization_header,
    };

    // STS API
    // const char *config = "{\"type\":\"SettingsConfiguration\",\"audio\":{\"input\":{\"encoding\":\"linear16\",\"sample_rate\":16000},\"output\":{\"encoding\":\"linear16\",\"sample_rate\":16000,\"container\":\"none\",\"buffer_size\":250}},\"agent\":{\"listen\":{\"model\":\"nova-2\"},\"speak\":{\"model\":\"aura-athena-en\"},\"think\":{\"provider\":\"open_ai\",\"model\":\"gpt-4o\",\"instructions\":\"You work taking orders at a drive-thru. Only respond in 2-3 sentences at most. Don\'t mention prices until the customer confirms that they\'re done ordering. The menu, including the names, descriptions, types, and prices for the items that you sell, is as follows:[[{\"name\":\"Krabby Patty\",\"description\":\"The signature burger of the Krusty Krab, made with a secret formula\",\"price\":2.99,\"category\":\"meal\"},{\"name\":\"Kelp Shake\",\"description\":\"A shake made with kelp juice\",\"price\":2.49,\"category\":\"beverage\"},{\"name\":\"Double Krabby Patty\",\"description\":\"A Krabby Patty with two patties.\",\"price\":3.99,\"category\":\"meal\"},{\"name\":\"Double Krabby Patty with Cheese\",\"description\":\"A Krabby Patty with two patties and a slice of cheese\",\"price\":4.49,\"category\":\"meal\"},{\"name\":\"Krabby Patty with Cheese\",\"description\":\"A Krabby Patty with a slice of cheese\",\"price\":3.49,\"category\":\"meal\"},{\"name\":\"Krabby Meal\",\"description\":\"Includes a Krabby Patty, fries, and a drink\",\"price\":5.99,\"category\":\"combo\"},{\"name\":\"Salty Sea Dog\",\"description\":\"A hot dog served with sea salt\",\"price\":2.49,\"category\":\"meal\"},{\"name\":\"Krusty Combo\",\"description\":\"Includes a Krabby Patty, Seaweed Salad, and a drink\",\"price\":6.99,\"category\":\"combo\"},{\"name\":\"Seaweed Salad\",\"description\":\"A fresh salad made with seaweed\",\"price\":2.49,\"category\":\"side\"},{\"name\":\"Bubbly buddy\",\"description\":\"A drink that is bubbly and refreshing\",\"price\":1.49,\"category\":\"beverage\"},{\"name\":\"Barnacle Fries\",\"description\":\"Fries made from barnacles\",\"price\":1.99,\"category\":\"side\"}]]\",\"functions\":[{\"name\":\"add_item\",\"description\":\"Add an item to an order for a beverage stand customer. Only items on the menu are valid items\",\"parameters\":{\"type\":\"object\",\"properties\":{\"item\":{\"type\":\"string\",\"description\":\"The name of the item that the user would like to order. The valid values are only those on the menu\"}},\"required\":[\"item\"]},\"url\":\"https://deepgram-workshop-server.glitch.me/calls/3e573b23-050b-456b-9822-9a31cda0cd3c/order/items\",\"method\":\"post\"}]}}}";
    // const char *uri = "wss://sts.sandbox.deepgram.com/demo/agent";
    // extern const char cacert_start[] asm("_binary_sts_sandbox_deepgram_com_pem_start");
    // esp_websocket_client_config_t websocket_cfg = {
    //     .uri = uri,
    // };

    shutdown_signal_timer = xTimerCreate("Websocket shutdown timer", NO_DATA_TIMEOUT_SEC * 1000 / portTICK_PERIOD_MS,
                                         pdFALSE, NULL, shutdown_signaler);
    shutdown_sema = xSemaphoreCreateBinary();

    websocket_cfg.cert_pem = cacert_start;

    ESP_LOGI(TAG, "Connecting to %s...", websocket_cfg.uri);

    client = esp_websocket_client_init(&websocket_cfg);
    esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, websocket_event_handler, (void *)client);

    esp_websocket_client_start(client);
    xTimerStart(shutdown_signal_timer, portMAX_DELAY);

    // STS API
    // int len = (int)strlen(config);
    // esp_websocket_client_send_text(client, config, len, portMAX_DELAY);
    
    /*
    // Deepgram Test with WAV File START
    ESP_LOGI(TAG, "XXX Open File");
    FILE *wav_file = fopen("/spiffs/Hi.wav", "rb");
    if (!wav_file) {
        ESP_LOGE(TAG, "Failed to open WAV file");
        return;
    }

    // Skip the WAV header
    fseek(wav_file, 44, SEEK_SET);

    char buffer[1024];
    size_t bytes_read;
    bool sent = false;
    while (sent == false) {

        ESP_LOGI(TAG, "XXX Checking if connected");
        if (esp_websocket_client_is_connected(client)) {
            ESP_LOGI(TAG, "XXX Sending bytes");
            while ((bytes_read = fread(buffer, 1, sizeof(buffer), wav_file)) > 0) {
                if (esp_websocket_client_is_connected(client)) {
                    // esp_websocket_client_send_bin(client, buffer, bytes_read, portMAX_DELAY);
                    esp_websocket_client_send_bin(client, buffer, bytes_read, portMAX_DELAY);
                    ESP_LOGI(TAG, "Sent %d bytes", bytes_read);
                } else {
                    ESP_LOGI(TAG, "XXX Not Connected");
                    ESP_LOGE(TAG, "Websocket not connected");
                    break;
                }
                vTaskDelay(100 / portTICK_PERIOD_MS); // Adjust delay as needed
            }

            fclose(wav_file);
            sent = true;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    */
    // End Deepgram END

    // Hello Websocket Test Code START
    // char data[32];
    // int i = 0;
    // while (i < 5) {
    //     if (esp_websocket_client_is_connected(client)) {
    //         int len = sprintf(data, "hello %04d", i++);
    //         ESP_LOGI(TAG, "Sending %s", data);
    //         esp_websocket_client_send_text(client, data, len, portMAX_DELAY);
    //     }
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
    // Hello Websocket Test Code END
    
    // xSemaphoreTake(shutdown_sema, portMAX_DELAY);
    // esp_websocket_client_close(client, portMAX_DELAY);
    // ESP_LOGI(TAG, "Websocket Stopped");
    // esp_websocket_client_destroy(client);
}
// websocket code END

static void audio_feed_task_websocket(void *arg) {
    ESP_LOGI(TAG, "Feed Task Started");

    while (WIFI_STATUS_CONNECTED_OK != wifi_connected_already()) {
        ESP_LOGI(TAG, "Waiting for network connection");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    websocket_start();
    ESP_LOGI(TAG, "XXX Websocket connected");
    size_t bytes_read = 0;
    esp_afe_sr_data_t *afe_data = (esp_afe_sr_data_t *)arg;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int feed_channel = 3;

    ESP_LOGI(TAG, "audio_chunksize=%d, feed_channel=%d", audio_chunksize, feed_channel);

    int16_t *audio_buffer = heap_caps_malloc(audio_chunksize * sizeof(int16_t) * feed_channel, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(audio_buffer);


    ESP_LOGI(TAG, "XXX Sending Audio to Websocket");
    while (true) {
        bsp_i2s_read((char *)audio_buffer, audio_chunksize * feed_channel * sizeof(int16_t), &bytes_read, portMAX_DELAY);
        g_sr_data->afe_in_buffer = audio_buffer;
        ESP_LOGI(TAG, "XXX audio_buffer.length: %d", bytes_read);
        // Send audio data over WebSocket
        if (esp_websocket_client_is_connected(client)) {
            ESP_LOGI(TAG, "XXX Sending Audio: %d", bytes_read);
            esp_websocket_client_send_bin(client, (char *)audio_buffer, bytes_read, portMAX_DELAY);
        }

        // Handle other tasks or cleanup if necessary
        if (xEventGroupGetBits(g_sr_data->event_group)) {
            break;  // or perform necessary shutdown procedures
        }
    }

    free(audio_buffer);
    ESP_LOGI(TAG, "Feed Task Ended");
}


// static void audio_feed_task(void *arg)
// {
//     ESP_LOGI(TAG, "Feed Task");
//     size_t bytes_read = 0;
//     esp_afe_sr_data_t *afe_data = (esp_afe_sr_data_t *) arg;
//     int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
//     int feed_channel = 3;
//     ESP_LOGI(TAG, "audio_chunksize=%d, feed_channel=%d", audio_chunksize, feed_channel);

//     /* Allocate audio buffer and check for result */
//     int16_t *audio_buffer = heap_caps_malloc(audio_chunksize * sizeof(int16_t) * feed_channel, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
//     assert(audio_buffer);
//     g_sr_data->afe_in_buffer = audio_buffer;

//     esp_err_t ret;

//     // Initialize I2S or similar setup to read audio data
//     // This might include configuring the I2S driver with i2s_driver_install and i2s_set_pin

//     ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "FFT initialization failed. Error = %i", ret);
//         return;
//     }

//     // Window coefficients
//     float wind[N_SAMPLES];
//     dsps_wind_hann_f32(wind, N);

//     while (true) {
//         if (g_sr_data->event_group && xEventGroupGetBits(g_sr_data->event_group)) {
//             xEventGroupSetBits(g_sr_data->event_group, FEED_DELETED);
//             vTaskDelete(NULL);
//         }

//         /* Read audio data from I2S bus */
//         bsp_i2s_read((char *)audio_buffer, audio_chunksize * I2S_CHANNEL_NUM * sizeof(int16_t), &bytes_read, portMAX_DELAY);

//         // Convert 16-bit audio samples from I2S to float and apply window
//         for (int i = 0; i < N; i++) {
//             // Assuming stereo audio, summing channels or picking one
//             float mono_sample = (audio_buffer[i * 2] + audio_buffer[i * 2 + 1]) / 2.0f;
//             y_cf[i * 2] = mono_sample * wind[i];  // Apply window
//             y_cf[i * 2 + 1] = 0;  // Imaginary part is zero
//         }

//         // Perform FFT
//         dsps_fft2r_fc32(y_cf, N);
//         dsps_bit_rev_fc32(y_cf, N);
//         dsps_cplx2reC_fc32(y_cf, N);  // Converts complex FFT output to real format

//         bool toneDetected = false;
//         // Output magnitudes
//         for (int i = 0; i < N / 2; i++) {
//             // ESP_LOGI(TAG, "Frequency: %.2f Hz, Magnitude: %f dB", frequency, magnitude);
//             if (i >= (int)(296 * N / 16000) && i < (int)(440 * N / 16000)) {  // Index corresponding to 147 Hz
//                 float frequency = (float)i * sampleRate / N;  // Frequency for each bin
//                 float magnitude = 10 * log10f((y_cf[i * 2 + 0] * y_cf[i * 2 + 0] + y_cf[i * 2 + 1] * y_cf[i * 2 + 1]) / N);
//                 if(magnitude > 65){
//                     ESP_LOGI(TAG, ">>>Magnitude at %f Hz: %f dB", frequency, magnitude);
//                     toneDetected = true;
//                 }
//             }
//         }

//         if(toneDetected){
//              ESP_LOGI(TAG, ">>> Detected car tone, trigger servo to press button");
//         }

//         findPeaks(y_cf, N, sampleRate);

//         /* Channel Adjust */
//         for (int  i = audio_chunksize - 1; i >= 0; i--) {
//             audio_buffer[i * 3 + 2] = 0;
//             audio_buffer[i * 3 + 1] = audio_buffer[i * 2 + 1];
//             audio_buffer[i * 3 + 0] = audio_buffer[i * 2 + 0];
//         }

//         /* Checking if WIFI is connected */
//         if (WIFI_STATUS_CONNECTED_OK == wifi_connected_already()) {

//             /* Feed samples of an audio stream to the AFE_SR */
//             afe_handle->feed(afe_data, audio_buffer);
//         }
//         audio_record_save(audio_buffer, audio_chunksize);
//     }
// }


static void audio_detect_task(void *arg)
{
    ESP_LOGI(TAG, "Detection task");
    static afe_vad_state_t local_state;
    static uint8_t frame_keep = 0;

    bool detect_flag = false;
    esp_afe_sr_data_t *afe_data = arg;

    while (true) {
        if (NEED_DELETE && xEventGroupGetBits(g_sr_data->event_group)) {
            xEventGroupSetBits(g_sr_data->event_group, DETECT_DELETED);
            vTaskDelete(g_sr_data->handle_task);
            vTaskDelete(NULL);
        }
        afe_fetch_result_t *res = afe_handle->fetch(afe_data);
        if (!res || res->ret_value == ESP_FAIL) {
            ESP_LOGW(TAG, "AFE Fetch Fail");
            continue;
        }
        if (res->wakeup_state == WAKENET_DETECTED) {
            ESP_LOGI(TAG, LOG_BOLD(LOG_COLOR_GREEN) "wakeword detected");
            sr_result_t result = {
                .wakenet_mode = WAKENET_DETECTED,
                .state = ESP_MN_STATE_DETECTING,
                .command_id = 0,
            };
            xQueueSend(g_sr_data->result_que, &result, 0);
        } else if (res->wakeup_state == WAKENET_CHANNEL_VERIFIED || manul_detect_flag) {
            detect_flag = true;
            if (manul_detect_flag) {
                manul_detect_flag = false;
                sr_result_t result = {
                    .wakenet_mode = WAKENET_DETECTED,
                    .state = ESP_MN_STATE_DETECTING,
                    .command_id = 0,
                };
                xQueueSend(g_sr_data->result_que, &result, 0);
            }
            frame_keep = 0;
            g_sr_data->afe_handle->disable_wakenet(afe_data);
            ESP_LOGI(TAG, LOG_BOLD(LOG_COLOR_GREEN) "AFE_FETCH_CHANNEL_VERIFIED, channel index: %d\n", res->trigger_channel_id);
        }

        if (true == detect_flag) {

            if (local_state != res->vad_state) {
                local_state = res->vad_state;
                frame_keep = 0;
            } else {
                frame_keep++;
            }

            if ((100 == frame_keep) && (AFE_VAD_SILENCE == res->vad_state)) {
                sr_result_t result = {
                    .wakenet_mode = WAKENET_NO_DETECT,
                    .state = ESP_MN_STATE_TIMEOUT,
                    .command_id = 0,
                };
                xQueueSend(g_sr_data->result_que, &result, 0);
                g_sr_data->afe_handle->enable_wakenet(afe_data);
                detect_flag = false;
                continue;
            }
        }
    }
    /* Task never returns */
    vTaskDelete(NULL);
}

esp_err_t app_sr_set_language(sr_language_t new_lang)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR is not running");

    if (new_lang == g_sr_data->lang) {
        ESP_LOGW(TAG, "nothing to do");
        return ESP_OK;
    } else {
        g_sr_data->lang = new_lang;
    }
    ESP_LOGI(TAG, "Set language %s", SR_LANG_EN == g_sr_data->lang ? "EN" : "CN");
    if (g_sr_data->model_data) {
        g_sr_data->multinet->destroy(g_sr_data->model_data);
    }
    char *wn_name = esp_srmodel_filter(models, ESP_WN_PREFIX, "");
    ESP_LOGI(TAG, "load wakenet:%s", wn_name);
    g_sr_data->afe_handle->set_wakenet(g_sr_data->afe_data, wn_name);
    return ESP_OK;
}

esp_err_t app_sr_start(bool record_en)
{
    sys_param = settings_get_parameter();

    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(NULL == g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR already running");

    g_sr_data = heap_caps_calloc(1, sizeof(sr_data_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_NO_MEM, TAG, "Failed create sr data");

    g_sr_data->result_que = xQueueCreate(3, sizeof(sr_result_t));
    ESP_GOTO_ON_FALSE(NULL != g_sr_data->result_que, ESP_ERR_NO_MEM, err, TAG, "Failed create result queue");

    g_sr_data->event_group = xEventGroupCreate();
    ESP_GOTO_ON_FALSE(NULL != g_sr_data->event_group, ESP_ERR_NO_MEM, err, TAG, "Failed create event_group");

    BaseType_t ret_val;
    models = esp_srmodel_init("model");
    afe_handle = (esp_afe_sr_iface_t *)&ESP_AFE_SR_HANDLE;
    afe_config_t afe_config = AFE_CONFIG_DEFAULT();

    afe_config.wakenet_model_name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);
    afe_config.aec_init = false;

    esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(&afe_config);
    g_sr_data->afe_handle = afe_handle;
    g_sr_data->afe_data = afe_data;

    g_sr_data->lang = SR_LANG_MAX;
    ret = app_sr_set_language(SR_LANG_EN);
    ESP_GOTO_ON_FALSE(ESP_OK == ret, ESP_FAIL, err, TAG,  "Failed to set language");

    // ret_val = xTaskCreatePinnedToCore(&audio_feed_task, "Feed Task", 8 * 1024, (void *)afe_data, 5, &g_sr_data->feed_task, 0);
    // ESP_GOTO_ON_FALSE(pdPASS == ret_val, ESP_FAIL, err, TAG,  "Failed create audio feed task");

    ret_val = xTaskCreatePinnedToCore(&audio_feed_task_websocket, "Feed Websocket Task", 8 * 1024, (void *)afe_data, 5, &g_sr_data->feed_websocket_task, 0);
    ESP_GOTO_ON_FALSE(pdPASS == ret_val, ESP_FAIL, err, TAG,  "Failed create audio feed task");

    ret_val = xTaskCreatePinnedToCore(&audio_detect_task, "Detect Task", 10 * 1024, (void *)afe_data, 5, &g_sr_data->detect_task, 1);
    ESP_GOTO_ON_FALSE(pdPASS == ret_val, ESP_FAIL, err, TAG,  "Failed create audio detect task");

    // TODO Deepgram - reference code for running tasks
    ret_val = xTaskCreatePinnedToCore(&sr_handler_task, "SR Handler Task", 8 * 1024, NULL, 5, &g_sr_data->handle_task, 0);
    ESP_GOTO_ON_FALSE(pdPASS == ret_val, ESP_FAIL, err, TAG,  "Failed create audio handler task");

    audio_record_init();

    return ESP_OK;
err:
    app_sr_stop();
    return ret;
}

esp_err_t app_sr_stop(void)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR is not running");
    xEventGroupSetBits(g_sr_data->event_group, NEED_DELETE);
    xEventGroupWaitBits(g_sr_data->event_group, NEED_DELETE | FEED_DELETED | DETECT_DELETED | HANDLE_DELETED, 1, 1, portMAX_DELAY);

    if (g_sr_data->result_que) {
        vQueueDelete(g_sr_data->result_que);
        g_sr_data->result_que = NULL;
    }

    if (g_sr_data->event_group) {
        vEventGroupDelete(g_sr_data->event_group);
        g_sr_data->event_group = NULL;
    }

    if (g_sr_data->fp) {
        fclose(g_sr_data->fp);
        g_sr_data->fp = NULL;
    }

    if (g_sr_data->model_data) {
        g_sr_data->multinet->destroy(g_sr_data->model_data);
    }

    if (g_sr_data->afe_data) {
        g_sr_data->afe_handle->destroy(g_sr_data->afe_data);
    }

    if (g_sr_data->afe_in_buffer) {
        heap_caps_free(g_sr_data->afe_in_buffer);
    }

    if (g_sr_data->afe_out_buffer) {
        heap_caps_free(g_sr_data->afe_out_buffer);
    }

    heap_caps_free(g_sr_data);
    g_sr_data = NULL;
    return ESP_OK;
}

esp_err_t app_sr_get_result(sr_result_t *result, TickType_t xTicksToWait)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR is not running");
    xQueueReceive(g_sr_data->result_que, result, xTicksToWait);
    return ESP_OK;
}

esp_err_t app_sr_start_once(void)
{
    ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG, "SR is not running");
    manul_detect_flag = true;
    return ESP_OK;
}
