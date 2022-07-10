/*
 * BLE MIDI Demo
 *
 * See ../README.md for usage hints
 *
 * =============================================================================
 *
 * MIT License
 *
 * Copyright (c) 2019 Thorsten Klose (tk@midibox.org)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * =============================================================================
 */


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "blemidi.h"
#include "tinyusb.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"


#define TAG "Knobs"


//--------------------------------------------------------------------+
// ADC driver related definitions and macros
//--------------------------------------------------------------------+

#define TIMES              256
#define GET_UNIT(x)        ((x>>3) & 0x1)


#define ADC_RESULT_BYTE     4
#define ADC_CONV_LIMIT_EN   0
#define ADC_CONV_MODE       ADC_CONV_BOTH_UNIT
#define ADC_OUTPUT_TYPE     ADC_DIGI_OUTPUT_FORMAT_TYPE2


static uint16_t adc1_chan_mask = BIT(2) | BIT(3);
static uint16_t adc2_chan_mask = BIT(0);
static adc_channel_t channel[3] = {ADC1_CHANNEL_2, ADC1_CHANNEL_3, (ADC2_CHANNEL_0 | 1 << 3)};


//--------------------------------------------------------------------+
// Event Groups
//--------------------------------------------------------------------+

static const int USB_SUSPEND = BIT0;
static EventGroupHandle_t  xUSBMidiEventGroup;


//--------------------------------------------------------------------+
// Task Handlers
//--------------------------------------------------------------------+

static TaskHandle_t xUSBMidiReadHandle = NULL;
static TaskHandle_t xUSBMidiWriteHandle = NULL;
static TaskHandle_t xBLEMidiWriteHandle = NULL;
static TaskHandle_t xADCReadHandle = NULL;


//--------------------------------------------------------------------+
// This task is periodically called to send MIDI over BLE
//--------------------------------------------------------------------+
static void vBLEMidiWriteTask(void *pvParameters)
{
  portTickType xLastExecutionTime;
  unsigned ctr = 0;


  // Initialise the xLastExecutionTime variable on task entry
  xLastExecutionTime = xTaskGetTickCount();

  while( 1 ) {
    ESP_ERROR_CHECK(esp_task_wdt_reset());
    vTaskDelayUntil(&xLastExecutionTime, 500 / portTICK_RATE_MS);

    blemidi_tick(); // for timestamp and output buffer handling

#if 1
    ctr += 1;
   // ESP_LOGI(TAG, "Sending MIDI Note #%d", ctr);


    {
      // TODO: more comfortable packet creation via special APIs
      uint8_t message[3] = { 0x90, 0x3c, 0x7f };
      blemidi_send_message(0, message, sizeof(message));
    }
    
    vTaskDelayUntil(&xLastExecutionTime, 500 / portTICK_RATE_MS);

    blemidi_tick(); // for timestamp and output buffer handling

    {
      // TODO: more comfortable packet creation via special APIs
      uint8_t message[3] = { 0x90, 0x3c, 0x00 };
      blemidi_send_message(0, message, sizeof(message));
    }
#endif
  }

  vTaskDelete(NULL);
}



//--------------------------------------------------------------------+
// MIDI over USB Tasks
//--------------------------------------------------------------------+

static void vUSBMidiReadTask(void *arg)
{
    // The MIDI interface always creates input and output port/jack descriptors
    // regardless of these being used or not. Therefore incoming traffic should be read
    // (possibly just discarded) to avoid the sender blocking in IO
    uint8_t packet[4];
    bool read = false;


    for (;;) {
        vTaskDelay(1);
        while (tud_midi_available()) {
            read = tud_midi_packet_read(packet);
            if (read) {
                ESP_LOGI(TAG, "Read - Time (ms since boot): %lld, Data: %02hhX %02hhX %02hhX %02hhX",
                         esp_timer_get_time(), packet[0], packet[1], packet[2], packet[3]);
            }
        }
        ESP_ERROR_CHECK(esp_task_wdt_reset());
    }
}

static void vUSBMidiWriteTask(void *arg)
{

 

    static uint8_t const cable_num = 0; // MIDI jack associated with USB endpoint
    static uint8_t const channel = 0; // 0 for channel 1


    for (;;)
    {        
      // Send on channel 1.
      uint8_t note_on[3] = {0x90 | channel, 74, 127};
      tud_midi_stream_write(cable_num, note_on, 3);
      ESP_ERROR_CHECK(esp_task_wdt_reset());
      vTaskDelay(1);
    }



}

//--------------------------------------------------------------------+
// Callbacks
//--------------------------------------------------------------------+


// This callback is called whenever a new MIDI message is received
void callback_midi_message_received(uint8_t blemidi_port, uint16_t timestamp, uint8_t midi_status, uint8_t *remaining_message, size_t len, size_t continued_sysex_pos)
{
  ESP_LOGI(TAG, "CALLBACK blemidi_port=%d, timestamp=%d, midi_status=0x%02x, len=%d, continued_sysex_pos=%d, remaining_message:", blemidi_port, timestamp, midi_status, len, continued_sysex_pos);
  esp_log_buffer_hex(TAG, remaining_message, len);

  // loopback received message
  {
    // TODO: more comfortable packet creation via special APIs

    // Note: by intention we create new packets for each incoming message
    // this shows that running status is maintained, and that SysEx streams work as well
    
    if( midi_status == 0xf0 && continued_sysex_pos > 0 ) {
      blemidi_send_message(0, remaining_message, len); // just forward
    } else {
      size_t loopback_message_len = 1 + len; // includes MIDI status and remaining bytes
      uint8_t *loopback_message = (uint8_t *)malloc(loopback_message_len * sizeof(uint8_t));
      if( loopback_message == NULL ) {
        // no memory...
      } else {
        loopback_message[0] = midi_status;
        memcpy(&loopback_message[1], remaining_message, len);

        blemidi_send_message(0, loopback_message, loopback_message_len);

        free(loopback_message);
      }
    }
  }
}




// Invoked when device is mounted
void tud_mount_cb(void)
{
  ESP_LOGI(TAG,"USB is mounted");

  // Subscribe tasks to TWDT, then check if it is subscribed
  if((esp_task_wdt_status(xUSBMidiReadHandle) != ESP_OK) && (esp_task_wdt_status(xUSBMidiWriteHandle) != ESP_OK)){
    ESP_ERROR_CHECK(esp_task_wdt_add(xUSBMidiReadHandle));
    ESP_ERROR_CHECK(esp_task_wdt_status(xUSBMidiReadHandle));
    ESP_ERROR_CHECK(esp_task_wdt_add(xUSBMidiWriteHandle));
    ESP_ERROR_CHECK(esp_task_wdt_status(xUSBMidiWriteHandle));
  }

  vTaskResume(xUSBMidiReadHandle);
  vTaskResume(xUSBMidiWriteHandle);


  xEventGroupClearBits(xUSBMidiEventGroup, USB_SUSPEND);

}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  ESP_LOGI(TAG,"USB is unmounted");

  if(!(0b10000000 && xEventGroupGetBits(xUSBMidiEventGroup))){
    vTaskSuspend(xUSBMidiReadHandle);
    vTaskSuspend(xUSBMidiWriteHandle);

    // Unsubscribe tasks to TWDT, then check if it is unsubscribed
    ESP_ERROR_CHECK(esp_task_wdt_delete(xUSBMidiReadHandle));
    ESP_ERROR_CHECK(esp_task_wdt_delete(xUSBMidiWriteHandle));
    xEventGroupSetBits(xUSBMidiEventGroup, USB_SUSPEND);
  }
}

// Invoked when usb bus is suspendeds
void tud_suspend_cb(bool remote_wakeup_en)
{
  ESP_LOGI(TAG,"USB is suspended");


  if(!(0b10000000 && xEventGroupGetBits(xUSBMidiEventGroup))){
    vTaskSuspend(xUSBMidiReadHandle);
    vTaskSuspend(xUSBMidiWriteHandle);

    // Unsubscribe tasks to TWDT, then check if it is unsubscribed
    ESP_ERROR_CHECK(esp_task_wdt_delete(xUSBMidiReadHandle));
    ESP_ERROR_CHECK(esp_task_wdt_delete(xUSBMidiWriteHandle));
    xEventGroupSetBits(xUSBMidiEventGroup, USB_SUSPEND);
  }
  
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  ESP_LOGI(TAG,"USB is resumed");

  // Subscribe tasks to TWDT, then check if it is subscribed
  if((esp_task_wdt_status(xUSBMidiReadHandle) != ESP_OK) && (esp_task_wdt_status(xUSBMidiWriteHandle) != ESP_OK)){
    ESP_LOGI(TAG,"USBMidi Task WDT is already subscribed");
    ESP_ERROR_CHECK(esp_task_wdt_add(xUSBMidiReadHandle));
    ESP_ERROR_CHECK(esp_task_wdt_status(xUSBMidiReadHandle));
    ESP_ERROR_CHECK(esp_task_wdt_add(xUSBMidiWriteHandle));
    ESP_ERROR_CHECK(esp_task_wdt_status(xUSBMidiWriteHandle));
  }

  vTaskResume(xUSBMidiReadHandle);
  vTaskResume(xUSBMidiWriteHandle);


  xEventGroupClearBits(xUSBMidiEventGroup, USB_SUSPEND);
  
}


static void continuous_adc_init(uint16_t adc1_chan_mask, uint16_t adc2_chan_mask, adc_channel_t *channel, uint8_t channel_num)
{
    adc_digi_init_config_t adc_dma_config = {
        .max_store_buf_size = 1024,
        .conv_num_each_intr = TIMES,
        .adc1_chan_mask = adc1_chan_mask,
        .adc2_chan_mask = adc2_chan_mask,
    };
    ESP_ERROR_CHECK(adc_digi_initialize(&adc_dma_config));

    adc_digi_configuration_t dig_cfg = {
        .conv_limit_en = ADC_CONV_LIMIT_EN,
        .conv_limit_num = 250,
        .sample_freq_hz = 10 * 1000,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        uint8_t unit = GET_UNIT(channel[i]);
        uint8_t ch = channel[i] & 0x7;
        adc_pattern[i].atten = ADC_ATTEN_DB_0;
        adc_pattern[i].channel = ch;
        adc_pattern[i].unit = unit;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%x", i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%x", i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%x", i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_digi_controller_configure(&dig_cfg));
}


static bool check_valid_data(const adc_digi_output_data_t *data)
{
    const unsigned int unit = data->type2.unit;
    if (unit > 2) return false;
    if (data->type2.channel >= SOC_ADC_CHANNEL_NUM(unit)) return false;

    return true;
}


static void vADCReadTask(void *pvParameters)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[TIMES] = {0};
    memset(result, 0xcc, TIMES);

 

    continuous_adc_init(adc1_chan_mask, adc2_chan_mask, channel, sizeof(channel) / sizeof(adc_channel_t));
    adc_digi_start();



    while(1) {
        ret = adc_digi_read_bytes(result, TIMES, &ret_num, ADC_MAX_DELAY);
        if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
            if (ret == ESP_ERR_INVALID_STATE) {
                /**
                 * @note 1
                 * Issue:
                 * As an example, we simply print the result out, which is super slow. Therefore the conversion is too
                 * fast for the task to handle. In this condition, some conversion results lost.
                 *
                 * Reason:
                 * When this error occurs, you will usually see the task watchdog timeout issue also.
                 * Because the conversion is too fast, whereas the task calling `adc_digi_read_bytes` is slow.
                 * So `adc_digi_read_bytes` will hardly block. Therefore Idle Task hardly has chance to run. In this
                 * example, we add a `vTaskDelay(1)` below, to prevent the task watchdog timeout.
                 *
                 * Solution:
                 * Either decrease the conversion speed, or increase the frequency you call `adc_digi_read_bytes`
                 */
            }

            //ESP_LOGI("ADC DMA", "ret is %x, ret_num is %d", ret, ret_num);
            for (int i = 0; i < ret_num; i += ADC_RESULT_BYTE) {
                adc_digi_output_data_t *p = (void*)&result[i];
    #if CONFIG_IDF_TARGET_ESP32
                //ESP_LOGI("ADC DMA", "Unit: %d, Channel: %d, Value: %x", 1, p->type1.channel, p->type1.data);
    #else
                if (ADC_CONV_MODE == ADC_CONV_BOTH_UNIT || ADC_CONV_MODE == ADC_CONV_ALTER_UNIT) {
                    if (check_valid_data(p)) {
                        //ESP_LOGI("ADC DMA", "Unit: %d,_Channel: %d, Value: %x", p->type2.unit+1, p->type2.channel, p->type2.data);
                    } else {
                        // abort();
                        //ESP_LOGI("ADC DMA", "Invalid data [%d_%d_%x]", p->type2.unit+1, p->type2.channel, p->type2.data);
                    }
                }
    #if CONFIG_IDF_TARGET_ESP32S2
                else if (ADC_CONV_MODE == ADC_CONV_SINGLE_UNIT_2) {
                    //ESP_LOGI("ADC DMA", "Unit: %d, Channel: %d, Value: %x", 2, p->type1.channel, p->type1.data);
                } else if (ADC_CONV_MODE == ADC_CONV_SINGLE_UNIT_1) {
                    //ESP_LOGI("ADC DMA", "Unit: %d, Channel: %d, Value: %x", 1, p->type1.channel, p->type1.data);
                }
    #endif  //#if CONFIG_IDF_TARGET_ESP32S2
    #endif
            }
            //See `note 1`
            ESP_ERROR_CHECK(esp_task_wdt_reset());
            vTaskDelay(1);
        } else if (ret == ESP_ERR_TIMEOUT) {
            /**
             * ``ESP_ERR_TIMEOUT``: If ADC conversion is not finished until Timeout, you'll get this return error.
             * Here we set Timeout ``portMAX_DELAY``, so you'll never reach this branch.
             */
            ESP_LOGW("ADC DMA", "No data, increase timeout or reduce conv_num_each_intr");
            vTaskDelay(1000);
        }

    }

    adc_digi_stop();
    ret = adc_digi_deinitialize();
    assert(ret == ESP_OK);
    vTaskDelete(NULL);
}




//--------------------------------------------------------------------+
// Application Starting Point
//--------------------------------------------------------------------+
void app_main()
{
  

  // install BLE MIDI service
  int status = blemidi_init(callback_midi_message_received);
  if( status < 0 ) {
    ESP_LOGE(TAG, "BLE MIDI Driver returned status=%d", status);
  } else {
    ESP_LOGI(TAG, "BLE MIDI Driver initialized successfully");
    // Write BLE MIDI packets
    ESP_LOGI(TAG, "BLE MIDI write task init");
    xTaskCreatePinnedToCore(vBLEMidiWriteTask, "BLE Midi Write", 4096, NULL, 1, &xBLEMidiWriteHandle,0);
    esp_task_wdt_add(xBLEMidiWriteHandle);
  }



  // install USB MIDI service  
  ESP_LOGI(TAG, "USB initialization");
  tinyusb_config_t const tusb_cfg = {
      .device_descriptor = NULL, // If device_descriptor is NULL, tinyusb_driver_install() will use Kconfig
      .string_descriptor = NULL,
      .external_phy = false // In the most cases you need to use a `false` value
  };
  ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
  ESP_LOGI(TAG, "USB initialization DONE");
  xUSBMidiEventGroup = xEventGroupCreate();
  // Write USB MIDI packets
  ESP_LOGI(TAG, "USB MIDI read task init");
  xTaskCreatePinnedToCore(vUSBMidiReadTask, "USB Midi Read", 2 * 1024, NULL, 1, &xUSBMidiReadHandle,0);
  // Read recieved USB MIDI packets
  ESP_LOGI(TAG, "USB MIDI write task init");
  xTaskCreatePinnedToCore(vUSBMidiWriteTask, "USB Midi Write", 2 * 1024, NULL, 1, &xUSBMidiWriteHandle,0);
  //Suspend until the USB is mounted
  vTaskSuspend(xUSBMidiReadHandle);
  vTaskSuspend(xUSBMidiWriteHandle);
  xEventGroupSetBits(xUSBMidiEventGroup, USB_SUSPEND);

  
  // Read ADC channels
  ESP_LOGI(TAG, "ADC reading task init");
  xTaskCreatePinnedToCore(vADCReadTask, "ADC Read", 4096, NULL, 1, &xADCReadHandle,1);
  esp_task_wdt_add(xADCReadHandle);


}