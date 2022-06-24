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

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "blemidi.h"
#include "tinyusb.h"
#include "adc_dma_read.h"

#define TAG "Knobs"


/* A combination of interfaces must have a unique product id, since PC will save device driver after the first plug.
 * Same VID/PID with different interface e.g MSC (first), then CDC (later) will possibly cause system error on PC.
 *
 * Auto ProductID layout's Bitmap:
 *   [MSB]       MIDI | HID | MSC | CDC          [LSB]
 */
#define _PID_MAP(itf, n) ( (CFG_TUD_##itf) << (n) )
#define USB_PID (0x4000 | _PID_MAP(CDC, 0) | _PID_MAP(MSC, 1) | _PID_MAP(HID, 2) | \
                 _PID_MAP(MIDI, 3) | _PID_MAP(VENDOR, 4) )


//--------------------------------------------------------------------+
// This task is periodically called to send MIDI over BLE
//--------------------------------------------------------------------+
static void task_ble_midi(void *pvParameters)
{
  portTickType xLastExecutionTime;
  unsigned ctr = 0;

  // Initialise the xLastExecutionTime variable on task entry
  xLastExecutionTime = xTaskGetTickCount();

  while( 1 ) {
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
}



//--------------------------------------------------------------------+
// This task is periodically called to send MIDI over USB
//--------------------------------------------------------------------+
static void usb_midi_task(void* arg)
{


    for(;;) {

        uint8_t const cable_num = 0; // MIDI jack associated with USB endpoint
        uint8_t const channel   = 0; // 0 for channel 1
        // The MIDI interface always creates input and output port/jack descriptors
        // regardless of these being used or not. Therefore incoming traffic should be read
        // (possibly just discarded) to avoid the sender blocking in IO
        uint8_t packet[4];
        while ( tud_midi_available() ) tud_midi_packet_read(packet);



        // Send Note On for current position at full velocity (127) on channel 1.
        uint8_t midi_cc[3] = { 0xB0 | channel, 0, 127 };
        tud_midi_stream_write(cable_num, midi_cc, 3);

    }
}


//--------------------------------------------------------------------+
// Device callbacks
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

}

// Invoked when device is unmounted
void tud_umount_cb(void)
{

}

// Invoked when usb bus is suspendeds
void tud_suspend_cb(bool remote_wakeup_en)
{

}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{

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
    xTaskCreate(task_ble_midi, "task_ble_midi", 4096, NULL, 8, NULL);
  }

  // install USB MIDI service  
  ESP_LOGI(TAG, "USB initialization");
  tusb_desc_device_t midi_descriptor = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0xCafe,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
  };
  tinyusb_config_t tusb_cfg = {
      .descriptor = &midi_descriptor,
      .string_descriptor = NULL,
      .external_phy = false // In the most cases you need to use a `false` value
  };
  ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
  ESP_LOGI(TAG, "USB initialization DONE");
  xTaskCreate(usb_midi_task, "usb_midi_task", 4096, NULL, 8, NULL);


  // Install ADC Service
  xTaskCreate(task_adc_dma_read, "task_adc_dma_read", 4096, NULL, 8, NULL);
  


#if 1
  // disable this for less debug messages (avoid packet loss on loopbacks)
  esp_log_level_set(TAG, ESP_LOG_NONE);
#endif
}