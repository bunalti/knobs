| Supported Targets | ESP32-S3 | 
| ----------------- | ----- | 

# ESP-IDF Knobs

This program is for controlling a MIDI knobs device housing 16 Knobs. It is possible to interface with either USB or BLE

## How to Use 

There is seperate programs for each task with its header and c file. Each task is executed in the main.c file using FreeRTOS.

### Hardware Required

* ESP32-S3 DevkitC or any other board housing ESP32-S3 chip
* Two USB cables
* 16 x Rotary Potentiometers

### Build and Flash

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)



## Example Output

```
I (550) BLEMIDI: create attribute table successfully, the number handle = 4

I (560) BLEMIDI: SERVICE_START_EVT, status 0, service_handle 40
I (560) BLEMIDI: advertising start successfully
I (560) Knobs: BLE MIDI Driver initialized successfully
I (570) Knobs: USB initialization
I (570) tusb_desc: 
┌─────────────────────────────────┐
│  USB Device Descriptor Summary  │
├───────────────────┬─────────────┤
│bDeviceClass       │ 0           │
├───────────────────┼─────────────┤
│bDeviceSubClass    │ 0           │
├───────────────────┼─────────────┤
│bDeviceProtocol    │ 0           │
├───────────────────┼─────────────┤
│bMaxPacketSize0    │ 64          │
├───────────────────┼─────────────┤
│idVendor           │ 0xcafe      │
├───────────────────┼─────────────┤
│idProduct          │ 0x4008      │
├───────────────────┼─────────────┤
│bcdDevice          │ 0x100       │
├───────────────────┼─────────────┤
│iManufacturer      │ 0x1         │
├───────────────────┼─────────────┤
│iProduct           │ 0x2         │
├───────────────────┼─────────────┤
│iSerialNumber      │ 0x3         │
├───────────────────┼─────────────┤
│bNumConfigurations │ 0x1         │
└───────────────────┴─────────────┘
I (740) TinyUSB: TinyUSB Driver installed
I (750) Knobs: USB initialization DONE
I (750) gpio: GPIO[3]| InputEn: 0| OutputEn: 0| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0 
I (760) ADC DMA: adc_pattern[0].atten is :0
I (760) ADC DMA: adc_pattern[0].channel is :2
I (770) ADC DMA: adc_pattern[0].unit is :0
```


