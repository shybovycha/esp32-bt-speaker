#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_bt.h"
#include "esp_gap_bt_api.h"
#include "esp_gap_ble_api.h"
#include "esp_a2dp_api.h"

#include <EEPROM.h>

// the audio DAC and amp configuration.
#include "driver/i2s.h"

#define EEPROM_LAST_CONN_ADDR_OFFSET 1

// the callback(processes bluetooth data).
// this is the most important function.
void bt_data_cb(const uint8_t *data, uint32_t len) {
   // number of 16 bit samples
   // int n = len / 2;

   // point to a 16bit sample
   // int16_t* data16 = (int16_t*) data;

   size_t i2s_bytes_write = 0;

   i2s_write(I2S_NUM_0, data, len, &i2s_bytes_write, 1);

   // create a variable (potentially processed) that we'll pass via I2S
   /*int16_t fy;

   // Records number of bytes written via I2S
   size_t i2s_bytes_write = 0;

   for (int i = 0; i < n; ++i) {
    // put the current sample in fy
    fy = *data16;

    //making this value larger will decrease the volume(Very simple DSP!).
    // fy /= 1;

    // Serial.println(fy);

    // write data to I2S buffer
    i2s_write(I2S_NUM_0, &fy, 2, &i2s_bytes_write, 10);

    //move to next memory address housing 16 bit data
    data16++;
  }*/
}

/*void bt_event_cb(esp_a2d_cb_event_t evt, esp_a2d_cb_param_t *param) {
  if (evt == ESP_A2D_CONNECTION_STATE_EVT && param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
    // store the connection address
    EEPROM.write(0, 1);

    for (int i = 0; i < ESP_BD_ADDR_LEN; ++i)
    {
       EEPROM.write(EEPROM_LAST_CONN_ADDR_OFFSET + i, (uint8_t) param->conn_stat.remote_bda[i]);
    }
  }
}*/

void setup() {
  Serial.begin(115200);

  /*esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

  // wait for BT controller to become available and idle
  if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_ENABLED) {
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE){
        if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
          Serial.printf("Initializing BT controller failed (1)\n");
        }

        while (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE);
    }

    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
        if (esp_bt_controller_enable(ESP_BT_MODE_BTDM)) {
            Serial.printf("Enabling BT controller failed (2)\n");
            return;
        }
    }

    if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_ENABLED){
      Serial.printf("Getting a grip of BT controller failed (3)\n");
      return;
    }
  }*/

  btStart();

  if (esp_bluedroid_init() != ESP_OK) {
    Serial.printf("Initializing Bluedroid failed\n");
    return;
  }

  if (esp_bluedroid_enable() != ESP_OK) {
    Serial.printf("Enabling Bluedroid failed\n");
    return;
  }

  Serial.printf("BT stack is up\n");

  // setup BT device descriptor
  esp_bt_cod_t cod;

  esp_bt_gap_get_cod(&cod);
  cod.minor |= 6; // headphones
  cod.major |= ESP_BT_COD_MAJOR_DEV_AV ;
  cod.service |= ESP_BT_COD_SRVC_RENDERING | ESP_BT_COD_SRVC_AUDIO;

  esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD);

  // set device name
  esp_bt_dev_set_device_name("ESP32 SPEAKER");

  // initialize i2s / adp
  // esp_a2d_register_callback(&bt_app_a2d_cb);
  // initialize A2DP sink and set the data callback(A2DP is bluetooth audio)
  esp_a2d_sink_register_data_callback(bt_data_cb);

  esp_a2d_sink_init();

  // set scan mode
  // set discoverable and connectable mode, wait to be connected
  esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

  // register BT event callback
  esp_ble_gap_register_callback(&bt_evt_cb);

  static const uint16_t BITRATE = 44100;

  // i2s configuration
  static const i2s_config_t i2s_config = {
    .mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = BITRATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = static_cast<i2s_comm_format_t>(I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // default interrupt priority
    .dma_buf_count = 32,
    .dma_buf_len = 512, // == dma_frame_num = channel_format (alias for num_channels) * bits_per_sample = bits
    .use_apll = false,
    .tx_desc_auto_clear = false
    // .fixed_mclk = -1
  };

  // i2s pinout
  static const i2s_pin_config_t pin_config = {
    .bck_io_num = 4, // BCLK
    .ws_io_num = 2, // LRCK
    .data_out_num = 5, // DIN
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  esp_err_t err;

  Serial.printf("Installing I2S driver\n");

  // now configure i2s with constructed pinout and config
  err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);

  if (err != ESP_OK) {
    Serial.printf("Failed installing driver: %d\n", err);
  }

  err = i2s_set_pin(I2S_NUM_0, &pin_config);

  if (err != ESP_OK) {
    Serial.printf("Failed setting i2s pin: %d\n", err);
  }

  Serial.printf("Starting bt\n");
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
}