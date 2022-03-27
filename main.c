#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
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
  static const uint16_t BITRATE = 44100;

  // i2s configuration
  static const i2s_config_t i2s_config = {
    .mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = BITRATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = static_cast<i2s_comm_format_t>(I2S_COMM_FORMAT_STAND_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // default interrupt priority
    .dma_buf_count = 64,
    .dma_buf_len = 256, // == dma_frame_num = channel_format (alias for num_channels) * bits_per_sample = bits
    .use_apll = true,
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

  // now configure i2s with constructed pinout and config
  err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);

  if (err != ESP_OK) {
    Serial.printf("Failed installing driver: %d\n", err);
  }

  err = i2s_set_pin(I2S_NUM_0, &pin_config);

  if (err != ESP_OK) {
    Serial.printf("Failed setting i2s pin: %d\n", err);
  }

  // i2s_set_clk(I2S_NUM_0, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
  // i2s_set_sample_rates(I2S_NUM_0, 44100);

  err = i2s_set_clk(I2S_NUM_0, BITRATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);

  if (err != ESP_OK) {
    Serial.printf("Failed setting i2s clk: %d\n", err);
  }

  err = i2s_set_sample_rates(I2S_NUM_0, BITRATE);

  if (err != ESP_OK) {
    Serial.printf("Failed setting i2s sample rate: %d\n", err);
  }

  // set up bluetooth classic via bluedroid
  btStart();

  esp_bluedroid_init();
  esp_bluedroid_enable();

  esp_bt_cod_t cod;
  esp_bt_gap_get_cod(&cod);
  cod.minor |= 6; // headphones
  cod.major |= ESP_BT_COD_MAJOR_DEV_AV ;
  cod.service |= ESP_BT_COD_SRVC_RENDERING | ESP_BT_COD_SRVC_AUDIO;
  esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD);

  // set up device name
  const char *dev_name = "ESP_SPEAKER";
  esp_bt_dev_set_device_name(dev_name);

  // initialize A2DP sink and set the data callback(A2DP is bluetooth audio)
  esp_a2d_sink_register_data_callback(bt_data_cb);
  /*esp_a2d_register_callback(bt_event_cb);

  esp_a2d_sink_init();

  esp_bd_addr_t lastConnectionAddr;

  uint8_t wasConnected = EEPROM.read(0);

  if (wasConnected) {
    for (int i = 0; i < ESP_BD_ADDR_LEN; ++i)
    {
      lastConnectionAddr[i] = EEPROM.read(EEPROM_LAST_CONN_ADDR_OFFSET + i);
    }

    err = esp_a2d_sink_connect(lastConnectionAddr);

    if (err != ESP_OK) {
      Serial.printf("Failed connecting to last connection: %d\n", err);
    }
  }*/

  // set discoverable and connectable mode, wait to be connected
  esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
}
