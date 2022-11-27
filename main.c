#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_bt.h"
#include "esp_gap_bt_api.h"
#include "esp_gap_ble_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_spp_api.h"
#include "nvs.h"
#include "nvs_flash.h"

// the audio DAC and amp configuration.
#include "driver/i2s.h"

#define APP_SIG_WORK_DISPATCH (0x01)

enum
{
  BT_APP_EVT_STACK_UP = 0,
};

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
    .tx_desc_auto_clear = false // avoiding noise in case of data unavailability
                                // .fixed_mclk = -1
};

i2s_channel_t i2s_channels = I2S_CHANNEL_STEREO;

xQueueHandle app_task_queue = nullptr;
xTaskHandle app_task_handle = nullptr;
int event_queue_size = 20;
int event_stack_size = 3072;

esp_a2d_audio_state_t audio_state;

esp_bd_addr_t bda;
size_t bda_size = sizeof(bda);

i2s_port_t i2s_port = I2S_NUM_0;

unsigned long reconnect_timout = 0;
unsigned int default_reconnect_timout = 10000;

const char *bt_name = "ESPEAKER_test";

// i2s pinout
static const i2s_pin_config_t pin_config = {
    .bck_io_num = 33,   // BCLK
    .ws_io_num = 25,    // LRCK
    .data_out_num = 32, // DIN
    .data_in_num = I2S_PIN_NO_CHANGE};

typedef void (*app_callback_t)(uint16_t event, void *param);

typedef struct
{
  uint16_t sig;      // signal to app_task
  uint16_t event;    // message event id
  app_callback_t cb; // context switch callback
  void *param;       // parameter area needs to be last
} app_msg_t;

const char *m_a2d_conn_state_str[4] = {"Disconnected", "Connecting", "Connected", "Disconnecting"};
const char *m_a2d_audio_state_str[3] = {"Suspended", "Stopped", "Started"};

esp_bd_addr_t last_connection = {0, 0, 0, 0, 0, 0};

#define BT_AV_TAG "BT_AV"

void app_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
  ESP_LOGD(BT_AV_TAG, "%s\n", __func__);

  switch (event)
  {
  case ESP_BT_GAP_AUTH_CMPL_EVT:
  {
    if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
    {
      ESP_LOGD(BT_AV_TAG, "Authentication success: %s\n", param->auth_cmpl.device_name);

      //  ESP_LOGD(BT_AV_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
    }
    else
    {
      ESP_LOGE(BT_AV_TAG, "Authentication failed, status: %d\n", param->auth_cmpl.stat);

      // reset pin_code data to "undefined" after authentication failure
      // just like when in disconnected state
      // pin_code_int = 0;
    }

    break;
  }

  case ESP_BT_GAP_PIN_REQ_EVT:
  {
    memcpy(bda, param->pin_req.bda, ESP_BD_ADDR_LEN);
    ESP_LOGD(BT_AV_TAG, "Partner address: %02x:%02x:%02x:%02x:%02x:%02x\n", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  }
  break;

  case ESP_BT_GAP_CFM_REQ_EVT:
  {
    memcpy(bda, param->cfm_req.bda, ESP_BD_ADDR_LEN);

    ESP_LOGD(BT_AV_TAG, "Partner address: %02x:%02x:%02x:%02x:%02x:%02x\n", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

    ESP_LOGD(BT_AV_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please confirm the passkey: %d\n", param->cfm_req.num_val);

    // pin_code_int = param->key_notif.passkey;
  }
  break;

  case ESP_BT_GAP_KEY_NOTIF_EVT:
  {
    ESP_LOGD(BT_AV_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey: %d\n", param->key_notif.passkey);

    // pin_code_int = param->key_notif.passkey;
  }

  break;

  case ESP_BT_GAP_KEY_REQ_EVT:
  {
    ESP_LOGD(BT_AV_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey\n");
  }

  break;

  case ESP_BT_GAP_READ_RSSI_DELTA_EVT:
  {
    break;
  }

  default:
  {
    ESP_LOGD(BT_AV_TAG, "event: %d\n", event);
    break;
  }
  }
}

void handle_audio_cfg(uint16_t event, void *p_param)
{
  ESP_LOGD(BT_AV_TAG, "%s evt %d\n", __func__, event);

  esp_a2d_cb_param_t *a2d = (esp_a2d_cb_param_t *)(p_param);

  ESP_LOGD(BT_AV_TAG, "a2dp audio_cfg_cb, codec type %d\n", a2d->audio_cfg.mcc.type);

  long sample_rate = 16000;

  char oct0 = a2d->audio_cfg.mcc.cie.sbc[0];

  if (oct0 & (0x01 << 6))
  {
    sample_rate = 32000;
  }
  else if (oct0 & (0x01 << 5))
  {
    sample_rate = 44100;
  }
  else if (oct0 & (0x01 << 4))
  {
    sample_rate = 48000;
  }

  ESP_LOGD(BT_AV_TAG, "a2dp audio_cfg_cb, configured sample_rate %d (calculated %d)\n", i2s_config.sample_rate, sample_rate);

  // for now only SBC stream is supported
  ESP_LOGD(BT_AV_TAG, "configure audio player %x-%x-%x-%x\n",
           a2d->audio_cfg.mcc.cie.sbc[0],
           a2d->audio_cfg.mcc.cie.sbc[1],
           a2d->audio_cfg.mcc.cie.sbc[2],
           a2d->audio_cfg.mcc.cie.sbc[3]);

  // setup sample rate and channels
  if (i2s_set_clk(i2s_port, i2s_config.sample_rate, i2s_config.bits_per_sample, i2s_channels) != ESP_OK)
  {
    ESP_LOGD(BT_AV_TAG, "i2s_set_clk failed with samplerate=%d\n", i2s_config.sample_rate);
  }
  else
  {
    ESP_LOGD(BT_AV_TAG, "audio player configured, samplerate=%d\n", i2s_config.sample_rate);
  }
}

void handle_audio_state(uint16_t event, void *p_param)
{
  ESP_LOGD(BT_AV_TAG, "%s evt %d\n", __func__, event);

  esp_a2d_cb_param_t *a2d = (esp_a2d_cb_param_t *)(p_param);

  ESP_LOGD(BT_AV_TAG, "A2DP audio state: %s\n", m_a2d_audio_state_str[a2d->audio_stat.state]);

  // callback on state change
  audio_state = a2d->audio_stat.state;

  if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state)
  {
    ESP_LOGD(BT_AV_TAG, "i2s_start\n");

    if (i2s_start(i2s_port) != ESP_OK)
    {
      ESP_LOGE(BT_AV_TAG, "Failed at i2s_start\n");
    }
  }
  else if (ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND == a2d->audio_stat.state || ESP_A2D_AUDIO_STATE_STOPPED == a2d->audio_stat.state)
  {
    ESP_LOGD(BT_AV_TAG, "i2s_stop\n");

    i2s_stop(i2s_port);
    i2s_zero_dma_buffer(i2s_port);
  }
}

bool has_last_connection()
{
  esp_bd_addr_t empty_connection = {0, 0, 0, 0, 0, 0};

  int result = memcmp(last_connection, empty_connection, ESP_BD_ADDR_LEN);

  return result != 0;
}

void handle_connection_state(uint16_t event, void *p_param)
{
  ESP_LOGD(BT_AV_TAG, "%s evt %d\n", __func__, event);
  esp_a2d_cb_param_t *a2d = (esp_a2d_cb_param_t *)(p_param);

  // determine remote BDA
  memcpy(bda, a2d->conn_stat.remote_bda, ESP_BD_ADDR_LEN);

  int connection_rety_count = 0;
  int try_reconnect_max_count = 1000;
  bool is_autoreconnect_allowed = true;

  ESP_LOGD(BT_AV_TAG, "partner address: %02x:%02x:%02x:%02x:%02x:%02x\n", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

  ESP_LOGD(BT_AV_TAG, "A2DP connection state: %s, [%02x:%02x:%02x:%02x:%02x:%02x]\n", m_a2d_conn_state_str[a2d->conn_stat.state], a2d->conn_stat.remote_bda[0], a2d->conn_stat.remote_bda[1], a2d->conn_stat.remote_bda[2], a2d->conn_stat.remote_bda[3], a2d->conn_stat.remote_bda[4], a2d->conn_stat.remote_bda[5]);

  switch (a2d->conn_stat.state)
  {
  case ESP_A2D_CONNECTION_STATE_DISCONNECTING:
    ESP_LOGD(BT_AV_TAG, "ESP_A2D_CONNECTION_STATE_DISCONNECTING\n");

    if (a2d->conn_stat.disc_rsn == ESP_A2D_DISC_RSN_NORMAL)
    {
      is_autoreconnect_allowed = false;
    }
    break;

  case ESP_A2D_CONNECTION_STATE_DISCONNECTED:
    ESP_LOGD(BT_AV_TAG, "ESP_A2D_CONNECTION_STATE_DISCONNECTED\n");

    if (a2d->conn_stat.disc_rsn == ESP_A2D_DISC_RSN_NORMAL)
    {
      is_autoreconnect_allowed = false;
    }

    // RECONNECTION MGMT
    // do not auto reconnect when disconnect was requested from device
    if (is_autoreconnect_allowed && has_last_connection())
    {
      if (connection_rety_count < try_reconnect_max_count)
      {
        ESP_LOGD(BT_AV_TAG, "Connection try number: %d\n", connection_rety_count);
        // make sure that any open connection is timing out on the target
        memcpy(bda, last_connection, ESP_BD_ADDR_LEN);
        reconnect();
      }
      else
      {
        ESP_LOGD(BT_AV_TAG, "Reconect retry limit reached\n");

        if (has_last_connection() && a2d->conn_stat.disc_rsn == ESP_A2D_DISC_RSN_NORMAL)
        {
          clean_last_connection();
        }
      }
    }
    break;

  case ESP_A2D_CONNECTION_STATE_CONNECTING:
    ESP_LOGD(BT_AV_TAG, "ESP_A2D_CONNECTION_STATE_CONNECTING\n");
    connection_rety_count++;
    break;

  case ESP_A2D_CONNECTION_STATE_CONNECTED:
    ESP_LOGD(BT_AV_TAG, "ESP_A2D_CONNECTION_STATE_CONNECTED\n");

    // checks if the address is valid
    connection_rety_count = 0;

    // record current connection
    set_last_connection(a2d->conn_stat.remote_bda);

    // unpause after reconnect
    // TODO: store the last play timestamp (in audio_evt) in NVS and unpause only if the time from last play is less than, say, 10 sec
    ESP_LOGD(BT_AV_TAG, "Trying to unpause\n");

    esp_avrc_ct_send_passthrough_cmd(0, ESP_AVRC_PT_CMD_PLAY, ESP_AVRC_PT_CMD_STATE_PRESSED);
    esp_avrc_ct_send_passthrough_cmd(0, ESP_AVRC_PT_CMD_PLAY, ESP_AVRC_PT_CMD_STATE_RELEASED);

    break;
  }
}

void clean_last_connection()
{
  ESP_LOGD(BT_AV_TAG, "%s\n", __func__);

  esp_bd_addr_t cleanBda = {0};

  set_last_connection(cleanBda);
}

void set_last_connection(esp_bd_addr_t bda)
{
  ESP_LOGD(BT_AV_TAG, "set_last_connection: %02x:%02x:%02x:%02x:%02x:%02x\n", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

  // same value, nothing to store
  if (memcmp(bda, last_connection, ESP_BD_ADDR_LEN) == 0)
  {
    ESP_LOGD(BT_AV_TAG, "No change in last connection\n");
    return;
  }

  nvs_handle nvs_last_bda_handle;
  esp_err_t err;

  err = nvs_open("connected_bda", NVS_READWRITE, &nvs_last_bda_handle);

  if (err != ESP_OK)
  {
    ESP_LOGE(BT_AV_TAG, "NVS OPEN ERROR\n");
  }

  err = nvs_set_blob(nvs_last_bda_handle, "last_bda", bda, ESP_BD_ADDR_LEN);

  if (err == ESP_OK)
  {
    err = nvs_commit(nvs_last_bda_handle);
  }
  else
  {
    ESP_LOGE(BT_AV_TAG, "NVS WRITE ERROR\n");
  }

  if (err != ESP_OK)
  {
    ESP_LOGE(BT_AV_TAG, "NVS COMMIT ERROR\n");
  }

  nvs_close(nvs_last_bda_handle);

  memcpy(last_connection, bda, ESP_BD_ADDR_LEN);
}

void av_hdl_a2d_evt(uint16_t event, void *p_param)
{
  ESP_LOGD(BT_AV_TAG, "%s evt %d\n", __func__, event);

  esp_a2d_cb_param_t *a2d = NULL;

  switch (event)
  {
  case ESP_A2D_CONNECTION_STATE_EVT:
  {
    ESP_LOGD(BT_AV_TAG, "%s ESP_A2D_CONNECTION_STATE_EVT\n", __func__);
    handle_connection_state(event, p_param);
  }
  break;

  case ESP_A2D_AUDIO_STATE_EVT:
  {
    ESP_LOGD(BT_AV_TAG, "%s ESP_A2D_AUDIO_STATE_EVT\n", __func__);
    handle_audio_state(event, p_param);
  }
  break;

  case ESP_A2D_AUDIO_CFG_EVT:
  {
    ESP_LOGD(BT_AV_TAG, "%s ESP_A2D_AUDIO_CFG_EVT\n", __func__);
    handle_audio_cfg(event, p_param);
  }
  break;

  default:
    ESP_LOGD(BT_AV_TAG, "%s unhandled evt %d\n", __func__, event);
    break;
  }
}

void app_a2d_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
  ESP_LOGD(BT_AV_TAG, "%s", __func__);

  switch (event)
  {
  case ESP_A2D_CONNECTION_STATE_EVT:
    ESP_LOGD(BT_AV_TAG, "%s ESP_A2D_CONNECTION_STATE_EVT\n", __func__);
    app_work_dispatch(av_hdl_a2d_evt, event, param, sizeof(esp_a2d_cb_param_t));
    break;

  case ESP_A2D_AUDIO_STATE_EVT:
    ESP_LOGD(BT_AV_TAG, "%s ESP_A2D_AUDIO_STATE_EVT\n", __func__);
    audio_state = param->audio_stat.state;
    app_work_dispatch(av_hdl_a2d_evt, event, param, sizeof(esp_a2d_cb_param_t));
    break;

  case ESP_A2D_AUDIO_CFG_EVT:
  {
    ESP_LOGD(BT_AV_TAG, "%s ESP_A2D_AUDIO_CFG_EVT\n", __func__);
    app_work_dispatch(av_hdl_a2d_evt, event, param, sizeof(esp_a2d_cb_param_t));
    break;
  }

  default:
    ESP_LOGD(BT_AV_TAG, "Invalid A2DP event: %d\n", event);
    break;
  }
}

void audio_data_callback(const uint8_t *data, uint32_t len)
{
  //  ESP_LOGD(BT_AV_TAG, "%s\n", __func__);

  // put data into ringbuffer
  size_t i2s_bytes_written = 0;

  // standard logic with 16 bits
  if (i2s_write(i2s_port, (void *)data, len, &i2s_bytes_written, portMAX_DELAY) != ESP_OK)
  {
    ESP_LOGD(BT_AV_TAG, "i2s_write has failed\n");
  }
}

void reconnect()
{
  int reconnect_timout = millis() + default_reconnect_timout;

  ESP_LOGD(BT_AV_TAG, "connect_to to %02x:%02x:%02x:%02x:%02x:%02x\n", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

  delay(100);

  esp_err_t err = esp_a2d_sink_connect(bda);

  if (err != ESP_OK)
  {
    ESP_LOGD(BT_AV_TAG, "esp_a2d_source_connect: %d\n", err);
  }
}

void av_hdl_stack_evt(uint16_t event, void *param)
{
  ESP_LOGD(BT_AV_TAG, "%s\n", __func__);

  esp_err_t result;

  switch (event)
  {
  case BT_APP_EVT_STACK_UP:
  {
    ESP_LOGD(BT_AV_TAG, "%s :: %s\n", __func__, "BT_APP_EVT_STACK_UP");

    esp_bt_cod_t cod;

    esp_bt_gap_get_cod(&cod);

    cod.minor |= 6; // headphones
    cod.major |= ESP_BT_COD_MAJOR_DEV_AV;
    cod.service |= ESP_BT_COD_SRVC_RENDERING | ESP_BT_COD_SRVC_AUDIO;

    esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD);

    /* set up device name */
    esp_bt_dev_set_device_name(bt_name);

    result = esp_avrc_ct_init();

    if (result == ESP_OK)
    {
      ESP_LOGD(BT_AV_TAG, "AVRCP controller initialized\n");
    }
    else
    {
      ESP_LOGE(BT_AV_TAG, "esp_avrc_ct_init: %d\n", result);
    }

    /* initialize A2DP sink */
    if (esp_a2d_register_callback(app_a2d_callback) != ESP_OK)
    {
      ESP_LOGD(BT_AV_TAG, "esp_a2d_register_callback\n");
    }

    if (esp_a2d_sink_register_data_callback(audio_data_callback) != ESP_OK)
    {
      ESP_LOGD(BT_AV_TAG, "esp_a2d_sink_register_data_callback\n");
    }

    if (esp_a2d_sink_init() != ESP_OK)
    {
      ESP_LOGD(BT_AV_TAG, "esp_a2d_sink_init\n");
    }

    // start automatic reconnect if relevant and stack is up
    if (has_last_connection())
    {
      ESP_LOGD(BT_AV_TAG, "reconnect\n");
      memcpy(bda, last_connection, ESP_BD_ADDR_LEN);
      reconnect();
    }

    /* set discoverable and connectable mode, wait to be connected */
    ESP_LOGD(BT_AV_TAG, "set_scan_mode_connectable(true)\n");

    if (esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE) != ESP_OK)
    {
      ESP_LOGD(BT_AV_TAG, "esp_bt_gap_set_scan_mode\n");
    }

    break;
  }

  default:
    ESP_LOGD(BT_AV_TAG, "%s unhandled evt %d\n", __func__, event);
    break;
  }
}

void app_task_handler(void *arg)
{
  ESP_LOGD(BT_AV_TAG, "%s\n", __func__);

  app_msg_t msg;

  while (true)
  {
    if (!app_task_queue)
    {
      ESP_LOGD(BT_AV_TAG, "%s, app_task_queue is null\n", __func__);

      delay(100);
    }
    else if (pdTRUE == xQueueReceive(app_task_queue, &msg, (portTickType)portMAX_DELAY))
    {
      ESP_LOGD(BT_AV_TAG, "%s, sig 0x%x, 0x%x\n", __func__, msg.sig, msg.event);

      switch (msg.sig)
      {
      case APP_SIG_WORK_DISPATCH:
        ESP_LOGD(BT_AV_TAG, "%s, APP_SIG_WORK_DISPATCH sig: %d\n", __func__, msg.sig);

        app_work_dispatched(&msg);
        break;
      default:
        ESP_LOGD(BT_AV_TAG, "%s, unhandled sig: %d\n", __func__, msg.sig);
        break;
      }

      if (msg.param)
      {
        free(msg.param);
      }
    }
    else
    {
      ESP_LOGD(BT_AV_TAG, "%s, xQueueReceive -> no data\n", __func__);
      delay(10);
    }
  }
}

bool app_work_dispatch(app_callback_t p_cback, uint16_t event, void *p_params, int param_len)
{
  ESP_LOGD(BT_AV_TAG, "%s event 0x%x, param len %d\n", __func__, event, param_len);

  app_msg_t msg;

  memset(&msg, 0, sizeof(app_msg_t));

  msg.sig = APP_SIG_WORK_DISPATCH;
  msg.event = event;
  msg.cb = p_cback;

  if (param_len == 0)
  {
    return app_send_msg(&msg);
  }
  else if (p_params && param_len > 0)
  {
    if ((msg.param = malloc(param_len)) != NULL)
    {
      memcpy(msg.param, p_params, param_len);
      return app_send_msg(&msg);
    }
  }

  return false;
}

void app_work_dispatched(app_msg_t *msg)
{
  ESP_LOGD(BT_AV_TAG, "%s\n", __func__);

  if (msg->cb)
  {
    msg->cb(msg->event, msg->param);
  }
}

bool app_send_msg(app_msg_t *msg)
{
  ESP_LOGD(BT_AV_TAG, "%s\n", __func__);

  if (msg == NULL || app_task_queue == NULL)
  {
    ESP_LOGD(BT_AV_TAG, "%s app_send_msg failed\n", __func__);
    return false;
  }

  if (xQueueSend(app_task_queue, msg, 10 / portTICK_RATE_MS) != pdTRUE)
  {
    ESP_LOGD(BT_AV_TAG, "%s xQueue send failed\n", __func__);
    return false;
  }

  return true;
}

void setup()
{
  Serial.begin(115200);

  esp_err_t nvs_err = nvs_flash_init();

  if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_LOGD(BT_AV_TAG, "Not enough space in NVS (non-volatile storage), trying to erase...\n");

    nvs_err = nvs_flash_erase();

    if (nvs_err != ESP_OK)
    {
      ESP_LOGE(BT_AV_TAG, "Failed to erase NVS (non-volatile storage)\n");
    }

    nvs_err = nvs_flash_init();
  }

  if (nvs_err != ESP_OK)
  {
    ESP_LOGE(BT_AV_TAG, "Failed to initialize NVS (non-volatile storage)\n");
  }

  nvs_handle nvs_last_connect_address_handle;

  // read last connection address
  nvs_err = nvs_open("connected_bda", NVS_READWRITE, &nvs_last_connect_address_handle);

  nvs_err = nvs_get_blob(nvs_last_connect_address_handle, "last_bda", bda, &bda_size);

  if (nvs_err != ESP_OK)
  {
    if (nvs_err == ESP_ERR_NVS_NOT_FOUND)
    {
      ESP_LOGE(BT_AV_TAG, "nvs_blob does not exist\n");
    }
    else
    {
      ESP_LOGE(BT_AV_TAG, "nvs_get_blob failed\n");
    }
  }

  nvs_close(nvs_last_connect_address_handle);

  if (nvs_err == ESP_OK)
  {
    memcpy(last_connection, bda, bda_size);

    ESP_LOGD(BT_AV_TAG, "Last connected address: %02x:%02x:%02x:%02x:%02x:%02x\n", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  }

  // initialize i2s
  ESP_LOGD(BT_AV_TAG, "Initialize I2S\n");

  // setup i2s
  if (i2s_driver_install(i2s_port, &i2s_config, 0, NULL) != ESP_OK)
  {
    ESP_LOGE(BT_AV_TAG, "Could not install I2S driver\n");
  }

  // pins are only relevant when music is not sent to internal DAC
  if (i2s_set_pin(i2s_port, &pin_config) != ESP_OK)
  {
    ESP_LOGE(BT_AV_TAG, "Could not setup I2S pins\n");
  }

  // setup bluetooth
  if (!btStart())
  {
    ESP_LOGE(BT_AV_TAG, "Failed to initialize Bluetooth controller\n");
  }

  ESP_LOGD(BT_AV_TAG, "Bluetooth controller initialized\n");

  esp_bluedroid_status_t bt_stack_status = esp_bluedroid_get_status();

  if (bt_stack_status == ESP_BLUEDROID_STATUS_UNINITIALIZED)
  {
    if (esp_bluedroid_init() != ESP_OK)
    {
      ESP_LOGE(BT_AV_TAG, "Failed to initialize bluedroid\n");
    }

    ESP_LOGD(BT_AV_TAG, "Bluedroid initialized\n");
  }

  while (bt_stack_status != ESP_BLUEDROID_STATUS_ENABLED)
  {
    if (esp_bluedroid_enable() != ESP_OK)
    {
      ESP_LOGE(BT_AV_TAG, "Failed to enable bluedroid, retrying...\n");

      delay(100);
    }
    else
    {
      ESP_LOGD(BT_AV_TAG, "Bluedroid enabled\n");
    }

    bt_stack_status = esp_bluedroid_get_status();
  }

  if (esp_bt_gap_register_callback(app_gap_callback) != ESP_OK)
  {
    ESP_LOGE(BT_AV_TAG, "GAP callback registration failed\n");
  }

  if (esp_spp_init(ESP_SPP_MODE_CB) != ESP_OK)
  {
    ESP_LOGE(BT_AV_TAG, "SPP initialization failed\n");
  }

  // create application task
  if (app_task_queue == NULL)
  {
    app_task_queue = xQueueCreate(event_queue_size, sizeof(app_msg_t));
  }

  UBaseType_t task_priority = configMAX_PRIORITIES - 10;
  int task_core = 1;

  if (app_task_handle == NULL)
  {
    if (xTaskCreatePinnedToCore(app_task_handler, "BtAppT", event_stack_size, NULL, task_priority, &app_task_handle, task_core) != pdPASS)
    {
      ESP_LOGE(BT_AV_TAG, "%s failed\n", __func__);
    }
  }

  // Bluetooth device name, connection mode and profile set up
  app_work_dispatch(av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0);

  // Set default parameters for Secure Simple Pairing
  esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
  esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
  esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

  // no callbacks
  esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
  esp_bt_pin_code_t pin_code;

  esp_bt_gap_set_pin(pin_type, 0, pin_code);
}

void loop()
{
  // put your main code here, to run repeatedly:
  delay(100);
}
