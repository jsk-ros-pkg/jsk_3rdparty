// Mainly copied from
// https://github.com/m5stack/M5-ProductExampleCodes/blob/master/Unit/PDM/PDM.ino
// We can do FFT by audio_to_spectrogram package, so fft.h is not included.

/*
    Description: Read the microphone data of the PDM Unit and display the audio frequency spectrum.
    Note: Remove the M5GO base when using this example, otherwise it will not work properly,PDM Unit connected PORT-A
*/

#include <driver/i2s.h>

#define PIN_CLK  22
#define PIN_DATA 21
#define MODE_MIC 0
#define AUDIO_SIZE 2048

static QueueHandle_t i2sstateQueue = nullptr;
uint8_t* microRawData = (uint8_t*)calloc(AUDIO_SIZE, sizeof(uint8_t));
size_t bytesread;
float volume;
uint8_t state = MODE_MIC;

void header(const char *string, uint16_t color)
{
    M5.Lcd.fillScreen(color);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.fillRect(0, 0, 320, 30, BLACK);
    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.drawString(string, 160, 3, 4); 
}

void drawVolume(float volume) {
  M5.Lcd.drawString("Volume", 10, 50, 4);
  M5.Lcd.drawString("                                    ", 10, 80, 4);
  M5.Lcd.drawString(String(volume, 3), 10, 80, 4);
  M5.Lcd.fillRect(10, 110, 240, 20, BLACK);
  int16_t width = int16_t(240 * min(volume / 100.0, 1.0));
  M5.Lcd.fillRect(10, 110, width, 20, WHITE);
}

typedef struct
{
    uint8_t state;
    void* audioPtr;
    uint32_t audioSize;
}i2sQueueMsg_t;

bool InitI2SSpakerOrMic(int mode)
{
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER),
        .sample_rate = 44100,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 2,
        .dma_buf_len = 128,
    };
    if (mode == MODE_MIC)
    {
        i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
    }

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_pin_config_t pin_config;
    pin_config.bck_io_num   = I2S_PIN_NO_CHANGE;
    pin_config.ws_io_num    = PIN_CLK;
    pin_config.data_out_num = I2S_PIN_NO_CHANGE;
    pin_config.data_in_num  = PIN_DATA;

    //Serial.println("Init i2s_set_pin");
    i2s_set_pin(I2S_NUM_0, &pin_config);
    //Serial.println("Init i2s_set_clk");
    i2s_set_clk(I2S_NUM_0, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);

    return true;
}


static void readMic()
{
    i2sQueueMsg_t QueueMsg;
    if( xQueueReceive(i2sstateQueue,&QueueMsg,(TickType_t)0) == pdTRUE)
    {
        //Serial.println("Queue Now");
        if( QueueMsg.state == MODE_MIC )
        {
            InitI2SSpakerOrMic(MODE_MIC);
            state = MODE_MIC;
        }
    }
    else if( state == MODE_MIC )
    {
        i2s_read(I2S_NUM_0, (char *)microRawData, AUDIO_SIZE, &bytesread, (100 / portTICK_RATE_MS));
    }
    else
    {
        delay(10);
    }
}


void calcVolume() {
  int16_t* buffptr = ( int16_t*)microRawData;
  float adc_data;
  int audio_size_16bit = AUDIO_SIZE / 2;
  for(int i=0; i<audio_size_16bit; i++) {
    adc_data = (float)map(buffptr[i], INT16_MIN, INT16_MAX, -2000, 2000);
    volume += adc_data * adc_data;
  }
  volume /= audio_size_16bit;
}


void microPhoneSetup()
{
  i2sstateQueue = xQueueCreate(5, sizeof(i2sQueueMsg_t));
  if( i2sstateQueue == 0 )
  {
      return;
  }

  InitI2SSpakerOrMic(MODE_MIC);
}


// If you connect PDM_SPM1423 mic to M5Stack Grove connector,
// you cannot use I2S and I2C devices simultaneously.
// In that case, try the following functions.
// Use I2S device -> enableI2C() -> Use I2C device -> disableI2C() -> Use I2S device
void enableI2C() {
  // Stop I2S
  i2s_stop(I2S_NUM_0);
  // Start I2C
  Wire.begin();
}


void disableI2C() {
  // Stop I2C
  Wire.endTransmission(true);
  // Start I2S
  i2s_driver_uninstall(I2S_NUM_0);
  InitI2SSpakerOrMic(MODE_MIC);
}
