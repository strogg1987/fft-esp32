#include <Arduino.h>

#include <Ticker.h>
#include <arduinoFFT.h>
#include <driver/i2s.h>
#include "FastLED.h"

#define NUM_LEDS 8
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define DATA_PIN 26
CRGB leds[NUM_LEDS];

const i2s_port_t I2S_PORT = I2S_NUM_0;
const int BLOCK_SIZE = 1024;
#define I2S_WS 15  // aka LRCL
#define I2S_SD 32  // aka DOUT
#define I2S_SCK 14 // aka BCLK
const int SAMPLE_RATE = 20480;

const double signalFrequency = 1000;
const double samplingFrequency = 44100;
const uint8_t amplitude = 1000;

double vReal[BLOCK_SIZE];
double vImag[BLOCK_SIZE];
int32_t samples[BLOCK_SIZE];

String labels[] = {"125", "250", "500", "1K", "2K", "4K", "8K", "16K"};

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

float k1 = 4;
int bands[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void fastL(void *params);
double fnt(int a, float p)
{
  auto z = pow(a >> 25, p);
  if (z < 255)
  {
    return z;
  }
  else
  {
    return 255;
  }
}
void setupMic()
{
  Serial.println("Configuring I2S...");
  esp_err_t err;

  const i2s_config_t i2s_config = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Receive, not transfer
      .sample_rate = 44100,                              // 10240 * 2 (20480) Hz
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,      // could only get it to work with 32bits
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,       // LEFT when pin is tied to ground.
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // Interrupt level 1
      .dma_buf_count = 8,                       // number of buffers
      .dma_buf_len = BLOCK_SIZE                 // samples per buffer
  };
  const i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_SCK, // BCLK aka SCK
      .ws_io_num = I2S_WS,   // LRCL aka WS
      .data_out_num = -1,    // not used (only for speakers)
      .data_in_num = I2S_SD  // DOUT aka SD
  };

  // Configuring the I2S driver and pins.
  // This function must be called before any I2S driver read/write operations.
  err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK)
  {
    Serial.printf("Failed installing driver: %d\n", err);
    while (true)
      ;
  }
  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK)
  {
    Serial.printf("Failed setting pin: %d\n", err);
    while (true)
      ;
  }
  Serial.println("I2S driver installed.");
}
xSemaphoreHandle ledSem;
void setup()
{
  ledSem = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(fastL, "LED", 8196, NULL, 2, NULL, 0);
  Serial.begin(115200);
  delay(1000);
  Serial.println("Setting up mic");
  setupMic();
  Serial.println("Mic setup completed");
  delay(1000);
}

void loop()
{
  // Read multiple samples at once and calculate the sound pressure
  int num_bytes_read = i2s_read_bytes(I2S_PORT,
                                      (char *)samples,
                                      BLOCK_SIZE,     // the doc says bytes, but its elements.
                                      portMAX_DELAY); // no timeout

  for (uint16_t i = 0; i < BLOCK_SIZE; i++)
  {
    vReal[i] = samples[i] << 8;
    vImag[i] = 0.0; // Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
  }

  FFT.Windowing(vReal, BLOCK_SIZE, FFT_WIN_TYP_HANN, FFT_FORWARD);
  FFT.Compute(vReal, vImag, BLOCK_SIZE, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, BLOCK_SIZE);
  for (int i = 0; i < 8; i++)
  {
    bands[i] = 0;
  }

  for (int i = 2; i < (BLOCK_SIZE / 2); i++)
  {                      // Don't use sample 0 and only first SAMPLES/2 are usable. Each array eleement represents a frequency and its value the amplitude.
    if (vReal[i] > 2000) // Add a crude noise filter, 10 x amplitude or more
    {
      if (i <= 4)
        bands[0] = max(bands[0], (int)(vReal[i] / amplitude)); // 125Hz
      if (i > 6 && i <= 10)
        bands[1] = max(bands[1], (int)(vReal[i] / amplitude)); // 250Hz
      if (i > 10 && i <= 15)
        bands[2] = max(bands[2], (int)(vReal[i] / amplitude)); // 500Hz
      if (i > 15 && i <= 30)
        bands[3] = max(bands[3], (int)(vReal[i] / amplitude)); // 1000Hz
      if (i > 30 && i <= 60)
        bands[4] = max(bands[4], (int)(vReal[i] / amplitude)); // 2000Hz
      if (i > 60 && i <= 106)
        bands[5] = max(bands[5], (int)(vReal[i] / amplitude)); // 4000Hz
      if (i > 106 && i <= 300)
        bands[6] = max(bands[6], (int)(vReal[i] / amplitude)); // 8000Hz
      if (i > 300)
        bands[7] = max(bands[7], (int)(vReal[i] / amplitude)); // 16000Hz
    }
  }
  xSemaphoreGive(ledSem);
}

void fastL(void *params)
{
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
  for (;;)
  {
    xSemaphoreTake(ledSem, portMAX_DELAY);
    leds[0] = CRGB(uint8_t(fnt(bands[0], k1) / 2), uint8_t(fnt(bands[0], k1) / 2), uint8_t(fnt(bands[0], k1) / 2)); // W
    leds[1] = CRGB(uint8_t(fnt(bands[1], k1)), 0, uint8_t(fnt(bands[1], k1) / 2));                                  // R+b
    leds[2] = CRGB(uint8_t(fnt(bands[2], k1)), 0, 0);                                                               // R
    leds[3] = CRGB(uint8_t(fnt(bands[3], k1)), uint8_t(fnt(bands[1], k1)), 0);                                  // R+g
    leds[4] = CRGB(0, uint8_t(fnt(bands[4], k1)), 0);                                                               // G
    leds[5] = CRGB(0, uint8_t(fnt(bands[5], k1)), uint8_t(fnt(bands[5], k1)));                                  // B+G
    leds[6] = CRGB(0, 0, uint8_t(fnt(bands[6], k1)));
    // leds[7] = CRGB(uint8_t(fnt(bands[6]+bands[7], k1+1)/3), 0, uint8_t(fnt(bands[6]+bands[7], k1)));                                                    // B
    leds[7] = CRGB(uint8_t(fnt(bands[7], k1)), 0, uint8_t(fnt(bands[7], k1))); // B+r
    // leds[7] = CRGB(uint8_t(fnt(bands[0], k1)), uint8_t(fnt(bands[0], k1)), uint8_t(fnt(bands[0], k1))); //W
    FastLED.show();
  }
}