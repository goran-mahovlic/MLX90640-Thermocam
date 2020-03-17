/**
 * Based on: 
 * - https://github.com/wilhelmzeuschner/arduino_thermal_camera_with_sd_and_img_processing.
 * - https://github.com/sparkfun/SparkFun_MLX90641_Arduino_Example
 * - https://github.com/melexis/mlx90641-library
 * - https://github.com/Bodmer/TFT_eSPI
 * 
 * Hardware:
 * - ESP32: https://www.espressif.com/en/products/hardware/esp32-devkitc/overview
 * - Sensor: https://shop.pimoroni.com/products/mlx90641-thermal-camera-breakout
 * - Display: https://www.amazon.de/gp/product/B07DPMV34R/, https://www.pjrc.com/store/display_ili9341.html
 * - PMOD: https://oshpark.com/shared_projects/QxI4a30m
 */


#include <TFT_eSPI.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#include "MLX90641_API.h"
#include "MLX90641_I2C_Driver.h"

#define EMMISIVITY 0.95
#define INTERPOLATE false

#define C_BLUE Display.color565(0,0,255)
#define C_RED Display.color565(255,0,0)
#define C_GREEN Display.color565(0,255,0)
#define C_WHITE Display.color565(255,255,255)
#define C_BLACK Display.color565(0,0,0)
#define C_LTGREY Display.color565(200,200,200)
#define C_DKGREY Display.color565(80,80,80)
#define C_GREY Display.color565(127,127,127)

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

const byte MLX90641_address = 0x33; //Default 7-bit unshifted address of the MLX90641
#define TA_SHIFT 8 //Default shift for MLX90641 in open air
paramsMLX90641 mlx90641;


TFT_eSPI Display = TFT_eSPI();


// Added for measure Temp
boolean measure = true;
float centerTemp;
unsigned long tempTime = millis();
unsigned long tempTime2 = 0;

// start with some initial colors
float minTemp = 10.0;
float maxTemp = 50.0;


// variables for interpolated colors
byte red, green, blue;

// variables for row/column interpolation
float intPoint, val, a, b, c, d, ii;
int x, y, i, j;


// array for the 16 x 12 measured tempValues
static float tempValues[16*12];

// Output size
#define O_WIDTH 32
#define O_HEIGHT 24
#define O_RATIO O_WIDTH/8

float **interpolated = NULL;
uint16_t *imageData = NULL;

void setup() {
  Serial.begin(115200);
  Serial.println("Hello.");

  // Connect thermal sensor.
  Wire.begin();
  Wire.setClock(400000); // Increase I2C clock speed to 400kHz
  Wire.beginTransmission((uint8_t)MLX90641_address);
  if (Wire.endTransmission() != 0) {
    Serial.println("MLX90641 not detected at default I2C address. Please check wiring.");
  }
  else {
    Serial.println("MLX90641 online!");
  }
  // Get device parameters - We only have to do this once
  int status;
  uint16_t eeMLX90641[832];
  status = MLX90641_DumpEE(MLX90641_address, eeMLX90641);
  
  if (status != 0) Serial.println("Failed to load system parameters");
  status = MLX90641_ExtractParameters(eeMLX90641, &mlx90641);
  if (status != 0) Serial.println("Parameter extraction failed");
  // Set refresh rate
  MLX90641_SetRefreshRate(MLX90641_address, 0x05); // Set rate to 8Hz effective - Works at 800kHz
  // Once EEPROM has been read at 400kHz we can increase
  Wire.setClock(800000);
  // Set up Display.
  pinMode(TFT_DC, OUTPUT);
  SPI.begin();
  Serial.println("Passed4");
  SPI.setFrequency(80000000L);
  Display.begin();
  //Display.setRotation(3);
  Display.fillScreen(C_BLACK);

  // Prepare interpolated array
  interpolated = (float **)malloc(O_HEIGHT * sizeof(float *));
  for (int i=0; i<O_HEIGHT; i++) {
    interpolated[i] = (float *)malloc(O_WIDTH * sizeof(float));
  }
  // Prepare imageData array
  imageData = (uint16_t *)malloc(O_WIDTH * O_HEIGHT * sizeof(uint16_t));
  // get the cutoff points for the color interpolation routines
  // note this function called when the temp scale is changed
  setAbcd();
  drawLegend();
}


void loop() {
  tempTime = millis();
  
  readTempValues();
  setTempScale();
  drawPicture();
  drawMeasurement();
}


// Read pixel data from MLX90641.
void readTempValues() {
  for (byte x = 0 ; x < 2 ; x++) // Read both subpages
  {
    uint16_t mlx90641Frame[834];
    int status = MLX90641_GetFrameData(MLX90641_address, mlx90641Frame);
    if (status < 0)
    {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
    }

    float vdd = MLX90641_GetVdd(mlx90641Frame, &mlx90641);
    float Ta = MLX90641_GetTa(mlx90641Frame, &mlx90641);

    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature

    MLX90641_CalculateTo(mlx90641Frame, &mlx90641, EMMISIVITY, tr, tempValues);
  }
}


int row;
float temp, temp2;

void interpolate() {
  for (row=0; row<12; row++) {
    for (x=0; x<O_WIDTH; x++) {
      temp  = tempValues[(15 - (x/3)) + (row*16) + 1];
      temp2 = tempValues[(15 - (x/3)) + (row*16)];
      interpolated[row*4][x] = lerp(temp, temp2, x%4/4.0);
    }
  }
  for (x=0; x<O_WIDTH; x++) {
    for (y=0; y<O_HEIGHT; y++) {
      temp  = interpolated[y-y%4][x];
      temp2 = interpolated[min((y-y%4)+4, O_HEIGHT-4)][x];
      interpolated[y][x] = lerp(temp, temp2, 1);//y%4/4.0);
    }
  }
}


// Linear interpolation
float lerp(float v0, float v1, float t) {
  return v0 + t * (v1 - v0);
}


void drawPicture() {
  if (INTERPOLATE) {
    interpolate();
    for (y=0; y<O_HEIGHT; y++) {
      for (x=0; x<O_WIDTH; x++) {
        imageData[(y*O_WIDTH) + x] = getColor(interpolated[y][x]);
      }
    }
    Display.pushImage(16, 16, O_WIDTH, O_HEIGHT, imageData);
  }
  else {
    for (y=0; y<12; y++) {
      for (x=0; x<16; x++) {
        Display.fillRect(16 + x*14, 16 + y*14, 14, 14, getColor(tempValues[(15-x) + (y*16)]));
      }
    }
  }
}



// Get color for temp value.
uint16_t getColor(float val) {
  /*
    pass in value and figure out R G B
    several published ways to do this I basically graphed R G B and developed simple linear equations
    again a 5-6-5 color display will not need accurate temp to R G B color calculation

    equations based on
    http://web-tech.ga-usa.com/2012/05/creating-a-custom-hot-to-cold-temperature-color-gradient-for-use-with-rrdtool/index.html

  */

  red = constrain(255.0 / (c - b) * val - ((b * 255.0) / (c - b)), 0, 255);

  if ((val > minTemp) & (val < a)) {
    green = constrain(255.0 / (a - minTemp) * val - (255.0 * minTemp) / (a - minTemp), 0, 255);
  }
  else if ((val >= a) & (val <= c)) {
    green = 255;
  }
  else if (val > c) {
    green = constrain(255.0 / (c - d) * val - (d * 255.0) / (c - d), 0, 255);
  }
  else if ((val > d) | (val < a)) {
    green = 0;
  }

  if (val <= b) {
    blue = constrain(255.0 / (a - b) * val - (255.0 * b) / (a - b), 0, 255);
  }
  else if ((val > b) & (val <= d)) {
    blue = 0;
  }
  else if (val > d) {
    blue = constrain(240.0 / (maxTemp - d) * val - (d * 240.0) / (maxTemp - d), 0, 240);
  }

  // use the displays color mapping function to get 5-6-5 color palet (R=5 bits, G=6 bits, B-5 bits)
  return Display.color565(red, green, blue);
}


void setTempScale() {
  minTemp = 255;
  maxTemp = 0;

  for (i = 0; i < 192; i++) {
    minTemp = min(minTemp, tempValues[i]);
    maxTemp = max(maxTemp, tempValues[i]);
  }

  setAbcd();
  drawLegend();
}


// Function to get the cutoff points in the temp vs RGB graph.
void setAbcd() {
  a = minTemp + (maxTemp - minTemp) * 0.2121;
  b = minTemp + (maxTemp - minTemp) * 0.3182;
  c = minTemp + (maxTemp - minTemp) * 0.4242;
  d = minTemp + (maxTemp - minTemp) * 0.8182;
}


// Draw a legend.
void drawLegend() {
  float inc = (maxTemp - minTemp) / 224.0;
  j = 0;
  for (ii = minTemp; ii < maxTemp; ii += inc) {
    Display.drawFastVLine(8+ + j++, 292, 20, getColor(ii));
  }

  Display.setTextFont(2);
  Display.setTextSize(1);
  Display.setCursor(8, 272);
  Display.setTextColor(TFT_WHITE, TFT_BLACK);
  Display.print(String(minTemp).substring(0, 5));

  Display.setCursor(192, 272);
  Display.setTextColor(TFT_WHITE, TFT_BLACK);
  Display.print(String(maxTemp).substring(0, 5));

  Display.setTextFont(NULL);
}


// Draw a circle + measured value.
void drawMeasurement() {
  // Mark center measurement
  Display.drawCircle(120, 8+84, 3, TFT_WHITE);
  // Measure and print center temperature
  centerTemp = (tempValues[96 - 16] + tempValues[96 - 11] + tempValues[96 + 11] + tempValues[96 + 12]) / 4;
  Display.setCursor(86, 214);
  Display.setTextColor(TFT_WHITE, TFT_BLACK);
  Display.setTextFont(2);
  Display.setTextSize(2);
  Display.print(String(centerTemp).substring(0, 5) + " °C");
}
