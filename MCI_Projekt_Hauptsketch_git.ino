#include <SPI.h>
#include "LCD_Driver.h"
#include "GUI_Paint.h"
#include "image.h"

// Accelerometer
#include "Wire.h"
const int MPU_ADDR = 0x68;
int16_t accelerometer_x, accelerometer_y, accelerometer_z;
int16_t last_accelerometer_x = 0, last_accelerometer_y = 0, last_accelerometer_z = 0;
const int SHAKE_THRESHOLD = 20000; // Schwellwert für die Schüttelerkennung
/*char tmp_str[7]; // Temporäre Variable für die Umwandlung
char* convert_int16_to_str(int16_t i) {
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}*/
unsigned long previousScan = 0;

// NFC-Reader
#include <Wire.h>
#include <PN532_I2C.h>
#include <PN532.h>
#include <NfcAdapter.h>
PN532_I2C pn532_i2c(Wire);
NfcAdapter nfc = NfcAdapter(pn532_i2c);
byte nuidPICC[4];

// Rotary Encoder - Rotation
#define CLK_PIN 2
#define DT_PIN 3
#define SW_PIN 4
#define DIRECTION_CW 0
#define DIRECTION_CCW 1
int counter = 0;
int direction = DIRECTION_CW;
int CLK_state;
int prev_CLK_state;

// Rotary Encoder - Button
#include <ezButton.h> 
ezButton button(SW_PIN);

// LED-Ring
#include <Adafruit_NeoPixel.h>
#define RING_PIN 6
#define NUMPIXELS 12
Adafruit_NeoPixel ring(NUMPIXELS, RING_PIN, NEO_GRB + NEO_KHZ800);
int colors[6][3] = { {255, 0, 0}, {255, 255, 0}, {0, 255, 0}, {0, 255, 255}, {0, 0, 255}, {255, 0, 255}};
int numColors = 6;

// Timer
unsigned long timerDuration = 0; // seconds
unsigned long previousMillis = 0;
unsigned long elapsed = 0;
unsigned long remaining = timerDuration;
unsigned int secondsPerActiveLED = 1;
bool timerRunning = false;

// Alertzustand
bool alerting = false;          // Gibt an, ob die Vibration aktiv ist
unsigned long alertStart;    // Startzeit der gesamten Vibration
unsigned long alertToggle;   // Zeitpunkt des nächsten Umschaltens (an/aus)
bool alertState = false;     // Zustand des Vibrationsmotors (an/aus)

boolean bitmapChosen = true;

void setup() {
  // Serial.println("Setup");
  pinMode(A0, OUTPUT);
  pinMode(CLK_PIN, INPUT);
  pinMode(DT_PIN, INPUT);
  prev_CLK_state = digitalRead(CLK_PIN);

  // Start NFC
  nfc.begin();

  // Configure Accelerometer
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Configure Ring
  ring.begin();
  ring.clear(); // Alle LEDs ausschalten
  ring.setBrightness(10);

  // Initialize Display
  Config_Init();
  LCD_Init();
  LCD_SetBacklight(1000);
  clearDisplay();
  initDisplay();
  updateRing(0);
  // updateTimeDisplay();
  bitmapChosen = false;
}

void loop() {

  button.loop();
  
  updateAcceleration();
  
  if (button.isPressed()) {
    toggleTimer();
  }
  if(timerRunning) {
    updateTimer();
    updateTimeDisplay();

  } else {

    if(!bitmapChosen) {
      readNFC();
    }

    if(remaining == 0 && bitmapChosen) {
      // Zeit kann wieder drauf geladen werden
      handleRotaryEncoder();
    }
  }
  updateAlert();
}

void readNFC() {
  // Serial.print("Reading");
  if (nfc.tagPresent()) {
    NfcTag tag = nfc.read();

    // tag.print();
    bool hasMessage = tag.hasNdefMessage();
    if(tag.hasNdefMessage()) {
      NdefMessage  message = tag.getNdefMessage();
      //Serial.println("Message found");

      int recordCount = message.getRecordCount();
      if(recordCount > 0) {
        //Serial.println("Message has records");
        for(int i = 0; i < recordCount; i++) {
          NdefRecord record = message.getRecord(i);
          // Serial.print("Record: ");
          // record.print();
          int payloadLength = record.getPayloadLength();
          byte payloadEx[payloadLength];
          record.getPayload(payloadEx);
        
          if(payloadLength > 0) {
            drawBitmap(payloadEx, payloadLength);
          }
        }
      } else{
        //Serial.println("Message has no records");
      }

    } else {
      //Serial.print("Message not found");
    }
  } else {
    // Serial.print("Not present");
  }
}

void handleAccelerometer() {
  long currentMillis = millis();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 7 * 2, true);

  accelerometer_x = Wire.read() << 8 | Wire.read();
  accelerometer_y = Wire.read() << 8 | Wire.read();
  accelerometer_z = Wire.read() << 8 | Wire.read();

  // Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
  // Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
  // Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));

  // Schüttelerkennung
  int delta_x = abs(accelerometer_x - last_accelerometer_x);
  int delta_y = abs(accelerometer_y - last_accelerometer_y);
  int delta_z = abs(accelerometer_z - last_accelerometer_z);
  
  if (delta_x > SHAKE_THRESHOLD || delta_y > SHAKE_THRESHOLD || delta_z > SHAKE_THRESHOLD) {
    
    if(currentMillis - previousScan < 1500) {
      // Serial.print("SHAKE DETECTED!");
      taskDone();
    }
  }
  
  last_accelerometer_x = accelerometer_x;
  last_accelerometer_y = accelerometer_y;
  last_accelerometer_z = accelerometer_z;
  previousScan = currentMillis;
}

void handleRotaryEncoder() {

  CLK_state = digitalRead(CLK_PIN);

  // If the state of CLK is changed, then pulse occurred
  // React to only the rising edge (from LOW to HIGH) to avoid double count
  if (CLK_state != prev_CLK_state && CLK_state == HIGH) {
    // if the DT state is HIGH
    // the encoder is rotating in counter-clockwise direction => increase the counter
    if (digitalRead(DT_PIN) == LOW) {
      counter--;
      if (counter == -1) {
        counter = 0;
      }
    } else {
      counter++;
    }

    fillRing(counter);
    
  }

  // save last CLK state
  prev_CLK_state = CLK_state;
}

void updateTimer() {
  if (timerRunning) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 1000) {
      elapsed++;
      previousMillis = currentMillis;
      if (elapsed >= timerDuration) {
        timesUp();
        timerRunning = false;
        elapsed = timerDuration;
      }
      remaining = timerDuration - elapsed;
    }
  }
}

void updateAcceleration() {
  unsigned long currentMillis = millis();
    if (currentMillis - previousScan >= 1000) {
      handleAccelerometer();
    }
}

void DrawImageFromNewPayload(const byte *payload, int payloadLength,  UWORD xStart, UWORD yStart) {

  int scale = 4;
  int row = -1;

  for (int i = 0; i < payloadLength; i++) {
      
    byte pixelByte = payload[i];
    bool pixelWhite = (pixelByte == 0xFF);  // Wenn Byte 0xFF, dann weiße Pixel
    bool pixelBlack = (pixelByte == 0x00);  // Wenn Byte 0x00, dann schwarze Pixel

    if(i%4 == 0) row++; // Alle zwei Bytes eine neue Reihe

    /*
    Serial.print("Byte in HEX: ");
    Serial.print(payload[i]);
    Serial.print("Byte in BIN: ");
    */
    for (int j = 7; j >= 0; j--) {

      bool pixelWhite = (payload[i] >> j) & 1;
      // Serial.print(pixelWhite);  // Bitweise Ausgabe

      int offset = i % 4;
      int xCurrent = xStart + (7 - j) * scale;  // Korrekte Position (MSB → LSB)
      xCurrent = xCurrent + 8 * offset * scale;
      int xNext = xStart + (8 - j) * scale;     // Nächste Position (falls vorhanden)
      xNext = xNext + 8 * offset * scale;

      if (pixelWhite) {
          Paint_DrawRectangle(xCurrent, yStart + row * scale, xNext, yStart + row * scale + scale, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
          // Serial.print(" Printing white from ");
      } else {
          // Hintergrund ist bereits schwarz
          // Paint_DrawRectangle(xCurrent, yStart + row * scale, xNext, yStart + row * scale + scale, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
          // Serial.print(" Printing black from ");
      }

      /*
      Serial.print(xCurrent);
      Serial.print(" to ");
      Serial.print(xNext);
      Serial.println();
      */
    }

  }

}

void updateTimeDisplay() {
  int minutes = remaining / 60;
  int seconds = remaining % 60;
  writeTime(formatTime(minutes, seconds).c_str());
  float percentage = (float)remaining / (float)timerDuration; // Division als float durchführen
  int pixels = (int)(percentage * NUMPIXELS);  // Berechne die Anzahl der Pixel basierend auf dem Prozentsatz
  updateRing(pixels, 255, 255, 255);  // Update den Ring mit der berechneten Anzahl von LEDs
  // Idee eine effizientere Funktion zu nutzen... Hat nicht funktioniert
  /*
  PAINT_TIME *time;
  time->Min = 12;  // Beispiel: 12 Minuten
  time->Sec = 34;  // Beispiel: 34 Sekunden
  Paint_DrawTime_MMSS(80, 178, time, &Font24, WHITE, BLACK);
  */
}

void writeTime(const char *c) {
  Paint_DrawString_EN(80, 178, c, &Font24, BLACK, WHITE);
}

// Not yet implemented
void clearTime() {
  Paint_DrawRectangle(80, 178, 80 + Font24.Width * strlen("00:00"), 178 + 24, WHITE, DOT_PIXEL_1X1, 1);
}

String formatTime(int minutes, int seconds) {
  char buffer[6];
  sprintf(buffer, "%02d:%02d", minutes, seconds);
  return String(buffer);
}

void toggleTimer() {
  // Serial.print("Button pressed");
  if(timerRunning) {
    // just pause timer
    timerRunning = false;
  } else {
    // Check if Timer needs to be reset
    if(remaining == 0) {

      // Reset
      resetTimer();
    } 
    timerRunning = true;
  }
}

void resetTimer() {
  timerDuration = counter * secondsPerActiveLED;
  remaining = timerDuration;
  elapsed = 0;
  counter = 0;
  previousMillis = millis();
}

void timesUp() {
  alert();
}

void taskDone() {
  clearDisplay();
  timerRunning = false;
  elapsed = timerDuration;
  remaining = 0;
  counter = 0;
  rainbow(5);
  updateRing(0);
  initDisplay();
}

void alert() {
  alerting = true;                 // Vibration aktivieren
  alertStart = millis();        // Startzeit speichern
  alertToggle = millis();       // Zeitpunkt des ersten Umschaltens setzen
  alertState = false;           // Initialen Zustand (aus)
}

void updateAlert() {
  unsigned long currentMillis = millis();

  // Überprüfen, ob die Vibration noch aktiv ist
  if (alerting) {
    // Umschalten der Vibration alle 1 Sekunde
    if (currentMillis - alertToggle >= 1000) {
      alertToggle = currentMillis;  // Zeitpunkt des nächsten Umschaltens setzen
      alertState = !alertState; // Zustand umschalten

      digitalWrite(A0, alertState ? HIGH : LOW);  // Schalte den Vibrationsmotor
      
      if(alertState) {
        updateRing(NUMPIXELS, 255, 0, 0);
      } else {
        updateRing(0);
      }
    }

    // Überprüfen, ob die Gesamtdauer der Vibration abgelaufen ist (10 Sekunden)
    if (currentMillis - alertStart >= 10000) {
      alerting = false;            // Vibration beenden
      digitalWrite(A0, LOW);        // Motor ausschalten
      updateRing(0);
      clearDisplay();
      initDisplay();
    }
  }
}

void clearDisplay() {
  Paint_NewImage(LCD_WIDTH, LCD_HEIGHT, 0, BLACK);
  Paint_Clear(BLACK);
  bitmapChosen = false;
}

void initDisplay() {
  Paint_DrawCircle(120,120, 120, WHITE ,DOT_PIXEL_2X2,DRAW_FILL_EMPTY);
}

void drawBitmap(const byte* payload, int payloadLength) {
  // clearBitmap();
  clearDisplay();
  DrawImageFromNewPayload(payload, payloadLength,  54, 34);
  initDisplay();
  bitmapChosen = true;
}

void updateRing(int count, int r, int g, int b) {
  ring.clear(); // Alle LEDs ausschalten

  // LEDs bis zur aktiven Anzahl einschalten
  for (int i = 0; i < count; i++) {
    ring.setPixelColor(i, ring.Color(r, g, b));
  }

  ring.show(); // Änderungen anzeigen
}

void updateRing(int count) {
  updateRing(count, 255, 0, 0);
}

void fillRing(int count) {

  if(count == 0) {
    // Serial.println("Is null");
    updateRing(0);
    return;
  }

  int pixels = count % NUMPIXELS;
  if(pixels == 0) pixels = 12;
  int index = (count - 1) / NUMPIXELS;
  index = index % numColors;
  /*
  Serial.print("Counter: ");
  Serial.print(count);
  Serial.print("Index: ");
  Serial.print(index);
  Serial.print("Pixels: ");
  Serial.print(pixels);
  Serial.println();
  */
  int r = colors[index][0];
  int g = colors[index][1];
  int b = colors[index][2];

  updateRing(pixels, r, g, b);
}

void rainbow(int wait) {
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    for(int i=0; i<ring.numPixels(); i++) {
      int pixelHue = firstPixelHue + (i * 65536L / ring.numPixels());
      ring.setPixelColor(i, ring.gamma32(ring.ColorHSV(pixelHue)));
    }
    ring.show();
    delay(wait);
  }
}

