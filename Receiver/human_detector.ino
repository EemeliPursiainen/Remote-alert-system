#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <SPI.h>
#include <LoRa.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const long freq = 868E6;
const int SF = 7;
const long bw = 31.25E3;

unsigned long time1;

int buzzerPin = 30;
int ledPin = 13;
int batMon = A5;

int rxBatInt = 0, rxBat = 0, rssi = 0;
String txBat = "0000";
String message = "";

void motion();
void updateDisplay();

void setup ()
{
  TXLED0;
  RXLED0;

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  pinMode(batMon, INPUT);

  for (int i = 0; i < 2000; i++)  {
    rxBatInt = rxBatInt * 0.9 + analogRead(batMon) * 0.1;
    delay(1);
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  updateDisplay();

  Serial.begin(115200);

  //lora setup
  LoRa.setPins(SS, 11, 10); // CS, reset, IRQ pin

  while (!LoRa.begin(freq)) {
  }

  //lora settings
  LoRa.setSpreadingFactor(SF);
  LoRa.setSignalBandwidth(bw);
  LoRa.setTxPower(20);
  LoRa.disableCrc();

  time1 = millis();
}

void loop()
{
  digitalWrite(buzzerPin, LOW);
  
  rxBatInt = rxBatInt * 0.95 + analogRead(batMon) * 0.05;
  rxBat = rxBatInt * 6.75;
  Serial.println(rxBat);

  if (millis() - 1000 < time1)  {
    updateDisplay();
  }


  message = "";
  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    Serial.print("PACKET   ");
    message = "";
    while (LoRa.available()) {
      message = message + ((char)LoRa.read());
    }
    sendAck(message);
    time1 = millis();
  }
  //if(message.charAt(0) == 'H' && message.charAt(6) != 'L')  {
  if (message.charAt(0) == 'H')  {
    motion();
  } else   {
    Serial.println("Message preamble not recognized");
    Serial.println(message);
  }

}

void motion() {
  Serial.print("Human detected   voltage = ");
  Serial.print(message.substring(1, 5));
  digitalWrite(ledPin, HIGH);
  digitalWrite(buzzerPin, HIGH);
  delay(150);
  digitalWrite(buzzerPin, LOW);
  delay(100);
  digitalWrite(buzzerPin, HIGH);
  delay(150);
  digitalWrite(buzzerPin, LOW);
  digitalWrite(ledPin, LOW);
  Serial.println();

  txBat = message.substring(1, 5);
  rssi = LoRa.packetRssi();

  updateDisplay();
}

void updateDisplay()  {
  int numx = 80;
  int row2y = 18;
  int row3y = 36;

  display.clearDisplay();
  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println(F("RX bat"));
  display.setCursor(numx, 0);
  display.println(rxBat);

  display.setCursor(0, row2y);
  display.println(F("TX bat"));
  display.setCursor(numx, row2y);
  display.println(txBat);

  display.setCursor(0, row3y);
  display.println(F("rssi"));
  display.setCursor(50, row3y);
  display.println(rssi);
  display.setTextSize(1);
  display.setCursor(100, row3y);
  display.println("dBm");

  display.display();      // Show initial text
  delay(100);
}

//Lora Acknowledgement function
void sendAck(String message) {
  int check = 0;
  for (unsigned int i = 0; i < message.length(); i++) {
    check += message[i];
  }
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();
}
