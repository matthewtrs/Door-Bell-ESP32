#include "ESP_I2S.h"
#include <SD.h>
#include "SPI.h"
#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>

#define I2S_DATA_PIN 22  // I2S data pin (connect to SD)
#define I2S_CLOCK_PIN 14 // I2S clock pin (connect to SCK)
#define I2S_LR_PIN 15    // I2S word select pin (connect to WS)
#define I2S_LR_CONTROL_PIN 13 // Left/Right control pin (LOW for left or HIGH for right)

#define SD_CS_PIN 5 // Chip select pin for SD card
#define ESPNOW_WIFI_CHANNEL 6

I2SClass I2S;

// Wi-Fi Credentials
const char* ssid = "Met"; // Your WiFi SSID
const char* password = "30031973"; // Your WiFi password
int count = 0;
String messages = "";

// Set the initial time (e.g., 08:00)
int hours = 8;
int minutes = 0;

// Slave Address
uint8_t broadcastAddress[] = {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF};

struct __attribute__((packed)) dataPacket {
  int state;
};

esp_now_peer_info_t peerInfo;

// Function to send data ESP-NOW
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Function to update time
void updateTime(void *parameter) {
  while (true) {
    vTaskDelay(60000 / portTICK_PERIOD_MS); // Delay 1 minute
    minutes++;
    if (minutes >= 60) {
      minutes = 0;
      hours++;
      if (hours >= 24) {
        hours = 0;
      }
    }
  }
}

// ESP-NOW message structure
typedef struct struct_message {
  int gpio4State;
} struct_message;

struct_message myData;

WebServer server(80);

// Function to write WAV header
void writeWavHeader(fs::File &file, uint32_t dataSize, uint16_t numChannels, uint32_t sampleRate) {
    // Write WAV header
    file.write((const uint8_t*)"RIFF", 4);
    file.write((uint8_t)((dataSize + 36) & 0xFF));
    file.write((uint8_t)(((dataSize + 36) >> 8) & 0xFF));
    file.write((uint8_t)(((dataSize + 36) >> 16) & 0xFF));
    file.write((uint8_t)(((dataSize + 36) >> 24) & 0xFF));
    file.write((const uint8_t*)"WAVE", 4);
    file.write((const uint8_t*)"fmt ", 4);
    file.write((uint8_t)16);  // Subchunk1Size (16 for PCM)
    file.write((uint8_t)0);
    file.write((uint8_t)0);
    file.write((uint8_t)0);
    file.write((uint8_t)1);   // AudioFormat (1 for PCM)
    file.write((uint8_t)0);
    file.write((uint8_t)numChannels); // NumChannels
    file.write((uint8_t)0);
    file.write((uint8_t)(sampleRate & 0xFF)); // SampleRate
    file.write((uint8_t)((sampleRate >> 8) & 0xFF));
    file.write((uint8_t)((sampleRate >> 16) & 0xFF));
    file.write((uint8_t)((sampleRate >> 24) & 0xFF));
    file.write((uint8_t)((sampleRate * numChannels * 2) & 0xFF)); // ByteRate
    file.write((uint8_t)(((sampleRate * numChannels * 2) >> 8) & 0xFF));
    file.write((uint8_t)(((sampleRate * numChannels * 2) >> 16) & 0xFF));
    file.write((uint8_t)(((sampleRate * numChannels * 2) >> 24) & 0xFF));
    file.write((uint8_t)(numChannels * 2)); // BlockAlign
    file.write((uint8_t)0);
    file.write((uint8_t)16);  // BitsPerSample
    file.write((uint8_t)0);
    file.write((const uint8_t*)"data", 4);
    file.write((uint8_t)(dataSize & 0xFF)); // Subchunk2Size (data size)
    file.write((uint8_t)((dataSize >> 8) & 0xFF));
    file.write((uint8_t)((dataSize >> 16) & 0xFF));
    file.write((uint8_t)((dataSize >> 24) & 0xFF));
}

void setup() {
  // Start the serial communication
  Serial.begin(115200);

  // Set GPIO pins as input
  pinMode(4, INPUT);
  pinMode(34, INPUT);
  pinMode(35, INPUT);

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  Serial.println("SD card initialized.");

  // Initialize I2S for INMP441 microphone
  I2S.setPins(I2S_DATA_PIN, I2S_LR_PIN, I2S_CLOCK_PIN);
    if (!I2S.begin(I2S_MODE_STD, 16000, I2S_DATA_BIT_WIDTH_24BIT, I2S_SLOT_MODE_MONO)) {
      Serial.println("Error initializing I2S");
    return; }
  Serial.println("I2S initialized");

  // Set the L/R control pin to LOW for left channel (mono)
  pinMode(I2S_LR_CONTROL_PIN, OUTPUT);
  digitalWrite(I2S_LR_CONTROL_PIN, LOW);

  // Setup ESP-NOW wifi mode
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected");

  // Serve the recorded audio file
  server.on("/audio", HTTP_GET, []() {
    File audioFile = SD.open("/ESP32_DATA/voice_record.wav", FILE_READ);
    if (audioFile) {
      server.streamFile(audioFile, "audio/wav");
      audioFile.close();
    } else {
      server.send(404, "text/plain", "File not found");
    }
  });

  // Serve a simple HTML page with a button to download the file
  server.on("/", HTTP_GET, []() {
    String html = "<!DOCTYPE html><html><head><title>ESP32 Buzz Record</title></head><body>";
    html += "<h1>ESP32 Buzz Record</h1>";
    html += "<div id='buzzRecords'>";
    html += messages; // Display buzz records
    html += "</div>";
    html += "<p><a href='/audio'><button>Download Recorded Audio</button></a></p>";
    html += "</body></html>";
    server.send(200, "text/html", html);
  });

  // Start the server
  server.begin();
  Serial.println("Server started");

  // Create a FreeRTOS task for the clock update
  xTaskCreatePinnedToCore(
    updateTime,   // Function to implement the task
    "Update Time", // Name of the task
    1000,         // Stack size in words
    NULL,         // Task input parameter
    1,            // Priority of the task
    NULL,         // Task handle
    0);           // Core where the task should run (0 in this case)
}

void loop() {
  // Read the state of the GPIO pins
  int gpioState = LOW;

  if (digitalRead(4) == HIGH) {
    gpioState = 4;  // Button 1 (GPIO4)
  } else if (digitalRead(34) == HIGH) {
    gpioState = 34; // Button 2 (GPIO34)
  } else if (digitalRead(35) == HIGH) {
    gpioState = 35; // Button 3 (GPIO35)
  }

  // Use switch-case to handle each GPIO button press
  switch (gpioState) {
    case 4:
      Serial.println("BUZZ");
      espnowmaster();
      timebuzz();
      break;

    case 34:
      Serial.println("Recording...");
      recordVoiceToSD();
      break;

    case 35:
      Serial.println("Emergency");
      // Add emergency actions here, like sending a notification
      delay(500);
      break;

    default:
      // Default case for when no button is pressed
      break;
  }

  server.handleClient();  // Handle HTTP requests
}

void recordVoiceToSD() {
  // Prepare file on SD card for saving the audio data
  File audioFile = SD.open("/ESP32_DATA/voice_record.wav", FILE_WRITE);
  if (!audioFile) {
    Serial.println("Error opening file for recording.");
    return;
  }

  // Write WAV header
  writeWavHeader(audioFile, 0, 1, 16000); // dataSize=0, numChannels=1 (mono), sampleRate=16000

  // Set up buffer for I2S data
  int32_t buffer[512];  // Buffer for 24-bit audio

  unsigned long startMillis = millis();
  // Record for up to 20 seconds
  while (millis() - startMillis < 20000) {
    // Read I2S data into the buffer
    size_t bytesRead = 0;
    for (int i = 0; i < sizeof(buffer) / sizeof(buffer[0]); i++) {
      int32_t sample = I2S.read();
      if (sample != -1) {
        buffer[i] = sample;
        bytesRead += 3; // Each sample is 3 bytes (24-bit)
      }
    }

    if (bytesRead > 0) {
      // Write data to the SD card
      audioFile.write((uint8_t *)buffer, bytesRead);
    }
  }

  // Finalize WAV file size in header
  unsigned long fileSize = audioFile.size();
  audioFile.seek(4);
  audioFile.write((fileSize - 8) & 0xFF);  // Update the file size in the header
  audioFile.seek(40);
  audioFile.write((fileSize - 44) & 0xFF); // Update the data size in the header

  audioFile.close();
  Serial.println("Recording saved to SD card.");
}

void espnowmaster() {
  dataPacket packet;
  packet.state = digitalRead(4);

  // Send the ESP-NOW message
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&packet, sizeof(packet));

  // Check the result of the send operation
  if (result != ESP_OK) {
    Serial.println("Failed to send ESP-NOW message.");
  }

  delay(30); // Small delay to avoid multiple triggers in quick succession
}

void timebuzz() {
  // Format the time and add it to the messages string
  String timeString = String(hours) + ":" + (minutes < 10 ? "0" : "") + String(minutes);
  messages += "<p>BUZZ at " + timeString + "</p>";
  count++;
  if (count > 5) {
    messages = ""; // Clear the messages after 5 records
    count = 0;
  }
}
