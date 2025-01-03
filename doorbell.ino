#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <driver/i2s.h>

// I2S pins
#define I2S_DATA_PIN 22
#define I2S_CLOCK_PIN 14
#define I2S_LR_PIN 15

// SD card CS pin
#define SD_CS_PIN 5

// WiFi credentials
const char* ssid = "Met";
const char* password = "30031973";

// Web server
WebServer server(80);

// ESP-NOW Peer (ESP-01)
uint8_t esp01Address[] = {0xFC, 0xF5, 0xC4, 0xA7, 0x0A, 0x1C}; // ESP-01's MAC address

// ESP-NOW Message Structure
struct __attribute__((packed)) espnowMessage {
  char type[10]; // "BUZZER" or "FIRE"
  int state;     // 1 (HIGH) or 0 (LOW)
};

// Variables for web interface
String messages = "";
int recordingCounter = 0;       // Counter for unique filenames
bool newVoiceMessageAvailable = false; // Flag for new voice message

// I2S configuration
const i2s_config_t i2s_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
  .sample_rate = 16000,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
  .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
  .communication_format = I2S_COMM_FORMAT_I2S_MSB,
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 16,
  .dma_buf_len = 1024,
  .use_apll = false,
  .tx_desc_auto_clear = false,
  .fixed_mclk = 0
};

const i2s_pin_config_t pin_config = {
  .bck_io_num = I2S_CLOCK_PIN,
  .ws_io_num = I2S_LR_PIN,
  .data_out_num = -1, // Not used for RX
  .data_in_num = I2S_DATA_PIN
};

// State Machine States
enum State {
  IDLE,
  BUZZER_ACTIVE,
  RECORDING,
  EMERGENCY,
  FIRE_ALERT
};

State currentState = IDLE; // Initial state

// Function to handle received ESP-NOW data
void OnDataRecv(const esp_now_recv_info_t *mac, const uint8_t *incomingData, int len) {
  espnowMessage receivedMsg;
  memcpy(&receivedMsg, incomingData, sizeof(receivedMsg));

  if (strcmp(receivedMsg.type, "FIRE") == 0 && receivedMsg.state == 1) {
    currentState = FIRE_ALERT; // Transition to FIRE_ALERT state
    messages = "FIRE ALERT"; // Update the web interface message
    Serial.println("FIRE ALERT received from ESP-01");
  }
}

// Function to send ESP-NOW data
void sendEspNowMessage(const char* type, int state) {
  espnowMessage msg;
  strcpy(msg.type, type);
  msg.state = state;

  // Send the ESP-NOW message
  esp_err_t result = esp_now_send(esp01Address, (uint8_t *)&msg, sizeof(msg));

  // Check the result of the send operation
  if (result != ESP_OK) {
    Serial.println("Failed to send ESP-NOW message.");
  } else {
    Serial.println("Message sent to ESP-01.");
  }
}

// Function to record voice to SD card
void recordVoiceToSD() {
  int retryCount = 0;
  const int maxRetries = 3; // Maximum number of retries
  bool fileSavedSuccessfully = false;

  while (retryCount < maxRetries && !fileSavedSuccessfully) {
    // Generate a unique filename
    String filename = "/ESP32_DATA/voice_record_" + String(recordingCounter) + ".wav";
    recordingCounter++; // Increment the counter for the next recording

    // Open the file for writing
    File audioFile = SD.open(filename, FILE_WRITE);
    if (!audioFile) {
      Serial.println("Error opening file for recording.");
      retryCount++;
      continue; // Skip to the next retry
    }

    // Write WAV header
    writeWAVHeader(audioFile);

    // Read and save I2S data
    int16_t buffer[512];
    size_t bytesRead = 0;
    unsigned long startMillis = millis();

    while (millis() - startMillis < 20000) { // Record for 20 seconds
      i2s_read(I2S_NUM_0, buffer, sizeof(buffer), &bytesRead, portMAX_DELAY);
      audioFile.write((uint8_t*)buffer, bytesRead);
    }

    // Update file sizes in header
    unsigned long fileSize = audioFile.size();
    audioFile.seek(4);
    audioFile.write((uint8_t*)&fileSize, 4);
    audioFile.seek(40);
    unsigned long dataChunkSize = fileSize - 44;
    audioFile.write((uint8_t*)&dataChunkSize, 4);

    audioFile.close();
    Serial.println("Recording saved to SD card: " + filename);

    // Verify that the file is really available on the SD card
    if (verifyFileExists(filename)) {
      Serial.println("File verification successful: " + filename);
      fileSavedSuccessfully = true; // Mark as successfully saved
      newVoiceMessageAvailable = true; // Set flag for new voice message
    } else {
      Serial.println("File verification failed: " + filename);
      retryCount++; // Increment retry count
    }
  }

  if (!fileSavedSuccessfully) {
    Serial.println("Failed to save recording after " + String(maxRetries) + " retries.");
  }
}

// Function to verify if a file exists on the SD card
bool verifyFileExists(const String& filename) {
  File file = SD.open(filename, FILE_READ);
  if (file) {
    file.close();
    return true; // File exists and is accessible
  } else {
    return false; // File does not exist or cannot be opened
  }
}

// Function to write WAV header
void writeWAVHeader(File& file) {
  uint32_t fileSize = 36 + 1000 * 512; // Placeholder values
  uint32_t dataChunkSize = 1000 * 512;
  uint16_t audioFormat = 1, numChannels = 1, bitsPerSample = 16;
  uint32_t sampleRate = 16000, byteRate = sampleRate * numChannels * bitsPerSample / 8;
  uint16_t blockAlign = numChannels * bitsPerSample / 8;

  file.write((const uint8_t*)"RIFF", 4);
  file.write((uint8_t*)&fileSize, 4);
  file.write((const uint8_t*)"WAVE", 4);
  file.write((const uint8_t*)"fmt ", 4);
  uint32_t subChunk1Size = 16;
  file.write((uint8_t*)&subChunk1Size, 4);
  file.write((uint8_t*)&audioFormat, 2);
  file.write((uint8_t*)&numChannels, 2);
  file.write((uint8_t*)&sampleRate, 4);
  file.write((uint8_t*)&byteRate, 4);
  file.write((uint8_t*)&blockAlign, 2);
  file.write((uint8_t*)&bitsPerSample, 2);
  file.write((const uint8_t*)"data", 4);
  file.write((uint8_t*)&dataChunkSize, 4);
}

// Function to update buzz records
void timebuzz() {
  // Format the time and add it to the messages string
  String timeString = String(millis() / 1000) + " seconds";
  messages += "<p>BUZZ at " + timeString + "</p>";
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

  // Initialize I2S
  esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.println("Failed to install I2S driver");
    return;
  }
  i2s_set_pin(I2S_NUM_0, &pin_config);
  Serial.println("I2S initialized");

  // Setup ESP-NOW wifi mode
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback for receiving ESP-NOW messages
  esp_now_register_recv_cb(OnDataRecv);

  // Add ESP-01 as a peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, esp01Address, 6);
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

  // Print the IP address
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  // Set up HTTP routes
  server.on("/", HTTP_GET, []() {
    String html = "<!DOCTYPE html><html><head><title>ESP32 Doorbell</title></head><body>";
    html += "<h1>ESP32 Doorbell</h1>";
    html += "<div id='buzzRecords'>";
    html += messages; // Display buzz records or "FIRE ALERT"
    html += "</div>";

    // Add voice message indicator
    if (newVoiceMessageAvailable) {
      html += "<p><strong>1 Voice message available</strong></p>";
    }

    html += "<p><a href='/download?file=" + String(recordingCounter - 1) + "'><button>Download Latest Recording</button></a></p>";
    html += "</body></html>";
    server.send(200, "text/html", html);
  });

  server.on("/download", HTTP_GET, []() {
    String fileIndex = server.arg("file");
    String filename = "/ESP32_DATA/voice_record_" + fileIndex + ".wav";

    File audioFile = SD.open(filename, FILE_READ);
    if (audioFile) {
      server.sendHeader("Content-Type", "audio/wav");
      server.streamFile(audioFile, "audio/wav");
      audioFile.close();

      // Reset the flag after the file is accessed
      newVoiceMessageAvailable = false;
    } else {
      server.send(404, "text/plain", "File not found");
    }
  });

  // Start the server
  server.begin();
  Serial.println("Server started");
}

void loop() {
  // Handle HTTP requests
  server.handleClient();

  // State Machine Logic
  switch (currentState) {
    case IDLE:
      // Check for button presses
      if (digitalRead(4) == HIGH) {
        currentState = BUZZER_ACTIVE;
      } else if (digitalRead(34) == HIGH) {
        currentState = RECORDING;
      } else if (digitalRead(35) == HIGH) {
        currentState = EMERGENCY;
      }
      break;

    case BUZZER_ACTIVE:
      Serial.println("BUZZ");
      sendEspNowMessage("BUZZER", 1); // Send "BUZZER ON" to ESP-01
      timebuzz();
      delay(500);
      currentState = IDLE; // Return to IDLE state
      break;

    case RECORDING:
      Serial.println("Recording...");
      recordVoiceToSD();
      currentState = IDLE; // Return to IDLE state
      break;

    case EMERGENCY:
      Serial.println("Emergency");
      // Add emergency actions here
      delay(500);
      currentState = IDLE; // Return to IDLE state
      break;

    case FIRE_ALERT:
      // Handle fire alert (e.g., display message, trigger alarms)
      Serial.println("FIRE ALERT active");
      delay(1000); // Simulate handling
      currentState = IDLE; // Return to IDLE state
      break;
  }
}
