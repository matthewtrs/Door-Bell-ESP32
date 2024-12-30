#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <driver/i2s.h>
#include "time.h"
#include "esp_sntp.h"

// I2S pins
#define I2S_DATA_PIN 22
#define I2S_CLOCK_PIN 14
#define I2S_LR_PIN 15

// SD card CS pin
#define SD_CS_PIN 5

// WiFi credentials
const char *ssid = "Met";
const char *password = "30031973";

// NTP Server settings
const char *ntpServer1 = "pool.ntp.org";
const char *ntpServer2 = "time.nist.gov";

// Timezone for Western Indonesia Time (WIB), UTC+7, without daylight saving adjustments
const char *time_zone = "WIB-7";

// Callback function (gets called when time adjusts via NTP)
void timeavailable(struct timeval *t) {
    Serial.println("Got time adjustment from NTP!");
    printLocalTime();
}

void printLocalTime() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("No time available (yet)");
        return;
    }
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

// Web server
WebServer server(80);

// I2S configuration
const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 512,
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

int messageCount = 0;  // Track the number of messages
std::vector<String> bellTimes; // Track the bell ring times

void updateInboxInHTML() {
    String html = "<html><body style='font-family: Arial, sans-serif;'>";
    html += "<h1>ESP32 Dashboard</h1>";

    // Case 34
    html += "<div style='margin-bottom: 20px;'>";
    html += "<h2>Case 34</h2>";
    html += "<p>" + String(messageCount) + " inbox</p>";
    for (int i = 1; i <= messageCount; i++) {
        html += "<p><a href='/message/" + String(i) + "'>View Msg " + String(i) + "</a></p>";
    }
    html += "</div>";

    html += "</body></html>";

    // Tampilkan HTML ke dalam log serial (atau Anda bisa memperbarui halaman web)
    Serial.println(html);
}

void updateBellTimesInHTML();

void core0Task(void *pvParameters) {
    while (true) {
        server.handleClient();  // Handle HTTP requests
        delay(1000); // Add delay
        Serial.print("Free heap (Core 0): ");
        Serial.println(ESP.getFreeHeap());
    }
}

void core1Task(void *pvParameters) {
    int gpioState = LOW;
    while (true) {
        // Read the state of the GPIO pins
        gpioState = LOW;
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
                // Store the current time in the list and update the HTML
                bellTimes.push_back(printLocalTime());
                updateBellTimesInHTML();
                delay(500); // Add a small delay to avoid multiple prints
                break;

            case 34:
                Serial.println("Recording...");
                // Start recording audio from INMP441 and save to SD card
                recordVoiceToSD();
                // After recording, update the message count and upload to HTML
                messageCount++;
                updateInboxInHTML();
                break;

            case 35:
                Serial.println("Emergency");
                // Redirect to the emergency route to trigger the emergency page
                server.sendHeader("Location", "/emergency", true);
                server.send(302, "text/plain", "");
                delay(500);
                break;

            default:
                // Default case for when no button is pressed
                break;
        }
    }
}

void setup() {
    Serial.begin(115200);

    // Set GPIO pins as input
    pinMode(4, INPUT);
    pinMode(34, INPUT);
    pinMode(35, INPUT);

    analogReadResolution(12); // Resolusi ADC hingga 12-bit (default 10-bit)

    // Initialize SD card
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD card initialization failed!");
        return;
    }
    Serial.println("SD card initialized.");

    // Check if the directory exists, and create it if not
    if (!SD.exists("/ESP32_DATA")) {
        Serial.println("Directory /ESP32_DATA does not exist. Creating directory...");
        if (SD.mkdir("/ESP32_DATA")) {
            Serial.println("Directory created successfully.");
        } else {
            Serial.println("Failed to create directory.");
            return;
        }
    }

    // Initialize I2S
    esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.println("Failed to install I2S driver");
        return;
    }
    i2s_set_pin(I2S_NUM_0, &pin_config);
    Serial.println("I2S initialized");

    // Connect to WiFi
    Serial.printf("Connecting to %s ", ssid);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println(" CONNECTED");

    // Set NTP servers
    esp_sntp_servermode_dhcp(1);  // Optional: Set SNTP server from DHCP

    // Set time sync callback
    sntp_set_time_sync_notification_cb(timeavailable);

    // Configure time zone and daylight saving rules (Indonesia does not use daylight saving time)
    configTzTime(time_zone, ntpServer1, ntpServer2);  // Use the correct timezone

    // Set up HTTP routes
    server.on("/audio", HTTP_GET, []() {
        File audioFile = SD.open("/ESP32_DATA/voice_record.wav", FILE_READ);
        if (audioFile) {
            server.streamFile(audioFile, "audio/wav");
            audioFile.close();
        } else {
            server.send(404, "text/plain", "File not found");
        }
    });

    server.on("/", HTTP_GET, []() {
        String html = "<html><body style='font-family: Arial, sans-serif;'>";
        html += "<h1>ESP32 Dashboard</h1>";

        // Case 4
        html += "<div style='margin-bottom: 20px;'>";
        html += "<h2>Case 4</h2>";
        for (const auto& time : bellTimes) {
            html += "<p><img src='https://example.com/bell_icon.png' alt='Bell Icon' style='width:16px;height:16px;'> Bel bunyi jam: " + time + "</p>";
        }
        html += "</div>";

        // Case 34
        html += "<div style='margin-bottom: 20px;'>";
        html += "<h2>Case 34</h2>";
        if (messageCount > 0) {
            html += "<p><a href='/messages' style='color: blue;'>" + String(messageCount) + " messages left</a></p>";
        } else {
            html += "<p style='color: gray;'>0 messages left</p>";
        }
        html += "</div>";

        // Case 35
        html += "<div style='margin-bottom: 20px;'>";
        html += "<h2>Case 35</h2>";
        html += "<p><a href='/emergency'><button style='color: white; background: red; padding: 10px;'>Trigger Emergency</button></a></p>";
        html += "</div>";

        html += "</body></html>";
        server.send(200, "text/html", html);
    });

        server.on("/emergency", HTTP_GET, []() {
        String html = "<html><head><style>";
        html += "@keyframes flash { 50% { opacity: 0.5; } }";
        html += "body { animation: flash 1s infinite; background-color: red; color: white; font-family: Arial, sans-serif; display: flex; justify-content: center; align-items: center; height: 100vh; margin: 0; }";
        html += "</style>";
        html += "<script>function playSound() { var audio = new Audio('https://example.com/alert_sound.mp3'); audio.play(); }</script>";
        html += "</head><body onload='playSound()'>";
        html += "<div>";
        html += "<h1 style='font-size: 5em;'>Emergency!</h1>";
        html += "<img src='https://example.com/emergency_icon.png' alt='Emergency Icon' style='width:64px;height:64px;'>";
        html += "<p><button onclick='muteSound()' style='padding: 10px;'>Mute</button></p>";
        html += "<p><button style='padding: 10px; background: white; color: red;'>Acknowledge</button></p>";
        html += "<p><button style='padding: 10px; background: white; color: red;'>Notify Authorities</button></p>";
        html += "</div>";
        html += "<script>function muteSound() { var audio = new Audio('https://example.com/alert_sound.mp3'); audio.pause(); }</script>";
        html += "</body></html>";
        server.send(200, "text/html", html);
    });

    // Create tasks for Core 0 and Core 1
    xTaskCreatePinnedToCore(core0Task, "Core0Task", 10000, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(core1Task, "Core1Task", 10000, NULL, 1, NULL, 1);

    Serial.println("Setup completed.");
}

void loop() {
    // Empty loop as tasks are handled by FreeRTOS
}

void updateBellTimesInHTML() {
    String html = "<html><body style='font-family: Arial, sans-serif;'>";
    html += "<h1>ESP32 Dashboard</h1>";

    // Case 4
    html += "<div style='margin-bottom: 20px;'>";
    html += "<h2>Case 4</h2>";
    for (const auto& time : bellTimes) {
        html += "<p><img src='https://example.com/bell_icon.png' alt='Bell Icon' style='width:16px;height:16px;'> Bel bunyi jam: " + time + "</p>";
    }
    html += "</div>";

    // Case 34
    html += "<div style='margin-bottom: 20px;'>";
    html += "<h2>Case 34</h2>";
    html += "<p>" + String(messageCount) + " inbox</p>";
    for (int i = 1; i <= messageCount; i++) {
        html += "<p><a href='/message/" + String(i) + "'>View Msg " + String(i) + "</a></p>";
    }
    html += "</div>";

    // Case 35
    html += "<div style='margin-bottom: 20px;'>";
    html += "<h2>Case 35</h2>";
    html += "<p><a href='/emergency'><button style='color: white; background: red; padding: 10px;'>Trigger Emergency</button></a></p>";
    html += "</div>";

    html += "</body></html>";
    server.send(200, "text/html", html);
}

void recordVoiceToSD() {
    File audioFile = SD.open("/ESP32_DATA/voice_record.wav", FILE_WRITE);
    if (!audioFile) {
        Serial.println("Error opening file for recording.");
        return;
    }

    // Write WAV header
    writeWAVHeader(audioFile);

    // Read and save I2S data
    int16_t buffer[512];
    size_t bytesRead = 0;
    unsigned long startMillis = millis();

    while (millis() - startMillis < 20000) {  // Record for 20 seconds
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
    Serial.println("Recording saved to SD card.");
}

void writeWAVHeader(File& file) {
    uint32_t fileSize = 36 + 1000 * 512;  // Placeholder values
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
