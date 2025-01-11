# Smart Doorbell Project Documentation

## Universitas Indonesia
**Final Project Report: Embedded Systems 02-01**

**Author:** Matthew Tristan Hutapea  
**Department:** Faculty of Engineering, Electrical Engineering Program  
**Location:** Depok, West Java 16424  
**Date:** January 2025

---

## Abstract
This project focuses on developing a smart doorbell system based on ESP32, equipped with time logging, audio recording, and remote notification features. Utilizing IoT technology, it enhances home security and convenience. Through a web interface, users can access time logs and audio recordings, while ESP-NOW communication supports remote notifications without reliance on external Wi-Fi networks. Testing confirmed the system's functionality, with real-time operation and practical applications such as analyzing courier arrival data.

---

## Introduction

### Background
Home security is a critical aspect of modern life. Advances in IoT technology enable innovative solutions to enhance security and comfort. Traditional doorbells only provide local notifications via sound, but modern requirements include time logging, remote notifications, and audio recording to improve security and convenience. This project aims to develop an ESP32-based smart doorbell with advanced features, including a web interface, ESP-NOW communication, and a state machine-based system.

### Objectives
1. Develop a smart doorbell system capable of logging time and recording audio.
2. Implement ESP32 communication using ESP-NOW protocol.
3. Create a web-based interface for easy access to time logs and audio recordings.
4. Apply state machine methodology for structured and efficient system management.

### Benefits
- Allows monitoring guest arrivals even when away from home.
- Provides an energy-efficient wireless communication solution.
- Facilitates data access through a user-friendly web interface.
- Enhances home security with audio recording for documentation and monitoring.

---

## Methodology

### System Design
Key components:
1. **ESP32 DevKit V1**: Microcontroller managing the system.
2. **Push Button**: Triggers time logging and audio recording.
3. **INMP441 Microphone**: Captures high-quality guest audio.
4. **SD Card Module**: Stores audio recordings in WAV format.
5. **Power Supply**: Powers all system components.

### Implementation Steps
1. **System Initialization**: Programming ESP32 for input/output management and initializing ESP-NOW communication.
2. **State Machine Development**:
   - **Idle State**: Waits for button press.
   - **Logging State**: Records timestamp.
   - **Recording State**: Records 20 seconds of audio.
   - **Web State**: Displays logs and audio via web interface.
3. **System Testing**:
   - Module testing to verify individual component functionality.
   - Integration testing to ensure cohesive system operation.

---

## Results and Analysis

### Testing Outcomes
- **Time Logging**: Accurate timestamp recording for each button press.
- **Audio Recording**: Successfully stored in WAV format with good quality, though slightly affected by noisy environments.
- **Remote Notifications**: Notifications delivered via ESP-NOW with sub-1-second latency.
- **Web Interface**: Operated smoothly across browsers, providing real-time log and audio access with intuitive usability.

### Courier Arrival Analysis
Courier arrivals were highest from 10:00–12:00 (20 arrivals), remained significant from 13:00–15:00 (18 arrivals), and tapered off by 15:00–17:00 (12 arrivals). This data suggests peak activity occurs late morning to early afternoon, useful for optimizing logistics resources.

---

## Discussion

### Successes
- Stable and efficient system operation.
- Optimal performance of core features (time logging, audio recording, remote notifications, and web interface).
- User-friendly web interface.

### Challenges
- Audio quality was suboptimal in noisy environments.
- Initial ESP-NOW device synchronization caused slight delays.

### Recommendations
1. **Improve Audio Quality**: Replace INMP441 microphone with a higher-sensitivity model.
2. **Enhance Web Security**: Implement authentication for access control.
3. **Optimize ESP-NOW**: Develop faster synchronization algorithms.
4. **Introduce Emergency State Responsiveness**: Improve system readiness for critical events.

---

## Conclusion
The ESP32-based smart doorbell successfully incorporates features like a web interface, ESP-NOW communication, and state machine management, offering practical and energy-efficient home security. Features such as time logging, audio recording, and remote notifications performed effectively, allowing users to monitor guests locally and remotely.

### Future Improvements
1. **Hardware Upgrades**:
   - Use a higher-sensitivity microphone.
   - Adopt faster SD card modules.
2. **Smartphone Integration**:
   - Enable real-time notifications and log access via mobile apps.
3. **Advanced Security**:
   - Add facial recognition for guest identification.
   - Strengthen web interface authentication.

---

## References
- Espressif Systems. (2023). ESP32 Documentation. Retrieved from [Espressif](https://www.espressif.com/en/products/socs/esp32)
- Adafruit. (2023). INMP441 Usage Guide. Retrieved from [Adafruit Learn](https://learn.adafruit.com/inmp441-i2s-microphone)
- Espressif Systems. (2023). ESP-NOW Communication Guide. Retrieved from [Espressif](https://www.espressif.com/en)

