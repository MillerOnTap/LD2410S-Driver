#include "LD2410S.h"

/**
 * @brief Default Destructor for the LD2410S class.
 *
 */
LD2410S::~LD2410S() = default;

/**
 * @brief Initializes the LD2410S sensor with specified RX and TX pins and a baud rate.
 * @param rxPin GPIO pin number used for receiving data from the sensor.
 * @param txPin GPIO pin number used for transmitting data to the sensor.
 * @param baud The baud rate for serial communication with the LD2410S sensor.
 * @return true if initialization and configuration succeed; false for a fail
 */
bool LD2410S::begin(int rxPin, int txPin, uint32_t baud) {
  _serial.begin(baud, SERIAL_8N1, rxPin, txPin);
  resetParser();
  _begun = true;
  return true;
}

/**
 * @brief Ends the Serial communication with the LD2410S sensor.
 */
void LD2410S::end() {
  if (_begun) { _serial.end(); _begun = false; }
  resetParser();
}

/**
 * @brief Switches the LD2410S device to minimal output mode.
 * @param retries Number of times to retry the command if it fails.
 * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
 *
 * @return true if the device successfully switches to minimal output mode.
 */
bool LD2410S::switchToMinimalMode(size_t retries, uint32_t timeoutMs)
{
  
    uint8_t cmdMJR = 0x7A;
    uint8_t cmdMNR = 0x00;

    uint8_t payload[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Parameter to set standard mode

    uint8_t buf[8];
    uint16_t n = 0;

    for (size_t i = 0; i < retries; ++i) {
        sendCommand(cmdMJR, cmdMNR, payload, sizeof(payload)/ sizeof(payload[0]));
        if (waitForAckAndCopy(cmdMJR, 0x01, buf, sizeof(buf), &n, timeoutMs)) {
            break;
        }
        if (i < retries - 1) {
            LD2410S_LOG_LINE("Switch to minimal output mode failed after retries");
        }
        vTaskDelay(pdMS_TO_TICKS(250)); // Wait before retrying
    }

        if (n < 2) {
            LD2410S_LOG_LINE("Error: Incomplete response for switch to minimal mode.");
            return false;
        }

      // Check status (00 00 == OK)
    if (buf[0] != 0x00 || buf[1] != 0x00) {
        LD2410S_LOG("Error: Unexpected response for switch to minimal mode ACK: %02X %02X\n", buf[0], buf[1]);
        return false;
    }

    
    return true;
}

/**
 * @brief Switches the LD2410S device to standard output mode.
 * @param retries The number of retries before sending false
 * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
 * @return true if the device successfully switches to standard output mode.
 */
bool LD2410S::switchToStandardMode(size_t retries, uint32_t timeoutMs)
{

    uint8_t cmdMJR = 0x7A;
    uint8_t cmdMNR = 0x00;

    uint8_t payload[6] = {0x00, 0x00, 0x01, 0x00, 0x00, 0x00}; // Parameter to set standard mode

    uint8_t buf[8];
    uint16_t n = 0;

    for (size_t i = 0; i < retries; ++i) {
        sendCommand(cmdMJR, cmdMNR, payload, sizeof(payload)/ sizeof(payload[0]));
        if (waitForAckAndCopy(cmdMJR, 0x01, buf, sizeof(buf), &n, timeoutMs)) {
            break;
        }
        if (i < retries - 1) {
            LD2410S_LOG_LINE("Switch to standard output mode failed after retries");
        }
        vTaskDelay(pdMS_TO_TICKS(250)); // Wait before retrying
    }

        if (n < 2) {
            LD2410S_LOG_LINE("Error: Incomplete response switch to standard mode.");
            return false;
        }

      // Check status (00 00 == OK)
    if (buf[0] != 0x00 || buf[1] != 0x00)  {
        LD2410S_LOG("Error: Unexpected response for switch to standard mode ACK: %20X %20X", buf[0], buf[1]);
        return false;
    }

    return true;
}

/**
 * @brief Reads the firmware version from the LD2410S device.
 * This function sends a command to the LD2410S device to request its firmware version.
 * It waits for a response and parses the major, minor, and patch version numbers from the received data.
 *
 * @param[out] major Reference to a uint16_t variable where the major version will be stored.
 * @param[out] minor Reference to a uint16_t variable where the minor version will be stored.
 * @param[out] patch Reference to a uint16_t variable where the patch version will be stored.
 * @param retries Number of retries to attempt if the command fails.
 * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
 * @return true if the firmware version was successfully read and parsed, false otherwise.
 */
bool LD2410S::readFirmwareVersion(uint16_t &major, uint16_t &minor, uint16_t &patch, size_t retries, uint32_t timeoutMs)
{

    uint8_t cmdMJR = 0x00; // Command Major
    uint8_t cmdMNR = 0x00; // Command Minor

    uint8_t buf[80]; 
    uint16_t n=0;
    for (size_t i = 0; i < retries; i++) {
        sendCommand(cmdMJR, cmdMNR, nullptr, 0);
        if (waitForAckAndCopy(cmdMJR, 0x01, buf, sizeof(buf), &n, timeoutMs)) {
            break;
        }
        if (i == retries - 1) {
            LD2410S_LOG_LINE("Error: Failed to read firmware after retries.");
            return false;
        }        
        vTaskDelay(pdMS_TO_TICKS(250));
        n = 0; // Reset buffer size for next attempt
    }

    if (n < 3 * 2 + 6) {
        LD2410S_LOG_LINE("Error: Incomplete response for read firmware.");
        return false;
    }


    major = static_cast<uint16_t>(buf[6]);
    minor = static_cast<uint16_t>(buf[8]);
    patch = static_cast<uint16_t>(buf[10]);

    _firmwareStr = String(major) + "." + String(minor) + "." + String(patch);


    return true;
}

/**
 * @brief Enters the configuration mode of the LD2410S device.
 * Entering this mode is required before any other commands can be sent.
 * @param retries Number of times to retry entering configuration mode if the command fails.
 * @param timeoutMs Timeout period in milliseconds to wait for a response.
 * @return true if the device successfully enters configuration mode.
 */
bool LD2410S::enterConfigMode(size_t retries, uint32_t timeoutMs) {

  uint8_t cmdMJR = 0xFF;
  uint8_t cmdMIN = 0x00;

  uint8_t payload[2] = {0x01, 0x00}; // Payload for entering config mode (if needed)

  uint8_t buf[80];
  uint16_t n = 0;
  for (size_t i = 0; i < retries; i++) {
    sendCommand(cmdMJR, cmdMIN, payload, sizeof(payload)/sizeof(payload[0]));
    // static const uint8_t ok[2] = {0x00, 0x00};
    if (waitForAckAndCopy(cmdMJR, 0x01, buf, sizeof(buf), &n, timeoutMs)) {
        break;
    }
    if (i == retries - 1) {
      LD2410S_LOG_LINE("Error: Failed to enter config mode after retries.");
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(250));
  }   
  
    if (n < 2) {
        LD2410S_LOG_LINE("Error: Incomplete response for ente config.");
        return false;
    }

      // Check status (00 00 == OK)
    if (buf[0] != 0x00 || buf[1] != 0x00) {
        LD2410S_LOG("Error: Enter Config ACK byte not correct. %02X %02X", buf[0], buf[1]);
        return false;
    }
    
    return true;
}

/**
 * @brief Exits the configuration mode of the LD2410S device.
 * @return true if the device successfully exits configuration mode.
 */
bool LD2410S::exitConfigMode(size_t retries, uint32_t timeoutMs)
{
    uint8_t cmdMJR = 0xFE;
    uint8_t cmdMIN = 0x00;

    for (int i = 0; i < retries; i++)
    {
        sendCommand(cmdMJR, cmdMIN, nullptr, 0);
        static const uint8_t ok[2] = {0x00, 0x00};
        if (waitForAck(cmdMJR, 0x01, ok, 2, timeoutMs))
            break;
        if (i == retries - 1){
            LD2410S_LOG_LINE("Error: Failed to exit config mode after retries.");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    return true;
  
}

/**
 * @brief Write serial number character array to sensor
 * @param serialNumber Serial Number is read from this array and written to the sensor
 * @param len Length of the input serial number character array
 * @param retries Number of retries to attempt if the command fails
 * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
 */
bool LD2410S::writeSerialNumber(char* serialNumber, size_t len, size_t retries, uint32_t timeoutMs) {

    if (len != 8) {
        LD2410S_LOG_LINE("Error: Serial number must be 8 bytes.");
        return false;
    }

    uint8_t cmdMJR = 0x10; // Command Major
    uint8_t cmdMNR = 0x00; // Command Minor

    uint8_t payload[10]; // 2 + 8 bytes for length and serial number characters
    payload[0] = 0x08; // Serial Number Length
    payload[1] = 0x00; 
    for (size_t i = 0; i < 8; ++i) {
        payload[i + 2] = static_cast<uint8_t>(serialNumber[i]);
    }

    uint8_t buf[80]; 
    uint16_t n=0;
    for (size_t i = 0; i < retries; i++) {
        sendCommand(cmdMJR, cmdMNR, payload, sizeof(payload)/sizeof(payload[0]));
        if (waitForAckAndCopy(cmdMJR, 0x01, buf, sizeof(buf), &n, timeoutMs)) {
            break;
        }
        if (i == retries - 1) {
            LD2410S_LOG_LINE("Error: Failed to read serial string after retries.");
            return false;
        }        
        vTaskDelay(pdMS_TO_TICKS(250));
        n = 0; // Reset buffer size for next attempt
    }

    if (n < 2) {
        LD2410S_LOG_LINE("Error: Incomplete response for read serial.");
        return false;
    }
    if (buf[0] != 0 || buf[1] != 0) {
        LD2410S_LOG("Error: Serial number ACK byte not correct. %02X %02X\n", buf[0], buf[1]);
        return false;   
    }
    return true;
}

/**
 * @brief Read serial number character array from sensor
 * @param serialNumber Serial number is read from the sensor and written into this array
 * @param len Length of the output serial number character array
 * @param retries Number of retries to attempt if the command fails
 * @param timeoutMs Timeout in milliseconds (default = 1000 ms) 
 * */  
bool LD2410S::readSerialNumber(char* serialNumber, size_t len, size_t retries, uint32_t timeoutMs) {

    if (len < 8) {
        LD2410S_LOG_LINE("Error: Serial number buffer too small, must be at least 8 characters.");
        return false;
    }

    uint8_t cmdMJR = 0x11; // Command Major
    uint8_t cmdMNR = 0x00; // Command Minor

    uint8_t buf[80]; 
    uint16_t n=0;
    for (size_t i = 0; i < retries; i++) {
        sendCommand(cmdMJR, cmdMNR, nullptr, 0);
        if (waitForAckAndCopy(cmdMJR, 0x01, buf, sizeof(buf), &n, timeoutMs)) {
            break;
        }
        if (i == retries - 1) {
            LD2410S_LOG_LINE("Error: Failed to read serial string after retries.");
            return false;
        }        
        vTaskDelay(pdMS_TO_TICKS(250));
        n = 0; // Reset buffer size for next attempt
    }

    if (n < 12) {
        LD2410S_LOG_LINE("Error: Incomplete response for read serial.");
        return false;
    }
    if (buf[0] != 0 && buf[1] != 0) {
        LD2410S_LOG("Error: Serial number ACK byte not correct. %02X %02X", buf[0], buf[1]);
        return false;   
    }

    for (int i=0; i<8; i++)
        serialNumber[i] = static_cast<char>(buf[i+4]);

    return true;
}

/**
 * @brief Reads the serial number string from the LD2410S device.
 @param[out] out Reference to a String object where the serial string will be stored.
 * @param[in] timeoutMs Timeout period in milliseconds to wait for a response.
 * @return true if the serial string was successfully read and parsed, false otherwise.
 */
bool LD2410S::readSerialString(String &out, size_t retries, uint32_t timeoutMs) {

    uint8_t cmdMJR = 0x11; // Command Major
    uint8_t cmdMNR = 0x00; // Command Minor

    uint8_t buf[80]; 
    uint16_t n=0;
    for (size_t i = 0; i < retries; i++) {
        sendCommand(cmdMJR, cmdMNR, nullptr, 0);
        if (waitForAckAndCopy(cmdMJR, 0x01, buf, sizeof(buf), &n, timeoutMs)) {
            break;
        }
        if (i == retries - 1) {
            LD2410S_LOG_LINE("Error: Failed to read serial string after retries.");
            return false;
        }        
        vTaskDelay(pdMS_TO_TICKS(250));
        n = 0; // Reset buffer size for next attempt
    }

  String s;
  if (!decodeAsciiBlock(buf, n, s)) {
    // fallback to hex string if not in the 00 00 len ascii... format
    char tmp[4]; s.reserve(n*2+3);
    for (uint16_t i=0;i<n;++i){ snprintf(tmp,sizeof(tmp),"%02X", buf[i]); s += tmp; if (i+1<n) s += ':'; }
  }
  _serialStr = out = s;
  return true;
}

    /**
     * @brief Write configuration parameters to sensor
     * @param farthest_distance Max detection distance in Meters (1 - 12 Meters)
     * @param nearest_distance Min detection distance in Meters (0 - 12 Meters)
     * @param delay_time Delay before entering static state in seconds (10 - 120 Seconds)
     * @param freq_status Report frequency for motion/static state in reports per scond (1 - 8 Hz)
     * @param freq_distance Report frequency for distance in reports per second (1 - 8 Hz)
     * @param respond_speed Sensor response speed (normal/fast) (5 for normal, 10 for fast)
     * @param retries Number of retries for reading
     * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
     * @return true if successful
     */
bool LD2410S::writeGenericParameters(uint32_t farthest_distance, uint32_t nearest_distance, uint32_t delay_time, 
                                            uint32_t freq_status, uint32_t freq_distance, uint32_t respond_speed,
                                            size_t retries, uint32_t timeoutMs)
{
    // Convert distances to gates (1 gate = 0.75m)
    uint32_t farthest_distance_gates = round(farthest_distance / .75); 
    uint32_t nearest_distance_gates  = round(nearest_distance / .75);
    uint8_t farthest_distance_param = static_cast<uint8_t>(std::max<uint32_t>(1u, std::min<uint32_t>(farthest_distance_gates, 16u)));
    uint8_t nearest_distance_param  = static_cast<uint8_t>(std::min<uint32_t>(nearest_distance_gates, 16u));
    // Clamp and convert other parameters
    uint8_t delay_time_param        = static_cast<uint8_t>(std::max<uint32_t>(10u, std::min<uint32_t>(delay_time, 120u)));
    uint8_t freq_status_param       = static_cast<uint8_t>(std::max<uint32_t>(5u, std::min<uint32_t>(freq_status * 10, 80u)));
    uint8_t freq_distance_param     = static_cast<uint8_t>(std::max<uint32_t>(5u, std::min<uint32_t>(freq_distance * 10, 80u)));
    uint8_t respond_speed_param     = (respond_speed > 5) ? 10 : 5;
  
    uint8_t cmdMJR = 0x70; // Command Major
    uint8_t cmdMNR = 0x00; // Command Minor

    // Compose all parameter payloads in a single buffer to minimize stack usage and serial writes
    uint8_t payload[] = {
        0x05, 0x00, farthest_distance_param, 0x00, 0x00, 0x00,
        0x0A, 0x00, nearest_distance_param,  0x00, 0x00, 0x00,
        0x06, 0x00, delay_time_param,        0x00, 0x00, 0x00,
        0x02, 0x00, freq_status_param,       0x00, 0x00, 0x00,
        0x0C, 0x00, freq_distance_param,     0x00, 0x00, 0x00,
        0x0B, 0x00, respond_speed_param,     0x00, 0x00, 0x00
    };
    // Clear the serial read buffer before sending the command

    uint8_t buf[8]; 
    uint16_t n=0;
    for (size_t i = 0; i < retries; i++) {
        sendCommand(cmdMJR, cmdMNR, payload, sizeof(payload) / sizeof(payload[0]));
        if (waitForAckAndCopy(cmdMJR, 0x01, buf, sizeof(buf), &n, timeoutMs)) {
            break;
        }
        if (i == retries - 1) {
            LD2410S_LOG_LINE("Error: Failed to write generic parameters after retries.");
            return false;
        }        
        vTaskDelay(pdMS_TO_TICKS(250));
        n = 0; // Reset buffer size for next attempt
    }

    if (n < 2) {
        LD2410S_LOG_LINE("Error: Incomplete response for write generic parameters.");
        return false;
    }

      // Check status (00 00 == OK)
    if (buf[0] != 0x00 || buf[1] != 0x00) {
        LD2410S_LOG("Error: Write generic parameters ACK byte not correct. %02X %02X", buf[0], buf[1]);
        return false;
    }
    
    return true;
}

/**
 * @brief Reads the common parameters from the LD2410S sensor.
 *
 * This function sends a command to the LD2410S sensor to request its common parameters,
 * waits for a response, and parses the returned data. The parameters retrieved include
 * the farthest and nearest detection distances, delay time, frequency status, frequency
 * distance, and response speed. The function returns true if the parameters are successfully
 * read and parsed, otherwise returns false.
 *
 * @param[out] farthest_distance The farthest detection distance in meters to be detected by the sensor.
 * @param[out] nearest_distance The nearest detection distance in meters to be detected by the sensor.
 * @param[out] delay_time The delay time parameter from the sensor.
 * @param[out] freq_status The frequency status parameter from the sensor per second.
 * @param[out] freq_distance The frequency distance parameter from the sensor per second.
 * @param[out] respond_speed The response speed parameter from the sensor 5 = minimal 10 = normal.
 * @param retries Number of retries to attempt if the command fails.
 * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
 * @return true if the parameters were successfully read and parsed, false otherwise.
 */
bool LD2410S::readCommonParameters(uint32_t &farthest_distance, uint32_t &nearest_distance, 
                uint32_t &delay_time, uint32_t &freq_status, 
                uint32_t &freq_distance, uint32_t &respond_speed, 
                size_t retries, uint32_t timeoutMs)
{

    uint8_t cmdMJR = 0x71; // Command Major
    uint8_t cmdMNR = 0x00; // Command Minor

    const uint8_t payload[] = {
        0x05, 0x00,  //Farthest Distance Gate 1-16 
        0x0A, 0x00,  //Nearest Distance Gate 0-16
        0x06, 0x00,  //Unmanned Delay Time 10-120
        0x02, 0x00,  //Frequency of Status Reporting  .5 to 8Hz * 10 5 to 80
        0x0C, 0x00,  //Frequency Distance Reporting .5 to 8Hz * 10 5 to 80
        0x0B, 0x00}; //Response Speed 5 Normal / 10 Fast
  
    uint8_t buf[80]; 
    uint16_t n=0;
    for (size_t i = 0; i < retries; i++) {
        sendCommand(cmdMJR, cmdMNR, payload, sizeof(payload) / sizeof(payload[0]));
        if (waitForAckAndCopy(cmdMJR, 0x01, buf, sizeof(buf), &n, timeoutMs)) {
            break;
        }
        if (i == retries - 1) {
            LD2410S_LOG_LINE("Error: Failed to read common parameters after retries.");
            return false;
        }        
        vTaskDelay(pdMS_TO_TICKS(250));
        n = 0; // Reset buffer size for next attempt
    }

    if (n < 2 + 6 * 4) {
        LD2410S_LOG_LINE("Error: Incomplete response for read common parameters.");
        return false;
    }

    // Check status (00 00 == OK)
    if (buf[0] != 0x00 || buf[1] != 0x00) return false;

    // Extract thresholds: first byte of each 4-byte block starting at offset 2
    farthest_distance = std::round(parseLEUint32(&buf[2 + 0 * 4]) * .75f);
    nearest_distance = std::round(parseLEUint32(&buf[2 + 1 * 4]) * .75f);
    delay_time = parseLEUint32(&buf[2 + 2 * 4]);
    freq_status = std::round(parseLEUint32(&buf[2 + 3 * 4]) / 10.0f);
    freq_distance = std::round(parseLEUint32(&buf[2 + 4 * 4]) / 10.0f);
    respond_speed = parseLEUint32(&buf[2 + 5 * 4]);

    return true;
}   

/**
 * @brief Sends a command to automatically update the trigger and hold thresholds along with the scan time 
 * in seconds.
 * @param triggerThresholdFactor The trigger threshold value (0-10).
 * @param holdThresholdFactor The hold threshold value (0-10).
 * @param scanTime The scan time value in seconds (120-240).
 * @param retries Number of retries to attempt if the command fails.
 * @param timeoutMs Timeout in milliseconds to wait for a response (default = 1000 ms).
 * @return true if the command was successfully sent and acknowledged; false otherwise.
 */

bool LD2410S::autoUpdateThresholds (uint32_t triggerThresholdFactor, uint32_t holdThresholdFactor, uint32_t scanTime, size_t retries, uint32_t timeoutMs) 
{
    // Clamp and convert other parameters
    uint8_t triggerThreshold_param = static_cast<uint8_t>(std::max(0u, std::min(triggerThresholdFactor, 10u)));
    uint8_t holdThreshold_param    = static_cast<uint8_t>(std::max(0u, std::min(holdThresholdFactor, 10u)));
    uint8_t scanTime_param          = static_cast<uint8_t>(std::max(120u, std::min(scanTime, 240u)));

    // Compose all parameter payloads in a single buffer to minimize stack usage and serial writes

    uint8_t cmdMJR = 0x09; // Command Major
    uint8_t cmdMNR = 0x00; // Command Minor

    uint8_t payload[] = {
        triggerThreshold_param, 0x00,
        holdThreshold_param, 0x00,
        scanTime_param, 0x00
    };

    uint8_t buf[80];
    uint16_t n = 0;

    for (size_t i = 0; i < retries; ++i) {
        sendCommand(cmdMJR, cmdMNR, payload, sizeof(payload));
        if (waitForAckAndCopy(cmdMJR, 0x01, buf, sizeof(buf), &n, timeoutMs)) {
            break;
        } 
        if (i == retries - 1) {
            LD2410S_LOG_LINE("Error: Failed to auto update thresholds after retries.");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(250));             
      
    }
    if (n < 2) {
        LD2410S_LOG_LINE("Error: Incomplete response for read common parameters.");
        return false;
    }

    // Check status (00 00 == OK)
    if (buf[0] != 0x00 || buf[1] != 0x00) {
        LD2410S_LOG("Error: Auto update thresholds ACK byte not correct. %02X %02X\n", buf[0], buf[1]);
        return false;
    }
    return true;
}

/**
 * @brief Get the progress of the automatic update
 * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
 * @return Progress percentage (0 to 100) or UINT32_MAX if the function
 * was unable to retrieve the progress in within 1 second.  
 */
uint32_t LD2410S::autoUpdateProgress(uint32_t timeoutMs) {
  uint8_t p[4]; uint16_t n = 0;
  if (waitForData(0x03, p, sizeof(p), &n, timeoutMs) && n >= 2) {
    uint16_t pct = (uint16_t)p[0];
    if (pct > 100) pct = 100;
    return pct;
  }
  return UINT32_MAX;
}

/**
 * @brief Sets the trigger threshold for each gate using an array of parameters.
 *
 * @param gateThresholds Array of gate thresholds to set (10-95).
 * @param len Length of the gateThresholds array (must be at least 16).
 * @return true if write successful, false otherwise.
 */
bool LD2410S::writeTriggerThresholds(const uint32_t* gateThresholds, size_t len, size_t retries,uint32_t timeoutMs)
{
    if (len < 16) {
        LD2410S_LOG_LINE("Error: gateThresholds array must have at least 16 elements.");
        return false;
    }

    uint8_t cmdMJR = 0x72; // Command Major
    uint8_t cmdMNR = 0x00; // Command Minor

    uint8_t payload[6 * 16]; // 4 header + 6 bytes per gate
    for (size_t i = 0; i < 16; ++i) {
        uint8_t threshold = static_cast<uint8_t>(std::max(10u, std::min(gateThresholds[i], 95u)));
        payload[i * 6 + 0] = static_cast<uint8_t>(i);
        payload[i * 6 + 1] = 0x00;
        payload[i * 6 + 2] = threshold;
        payload[i * 6 + 3] = 0x00;
        payload[i * 6 + 4] = 0x00;
        payload[i * 6 + 5] = 0x00;
    }

    
    for (int i = 0; i < retries; i++) {
        sendCommand(cmdMJR, cmdMNR, payload, sizeof(payload) / sizeof(payload[0]));
        if (waitForAck(cmdMJR, 0x01, timeoutMs)) {
            break;
        }
        if (i == retries - 1) {
            LD2410S_LOG_LINE("Error: Failed to write trigger thresholds after retries.");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    return true;
}

/**
 * @brief Reads the trigger threshold for each gate from the LD2410 sensor and populates an array with
 * each gate value.
 *
 * @param gateThresholds Array of gate thresholds to be received (10-95).
 * @param len Length of the gateThresholds array (must be at least 16).
 * @return true if read successful, false otherwise.
 */
bool LD2410S::readTriggerThresholds(uint32_t* gateThresholds, size_t len, size_t retries, uint32_t timeoutMs)
{
    if (len < 16) {
        LD2410S_LOG_LINE("Error: gateThresholds array must have at least 16 elements.");
        return false;
    }

    uint8_t cmdMJR = 0x73; // Command Major
    uint8_t cmdMNR = 0x00; // Command Minor

    uint8_t payload[2 * 16]; // 4 header + 2 bytes per gate
    for (size_t i = 0; i < 16; ++i) {
        payload[i * 2 + 0] = static_cast<uint8_t>(i);
        payload[i * 2 + 1] = 0x00;
    }
    uint8_t buf[80]; 
    uint16_t n=0;
    for (size_t i = 0; i < retries; i++) {
        sendCommand(cmdMJR, cmdMNR, payload, sizeof(payload) / sizeof(payload[0]));
        if (waitForAckAndCopy(cmdMJR, 0x01, buf, sizeof(buf), &n, timeoutMs)) {
            break;
        }
        if (i == retries - 1) {
            LD2410S_LOG_LINE("Error: Failed to read trigger thresholds after retries.");
            return false;
        }        
        vTaskDelay(pdMS_TO_TICKS(250));
        n = 0; // Reset buffer size for next attempt
    }

    if (n < 2 + 16 * 4) {
        LD2410S_LOG_LINE("Error: Incomplete response for trigger thresholds.");
        return false;
    }

      // Check status (00 00 == OK)
    if (buf[0] != 0x00 || buf[1] != 0x00) return false;

    // // Extract thresholds: first byte of each 4-byte block starting at offset 2
    for (size_t i = 0; i < 16; ++i) {
        gateThresholds[i] = static_cast<uint32_t>(buf[i * 4 + 2]);        
    }
    return true;

}

/**
 * @brief Sets the hold threshold for each gate using an array of parameters.
 *
 * @param gateThresholds Array of gate thresholds to set (10-95).
 * @param len Length of the gateThresholds array (must be at least 16).
 * @param retries Number of retries to attempt if the command fails.
 * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
 * @return true if write successful, false otherwise.
 */
bool LD2410S::writeHoldThresholds(const uint32_t* gateThresholds, size_t len, size_t retries, uint32_t timeoutMs)
{
    if (len < 16) {
        LD2410S_LOG_LINE("Error: gateThresholds array must have at least 16 elements.");
        return false;
    }

    uint8_t cmdMJR = 0x76; // Command Major
    uint8_t cmdMNR = 0x00; // Command Minor

    uint8_t payload[6 * 16]; // 4 header + 6 bytes per gate

    for (size_t i = 0; i < 16; ++i) {
        uint8_t threshold = static_cast<uint8_t>(std::max(10u, std::min(gateThresholds[i], 95u)));
        payload[i * 6 + 0] = static_cast<uint8_t>(i);
        payload[i * 6 + 1] = 0x00;
        payload[i * 6 + 2] = threshold;
        payload[i * 6 + 3] = 0x00;
        payload[i * 6 + 4] = 0x00;
        payload[i * 6 + 5] = 0x00;
    }

    for (int i = 0; i < retries; i++) {
        sendCommand(cmdMJR, cmdMNR, payload, sizeof(payload) / sizeof(payload[0]));
        if (waitForAck(cmdMJR, 0x01, timeoutMs)) {
            break;
        }
        if (i == retries - 1) {
            LD2410S_LOG_LINE("Error: Failed to write hold thresholds after retries.");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    
    return true;
}

/**
 * @brief Reads the hold threshold for each gate from the LD2410 sensor and populates an array with
 * each gate value.
 *
 * @param gateThresholds Array of gate thresholds to received (10-95).
 * @param len Length of the gateThresholds array (must be at least 16).
 * @param retries Number of retries to attempt if the command fails.
 * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
 * @return true if read successful, false otherwise.
 */
bool LD2410S::readHoldThresholds(uint32_t* gateThresholds, size_t len, size_t retries,uint32_t timeoutMs)
{
    if (len < 16) {
        LD2410S_LOG_LINE("Error: gateThresholds array must have at least 16 elements.");
        return false;
    }

    uint8_t cmdMJR = 0x77; // Command Major
    uint8_t cmdMNR = 0x00; // Command Minor

    uint8_t payload[2 * 16]; // 4 header + 2 bytes per gate
    for (size_t i = 0; i < 16; ++i) {
        payload[i * 2 + 0] = static_cast<uint8_t>(i);
        payload[i * 2 + 1] = 0x00;
    }

    uint8_t buf[80]; 
    uint16_t n=0;
    for (size_t i = 0; i < retries; i++) {
        sendCommand(cmdMJR, cmdMNR, payload, sizeof(payload) / sizeof(payload[0]));
        if (waitForAckAndCopy(cmdMJR, 0x01, buf, sizeof(buf), &n, timeoutMs)) {
            break;
        }
        if (i == retries - 1) {
            LD2410S_LOG_LINE("Error: Failed to read hold thresholds after retries.");
            return false;
        }        
        vTaskDelay(pdMS_TO_TICKS(250));
        n = 0; // Reset buffer size for next attempt
    }

    if (n < 2 + 16 * 4) {
        LD2410S_LOG_LINE("Error: Incomplete response for hold thresholds.");
        return false;
        }

        // Check status (00 00 == OK)
        if (buf[0] != 0x00 || buf[1] != 0x00) return false;

        // Extract thresholds: first byte of each 4-byte block starting at offset 2
        for (size_t i = 0; i < 16; ++i) {
            gateThresholds[i] = (buf[i * 4 + 2]);
        }
        return true;
}


/**
 * @brief Fill Event Structure with the latest data
 * @param out Event structure to be filled
 */
bool LD2410S::getLatest(LD2410SEvent& out) const {
    out = _latest; // POD copy
    return true;
}

/**
 * @brief Checks if the latest data has been updated then clears the flag.
 * @return true if the latest data has been updated
 */
bool LD2410S::wasUpdatedAndClear() {
    bool u = _updated;
    _updated = false;
    return u;
}

/** @brief Return the latest distance in meters */
float LD2410S::latestDistanceMeters() const {
    return _latest.distance_cm / 100.0f;
}

/** @brief Return the latest distance in feet */
float LD2410S::latestDistanceFeet() const {
    return _latest.distance_cm * 0.0328084f;
}

/**
 * @brief Sends a command to the LD2410S device.
 * @param cmdMaj Major command byte.
 * @param cmdMin Minor command byte.
 * @param payload Pointer to the command payload data.
 * @param n Length of the payload data.
 */
void LD2410S::sendCommand(uint8_t cmdMaj, uint8_t cmdMin, const uint8_t *payload, uint16_t n) {
  // Stream the frame directly to UART to avoid large stack buffers and overflow
  const uint16_t len = (uint16_t)(2 + n);
//   Serial.printf("LD2410S: Sending command: ");
//   for (int i = 0; i < 4; ++i) {
//     Serial.printf("%02X ", CMD_HDR[i]);
//   }
//   Serial.printf("%02X ", (len & 0xFF));      // length LSB
//   Serial.printf("%02X ", (len >> 8));        // length MSB
//   Serial.printf("%02X ", cmdMaj);         // command major
//   Serial.printf("%02X ", cmdMin);         // command minor
//   for (int i = 0; i < n; ++i) {
//     Serial.printf("%02X ", payload ? payload[i] : 0);
//   }
//   for (int i = 0; i < 4; ++i) {
//         Serial.printf("%02X ", CMD_TAIL[i]);
//   }
//   Serial.println();
  _serial.write(CMD_HDR, 4);                 // header
  _serial.write((uint8_t)(len & 0xFF));      // length LSB
  _serial.write((uint8_t)(len >> 8));        // length MSB
  _serial.write(cmdMaj);                     // command major
  _serial.write(cmdMin);                     // command minor
  if (n && payload) {
    _serial.write(payload, n);               // payload 
  }
  _serial.write(CMD_TAIL, 4);                // tail
  _serial.flush();
}

/**
 * @brief Wait for an acknowledgment from the LD2410S device.
 * @param ackMaj Major acknowledgment byte.
 * @param ackMin Minor acknowledgment byte.
 * @param timeoutMs Timeout period in milliseconds.
 * @return true if the expected acknowledgment is received, false otherwise.
 */
bool LD2410S::waitForAck(uint8_t ackMaj, uint8_t ackMin, uint32_t timeoutMs) {
  const uint32_t t0 = millis();
  while ((millis() - t0) < timeoutMs) {
    while (_serial.available()) {
    uint8_t b = (uint8_t)_serial.read();
      feed(b);      // keep command ACKs flowing
      feedData(b);  // also parse data frames in the background 
      if (_frameReady) {
        _frameReady = false;
        if (_lastCmdMaj == ackMaj && _lastCmdMin == ackMin) return true; // payload ignored
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return false;
}

/**
 * @brief Wait for an acknowledgment from the LD2410S device and copy the payload.
 * @param ackMaj Major acknowledgment byte.
 * @param ackMin Minor acknowledgment byte.
 * @param out Pointer to the buffer where the payload will be copied.
 * @param outCap Capacity of the output buffer.
 * @param outLen Pointer to a variable where the length of the copied payload will be stored.
 * @param timeoutMs Timeout period in milliseconds.
 * @return true if the expected acknowledgment is received and the payload is copied, false otherwise.
 */
bool LD2410S::waitForAckAndCopy(uint8_t ackMaj, uint8_t ackMin,
                                uint8_t* out, uint16_t outCap, uint16_t* outLen,
                                uint32_t timeoutMs){
  const uint32_t t0 = millis();
  while ((millis() - t0) < timeoutMs) {
    while (_serial.available()) {
      uint8_t b = (uint8_t)_serial.read();
      feed(b);      // keep command ACKs flowing
      feedData(b);  // also parse data frames in the background 
      if (_frameReady) {        
        _frameReady = false;        
        if (_lastCmdMaj == ackMaj && _lastCmdMin == ackMin) {
          uint16_t n = _lastPayloadLen; 
          if (n > outCap) n = outCap;
          if (outCap < n) {
            LD2410S_LOG("Warning: Output buffer will be truncated (%d bytes needed, %d bytes available).\n", n, outCap);
          }
          if (out && n) memcpy(out, _lastPayload, n);
          if (outLen) *outLen = n;
          return true;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return false;
}

/**
 * @brief Wait for an acknowledgment from the LD2410S device.
 * @param ackMaj Major acknowledgment byte.
 * @param ackMin Minor acknowledgment byte.
 * @param expPayload Expected payload data.
 * @param expLen Length of the expected payload.
 * @param timeoutMs Timeout period in milliseconds.
 * @return true if the expected acknowledgment is received, false otherwise.
 */
bool LD2410S::waitForAck(uint8_t ackMaj, uint8_t ackMin,
                        const uint8_t *expPayload, uint16_t expLen,
                        uint32_t timeoutMs) {
  const uint32_t t0 = millis();
  while ((millis() - t0) < timeoutMs) {
    while (_serial.available()) {
      uint8_t b = (uint8_t)_serial.read();
      feed(b);      // keep command ACKs flowing
      feedData(b);  // also parse data frames in the background 
      if (_frameReady) {
        _frameReady = false;
        if (_lastCmdMaj == ackMaj && _lastCmdMin == ackMin) {
          if (!expPayload) return true; // accept any payload
          if (expLen == _lastPayloadLen &&
              (expLen == 0 || memcmp(expPayload, _lastPayload, expLen) == 0)) {
            return true; // exact-match success
          }
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return false;
}

/**
 * @brief 
 */
bool LD2410S::waitForData(uint8_t type, uint8_t* out, uint16_t outCap, uint16_t* outLen,
                          uint32_t timeoutMs) {
  const uint32_t t0 = millis();
  if (outLen) *outLen = 0;
  while ((millis() - t0) < timeoutMs) {
    while (_serial.available()) {
      uint8_t b = (uint8_t)_serial.read();
      feed(b);      // don't starve command parser
      feedData(b);  // parse data/progress frames
      if (_dataReady) {
        _dataReady = false;
        if (_dataLen >= 1 && _dataBody[0] == type) {
          uint16_t n = (_dataLen - 1);
          if (out && outCap) { if (n > outCap) n = outCap; memcpy(out, _dataBody + 1, n); }
          if (outLen) *outLen = n;
          return true;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return false;
}

/**
 * @brief Get the last received frame information.
 * @param cmdMaj Major command byte.
 * @param cmdMin Minor command byte.
 * @param payload Pointer to the payload data.
 * @param len Length of the payload data.
 * @return true if the last frame is valid, false otherwise.
 */
bool LD2410S::lastFrame(uint8_t &cmdMaj, uint8_t &cmdMin, const uint8_t *&payload, uint16_t &len) const {
  if (!_lastValid) return false;
  cmdMaj = _lastCmdMaj; cmdMin = _lastCmdMin; payload = _lastPayload; len = _lastPayloadLen; return true;
}

/**
 * @brief Copy the last received payload data.
 * @param out Pointer to the output buffer.
 * @param outCap Capacity of the output buffer.
 * @param outLen Pointer to a variable where the length of the copied payload will be stored.
 * @return true if the payload is copied successfully, false otherwise.
 */
bool LD2410S::copyLastPayload(uint8_t* out, uint16_t outCap, uint16_t* outLen) const {
  if (!out) return false;  
  if (!_lastValid) return false;
  uint16_t n = _lastPayloadLen; if (n > outCap) n = outCap;
  if (out && n) memcpy(out, _lastPayload, n);
  if (outLen) *outLen = n;
  return true;
}
/**
 * @brief Reset the parser state.
 *
 * This function resets the internal state of the LD2410S parser, clearing all indices,
 * flags, and buffers to prepare for a new frame of data.
 */
void LD2410S::resetParser() {
  _st = State::FIND_HDR; _hdrIdx = 0; _tailIdx = 0; _need = 0; _bi = 0;
  _frameReady = false; _lastValid = false; _lastPayloadLen = 0;
}

/**
 * @brief Push the current frame into the parser.
 *
 * This function processes the current frame data and updates the internal state
 * of the LD2410S parser with the received command and payload information.
 */
void LD2410S::pushFrame() {
  if (_need >= 2) {
    _lastCmdMaj = _body[0];
    _lastCmdMin = _body[1];
    _lastPayloadLen = (uint16_t)(_need - 2);
    if (_lastPayloadLen > sizeof(_lastPayload)) _lastPayloadLen = sizeof(_lastPayload);
    memcpy(_lastPayload, _body + 2, _lastPayloadLen);
    _frameReady = true;
    _lastValid  = true;
  }
}

/**
 * @brief Feed a byte into the parser.
 * @param b The byte to feed.
 */
void LD2410S::feed(uint8_t b) {
  switch (_st) {
    case State::FIND_HDR:
      if (b == CMD_HDR[_hdrIdx]) { if (++_hdrIdx == 4) { _hdrIdx = 0; _st = State::LEN0; } }
      else { _hdrIdx = (b == CMD_HDR[0]) ? 1 : 0; }
      break;

    case State::LEN0: _need = b; _st = State::LEN1; break;
    case State::LEN1: _need |= (uint16_t)b << 8; _bi = 0; _st = State::BODY; break;

    case State::BODY:
        if (_bi < sizeof(_body)) _body[_bi] = b;   // store while we have room
        ++_bi;                                      // ALWAYS count the byte
        if (_bi >= _need) { _tailIdx = 0; _st = State::TAIL; }
        break;

    case State::TAIL:
      if (b == CMD_TAIL[_tailIdx]) {
        if (++_tailIdx == 4) { pushFrame(); _st = State::FIND_HDR; _hdrIdx = 0; _tailIdx = 0; _need = 0; _bi = 0; }
      } else {
        _st = State::FIND_HDR; _hdrIdx = (b == CMD_HDR[0]) ? 1 : 0; _tailIdx = 0; _need = 0; _bi = 0;
      }
      break;
  }
}

/**
 * @brief Feed a byte into the data parser.
 * @param b The byte to feed.
 */
void LD2410S::feedData(uint8_t b) {
  switch (_dst) {
    case DState::D_FIND_HDR:
      if (b == DATA_HDR[_dhdrIdx]) { if (++_dhdrIdx == 4) { _dhdrIdx = 0; _dst = DState::D_LEN0; } }
      else { _dhdrIdx = (b == DATA_HDR[0]) ? 1 : 0; }
      break;

    case DState::D_LEN0: _dneed = b; _dst = DState::D_LEN1; break;
    case DState::D_LEN1: _dneed |= (uint16_t)b << 8; _dbi = 0; _dst = DState::D_BODY; break;

    case DState::D_BODY:
        if (_dbi < sizeof(_dbody)) _dbody[_dbi] = b;   // store while we have room
        ++_dbi;                                      // ALWAYS count the byte
        if (_dbi >= _dneed) { _dtailIdx = 0; _dst = DState::D_TAIL; }
        break;

    case DState::D_TAIL:
      if (b == DATA_TAIL[_dtailIdx]) {
        if (++_dtailIdx == 4) {
          _dataLen = (_dneed > sizeof(_dataBody)) ? sizeof(_dataBody) : _dneed;
          memcpy(_dataBody, _dbody, _dataLen);
          _dataReady = true;
          _dst = DState::D_FIND_HDR; _dhdrIdx = 0; _dtailIdx = 0; _dneed = 0; _dbi = 0;
        }
      } else {
        _dst = DState::D_FIND_HDR; _dhdrIdx = (b == DATA_HDR[0]) ? 1 : 0; _dtailIdx = 0; _dneed = 0; _dbi = 0;
      }
      break;
  }
}

/**
 * @brief Main loop to process incoming data
 * @param max_bytes_per_call Maximum number of bytes to read per call (default = 128)
 */
void LD2410S::loop(size_t max_bytes_per_call) {
    size_t consumed = 0;
    const uint32_t now = millis();

    // Timeout: if we're mid-frame and it went quiet, reset
    if (_inFrame && (now - _lastByteMs) > FRAME_IDLE_TIMEOUT_MS) {
        _inFrame = false;
        _pos = 0;
    }

    while (_serial.available() && consumed < max_bytes_per_call) {
        uint8_t b = _serial.read();
        consumed++;
        _lastByteMs = now;

        if (!_inFrame) {
            if (b == HDR) {
                _inFrame = true;
                _pos = 0;
                _buf[_pos++] = b;
            }
            // else keep scanning for header
        } else {
            // In frame
            if (_pos < MAX_FRAME) {
                _buf[_pos++] = b;
            } else {
                // overflow: resync by treating this byte as potential new header
                _inFrame = (b == HDR);
                _pos = _inFrame ? ( _buf[0] = b, 1 ) : 0;
                continue;
            }
            if (b == FTR || _pos >= MAX_FRAME) {
                LD2410S_LOG_LINE("Minimal frame:");
                dumpFrame(_buf, _pos);

                LD2410SEvent evt;
                if (parseMinimalFrame(_buf, _pos, evt)) {
                    _latest = evt;
                    _updated = true;
                    if (_cb) _cb(evt);
                }
                // Resync: if last byte looked like header, keep it as start
                bool carryHeader = (_buf[_pos-1] == HDR);
                _inFrame = carryHeader;
                _pos = carryHeader ? ( _buf[0] = HDR, 1 ) : 0;
            }
        }
    }
}

/**
 * @brief Parser used for parsing the minimal frame format from the LD2410S
 * @param frame Pointer to the frame data.
 * @param len Length of the frame data.
 * @param out Reference to the output event structure.
 */
bool LD2410S::parseMinimalFrame(const uint8_t* frame, size_t len, LD2410SEvent& out) {
    if (len < MIN_FRAME) return false;
    if (frame[0] != HDR)  return false;
    if (frame[len-1] != FTR) return false;

    uint8_t target = frame[1];
    bool motion;
    if      (target == 0 || target == 1) motion = false;
    else if (target == 2 || target == 3) motion = true;
    else return false;

    uint16_t dist = (uint16_t(frame[3]) << 8) | frame[2];

    out.motion = motion;
    out.distance_cm = dist;
    out.ts_ms = millis();
    return true;
}

/** @brief Dump raw frame data for debugging */
void LD2410S::dumpFrame(const uint8_t* p, size_t n) {
#if LD2410S_ENABLE_LOG
    const size_t maxShow = n < 32 ? n : 32; // clamp
    for (size_t i = 0; i < maxShow; ++i) {
        LD2410S_LOG("%02X ", p[i]);
    }
    if (n > maxShow) LD2410S_LOG("... (%u bytes)", (unsigned)n);
    LD2410S_LOG_LINE("");
#endif
}

/**
 * @brief Print the last received payload in a human-readable format.
 * @param out Output stream to print to
 */
void LD2410S::debugPrintLastPayload(Stream &out) const {
  if (!_lastValid) {
    out.println("[LD2410S] No frame captured yet.");
    return;
  }
  out.printf("[LD2410S] Last frame cmd=%02X %02X, payloadLen=%u", _lastCmdMaj, _lastCmdMin, _lastPayloadLen);

  // Hex dump
  out.print("  Hex:   ");
  for (uint16_t i = 0; i < _lastPayloadLen; ++i) {
    char buf[4]; snprintf(buf, sizeof(buf), "%02X", _lastPayload[i]);
    out.print(buf); if (i + 1 < _lastPayloadLen) out.print(' ');
  }
  out.println();

  // ASCII preview
  out.print("  ASCII: ");
  for (uint16_t i = 0; i < _lastPayloadLen; ++i) {
    char c = (char)_lastPayload[i];
    out.print((c >= 32 && c <= 126) ? c : '.');
  }
  out.println();
}

/**
 * @brief Decode an ASCII block from the payload.
 * Format commonly observed: 00 00 <lenLE> <ascii bytes>
 * @param p Pointer to the payload data.
 * @param len Length of the payload data.
 * @param out Output string to store the decoded ASCII characters.
 * @return true if the block is decoded successfully, false otherwise.
 */
bool LD2410S::decodeAsciiBlock(const uint8_t* p, uint16_t len, String &out){
  if (len >= 4 && p[0]==0x00 && p[1]==0x00){
    const uint16_t n = (uint16_t)p[2] | ((uint16_t)p[3] << 8);
    if (4u + n <= len){
      out.reserve(n);
      out = "";
      for (uint16_t i=0;i<n;++i){ char c=(char)p[4+i]; out += ((c>=32&&c<=126)?c:'.'); }
      return true;
    }
  }
  return false;
}

/**
 * @brief Decode a version 4B block from the payload.
 * If payload is exactly 4 bytes, print as X.Y.Z (build)
 * @param p Pointer to the payload data.
 * @param len Length of the payload data.
 * @param out Output string to store the decoded version information.
 * @return true if the block is decoded successfully, false otherwise.
 */
bool LD2410S::decodeVersion4B(const uint8_t* p, uint16_t len, String &out){
  if (len == 4){
    char tmp[32];
    snprintf(tmp, sizeof(tmp), "%u.%u.%u (%u)", (unsigned)p[0], (unsigned)p[1], (unsigned)p[2], (unsigned)p[3]);
    out = tmp; return true;
  }
  return false;
}

/**
 * @brief Parses a 32-bit unsigned integer from a little-endian byte array.
 *
 * This function takes a pointer to a 4-byte array and interprets the bytes as a
 * 32-bit unsigned integer in little-endian order (least significant byte first).
 *
 * @param data Pointer to an array of at least 4 bytes containing the little-endian data.
 * @return The parsed 32-bit unsigned integer.
 */
uint32_t LD2410S::parseLEUint32(const uint8_t *data)
{
    return static_cast<uint32_t>(data[0]) |
          (static_cast<uint32_t>(data[1]) << 8) |
          (static_cast<uint32_t>(data[2]) << 16) |
          (static_cast<uint32_t>(data[3]) << 24);
}