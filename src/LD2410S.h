/** Original Author
 * @file LD2410S.h
 * @brief Driver library for LD2410S radar sensor (motion & distance)
 * @author PhuongNam720
 * @note License and original source can be found at https://github.com/phuongnamzz/HLK-LD2410S/
 * @version 1.0
 * @date 2025-07-03
 */
/** 
 * @file LD2410S.h
 * @brief Library for LD2410S radar sensor, Modified from the original to add 
 * support for the remaining documented features of the LD2410S. Added both Command and
 * Data Parsers.  Reworked most of the existing functions to utilize a
 * sendCommand() / waitForAck() architecture.
 *
 * @author MillerOnTap
 * @version 2.0
 * @date 2025-08-18
 */

#pragma once
#include <Arduino.h>
#include <atomic>

/**
 * @def LD2410S_ENABLE_LOG
 * @brief Enable or disable Serial debug logging (1 = enabled, 0 = disabled)
 */
#define LD2410S_ENABLE_LOG 0

#if LD2410S_ENABLE_LOG
#define LD2410S_LOG(...) Serial.printf(__VA_ARGS__)
#define LD2410S_LOG_LINE(x) Serial.println(x)
#else
#define LD2410S_LOG(...)
#define LD2410S_LOG_LINE(x)
#endif

namespace {
  static const uint8_t CMD_HDR[4]  = {0xFD, 0xFC, 0xFB, 0xFA};
  static const uint8_t CMD_TAIL[4] = {0x04, 0x03, 0x02, 0x01};
  static const uint8_t DATA_HDR[4]  = {0xF4, 0xF3, 0xF2, 0xF1};
  static const uint8_t DATA_TAIL[4] = {0xF8, 0xF7, 0xF6, 0xF5};
}





/**
 * @class LD2410S
 * @brief Class to communicate and configure the LD2410S radar sensor.
 */
class LD2410S
{
public:

    /**
     * @brief Constructor
     * @param port Reference to HardwareSerial object
     */
    explicit LD2410S(HardwareSerial &port = Serial1) : _serial(port) {}

    /**
     * @brief Destructor
     */
    ~LD2410S();

    struct MinimalData {
      uint8_t  target_state = 0;
      uint16_t distance_cm = 0;
      uint32_t seq = 0;  
      uint64_t ts_ms = 0;       
      bool motion = false;
    };

    struct ProgressData {
      uint8_t  percent = 0;      
      uint32_t seq = 0;
      uint64_t ts_ms = 0;
    };

    struct StandardData {
      uint8_t  target_state = 0;
      uint16_t distance_cm = 0;
      uint16_t reserved = 0;
      uint32_t energy[16] = {0}; 

      // Derived values, not directly reported from sensor
      uint32_t noise[16] = {0}; // EMA in (raw units)
      int16_t snr_db_q8[16] = {0}; //SNR in db * 256 (fixed-point)
      uint32_t seq = 0; //Updated every new frame
      uint64_t ts_ms = 0; // Timestamp in milliseconds
      bool motion = false; // True if motion detected
    };

    /**
     * @brief a Structure for the Auto configuration parameters
     */
    struct AutoCfgParams {
        uint16_t trigger = 2;
        uint16_t retention = 1;
        uint16_t scanTime = 0x0078;
    };


    /*
     * @brief Set default Auto Configuration parameters
     * @param p Configuration parameters to set
     */
    void setDefaults(const AutoCfgParams &p){ _defaults = p; }

    /**
     * @brief Get default Auto Configuration parameters
     * @return Default configuration parameters
     */
    AutoCfgParams defaults() const { return _defaults; }

    enum class Op : uint8_t { 
        EnterConfig, 
        ReadFirmware, 
        ReadSerial, 
        StartAutoThreshold 
    };
    
    /**
     * @brief Initialize serial communication and enter config mode
     * @param rxPin GPIO used for sensor TX
     * @param txPin GPIO used for sensor RX
     * @param baud Serial baudrate (default = 115200)
     * @return true if successful
     */
    bool begin(int rxPin, int txPin, uint32_t baud = 115200);

    /**
     * @brief End serial communication
     */
    void end();

    /**
     * @brief Switch to standard output mode (detailed)
     * @param retries number of times to retry before failing
     * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
     * @return true if success     
     */     
    bool switchToStandardMode(size_t retries = 4, uint32_t timeoutMs = 1000);

    /**
     * @brief Switch to minimal output mode (shorter packets)
     * @param retries number of times to retry before failing
     * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
     * @return true if success
     */
    bool switchToMinimalMode(size_t retries = 4, uint32_t timeoutMs = 1000);

    /**
     * @brief Read firmware version of the sensor
     * @param major Major version
     * @param minor Minor version
     * @param patch Patch version
     * @param retries Number of retries to attempt if the command fails
     * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
     * @return true if response received
     */        
    bool readFirmwareVersion(uint16_t &major, uint16_t &minor, uint16_t &patch
                            ,size_t retries = 4, uint32_t timeoutMs = 1000);

    /**
     * @brief Enter configuration mode
     * @return true if success
     * 
     * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
     */
    bool enterConfigMode(size_t retries = 4, uint32_t timeoutMs = 1000);

    /**
     * @brief Exit configuration mode
     * @return true if success
     * 
     * @param retries number of times to retry before failing
     * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
     */
    bool exitConfigMode(size_t retries = 4, uint32_t timeoutMs = 1000);

    /**
     * @brief Read serial number string from sensor
     * @param out Output string to store the serial string
     * @param retries Number of retries to attempt if the command fails
     * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
     * @return true if response received
     */
    bool readSerialString(String &out, size_t retries, uint32_t timeoutMs = 1000);

    /**
     * @brief Write serial number character array to sensor
     * @param serialNumber Serial Number is read from this array and written to the sensor
     * @param len Length of the input serial number character array
     * @param retries Number of retries to attempt if the command fails
     * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
     */
    bool writeSerialNumber(char* serialNumber, size_t len
                            ,size_t retries = 4, uint32_t timeoutMs = 1000);
    
    /**
     * @brief Read serial number character array from sensor
     * @param serialNumber Serial number is read from the sensor and written into this array
     * @param len Length of the output serial number character array
     * @param retries Number of retries to attempt if the command fails
     * @param timeoutMs Timeout in milliseconds (default = 1000 ms) 
     * */                        
    bool readSerialNumber(char* serialNumber, size_t len
                            ,size_t retries = 4, uint32_t timeoutMs = 1000);

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
    bool writeGenericParameters(uint32_t farthest_distance = 4, uint32_t nearest_distance = 0, 
                                        uint32_t delay_time = 10, uint32_t freq_status = 4, 
                                        uint32_t freq_distance = 4, uint32_t respond_speed = 10, 
                                        size_t retries = 4, uint32_t timeoutMs = 1000);    
    
    /**
     * @brief Read configuration parameters from sensor
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
    bool readCommonParameters(uint32_t &farthest_distance, uint32_t &nearest_distance, 
                                        uint32_t &delay_time, uint32_t &freq_status, 
                                        uint32_t &freq_distance, uint32_t &respond_speed, 
                                        size_t retries = 4, uint32_t timeoutMs = 1000);                                    
    
    /**
     * @brief Run the automatic update threshold parameters command
     * @param triggerThresholdFactor Factor for trigger threshold (0 to 10)
     * @param holdThresholdFactor Factor for hold threshold (0 to 10)
     * @param scanTime Scan time in seconds (120 to 240)
     * @param retries Number of retries to attempt if the command fails
     * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
     * @return true if successful
     */
    bool autoUpdateThresholds(uint32_t triggerThresholdFactor = 2, 
                                        uint32_t holdThresholdFactor = 1, 
                                        uint32_t scanTime = 120, size_t retries = 4, 
                                        uint32_t timeoutMs = 1000);

    /**
     * @brief Get the progress of the automatic update
     * @note It is my understanding that you must exit config mode before
     * the progress updates will be sent, so if this is not working try
     * and exit config mode then call this function periodically.
     * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
     * @return Progress percentage (0 to 100) or UINT32_MAX if the function
     * was unable to retrieve the progress in within 1 second.  You may have to check
     * this function several times before deciding it has failed.
     */
    uint32_t autoUpdateProgress(uint32_t timeoutMs = 1000);                                    
    
     /**
     * @brief Write trigger threshold parameters to sensor
     * @param gateThresholds Array of gate threshold values
     * @param len Length of the array (should be 16)
     * @param retries Number of retries for reading
     * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
     * @return true if successful
     */
    bool writeTriggerThresholds(const uint32_t* gateThresholds, size_t len, 
                                        size_t retries = 4, uint32_t timeoutMs = 1000);

    /**
     * @brief Read trigger threshold parameters from sensor
     * @param gateThresholds Array to store gate threshold values
     * @param len Length of the array (should be 16)
     * @param retries Number of retries for reading
     * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
     * @return true if successful
     */
    bool readTriggerThresholds(uint32_t* gateThresholds, size_t len, 
                                        size_t retries = 4, uint32_t timeoutMs = 1000);
    
    /**
     * @brief Write hold threshold parameters to sensor
     * @param gateThresholds Array of hold threshold values
     * @param len Length of the array (should be 16)
     * @param retries Number of retries for writing
     * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
     * @return true if successful
     */
    bool writeHoldThresholds(const uint32_t* gateThresholds, size_t len, 
                                    size_t retries = 4, uint32_t timeoutMs = 1000);
    /**
     * @brief Read hold threshold parameters from sensor
     * @param gateThresholds Array to store hold threshold values
     * @param len Length of the array (should be 16)
     * @param retries Number of retries for reading
     * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
     * @return true if successful
     */
    bool readHoldThresholds(uint32_t* gateThresholds, size_t len, 
                                    size_t retries = 4, uint32_t timeoutMs = 1000);                                   
     
    
    /** @brief Get the cached firmware version string
     * @return Cached firmware version string
     */
    const String& firmwareString() const { return _firmwareStr; }

    /**
     * @brief Get the cached serial number string
     * @return Cached serial number string
     */  
    const String& serialString()   const { return _serialStr;   }

    /** @brief Get the latest distance in meters */
    float latestDistanceMeters() const;

    /** @brief Get the latest distance in feet */
    float latestDistanceFeet()   const;

      /** @brief Return the latest timestamp in milliseconds */
    uint32_t latestTimestamp() const;

        /** @brief Return the latest distance in centimeters */
    uint32_t latestDistanceCm() const;

    /** @brief Return the latest motion detection status */
    bool latestMotionDetected() const;

    /** @brief Return the latest target state */
    uint8_t latestTargetState() const;

    /** @brief Get the latest minimal data motion information
     * @param motion Reference to a boolean variable to store the motion status
     * @param dist Reference to a uint16_t variable to store the distance
     * @param seqOut Pointer to a uint32_t variable to store the sequence number
     * @param tsOut Pointer to a uint64_t variable to store the timestamp
     */
    bool getMinimalData(bool& motion, uint8_t& target_state, uint16_t& dist, uint32_t* seqOut, uint64_t* tsOut) const;

    /**
     * @brief Get the latest minimal data
     * @param out Reference to a MinimalData structure to store the data
     * @return true if successful
     */
    bool getMinimalData(MinimalData& out) const {
      for (int tries = 0; tries < 2; ++tries) {
        uint32_t a = _minSeq.load(std::memory_order_acquire);
        if (a & 1) continue;                // writer in progress
        out = _minData;                          
        std::atomic_thread_fence(std::memory_order_acquire);
        uint32_t b = _minSeq.load(std::memory_order_acquire);
        if (a == b) return true;            
      }
      return false;                          
    }

    /**
     * @brief Get the latest standard data
     * @param motion Reference to a boolean variable to store the motion status
     * @param dist Reference to a uint16_t variable to store the distance
     * @param reserved Reference to a uint16_t variable to store the reserved value
     * @param energy Array to store the 16 measure gate energy levels
     * @param seqOut Pointer to a uint32_t variable to store the sequence number
     * @param tsOut Pointer to a uint64_t variable to store the timestamp
     * @param noise Array to store the 16 measure gate noise levels
     * @param snr Array to store the 16 measure gate SNR values
     * @return true if successful
     */
    bool getStandardData(bool& motion, uint8_t& target_state, uint16_t& dist,
                                  uint16_t& reserved, uint32_t energy[16],
                                  uint32_t* seqOut, uint64_t* tsOut, uint32_t noise[16], uint16_t snr[16]) const;

    /**
     * @brief Get the latest standard data
     * @param out Reference to a StandardData structure to store the data
     * @return true if successful
     */                                  
    bool getStandardData(StandardData& out) const {
      for (int tries = 0; tries < 2; ++tries) {
        uint32_t a = _stdSeq.load(std::memory_order_acquire);
        if (a & 1) continue;                
        out = _stdData;                          
        std::atomic_thread_fence(std::memory_order_acquire);
        uint32_t b = _stdSeq.load(std::memory_order_acquire);
        if (a == b) return true;
      }
      return false;                          
  }

    /**
     * @brief Get the latest progress data
     * @param pct Reference to a uint8_t variable to store the progress percentage
     * @param seqOut Pointer to a uint32_t variable to store the sequence number
     * @param tsOut Pointer to a uint64_t variable to store the timestamp
     * @return true if successful
     */
    bool getProgressData(uint8_t& pct, uint32_t* seqOut, uint64_t* tsOut) const;

    /**
     * @brief Reset the noise data
     * @note This will zero out the noise data and re-prime it
     */
    void resetNoise() {
      memset(_stdData.noise, 0, sizeof(_stdData.noise));
      _noisePrimed = false;
    }

    /**
     * @brief Set the noise EMA alpha value
     * @param a_q8 Alpha value in Q8 format (default ~16 (≈0.0625))
     */
    void setNoiseEmaAlphaQ8(uint8_t a_q8 = 16) {
      _noise_alpha_q8 = a_q8;
    }

    /**
     * @brief Set this to true if you want noise also updated during motion
     * otherwise it will only be updated when no motion is detected
     * @param enable True to enable updates on motion, false to disable
     */
    void setNoiseUpdateOnMotion(bool enable = false) {
        _noise_update_on_motion = enable;
    }

    /**
     * @brief Get the last frame information
     * @param cmdMaj Major command byte
     * @param cmdMin Minor command byte
     * @param payload Pointer to the payload data
     * @param len Length of the payload data
     * @return true if successful
     */
     bool lastFrame(uint8_t &cmdMaj, uint8_t &cmdMin,  
                    const uint8_t *&payload, uint16_t &len) const;

    /**
     * @brief Copy the last payload data to an output buffer
     * @param out Pointer to the output buffer
     * @param outCap Capacity of the output buffer
     * @param outLen Length of the copied data
     * @return true if successful
     */
     bool copyLastPayload(uint8_t* out, uint16_t outCap, uint16_t* outLen = nullptr) const;

    /*
    * @brief Send a command direct to the sensor
    * @param cmdMaj Major command byte
    * @param cmdMin Minor command byte
    * @param payload Pointer to the payload data
    * @param n Length of the payload data
    */
    void sendCommand(uint8_t cmdMaj, uint8_t cmdMin, 
                        const uint8_t *payload = nullptr, uint16_t n = 0);

    /**
     * @brief Wait for an acknowledgment from the sensor
     * @param ackMaj Major acknowledgment byte
     * @param ackMin Minor acknowledgment byte
     * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
     * @return true if acknowledgment received
     */
    bool waitForAck(uint8_t ackMaj, uint8_t ackMin, uint32_t timeoutMs = 1000);

    /**
     * @brief Wait for an acknowledgment and copy the response data
     * @param ackMaj Major acknowledgment byte
     * @param ackMin Minor acknowledgment byte
     * @param out Output buffer for the response data
     * @param outCap Capacity of the output buffer
     * @param outLen Length of the response data
     * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
     * @return true if acknowledgment received and data copied
     */
    bool waitForAckAndCopy(uint8_t ackMaj, uint8_t ackMin,
                         uint8_t* out, uint16_t outCap, uint16_t* outLen,
                         uint32_t timeoutMs = 1000);

    /**
     * @brief Wait for an acknowledgment
     * @param ackMaj Major acknowledgment byte
     * @param ackMin Minor acknowledgment byte
     * @param expPayload Expected payload data to be compared to received data
     * @param expLen Length of the expected payload
    * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
    * @return true if acknowledgment received and matches expected payload
    */
    bool waitForAck(uint8_t ackMaj, uint8_t ackMin,
                  const uint8_t *expPayload, uint16_t expLen,
                  uint32_t timeoutMs);

    /**
     * @brief wait on a data frame
     * @param type Major command byte of the expected data frame
     * @param out Output buffer for the data payload
     * @param outCap Capacity of the output buffer
     * @param outLen Length of the data payload
     * @param timeoutMs Timeout in milliseconds (default = 1000 ms)
     *  */              
    bool waitForData(uint8_t type, uint8_t* out, uint16_t outCap, uint16_t* outLen,
                   uint32_t timeoutMs = 1000);

    /**
     * @brief Print the last payload in hexadecimal format for debugging
     * @param out Output stream to print to
     */
    void debugPrintLastPayload(Stream &out = Serial) const;  
    
    /**
     * @brief Main loop to process incoming data
     * @note Call this often so incoming data is sent to the correct parsers
     * 
     */
    void loop();

    
private:

    /**
     * @brief Reset the parser state
     */
    void resetParser();

    /**
     * @brief Push the current frame to the parser
     */
    void pushFrame();

    /**
     * @brief Feed a byte to the parser
     * @param b Byte to feed
     */
    void feed(uint8_t b);

    /**
     * @brief Feed a byte to the data parser
     * @param b Byte to feed
     */
    void feedData(uint8_t b);

    /**
     * @brief Decode an ASCII block
     * @param p Pointer to the data
     * @param len Length of the data
     * @param out Output string
     * @return true if successful
     */
    static bool decodeAsciiBlock(const uint8_t* p, uint16_t len, String &out);

    /**
     * @brief Decode a version 4B block
     * @param p Pointer to the data
     * @param len Length of the data
     * @param out Output string
     * @return true if successful
     */
    static bool decodeVersion4B(const uint8_t* p, uint16_t len, String &out);

    /**
     * @brief Parse a minimal frame
     * @param frame Pointer to the frame data
     * @param len Length of the frame data
     * @param out Structure to fill with the parsed event data
     * @return true if successful
     */
    bool parseMinimalFrame(const uint8_t* frame, size_t len);

    /**
     * @brief Dump a frame for debugging
     * @param p Pointer to the data
     * @param n Length of the data
     */
    void dumpFrame(const uint8_t* p, size_t n);

    /**
     * @brief Parses a 32-bit unsigned integer from a little-endian byte array.
     *
     * This function takes a pointer to a 4-byte array and interprets the bytes as a
     * 32-bit unsigned integer in little-endian order (least significant byte first).
     *
     * @param data Pointer to an array of at least 4 bytes containing the little-endian data.
     * @return The parsed 32-bit unsigned integer.
     */
    static uint32_t parseLEUint32(const uint8_t *data);

    void parseProgressFrame(const uint8_t* frame, uint16_t len);

    void parseStandardFrame(const uint8_t* frame, uint16_t len);

    void primeNoiseIfNeeded(const uint32_t e[16]) {
      if (!_noisePrimed) {
          memcpy(_stdData.noise, e, 16 * sizeof(uint32_t));
          _noisePrimed = true;
        }
      }

    HardwareSerial &_serial; 

    bool _begun = false;    

    enum class State : uint8_t { 
        FIND_HDR, 
        LEN0, 
        LEN1, 
        BODY, 
        TAIL 
    };  

  //Command state
  State  _st = State::FIND_HDR;
  uint8_t _hdrIdx = 0;
  uint16_t _need = 0;     // bytes remaining in BODY
  uint16_t _bi = 0;       // body index
  uint8_t _tailIdx = 0;
  uint8_t _body[128];

  // Last captured command frame
  bool _frameReady = false;
  bool _lastValid  = false;
  uint8_t  _lastCmdMaj = 0;
  uint8_t  _lastCmdMin = 0;
  uint16_t _lastPayloadLen = 0;
  uint8_t  _lastPayload[120];
  
  enum class DState : uint8_t { 
      D_FIND_HDR, 
      D_LEN0, 
      D_LEN1, 
      D_BODY, 
      D_TAIL 
  };

  //Data State
  DState  _dst = DState::D_FIND_HDR;
  uint8_t _dhdrIdx = 0;
  uint16_t _dneed = 0;
  uint16_t _dbi = 0;
  uint8_t _dtailIdx = 0;
  uint8_t _dbody[128];

  bool _dataReady = false;
  uint16_t _dataLen = 0;      
  uint8_t  _dataBody[84];

  String _firmwareStr;
  String _serialStr;
  AutoCfgParams _defaults; 

  enum : uint8_t { 
    HDR = 0x6E, 
    FTR = 0x62 
  };
  static constexpr size_t MIN_FRAME = 5;
  static constexpr size_t MAX_FRAME = 64;
  static constexpr uint32_t FRAME_IDLE_TIMEOUT_MS = 30;

  uint8_t _buf[MAX_FRAME];
  size_t  _pos = 0;
  bool    _inFrame = false;
  uint32_t _lastByteMs = 0;

 
  uint8_t  _noise_alpha_q8 = 16; // ≈0.0625
  bool     _noise_update_on_motion = false;
  bool     _noisePrimed = false;
  std::atomic<uint32_t> _stdSeq{0}; 
  std::atomic<uint32_t> _minSeq{0}; // even=stable, odd=being written


enum class OutputMode : uint8_t { 
  Unknown, 
  Minimal, 
  Standard 
};

OutputMode _outMode = OutputMode::Unknown;

enum class MState : uint8_t { 
  FIND_HDR, 
  PAYLOAD 
}; 
MState  _mst   = MState::FIND_HDR;
uint8_t _mbuf[5];
uint8_t _mpos  = 0;

MinimalData  _minData{};
ProgressData _progData{};
StandardData _stdData{};

};

static inline uint32_t emaU32(uint32_t oldv, uint32_t newv, uint8_t a_q8){
  return oldv + ((int32_t(newv) - int32_t(oldv)) * a_q8 >> 8);
}

static inline int16_t dbToQ8(float db){        // dB -> Q8
  float x = db * 256.0f;
  if (x < -32768.0f) x = -32768.0f;
  if (x >  32767.0f) x =  32767.0f;
  return (int16_t)x;
}

