#ifndef IBUS_RECEIVER_H
#define IBUS_RECEIVER_H

#include <Arduino.h>
#include "ibus_config.h"

// Statistics for monitoring the health of the iBUS connection
struct IbusStats {
    uint32_t goodFrames;       // Total valid frames received
    uint32_t checksumErrors;   // Frames dropped due to checksum mismatch
    uint32_t resyncs;          // Times the UART parsing had to resync to the header
    uint16_t frameRateHz;      // Calculated frame rate in Hz
};

// Class handling the parsing and state of the iBUS serial protocol
class IbusReceiver {
public:
    IbusReceiver();
    
    // Initializes the hardware serial port for receiving iBUS data
    void begin();
    
    // Polls the serial port and processes incoming bytes. Should be called frequently.
    // Returns true if a new frame was successfully parsed during the call.
    bool poll();

    // Gets the current value of the specified channel (1-indexed). Returns failsafe value if in failsafe.
    uint16_t channel(uint8_t index) const;
    
    // Gets the raw received value of the channel, ignoring failsafe states.
    uint16_t channelReceived(uint8_t index) const;
    
    // Returns the channel value normalized to a -1.0 to +1.0 float range
    float channelNormalised(uint8_t index) const;
    
    // Returns the channel value mapped to a 0.0 to 1.0 float range
    float channelUnipolar(uint8_t index) const;

    // Checks if the receiver is currently in a failsafe condition
    bool isFailsafe() const;
    
    // Returns whether the sentinel channel indicates a fault
    bool sentinelConfigFault() const { return m_sentinelFault; }
    
    // Provides access to receiver performance statistics
    const IbusStats &stats() const { return m_stats; }

private:
    // Parsing state machine states
    enum class RxState : uint8_t { WaitLength, WaitCommand, Payload };

    void feed(uint8_t byte);      // Processes a single incoming byte
    void handleFrame();           // Processes a fully received frame
    void resync();                // Resets the parser state machine
    void updateFrameRate();       // Computes the incoming frame rate periodically

    HardwareSerial m_uart;        // The serial port instance used for communication
    RxState  m_state;             // Current state of the parser
    uint8_t  m_buffer[IBUS_FRAME_LEN]; // Buffer for assembling incoming frames
    uint8_t  m_index;             // Current position in the buffer
    uint32_t m_lastByteMicros;    // Timestamp of the last received byte for gap detection

    uint16_t m_channels[IBUS_CHANNEL_COUNT]; // Decoded channel values
    uint32_t m_lastFrameMillis;   // Timestamp of the last successful frame
    bool     m_sentinelFault;     // Sentinel channel fault flag

    uint16_t m_lastRaw[IBUS_CHANNEL_COUNT]; // Previous frame's values for movement detection
    bool     m_haveBaseline;      // Indicates if we have received at least one valid frame
    bool     m_live;              // Indicates if the receiver considers the link 'live' (not frozen)
    uint32_t m_lastChangeMillis;  // Time since the last detected channel movement
    uint8_t  m_moveCount;         // Counter for channel movements
    uint32_t m_moveWindowMillis;  // Window timer for counting movements

    IbusStats m_stats;            // Receiver statistics object
    uint32_t  m_rateWindowMillis; // Timer for frame rate calculation
    uint32_t  m_rateWindowFrames; // Counter for frames within the current rate window
};

#endif /* IBUS_RECEIVER_H */