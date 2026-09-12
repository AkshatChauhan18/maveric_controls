#include "ibus_receiver.h"

namespace {
// Internal Constants and Helper Functions
constexpr size_t kRxBufferSize = 256;      // UART RX buffer size
constexpr uint32_t kRateWindowMs = 1000;   // Window in ms for calculating frame rate
constexpr uint16_t kChannelValueMask = 0x0FFF; // 12-bit mask for channel data

// Utility to clamp a float between a minimum and maximum value
float clamp(float value, float low, float high) {
    if (value < low)  return low;
    if (value > high) return high;
    return value;
}

// Default values to apply if the receiver enters failsafe mode
constexpr uint16_t kFailsafeValues[IBUS_CHANNEL_COUNT] = IBUS_FS_VALUES;

// Structure for snapping stick values to perfect center/min/max
struct Landmark { uint16_t value; uint16_t window; };

// Applies deadband snapping around the ends and center of the stick travel
uint16_t snapToLandmark(uint16_t value) {
    constexpr Landmark kLandmarks[] = {
        { IBUS_CH_MIN, IBUS_SNAP_END_US },
        { IBUS_CH_MID, IBUS_SNAP_US     },
        { IBUS_CH_MAX, IBUS_SNAP_END_US },
    };
    for (const Landmark &lm : kLandmarks) {
        // Calculate absolute distance to the landmark
        const uint16_t delta = (value > lm.value) ? (value - lm.value) : (lm.value - value);
        // If within the window, snap exactly to the landmark value
        if (delta <= lm.window) return lm.value;
    }
    return value; // If outside all windows, return raw value
}
} // namespace

// Constructor: Initializes members and sets channels to their failsafe defaults initially
IbusReceiver::IbusReceiver()
    : m_uart(IBUS_UART_NUM), m_state(RxState::WaitLength), m_buffer{}, m_index(0),
      m_lastByteMicros(0), m_channels{}, m_lastFrameMillis(0), m_sentinelFault(false),
      m_lastRaw{}, m_haveBaseline(false), m_live(false), m_lastChangeMillis(0),
      m_moveCount(0), m_moveWindowMillis(0), m_stats{}, m_rateWindowMillis(0), m_rateWindowFrames(0)
{
    for (uint8_t i = 0; i < IBUS_CHANNEL_COUNT; ++i) {
        m_channels[i] = kFailsafeValues[i];
    }
}

// Starts the serial port and prepares the parser
void IbusReceiver::begin() {
    m_uart.setRxBufferSize(kRxBufferSize);
    // Configures the UART with the specific baud rate and pins required for iBUS
    m_uart.begin(IBUS_BAUD, SERIAL_8N1, IBUS_RX_PIN, IBUS_TX_PIN);
    m_state = RxState::WaitLength;
    m_index = 0;
    m_lastByteMicros = micros();
    m_rateWindowMillis = millis();
}

// Polls the UART for incoming bytes and feeds them to the parser
bool IbusReceiver::poll() {
    const uint32_t framesBefore = m_stats.goodFrames;
    int available = m_uart.available();
    
    // Process all bytes currently in the RX buffer
    while (available-- > 0) {
        const int byte = m_uart.read();
        if (byte < 0) break;
        feed(static_cast<uint8_t>(byte));
    }
    
    // Update the calculated frame rate stats
    updateFrameRate();
    
    // Return true if at least one new frame was successfully parsed
    return m_stats.goodFrames != framesBefore;
}

// Processes a single incoming byte using a state machine
void IbusReceiver::feed(uint8_t byte) {
    const uint32_t now = micros();
    const uint32_t gap = now - m_lastByteMicros;
    m_lastByteMicros = now;

    // Detect large time gaps between bytes to resync to the start of a frame
    if (m_state != RxState::WaitLength && gap > IBUS_BYTE_GAP_US) {
        resync();
    }

    // iBUS parsing state machine
    switch (m_state) {
    case RxState::WaitLength:
        // Waiting for the first byte of a frame (0x20 length byte)
        if (byte == IBUS_HEADER_LEN) {
            m_buffer[0] = byte;
            m_index = 1;
            m_state = RxState::WaitCommand;
        }
        break;
    case RxState::WaitCommand:
        // Waiting for the second byte (0x40 command byte)
        if (byte == IBUS_HEADER_CMD) {
            m_buffer[1] = byte;
            m_index = 2;
            m_state = RxState::Payload;
        } else if (byte == IBUS_HEADER_LEN) {
            // Might be offset, stay in WaitCommand but reset index
            m_index = 1;
        } else {
            resync(); // Invalid header, start over
        }
        break;
    case RxState::Payload:
        // Reading the channel data and checksum bytes
        m_buffer[m_index++] = byte;
        // Check if we have received a full frame (32 bytes)
        if (m_index >= IBUS_FRAME_LEN) {
            handleFrame(); // Attempt to process the full frame
            m_state = RxState::WaitLength; // Reset state for the next frame
            m_index = 0;
        }
        break;
    }
}

// Validates and extracts channel data from a fully received frame
void IbusReceiver::handleFrame() {
    // Calculate the checksum by summing the first 30 bytes
    uint16_t sum = 0;
    for (uint8_t i = 0; i < IBUS_FRAME_LEN - 2; ++i) {
        sum += m_buffer[i];
    }
    
    // The expected checksum is 0xFFFF minus the calculated sum
    const uint16_t expected = static_cast<uint16_t>(0xFFFF - sum);
    // The received checksum is in the last two bytes (Little Endian)
    const uint16_t received = static_cast<uint16_t>(m_buffer[30]) | (static_cast<uint16_t>(m_buffer[31]) << 8);

    // If the checksum doesn't match, drop the frame and increment error stat
    if (expected != received) {
        ++m_stats.checksumErrors;
        return;
    }

    bool moved = false; // Flag to detect if any channel changed value
    
    // Decode each channel (2 bytes per channel, Little Endian)
    for (uint8_t ch = 0; ch < IBUS_CHANNEL_COUNT; ++ch) {
        const uint8_t offset = 2 + (ch * 2);
        const uint16_t raw = (static_cast<uint16_t>(m_buffer[offset]) | (static_cast<uint16_t>(m_buffer[offset + 1]) << 8)) & kChannelValueMask;

        if (m_haveBaseline && raw != m_lastRaw[ch]) {
            moved = true; // A stick/switch has moved
        }
        m_lastRaw[ch] = raw;
        // Snap the raw value to deadbands before storing as the current channel state
        m_channels[ch] = snapToLandmark(raw);
    }

    m_haveBaseline = true;
    const uint32_t now = millis();

    // Logic to detect a "frozen" receiver (e.g., receiver loses link but keeps outputting the last frame)
#if !IBUS_FREEZE_TIMEOUT_MS
    (void)moved;
    m_live = true; // Freeze detection disabled, assume always live if receiving frames
#else
    // If no movement for a long time, consider it non-live (frozen)
    if (m_live && (now - m_lastChangeMillis) > IBUS_FREEZE_TIMEOUT_MS) {
        m_live = false;
        m_moveCount = 0;
    }
    // If movement detected, require a certain number of moves within a window to declare it 'live' again
    if (moved) {
        if (now - m_moveWindowMillis > IBUS_LIVE_WINDOW_MS) {
            m_moveWindowMillis = now;
            m_moveCount = 0; // Reset counter if window expired
        }
        if (m_moveCount < IBUS_LIVE_MOVES) ++m_moveCount;
        if (m_moveCount >= IBUS_LIVE_MOVES) m_live = true;
        m_lastChangeMillis = now;
    }
#endif

    m_lastFrameMillis = now;
    ++m_stats.goodFrames;
    ++m_rateWindowFrames;
}

// Resets the parser state due to a timeout or invalid data
void IbusReceiver::resync() {
    if (m_state != RxState::WaitLength) ++m_stats.resyncs;
    m_state = RxState::WaitLength;
    m_index = 0;
}

// Periodically calculates the frame rate (Hz) for statistics
void IbusReceiver::updateFrameRate() {
    const uint32_t now = millis();
    if (now - m_rateWindowMillis >= kRateWindowMs) {
        const uint32_t elapsed = now - m_rateWindowMillis;
        m_stats.frameRateHz = static_cast<uint16_t>((m_rateWindowFrames * 1000UL) / elapsed);
        m_rateWindowFrames = 0;
        m_rateWindowMillis = now;
    }
}

// Returns the value of a channel (1-14). Returns failsafe value if in failsafe condition.
uint16_t IbusReceiver::channel(uint8_t index) const {
    if (index < 1 || index > IBUS_CHANNEL_COUNT) return IBUS_CH_DEFAULT;
    if (isFailsafe()) return kFailsafeValues[index - 1];
    return m_channels[index - 1];
}

// Returns the raw received value of a channel without applying failsafe overrides
uint16_t IbusReceiver::channelReceived(uint8_t index) const {
    if (index < 1 || index > IBUS_CHANNEL_COUNT) return IBUS_CH_DEFAULT;
    return m_channels[index - 1];
}

// Maps a channel to a bipolar range [-1.0, 1.0], assuming 1500 is center
float IbusReceiver::channelNormalised(uint8_t index) const {
    const float value = static_cast<float>(channel(index));
    const float span  = static_cast<float>(IBUS_CH_MAX - IBUS_CH_MID);
    return clamp((value - static_cast<float>(IBUS_CH_MID)) / span, -1.0f, 1.0f);
}

// Maps a channel to a unipolar range [0.0, 1.0], using the min/max endpoints
float IbusReceiver::channelUnipolar(uint8_t index) const {
    const float value = static_cast<float>(channel(index));
    const float span  = static_cast<float>(IBUS_CH_MAX - IBUS_CH_MIN);
    return clamp((value - static_cast<float>(IBUS_CH_MIN)) / span, 0.0f, 1.0f);
}

// Determines if the receiver should be treated as in a failsafe state
bool IbusReceiver::isFailsafe() const {
    // Not live (e.g., frozen outputs)
    if (!m_live) return true;
    // Sentinel fault flag
    if (m_sentinelFault) return true;

    // Timeout: Haven't received a valid frame recently
    const uint32_t now = millis();
    if (now - m_lastFrameMillis > IBUS_SIGNAL_TIMEOUT_MS) return true;

    // Sentinel channel threshold logic (e.g., switch mapped to ch8 triggered failsafe)
#if IBUS_FS_SENTINEL_CHANNEL
    if (m_channels[IBUS_FS_SENTINEL_CHANNEL - 1] <= IBUS_FS_SENTINEL_MAX_US) return true;
#endif

    // Explicit freeze timeout check
#if IBUS_FREEZE_TIMEOUT_MS
    if ((now - m_lastChangeMillis) > IBUS_FREEZE_TIMEOUT_MS) return true;
#endif

    return false; // Signal is good and active
}