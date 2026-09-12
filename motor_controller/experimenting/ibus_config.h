#ifndef IBUS_CONFIG_H
#define IBUS_CONFIG_H

#include <Arduino.h>

// iBUS Hardware & Pin Configuration
// FlySky iBUS uses a serial protocol. This configures the ESP32 UART.
#define IBUS_RX_PIN               34       // UART2 RX Pin (Connects to FS-iA6B iBUS output)
#define IBUS_TX_PIN               -1       // TX pin is disabled since we only receive
#define IBUS_UART_NUM             2        // We use Hardware Serial UART2 on the ESP32
#define IBUS_BAUD                 115200   // Standard baud rate for iBUS protocol

// iBUS Frame Protocol Definitions
#define IBUS_FRAME_LEN            32       // Standard length of an iBUS frame in bytes
#define IBUS_HEADER_LEN           0x20     // First header byte expected in every frame
#define IBUS_HEADER_CMD           0x40     // Second header byte expected
#define IBUS_CHANNEL_COUNT        14       // Maximum number of channels processed
#define IBUS_BYTE_GAP_US          3000     // Microsecond threshold to detect inter-frame gap for resync

// Channel Pulse Width Boundaries (Microseconds)
// Defines the valid range of standard RC PWM signals (usually 1000us - 2000us)
#define IBUS_CH_MIN               1000
#define IBUS_CH_MID               1500     // Center stick position
#define IBUS_CH_MAX               2000
#define IBUS_CH_DEFAULT           1500     // Default value for uninitialized channels

// Snapping / Deadband Windows
// Small windows around landmarks to prevent jitter at min/mid/max stick positions
#define IBUS_SNAP_US              5        // Deadband at center position
#define IBUS_SNAP_END_US          15       // Deadband at the minimum and maximum ends

// Safety & Failsafe Timings
#define IBUS_SIGNAL_TIMEOUT_MS    100      // Time without a frame before declaring signal loss failsafe
#define IBUS_FREEZE_TIMEOUT_MS    0        // Max time channels can stay exactly the same (0 = Disabled)
#define IBUS_LIVE_MOVES           5        // Number of channel changes needed to consider signal 'live'
#define IBUS_LIVE_WINDOW_MS       1000     // Time window for counting 'live' moves

// Sentinel Channel Failsafe Config
// Use a specific channel (like a switch) as an active failsafe trigger
#define IBUS_FS_SENTINEL_CHANNEL        8
#define IBUS_FS_SENTINEL_MAX_US         1000     // If channel 8 is at or below 1000, trigger failsafe
#define IBUS_FS_SENTINEL_CONFIG_MAX_US  0        // Optional config fault threshold

// Default failsafe fallback values for each channel if signal is lost
#define IBUS_FS_VALUES { \
    1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500, \
    1500, 1500, 1500, 1500, 1500, 1500 \
}

#endif /* IBUS_CONFIG_H */