#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <glob.h>
#include <string>
#include <vector>

using namespace std;

// The exact same struct used in the Arduino code
#pragma pack(push, 1)
struct KartCommand {
    int8_t  steering;   // -100 to 100
    uint8_t throttle;   // 0 to 100
    uint8_t brake;      // 0 or 1
};
#pragma pack(pop)

// Global state variables
atomic<int> current_steering(0);
atomic<int> current_throttle(0);
atomic<int> current_brake(0);
atomic<bool> running(true);

// Function to set terminal to non-blocking "raw" mode
void setRawMode(bool enable) {
    static struct termios oldt, newt;
    if (enable) {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        // VMIN=0, VTIME=1 (0.1s timeout) to allow detecting when keys stop being pressed
        newt.c_cc[VMIN] = 0;
        newt.c_cc[VTIME] = 1; 
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    } else {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
}

// Thread to constantly read keyboard inputs
void keyboardThread() {
    char c;
    auto last_w_time = chrono::steady_clock::now();
    bool w_pressed = false;

    auto last_space_time = chrono::steady_clock::now();
    bool space_pressed = false;

    // Steering auto-return: track A/D key press state
    auto last_a_time = chrono::steady_clock::now();
    bool a_pressed = false;
    auto last_d_time = chrono::steady_clock::now();
    bool d_pressed = false;
    auto last_steer_decay_time = chrono::steady_clock::now();

    while (running) {
        if (read(STDIN_FILENO, &c, 1) > 0) {
            c = tolower(c);
            // W: Throttle
            if (c == 'w') {
                current_throttle = min(100, current_throttle + 1); // Very slow Ramp up
                last_w_time = chrono::steady_clock::now();
                w_pressed = true;
            }
            
            // A / D: Steering (auto-returns to center when released)
            static auto last_steer_time = chrono::steady_clock::now();
            auto now_steer = chrono::steady_clock::now();
            
            if (c == 'a') {
                if (chrono::duration_cast<chrono::milliseconds>(now_steer - last_steer_time).count() > 40) {
                    current_steering = max(-100, current_steering - 1); // Ultra slow steering
                    last_steer_time = now_steer;
                }
                last_a_time = chrono::steady_clock::now();
                a_pressed = true;
            } else if (c == 'd') {
                if (chrono::duration_cast<chrono::milliseconds>(now_steer - last_steer_time).count() > 40) {
                    current_steering = min(100, current_steering + 1); // Ultra slow steering
                    last_steer_time = now_steer;
                }
                last_d_time = chrono::steady_clock::now();
                d_pressed = true;
            }

            // Spacebar: Brakes (Hold to brake)
            if (c == ' ') {
                current_brake = 1;
                last_space_time = chrono::steady_clock::now();
                space_pressed = true;
            }
            
            // Q: Quit
            if (c == 'q') {
                running = false;
            }
        }

        auto now = chrono::steady_clock::now();

        // Safety: If 'W' hasn't been seen for 250ms, assume it was released
        if (w_pressed) {
            if (chrono::duration_cast<chrono::milliseconds>(now - last_w_time).count() > 250) {
                w_pressed = false; // Mark as released, begin decay
            }
        }

        // Decay throttle slowly when W is released
        if (!w_pressed && current_throttle > 0) {
            static auto last_decay_time = chrono::steady_clock::now();
            if (chrono::duration_cast<chrono::milliseconds>(now - last_decay_time).count() > 20) {
                current_throttle = max(0, current_throttle - 1);
                last_decay_time = now;
            }
        }

        // Safety: If 'Space' hasn't been seen for 250ms, assume it was released
        if (space_pressed) {
            if (chrono::duration_cast<chrono::milliseconds>(now - last_space_time).count() > 250) {
                current_brake = 0;
                space_pressed = false;
            }
        }

        // Safety: If 'A' hasn't been seen for 250ms, assume it was released
        if (a_pressed) {
            if (chrono::duration_cast<chrono::milliseconds>(now - last_a_time).count() > 250) {
                a_pressed = false;
            }
        }

        // Safety: If 'D' hasn't been seen for 250ms, assume it was released
        if (d_pressed) {
            if (chrono::duration_cast<chrono::milliseconds>(now - last_d_time).count() > 250) {
                d_pressed = false;
            }
        }

        // Decay steering toward center (0) when neither A nor D is held
        if (!a_pressed && !d_pressed && current_steering != 0) {
            if (chrono::duration_cast<chrono::milliseconds>(now - last_steer_decay_time).count() > 20) {
                int steer = current_steering.load();
                if (steer > 0) {
                    current_steering = max(0, steer - 1);
                } else {
                    current_steering = min(0, steer + 1);
                }
                last_steer_decay_time = now;
            }
        }
        
        // Small sleep to prevent CPU hogging, but fast enough to read rapid keystrokes
        this_thread::sleep_for(chrono::milliseconds(10));
    }
}

// Auto-detect the ESP32 serial port by scanning common USB-serial device patterns
string findESP32Port() {
    // Common patterns for ESP32 USB-serial devices on macOS and Linux
    const char* patterns[] = {
        "/dev/cu.usbserial*",   // macOS (CP210x, FTDI, CH340)
        "/dev/cu.SLAB*",        // macOS (Silicon Labs CP210x alternate)
        "/dev/cu.wchusbserial*",// macOS (CH340/CH910)
        "/dev/ttyUSB*",         // Linux (CP210x, FTDI, CH340)
        "/dev/ttyACM*",         // Linux (native USB CDC)
        nullptr
    };
    
    glob_t results;
    for (int i = 0; patterns[i] != nullptr; ++i) {
        if (glob(patterns[i], (i == 0 ? 0 : GLOB_APPEND), nullptr, &results) == 0) {
            // Continue to collect all matches
        }
    }
    
    string found;
    if (results.gl_pathc > 0) {
        found = results.gl_pathv[0]; // Pick the first match
    }
    globfree(&results);
    return found;
}

int main() {
    // Auto-detect the ESP32 serial port
    string portName = findESP32Port();
    
    if (!portName.empty()) {
        cout << "Auto-detected ESP32 on: " << portName << endl;
    } else {
        cerr << "ERROR: Could not auto-detect ESP32 port. Port not connected." << endl;
        return 1;
    }
    
    int serial_fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    
    if (serial_fd == -1) {
        cerr << "ERROR: Could not open serial port. Port not connected." << endl;
        return 1;
    }
    
    // Configure Serial Port to 115200 baud
    struct termios options;
    tcgetattr(serial_fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;     // 8 data bits
    options.c_cflag &= ~PARENB; // No parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit
    tcsetattr(serial_fd, TCSANOW, &options);
    cout << "Serial port opened successfully." << endl;

    setRawMode(true);
    
    cout << "\n=== Laptop Kart Controller ===" << endl;
    cout << " W     : Accelerate (Release to stop)" << endl;
    cout << " A / D : Steering (Left / Right)" << endl;
    cout << " Space : Brake (Hold to brake)" << endl;
    cout << " Q     : Quit" << endl;
    cout << "==============================" << endl;

    // Start keyboard reader thread
    thread kb_thread(keyboardThread);

    KartCommand cmd;
    
    // Main transmission loop (20Hz)
    while (running) {
        cmd.steering = (int8_t)current_steering.load();
        cmd.throttle = (uint8_t)current_throttle.load();
        cmd.brake    = (uint8_t)current_brake.load();

        ssize_t bytes_written = write(serial_fd, &cmd, sizeof(KartCommand));
        if (bytes_written < (ssize_t)sizeof(KartCommand)) {
            cerr << "\n\nERROR: Serial write failed." << endl;
            cerr << "EMERGENCY STOP: Sending zero-command and exiting..." << endl;
            // Attempt to send a zero-command
            KartCommand zero_cmd = {0, 0, 0};
            write(serial_fd, &zero_cmd, sizeof(KartCommand));
            running = false;
            break;
        }

        // Print status to console (carriage return overwrites current line)
        printf("\r[Status] Throttle: %3d%% | Steer: %4d%% | Brake: %s    ", 
               cmd.throttle, cmd.steering, cmd.brake ? "ON " : "OFF");
        fflush(stdout);

        this_thread::sleep_for(chrono::milliseconds(50)); // 50ms = 20Hz
    }

    cout << "\n\nExiting..." << endl;
    
    setRawMode(false);
    if (serial_fd != -1) {
        close(serial_fd);
    }
    kb_thread.join();
    
    return 0;
}
