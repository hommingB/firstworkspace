#pragma once

#include <libserial/SerialPort.h>
#include <string>
#include <stdexcept>
#include <iostream>
LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
// Just handle some common baud rates
    switch (baud_rate)
    {
        case 1200: return LibSerial::BaudRate::BAUD_1200;
        case 1800: return LibSerial::BaudRate::BAUD_1800;
        case 2400: return LibSerial::BaudRate::BAUD_2400;
        case 4800: return LibSerial::BaudRate::BAUD_4800;
        case 9600: return LibSerial::BaudRate::BAUD_9600;
        case 19200: return LibSerial::BaudRate::BAUD_19200;
        case 38400: return LibSerial::BaudRate::BAUD_38400;
        case 57600: return LibSerial::BaudRate::BAUD_57600;
        case 115200: return LibSerial::BaudRate::BAUD_115200;
        case 230400: return LibSerial::BaudRate::BAUD_230400;
        default:
        std:: cout << "Error! Baud rate " << baud_rate << " not supported! Default to 115200" << std :: endl;
        return LibSerial::BaudRate::BAUD_115200;
    }
}
class Esp32Serial {
public:
    explicit Esp32Serial() = default;

    ~Esp32Serial() {
        disconnect();
    }

    // ---------------- Connection management ----------------

    void connect(const std::string& device,
                 int baud,
                 size_t time_out = 100) {
        device_ = device;
        baud_ = convert_baud_rate(baud);
        time_out_ = time_out;

        if (connected_) {
            return;
        }

        serial_.Open(device_);
        serial_.SetBaudRate(baud_);
        serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_.SetParity(LibSerial::Parity::PARITY_NONE);
        serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

        connected_ = true;

        // Optional: sanity check
        if (!ping()) {
            throw std::runtime_error("ESP32 did not respond to PING");
        }
    }

    void disconnect() {
        if (connected_ && serial_.IsOpen()) {
            serial_.Close();
        }
        connected_ = false;
    }

    bool isConnected() const {
        return connected_ && serial_.IsOpen();
    }

    // ---------------- Motor control API ----------------

    // Set motor output (e.g. PWM or velocity)
    void setValue(std::string &motor_name1, double value1, std::string &motor_name2, double value2) {
        ensureConnected();

        sendCommand("SET " + motor_name1 +
                    " " + std::to_string(value1) + 
                    " " + motor_name1 + " " + std::to_string(value1));

        expectOK();
    }

    // Read encoder count
    void readEncoder(const std::vector<std::string_view> &motor_names, int &L_enc, int &R_enc) {
        ensureConnected();
        sendCommand("ENC");

        std::string response = readLine(); 

        char name1[32], name2[32];
        int val1, val2;

        if (sscanf(response.c_str(), "ENC %31s %d %31s %d", name1, &val1, name2, &val2) != 4) {
            throw std::runtime_error("Invalid format: " + response);
        }

        bool foundL = false, foundR = false;

        if (name1 == motor_names[0]) { L_enc = val1; foundL = true; }
        else if (name1 == motor_names[1]) { R_enc = val1; foundR = true; }

        if (name2 == motor_names[0]) { L_enc = val2; foundL = true; }
        else if (name2 == motor_names[1]) { R_enc = val2; foundR = true; }

        if (!foundL || !foundR) {
            throw std::runtime_error("Encoder name mismatch or missing");
        }
    }

    // Emergency stop (optional but realistic)
    void stopAll() {
        ensureConnected();
        sendCommand("STOP");
        expectOK();
    }

private:
    // ---------------- Internal helpers ----------------

    void ensureConnected() const {
        if (!isConnected()) {
            throw std::runtime_error("ESP32 not connected");
        }
    }

    void sendCommand(const std::string& cmd) {
        serial_.Write(cmd + "\n");
    }

    std::string readLine() {
        std::string line;
        try
        {
            serial_.ReadLine(line, '\n', time_out_);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << ": The Readline() call has timed out." << '\n';
        }
        
        return line;
    }

    void expectOK() {
        std::string response = readLine();
        if (response != "OK") {
            throw std::runtime_error("ESP32 error: " + response);
        }
    }

    bool ping() {
        sendCommand("PING");
        return readLine() == "PONG";
    }

private:
    std::string device_;
    LibSerial::BaudRate baud_;
    LibSerial::SerialPort serial_;
    bool connected_;
    size_t time_out_;
};
