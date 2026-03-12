#pragma once

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>

#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <cstring>
#include <chrono>

static speed_t convert_baud_rate(int baud)
{
    switch (baud)
    {
        case 1200: return B1200;
        case 2400: return B2400;
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
#ifdef B57600
        case 57600: return B57600;
#endif
#ifdef B115200
        case 115200: return B115200;
#endif
#ifdef B230400
        case 230400: return B230400;
#endif
        default:
            std::cerr << "Unsupported baud rate, defaulting to 115200\n";
            return B115200;
    }
}

class Esp32Serial
{
public:
    Esp32Serial() = default;

    ~Esp32Serial()
    {
        disconnect();
    }

    // ---------- Connection ----------

    void connect(const std::string& device,
                 int baud,
                 size_t timeout_ms = 100)
    {
        if (connected_) return;

        device_ = device;
        baud_ = convert_baud_rate(baud);
        timeout_ms_ = timeout_ms;

        fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0)
            throw std::runtime_error("Failed to open serial port");

        configurePort();

        connected_ = true;

        if (!ping())
            throw std::runtime_error("ESP32 did not respond to PING");
    }

    void disconnect()
    {
        if (connected_)
        {
            close(fd_);
            connected_ = false;
        }
    }

    bool isConnected() const
    {
        return connected_;
    }

    // ---------- API ----------

    void setValue(std::string &motor_name1, double value1, std::string &motor_name2, double value2)
    {
        ensureConnected();
        sendCommand("SET " + motor_name1 +
                    " " + std::to_string(value1) + 
                    " " + motor_name2 + " " + std::to_string(value2));
        expectOK();
    }

    void readEncoder(const std::vector<std::string_view>& motor_names,
                     int& L_enc, int& R_enc)
    {
        ensureConnected();
        sendCommand("ENC");

        std::string response = readLine();

        char name1[32], name2[32];
        int val1, val2;

        if (sscanf(response.c_str(),
                   "ENC %31s %d %31s %d",
                   name1, &val1, name2, &val2) != 4)
        {
            throw std::runtime_error("Invalid format: " + response);
        }

        bool foundL = false, foundR = false;

        if (motor_names[0] == name1) { L_enc = val1; foundL = true; }
        if (motor_names[1] == name1) { R_enc = val1; foundR = true; }
        if (motor_names[0] == name2) { L_enc = val2; foundL = true; }
        if (motor_names[1] == name2) { R_enc = val2; foundR = true; }

        if (!foundL || !foundR)
            throw std::runtime_error("Encoder name mismatch");
    }

    void stopAll()
    {
        ensureConnected();
        sendCommand("STOP");
        expectOK();
    }

private:
    // ---------- Serial config ----------

    void configurePort()
    {
        struct termios tty{};
        if (tcgetattr(fd_, &tty) != 0)
            throw std::runtime_error("tcgetattr failed");

        cfsetospeed(&tty, baud_);
        cfsetispeed(&tty, baud_);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_iflag = 0;
        tty.c_oflag = 0;
        tty.c_lflag = 0;

        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0)
            throw std::runtime_error("tcsetattr failed");
    }

    // ---------- IO helpers ----------

    void ensureConnected() const
    {
        if (!connected_)
            throw std::runtime_error("Not connected");
    }

    void sendCommand(const std::string& cmd)
    {
        while (waitReadable()) readLine();
        std::string data = cmd + "\n";
        write(fd_, data.c_str(), data.size());
    }

    std::string readLine()
    {
        std::string result;
        char c;

        while (true)
        {
            if (!waitReadable())
                throw std::runtime_error("Timeout waiting for serial data");

            ssize_t n = read(fd_, &c, 1);
            if (n <= 0)
                throw std::runtime_error("Serial read failed");

            if (c == '\n')
                break;

            if (c != '\r')   // strip CR
                result += c;
        }

        return result;
    }


    bool waitReadable()
    {
        fd_set set;
        FD_ZERO(&set);
        FD_SET(fd_, &set);

        struct timeval timeout;
        timeout.tv_sec  = timeout_ms_ / 1000;
        timeout.tv_usec = (timeout_ms_ % 1000) * 1000;

        int rv = select(fd_ + 1, &set, nullptr, nullptr, &timeout);
        return rv > 0;
    }

    void expectOK()
    {
        std::string r = readLine();
        if (r != "OK")
            throw std::runtime_error("ESP32 error: " + r);
    }

    bool ping()
    {
        sendCommand("PING");

        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);

        while (std::chrono::steady_clock::now() < deadline)
        {
            auto r = readLine();
            if (r.find("PONG") != std::string::npos)
                return true;
        }

        return false;
    }


private:
    std::string device_;
    speed_t baud_;
    int fd_ = -1;
    bool connected_ = false;
    size_t timeout_ms_ = 100;
};
