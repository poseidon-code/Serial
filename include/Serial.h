#pragma once

#include <fcntl.h>
#include <functional>
#include <string>
#include <termios.h>
#include <thread>


class Serial {
private:
    std::string com_port;
    struct termios tty;
    unsigned int size;

public:
    int serial_port;
    Serial(const std::string com_port, const speed_t baudrate, const unsigned int size);
    ~Serial();
    
    ssize_t  Read(unsigned char* buffer);
    ssize_t  Read(std::function<void(unsigned char*, const int)> callback);
    int Read(std::stop_token stop, std::function<void(unsigned char*, const int)> callback, const unsigned char header, const unsigned char footer);
    ssize_t Write(const unsigned char* data);
};
