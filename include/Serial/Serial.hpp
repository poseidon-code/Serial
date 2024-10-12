#pragma once

#include <functional>
#include <string>
#include <system_error>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>



class Serial {
private:
    struct termios tty;
    unsigned int size;

public:
    int serial_port;

    Serial(const std::string com_port, const speed_t baudrate, const unsigned int size) {
        this->serial_port = open(com_port.c_str(), O_RDWR | O_NOCTTY);
        this->size = size;

        if (this->serial_port < 0) throw std::runtime_error("failed to open serial port");

        if (tcgetattr(this->serial_port, &this->tty) != 0) {
            close(this->serial_port);
            throw std::runtime_error("failed to fetch serial port attributes");
        }


        this->tty.c_cflag &= ~PARENB;             // no parity
        this->tty.c_cflag &= ~CSTOPB;             // one stop bit
        this->tty.c_cflag &= ~CSIZE;              // reset character size bits
        this->tty.c_cflag |= CS8;                 // 8 bits per byte
        this->tty.c_cflag &= ~CRTSCTS;            // no flow control
        this->tty.c_cflag |= CLOCAL | CREAD;      // enable receiver, ignore modem control signals


        // Disable special handling of received bytes
        this->tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);

        // disable input processing
        this->tty.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | ISIG | IEXTEN);

        this->tty.c_oflag &= ~OPOST; // disable output processing
        this->tty.c_oflag &= ~ONLCR;

        this->tty.c_cc[VMIN] = this->size; // minimum number of charaters to read


        cfsetospeed(&this->tty, baudrate); // sets write baudrate
        cfsetispeed(&this->tty, baudrate); // sets read baudrate

        fcntl(this->serial_port, F_SETFL, O_NONBLOCK); // set non-blocking mode


        if (tcsetattr(this->serial_port, TCSANOW, &this->tty) != 0) {
            close(this->serial_port);
            throw std::runtime_error("failed to set serial port attributes");
        }
    }


    ~Serial() {
        if (this->serial_port >= 0) close(this->serial_port);
    }


    ssize_t Read(unsigned char* buffer) {
        return read(this->serial_port, &buffer, this->size);
    }


    ssize_t Read(std::function<void(unsigned char*, const ssize_t)> callback) {
        unsigned char buffer[this->size] = {0};

        ssize_t bytes_read = this->Read(buffer);
        if (bytes_read > 0) callback(buffer, bytes_read);

        return bytes_read;
    }


    ssize_t Write(const unsigned char* data) {
        return write(this->serial_port, data, this->size);
    }
};
