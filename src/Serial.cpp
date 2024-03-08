#include <algorithm>
#include <execution>
#include <sstream>
#include <system_error>
#include <unistd.h>

#include "Serial.h"


Serial::Serial(const std::string com_port, const speed_t baudrate, const unsigned int size) {
    this->com_port = com_port;
    this->serial_port = open(com_port.c_str(), O_RDWR | O_NOCTTY);
    this->size = size;

    if (this->serial_port < 0) {
        std::ostringstream oss;
        oss << "failed to open serial port" << "(" << this->com_port << ")" << std::endl;
        throw std::runtime_error(oss.str());
    }

    if (tcgetattr(this->serial_port, &this->tty) != 0) {
        close(this->serial_port);
        std::ostringstream oss;
        oss << "failed to fetch serial port attributes" << "(" << this->com_port << ")" << std::endl;
        throw std::runtime_error(oss.str());
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
        std::ostringstream oss;
        oss << "failed to set serial port attributes" << "(" << com_port << ")" << std::endl;
        throw std::runtime_error(oss.str());
    }
}


Serial::~Serial() {
    if (this->serial_port >= 0) close(this->serial_port);
}


ssize_t Serial::Read(unsigned char* buffer) {
    return read(this->serial_port, &buffer, this->size);
}


ssize_t Serial::Read(std::function<void(unsigned char*, const int)> callback) {
    unsigned char buffer[this->size] = {0};
    ssize_t bytes_read = 0;

    bytes_read = read(this->serial_port, &buffer, this->size);
    if (bytes_read > 0) callback(buffer, bytes_read);

    return bytes_read;
}


int Serial::Read(std::stop_token stop, std::function<void(unsigned char*, const int)> callback, const unsigned char header, const unsigned char footer) {
    int bytes_read = 0;
    unsigned char buffer[this->size] = {0};
    bool header_found = false;


    while (!stop.stop_requested()) {
        unsigned char t_byte;
        bytes_read = read(this->serial_port, &t_byte, 1);

        if (bytes_read > 0) {
            if (!header_found) {
                if (t_byte == header) {
                    header_found = true;
                    bytes_read = 0;
                    buffer[bytes_read++] = t_byte;
                }
            } else {
                buffer[bytes_read++] = t_byte;

                if (bytes_read == this->size) {
                    if (t_byte == footer) callback(buffer, bytes_read);
                    header_found = false;
                    bytes_read = 0;
                    std::fill_n(std::execution::par_unseq, buffer, this->size, 0);
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return EXIT_SUCCESS;
}


ssize_t Serial::Write(const unsigned char* data) {
    ssize_t bytes_written = 0;

    bytes_written = write(this->serial_port, data, this->size);

    return bytes_written;
}
