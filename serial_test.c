#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>

#define DATA_SIZE (1024 * 1000)

int total_bits_per_frame = 10; // 1 start bit + 8 data bits + 1 stop bit

// Function to configure the serial port
int setup_serial(const char *port, speed_t baudrate) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("open");
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8 data bits
    tty.c_iflag &= ~IGNBRK;         // Disable break processing
    tty.c_lflag = 0;                // No signal processing
    tty.c_oflag = 0;                // Disable output processing
    tty.c_cc[VMIN]  = 1;            // Read at least 1 byte each time
    tty.c_cc[VTIME] = 10;           // 1 second timeout

    tty.c_cflag |= (CLOCAL | CREAD);  // Enable receiver
    tty.c_cflag &= ~(PARENB | PARODD); // No parity
    tty.c_cflag &= ~CSTOPB;            // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;           // No hardware flow control

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    return fd;
}

// Thread function for sending data
void *send_data(void *arg) {
    int fd = *((int *)arg);
    char *data = malloc(DATA_SIZE);
    memset(data, '0', DATA_SIZE);

    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    int bytes_sent = write(fd, data, DATA_SIZE);
    if (bytes_sent != DATA_SIZE) {
        perror("write");
    }

    tcdrain(fd);  // Wait for the transmission to complete
    clock_gettime(CLOCK_MONOTONIC, &end);

    double elapsed_time = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
    double baud_rate = (DATA_SIZE * total_bits_per_frame) / elapsed_time;

    printf("Transmission completed: Time taken %.2f seconds, Actual transmission baud rate: %.2f bps\n", elapsed_time, baud_rate);

    free(data);
    return NULL;
}

// Thread function for receiving data
void *receive_data(void *arg) {
    int fd = *((int *)arg);
    char *received_data = malloc(DATA_SIZE);
    int total_bytes_received = 0;

    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    while (total_bytes_received < DATA_SIZE) {
        int bytes_received = read(fd, received_data + total_bytes_received, DATA_SIZE - total_bytes_received);
        if (bytes_received < 0) {
            perror("read");
            break;
        }
        total_bytes_received += bytes_received;
    }

    clock_gettime(CLOCK_MONOTONIC, &end);

    double elapsed_time = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
    double baud_rate = (total_bytes_received * total_bits_per_frame) / elapsed_time;

    printf("Reception completed: Time taken %.2f seconds, Total bytes received: %d, Actual reception baud rate: %.2f bps\n", elapsed_time, total_bytes_received, baud_rate);

    free(received_data);
    return NULL;
}

// Converts the input baud rate to the standard value in termios
speed_t get_baud_rate(int baudrate) {
    switch (baudrate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: 
            fprintf(stderr, "Unsupported baud rate: %d\n", baudrate);
            exit(EXIT_FAILURE);
    }
}

int main(int argc, char *argv[]) {
    if (argc != 4) {
        fprintf(stderr, "Usage: %s <serial port 1> <serial port 2> <baud rate>\n", argv[0]);
        return 1;
    }

    // Get command line arguments
    const char *tx_port = argv[1];
    const char *rx_port = argv[2];
    int baudrate = atoi(argv[3]);
    speed_t baud = get_baud_rate(baudrate);

    // Set up serial ports
    int tx_fd = setup_serial(tx_port, baud);
    int rx_fd = setup_serial(rx_port, baud);
    if (tx_fd < 0 || rx_fd < 0) {
        return 1;
    }

    // Create send and receive threads
    pthread_t send_thread, receive_thread;
    pthread_create(&send_thread, NULL, send_data, &tx_fd);
    pthread_create(&receive_thread, NULL, receive_data, &rx_fd);

    // Wait for threads to complete
    pthread_join(send_thread, NULL);
    pthread_join(receive_thread, NULL);

    // Close serial ports
    close(tx_fd);
    close(rx_fd);

    return 0;
}
