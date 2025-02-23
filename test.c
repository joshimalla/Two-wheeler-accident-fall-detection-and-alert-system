#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>

#define ADXL345_I2C_ADDR  0x53  // ADXL345 I2C address
#define MOTOR_PIN         60    // GPIO_60 (P9_12) for MOSFET control

// ADXL345 Register Addresses
#define POWER_CTL         0x2D
#define DATA_FORMAT       0x31
#define DATAX0           0x32

int i2c_fd;

// Function to initialize ADXL345
void init_adxl345() {
    i2c_fd = open("/dev/i2c-2", O_RDWR); // Open I2C bus
    if (i2c_fd < 0) {
        perror("Failed to open I2C");
        exit(1);
    }
    if (ioctl(i2c_fd, I2C_SLAVE, ADXL345_I2C_ADDR) < 0) {
        perror("Failed to set I2C address");
        exit(1);
    }

    // Set ADXL345 to measurement mode
    char config[2] = {POWER_CTL, 0x08};
    write(i2c_fd, config, 2);
}

// Function to read accelerometer data
int16_t read_acceleration() {
    char reg = DATAX0;
    write(i2c_fd, &reg, 1);

    char data[6];
    read(i2c_fd, data, 6);

    int16_t x = ((int16_t)data[1] << 8) | data[0];
    return x;
}

int main() {
    // Initialize GPIO
    if (wiringPiSetupGpio() == -1) {
        printf("GPIO Initialization Failed!\n");
        return 1;
    }
    
    pinMode(MOTOR_PIN, OUTPUT);
    digitalWrite(MOTOR_PIN, LOW); // Start with motor OFF

    // Initialize ADXL345
    init_adxl345();
    printf("System Initialized. Monitoring for falls...\n");

    while (1) {
        int16_t x_accel = read_acceleration();
        
        if (abs(x_accel) > 500) { // Fall threshold
            printf("Fall detected! Turning motor OFF\n");
            digitalWrite(MOTOR_PIN, LOW);
        } else {
            printf("No fall. Motor ON\n");
            digitalWrite(MOTOR_PIN, HIGH);
        }
        
        usleep(500000); // 500ms delay
    }

    close(i2c_fd);
    return 0;
}

