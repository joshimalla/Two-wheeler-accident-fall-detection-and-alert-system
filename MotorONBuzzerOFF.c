#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <math.h>

#define I2C_BUS "/dev/i2c-2"   // I2C bus for BeagleBone Black
#define ADXL345_ADDR 0x53      // ADXL345 I2C address
#define POWER_CTL 0x2D         // Power control register
#define DATA_FORMAT 0x31       // Data format register
#define DATAX0 0x32            // X-axis data register
#define THRESHOLD 2.5          // Acceleration threshold (in g)

// GPIO Definitions
#define BUZZER_GPIO 60
#define MOTOR_GPIO 66

// Function to initialize ADXL345
void init_adxl345(int file) {
    char config[2];

    // Enable measurement mode
    config[0] = POWER_CTL;
    config[1] = 0x08;
    write(file, config, 2);

    // Set data format (±16g, 4mg/LSB)
    config[0] = DATA_FORMAT;
    config[1] = 0x08;
    write(file, config, 2);
}

// Function to read acceleration data from ADXL345
void read_acceleration(int file, float *x, float *y, float *z) {
    char reg = DATAX0;
    char data[6];

    // Request data from ADXL345
    write(file, &reg, 1);
    read(file, data, 6);

    // Convert data to 16-bit values
    int x_raw = ((int)data[1] << 8) | (int)data[0];
    int y_raw = ((int)data[3] << 8) | (int)data[2];
    int z_raw = ((int)data[5] << 8) | (int)data[4];

    // Convert to acceleration in g (assuming ±16g range, 4mg/LSB)
    *x = x_raw * 0.004;
    *y = y_raw * 0.004;
    *z = z_raw * 0.004;
}

// Function to write GPIO value
void write_gpio_value(int gpio, int value) {
    char path[50];
    FILE *fp;

    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
    fp = fopen(path, "w");
    if (fp == NULL) {
        perror("Failed to open GPIO value file");
        exit(1);
    }
    fprintf(fp, "%d", value);
    fclose(fp);
}

// Function to initialize GPIO
void setup_gpio(int gpio) {
    char path[50];
    FILE *fp;

    // Export GPIO
    snprintf(path, sizeof(path), "/sys/class/gpio/export");
    fp = fopen(path, "w");
    if (fp) {
        fprintf(fp, "%d", gpio);
        fclose(fp);
    }

    // Set GPIO direction
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio);
    fp = fopen(path, "w");
    if (fp == NULL) {
        perror("Failed to set GPIO direction");
        exit(1);
    }
    fprintf(fp, "out");
    fclose(fp);
}

int main() {
    int file;
    float x, y, z, acceleration;

    // Open I2C device
    if ((file = open(I2C_BUS, O_RDWR)) < 0) {
        perror("Failed to open I2C bus");
        return 1;
    }

    // Connect to ADXL345
    if (ioctl(file, I2C_SLAVE, ADXL345_ADDR) < 0) {
        perror("Failed to connect to ADXL345");
        return 1;
    }

    // Initialize ADXL345
    init_adxl345(file);

    // Setup GPIOs
    setup_gpio(BUZZER_GPIO);
    setup_gpio(MOTOR_GPIO);

    while (1) {
        // Read acceleration
        read_acceleration(file, &x, &y, &z);

        // Calculate resultant acceleration
        acceleration = sqrt(x * x + y * y + z * z);
        printf("Acceleration: X=%.2fg Y=%.2fg Z=%.2fg | Resultant=%.2fg\n", x, y, z, acceleration);

        // Compare with threshold
        if (acceleration > THRESHOLD) {
            printf("Fall detected! Motor ON, Buzzer OFF.\n");
            write_gpio_value(MOTOR_GPIO, 1);  // Motor ON
            write_gpio_value(BUZZER_GPIO, 0); // Buzzer OFF
        } else {
            printf("No fall detected. Motor OFF, Buzzer ON.\n");
            write_gpio_value(MOTOR_GPIO, 0);  // Motor OFF
            write_gpio_value(BUZZER_GPIO, 1); // Buzzer ON
        }

        // Wait for 1 second before next reading
        sleep(1);
    }

    // Close I2C file
    close(file);
    return 0;
}
