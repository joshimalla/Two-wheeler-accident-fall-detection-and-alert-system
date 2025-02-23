#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <linux/i2c-dev.h>
#include <string.h>

#define ADXL345_SLAVE_ADDR 0x53    // Slave address of I2C
#define I2C_FILE_PATH "/dev/i2c-2" // I2C device file directory

// Registers from the ADXL345 Datasheet
#define ADXL345_REG_BW_RATE 0x2C     // Bandwidth rate control register
#define ADXL345_REG_POWERCTL 0x2D    // Power control register
#define ADXL345_REG_DATA_FORMAT 0x31 // Data format register
#define ADXL345_REG_DATAX0 0x32      // Start of data registers

// Function to initialize the ADXL345
int adxl345_init()
{
    char writeBuffer[2]; // Two-element buffer to hold register address and data
    int file;            // Local file descriptor

    // Open the I2C file
    if ((file = open(I2C_FILE_PATH, O_RDWR)) < 0)
    {
        perror("Failed to open the I2C bus\n");
        return -1;
    }

    // Connect to the ADXL345 slave
    if (ioctl(file, I2C_SLAVE, ADXL345_SLAVE_ADDR) < 0)
    {
        perror("Failed to connect to the sensor\n");
        close(file);
        return -1;
    }

    // Configure the BW_RATE register (3200 Hz, normal mode)
    writeBuffer[0] = ADXL345_REG_BW_RATE;
    writeBuffer[1] = 0x0F;
    if (write(file, writeBuffer, 2) != 2)
    {
        perror("Failed to write to the BW_RATE register\n");
        close(file);
        return -1;
    }

    // Configure the POWER_CTL register (enable measurement mode)
    writeBuffer[0] = ADXL345_REG_POWERCTL;
    writeBuffer[1] = 0x08;
    if (write(file, writeBuffer, 2) != 2)
    {
        perror("Failed to write to the POWER_CTL register\n");
        close(file);
        return -1;
    }

    // Configure the DATA_FORMAT register (full resolution, ±2g range)
    writeBuffer[0] = ADXL345_REG_DATA_FORMAT;
    writeBuffer[1] = 0x08;
    if (write(file, writeBuffer, 2) != 2)
    {
        perror("Failed to write to the DATA_FORMAT register\n");
        close(file);
        return -1;
    }

    printf("ADXL345 initialized successfully\n");

    // Close the I2C file
    close(file);

    return 0;
}

// Function to read accelerometer data
int adxl345_read(short *acc_x, short *acc_y, short *acc_z)
{
    int file;                                   // File descriptor for I2C
    char buffer[6];                             // Buffer to hold X, Y, Z axis data (6 bytes)
    char reg_pointer[1] = {ADXL345_REG_DATAX0}; // Start register address

    // Open the I2C file
    if ((file = open(I2C_FILE_PATH, O_RDWR)) < 0)
    {
        perror("Failed to open the I2C bus");
        return -1;
    }

    // Connect to the ADXL345 slave
    if (ioctl(file, I2C_SLAVE, ADXL345_SLAVE_ADDR) < 0)
    {
        perror("Failed to connect to the sensor");
        close(file);
        return -1;
    }

    // Set the register pointer to the start of the data registers
    if (write(file, reg_pointer, 1) != 1)
    {
        perror("Failed to set the register pointer");
        close(file);
        return -1;
    }

    // Read 6 bytes of accelerometer data (X0, X1, Y0, Y1, Z0, Z1)
    if (read(file, buffer, 6) != 6)
    {
        perror("Failed to read accelerometer data");
        close(file);
        return -1;
    }

    // Combine high and low bytes to get 16-bit values for each axis
    *acc_x = (buffer[1] << 8) | buffer[0]; // X-axis
    *acc_y = (buffer[3] << 8) | buffer[2]; // Y-axis
    *acc_z = (buffer[5] << 8) | buffer[4]; // Z-axis

    // Close the I2C file
    close(file);

    return 0;
}

int main()
{

	// uart
	    int uart1_filestream = -1;

    // Open the UART1 device file
    uart1_filestream = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);

    if (uart1_filestream == -1) {
        printf("Error - Unable to open UART. Ensure it is not in use by another application\n");
        return -1;
    }

    // Configure the UART
    struct termios options;
    tcgetattr(uart1_filestream, &options);
    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart1_filestream, TCIFLUSH);
    tcsetattr(uart1_filestream, TCSANOW, &options);






    short acc_x, acc_y, acc_z;

    // Initialize the ADXL345
    if (adxl345_init() != 0)
    {
        printf("Failed to initialize the ADXL345\n");
        return -1;
    }

    // Read accelerometer data
    while (1)
    {
        if (adxl345_read(&acc_x, &acc_y, &acc_z) != 0)
        {
            printf("Failed to read accelerometer data\n");
            return -1;
        }

	else
	{

        // Print the accelerometer data
        printf("Accelerometer Data:\n");
        printf("X: %d\n", acc_x);
        printf("Y: %d\n", acc_y);
        printf("Z: %d\n", acc_z);

	if(acc_z < 170)
	{
		unsigned char tx_buffer[20];
    		unsigned char *p_tx_buffer;
    		p_tx_buffer = &tx_buffer[0];
    		strcpy(p_tx_buffer, "Hello");
    		int count = write(uart1_filestream, &tx_buffer[0], strlen(p_tx_buffer));

    		if (count < 0) {
       		 printf("UART TX error\n");
    	}

    // Close the UART
    close(uart1_filestream);
	}

        sleep(1);
	}
    }

    return 0;
}
