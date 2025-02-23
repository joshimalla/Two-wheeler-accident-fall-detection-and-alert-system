#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define MOTOR_PIN "60" // Example: P9_12 (Check your BBB pin mapping)

void writeGPIO(const char *filename, const char *value) {
    char path[50];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%s/%s", MOTOR_PIN, filename);
    FILE *f = fopen(path, "w");
    if (f) {
        fprintf(f, "%s", value);
        fclose(f);
    }
}

int main() {
    // Export GPIO
    writeGPIO("export", MOTOR_PIN);
    usleep(100000);

    // Set GPIO as Output
    writeGPIO("direction", "out");

    // Turn Motor ON
    writeGPIO("value", "1");
    printf("Motor ON\n");
    sleep(5); // Run motor for 5 seconds

    // Turn Motor OFF
    writeGPIO("value", "0");
    printf("Motor OFF\n");

    return 0;
}

