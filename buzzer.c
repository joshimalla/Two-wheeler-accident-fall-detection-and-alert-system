#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define GPIO_PIN "44"  // BBB P8_12 corresponds to GPIO 44
#define GPIO_PATH "/sys/class/gpio/gpio" GPIO_PIN "/"

void write_gpio(const char *filename, const char *value) {
    char path[50];
    snprintf(path, sizeof(path), GPIO_PATH "%s", filename);
    FILE *fp = fopen(path, "w");
    if (fp == NULL) {
        perror("Error writing to GPIO");
        return;
    }
    fprintf(fp, "%s", value);
    fclose(fp);
}

void setup_gpio() {
    FILE *export_fp = fopen("/sys/class/gpio/export", "w");
    if (export_fp == NULL) {
        perror("Error exporting GPIO");
        return;
    }
    fprintf(export_fp, GPIO_PIN);
    fclose(export_fp);

    usleep(200000);  // Allow time for the system to create GPIO directory

    write_gpio("direction", "out");  // Set GPIO as output
}

int main() {
    setup_gpio();

    printf("Turning Buzzer ON\n");
    write_gpio("value", "1");
    sleep(5);  // Buzzer ON for 2 seconds

    printf("Turning Buzzer OFF\n");
    write_gpio("value", "0");

    return 0;
}

