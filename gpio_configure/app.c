#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#define GPIO_DEV "/dev/gpio_dev"  // The device file for GPIO control

// Command constants
#define GPIO_SET_PIN_HIGH  1
#define GPIO_SET_PIN_LOW   2
#define GPIO_READ_PIN      3

// Struct to hold GPIO command and pin number
typedef struct {
    int command;
    int pin;
} gpio_command_t;

// Print the menu to the user
void print_menu() {
    printf("\nGPIO Control Menu\n");
    printf("1. Set GPIO pin high\n");
    printf("2. Set GPIO pin low\n");
    printf("3. Read GPIO pin state\n");
    printf("4. Read Shorted GPIO pin state\n");
    printf("5. Exit\n");
    printf("Enter your choice: ");
}

int main() {
    int choice;
    int gpio_fd;  // File descriptor for the GPIO device
    int state;

    // Open the GPIO device
    gpio_fd = open(GPIO_DEV, O_RDWR);
    if (gpio_fd == -1) {
        perror("Failed to open GPIO device");
        return 1;
    }

    while (1) {
        // Show the menu
        print_menu();
		char state[2];
        // Get user input for menu choice
        if (scanf("%d", &choice) != 1) {
            printf("Invalid input! Please enter a valid option.\n");
            while(getchar() != '\n');  // Clear the input buffer
            continue;
        }
		char val[2];
        switch (choice) {
            case 1:  // Set GPIO pin high
                strcpy(val, "1");
                if (write(gpio_fd, val, sizeof(val)) == -1) {
                    perror("Failed to set GPIO pin high");
                } else {
                    printf("GPIO pin 22 set to high.\n");
                }
                break;

            case 2:  // Set GPIO pin low
                strcpy(val, "0");
                if (write(gpio_fd, &val, sizeof(val)) == -1) {
                    perror("Failed to set GPIO pin low");
                } else {
                    printf("GPIO pin 22 set to low.\n");
                }
                break;

            case 3:  // Read GPIO pin state
				 strcpy(state, "a");
                 if (read(gpio_fd, &state, sizeof(state)) == -1) {
                    perror("Failed to read GPIO pin state");
                 } else {
                    printf("GPIO pin 22 state is: %s\n", (state[0] == '1') ? "High" : "Low");
                 }
                break;
			case 4: //Read the other GPIO pin state
				 strcpy(state, "b");
                 if (read(gpio_fd, &state, sizeof(state)) == -1) {
                    perror("Failed to read GPIO pin state");
                 } else {
                    printf("GPIO pin 23 state is: %s\n", (state[0] == '1') ? "High" : "Low");
                 }
                break;
            case 5:  // Exit the program
                printf("Exiting the program...\n");
                close(gpio_fd);  // Close the GPIO device
                return 0;

            default:
                printf("Invalid choice! Please enter a number between 1 and 40.\n");
        }
    }

    close(gpio_fd);  // Make sure to close the device file
    return 0;
}
