#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>

#define GPIO_BASE        0xFE200000      // Base address for GPIO on Raspberry Pi 4
#define GPSET0           (GPIO_BASE + 0x1C)  // GPIO Pin Output Set Register
#define GPCLR0           (GPIO_BASE + 0x28)  // GPIO Pin Output Clear Register
#define GPLEV0           (GPIO_BASE + 0x34)  // GPIO Pin Level (Read Register)
#define DEVICE_NAME      "gpio_dev"  // Device name for the driver
#define GPIO_MAJOR       0              // Dynamic major number (0 means kernel will allocate)
#define GPIO_SIZE        0xB4 
#define PIN 22
#define PIN2 23
#define GPIO_PUP_PDN_CNTRL_REG0 (GPIO_BASE + 0xE8)  // Base address for GPIO pull control
#define GPIO22 22
#define GPFSEL2          (GPIO_BASE + 0x08)


struct gpio_command {
    int command;  // Command to set GPIO pin (high/low)
    int pin;      // GPIO pin number (0-40)
};

// Pointer to GPIO register memory locations
volatile unsigned int *gpSet  = NULL;  // Pointer to GPSET0 register
volatile unsigned int *gpClr  = NULL;  // Pointer to GPCLR0 register
volatile unsigned int *gpLev  = NULL;  // Pointer to GPLEV0 register

volatile unsigned int *gpio_base = NULL;
void *gpio_map;
// Declare the device and class
static dev_t gpio_dev_number;
static struct cdev gpio_cdev;
static struct class *gpio_class = NULL;
static struct device *gpio_device = NULL;



// Function to set GPIO pin to high
void gpio_set(int pin) {
    *(gpSet) |= (1 << pin);  // Set the GPIO pin to high
    udelay(2000);
    printk(KERN_INFO "Executed the setting! the value of bit is %d\n", *(gpLev) & (1 << pin));
}

// Function to set GPIO pin to low
void gpio_clear(int pin) {
    *(gpClr) = (1 << pin);  // Set the GPIO pin to low
}

// Function to read GPIO pin state (high or low)
int gpio_read(int pin) {
    printk(KERN_INFO "The state of pin is %d\n", *(gpLev) & (1 << pin));
    return (*(gpLev) & (1 << pin)) ? 1 : 0;  // Read the GPIO pin state
}

// Open function for the GPIO device
static int gpio_open(struct inode *inode, struct file *file) {
    printk(KERN_INFO "GPIO Driver: Device opened\n");
    return 0;
}

// Read function for the GPIO device
static ssize_t gpio_read_file(struct file *file, char __user *buf, size_t len, loff_t *offset) {
    int state;
    if(buf[0] == 'a'){
        state = gpio_read(PIN);
    }
    else{
        state = gpio_read(PIN2);
    }
    char level[2];
    if(state == 1){
        level[0] = '1';
    }
    else{
        level[0] = '0';
    }
    level[1] ='\n';
    printk("State value in gpio_red_file is %d\n", state);
    // Write the state (high or low) to the user-space buffer
    if (copy_to_user(buf, &level, 2)) {
        return -EFAULT;
    }
    
    printk(KERN_INFO "GPIO Driver: Read pin %d, state: %d\n", PIN, state);
    return sizeof(level);
}

// Write function for the GPIO device
static ssize_t gpio_write_file(struct file *file, const char __user *buf, size_t len, loff_t *offset) {
      // Use the file offset to determine the pin
    //int value;

    char value[2];
    
    // Read the command from user-space
    if (copy_from_user(&value, buf, 1)) {
        return -EFAULT;
    }
    //printk(KERN_INFO "Value of pin : %d\n", command.pin);
    if (value[0] == '1') {
        gpio_set(PIN);  // Set the GPIO pin high
    } else {
        gpio_clear(PIN);  // Set the GPIO pin low
    }
    
    printk(KERN_INFO "GPIO Driver: Wrote pin %d, state: %c\n", PIN, value[0]);
    return sizeof(len);
}

// Release function for the GPIO device
static int gpio_release(struct inode *inode, struct file *file) {
    printk(KERN_INFO "GPIO Driver: Device released\n");
    return 0;
}

// File operations structure
static struct file_operations gpio_fops = {
    .owner = THIS_MODULE,
    .open = gpio_open,
    .read = gpio_read_file,
    .write = gpio_write_file,
    .release = gpio_release,
};

// GPIO initialization function
static int __init gpio_driver_init(void) {
    int result;

    printk(KERN_INFO "GPIO Driver: Initializing...\n");

    // Register a character device
    result = alloc_chrdev_region(&gpio_dev_number, 0, 1, DEVICE_NAME);
    if (result < 0) {
        printk(KERN_ERR "GPIO Driver: Failed to allocate a major number\n");
        return result;
    }

    // Create device class
    gpio_class = class_create("gpio_class");
    if (IS_ERR(gpio_class)) {
        unregister_chrdev_region(gpio_dev_number, 1);
        return PTR_ERR(gpio_class);
    }

    // Create device file
    gpio_device = device_create(gpio_class, NULL, gpio_dev_number, NULL, DEVICE_NAME);
    if (IS_ERR(gpio_device)) {
        class_destroy(gpio_class);
        unregister_chrdev_region(gpio_dev_number, 1);
        return PTR_ERR(gpio_device);
    }

    // Initialize the GPIO device
    cdev_init(&gpio_cdev, &gpio_fops);
    gpio_cdev.owner = THIS_MODULE;
    result = cdev_add(&gpio_cdev, gpio_dev_number, 1);
    if (result) {
        device_destroy(gpio_class, gpio_dev_number);
        class_destroy(gpio_class);
        unregister_chrdev_region(gpio_dev_number, 1);
        return result;
    }
    
    
    /** This section sets the pin's pull configuration to none **/
    volatile uint32_t *gpio_pup_pdn_cntrl_reg0 = ioremap(GPIO_PUP_PDN_CNTRL_REG0, sizeof(uint32_t));
    if (!gpio_pup_pdn_cntrl_reg0) {
        pr_err("GPIO Driver: Failed to map GPIO_PUP_PDN_CNTRL_REG0\n");
        return -ENOMEM;
    }
    uint32_t before = readl(gpio_pup_pdn_cntrl_reg0);
    pr_info("GPIO Driver: Before value: 0x%08x\n", before);

    writel(before & ~3 << ((GPIO22 - 16) * 2), gpio_pup_pdn_cntrl_reg0);

    uint32_t after = readl(gpio_pup_pdn_cntrl_reg0);
    pr_info("GPIO Driver: After value: 0x%08x\n", after);

    if(gpio_pup_pdn_cntrl_reg0){
        iounmap(gpio_pup_pdn_cntrl_reg0);
    }
    
    
    
    /** This section configures the pin as a output pin **/
    volatile uint32_t *gpfsel2 = ioremap(GPFSEL2, sizeof(uint32_t));
    if (!gpfsel2) {
        pr_err("GPIO Driver: Failed to map GPFSEL2\n");
    return -ENOMEM;
    }

    uint32_t val = readl(gpfsel2);
    pr_info("GPIO Driver: GPFSEL2 before value: 0x%08x\n", val);

    // Clear the 3 bits for GPIO22 (bits 6-8)
    val &= ~(7 << ((22 % 10) * 3));
    // Set as output (001)
    val |= (1 << ((22 % 10) * 3));

    writel(val, gpfsel2);

    uint32_t aft = readl(gpfsel2);
    pr_info("GPIO Driver: GPFSEL2 after value: 0x%08x\n", aft);

    if(gpfsel2) {
        iounmap(gpfsel2);
    }
    
    
    
    // Map the GPIO registers into kernel space memory
    //set_pull_state(22, GPIO_PULL_NONE);
    gpio_map = ioremap(GPIO_BASE, GPIO_SIZE);  // Use ioremap to map GPIO base to virtual address
    if (!gpio_map) {
        printk(KERN_ERR "GPIO Driver: Error mapping GPIO registers\n");
        return -ENOMEM;
    }

    // Assign register pointers
    gpSet = (volatile unsigned int *)(gpio_map + 0x1C);  // GPSET0
    gpClr = (volatile unsigned int *)(gpio_map + 0x28);  // GPCLR0
    gpLev = (volatile unsigned int *)(gpio_map + 0x34);  // GPLEV0

    printk(KERN_INFO "GPIO Driver: Initialized successfully\n");
    return 0;  // Success
}

// GPIO cleanup function
static void __exit gpio_driver_exit(void) {
    printk(KERN_INFO "GPIO Driver: Cleaning up...\n");

    // Remove device and class
    device_destroy(gpio_class, gpio_dev_number);
    class_destroy(gpio_class);
    if (gpio_map) {
        iounmap(gpio_map);
    }
    // Unregister character device
    cdev_del(&gpio_cdev);
    unregister_chrdev_region(gpio_dev_number, 1);

    printk(KERN_INFO "GPIO Driver: Cleanup complete\n");
}

// Register the driver functions
module_init(gpio_driver_init);  // Function to call during module loading
module_exit(gpio_driver_exit);  // Function to call during module unloading

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sireesha");
MODULE_DESCRIPTION("Simple GPIO Driver for Raspberry Pi with file operations");
