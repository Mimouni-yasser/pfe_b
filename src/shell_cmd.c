
#ifdef qvdsfbdg
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <string.h>


static int cmd_devices(const struct shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "got argc = %d", argc);
    if (argc < 12) {
        shell_print(shell, "Usage: devices -i interface -p pin -a i2c_address -id id -r {registers 0 to 4 values}");
        shell_print(shell, "Example: devices -i i2c -p 1 -a 0x50 -id 1 -r 0x10 0x20 0x30 0x40 0x50");
        return -EINVAL;
    }

    const char *interface = NULL;
    uint32_t pin = 0;
    uint32_t i2c_address = 0;
    uint32_t id = 0;
    uint32_t registers[5] = {0};

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            interface = argv[++i];
        } else if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
            pin = strtoul(argv[++i], NULL, 10);
        } else if (strcmp(argv[i], "-a") == 0 && i + 1 < argc) {
            i2c_address = strtoul(argv[++i], NULL, 16);
        } else if (strcmp(argv[i], "-id") == 0 && i + 1 < argc) {
            id = strtoul(argv[++i], NULL, 10);
        } else if (strcmp(argv[i], "-r") == 0 && i + 3 < argc) {
            for (int j = 0; j < 5; j++) {
                registers[j] = strtoul(argv[++i], NULL, 16);
            }
        } else {
            shell_print(shell, "Invalid argument: %s", argv[i]);
            return -EINVAL;
        }
    }

    // Handle the device configuration here
    shell_print(shell, "Configuring device:");
    shell_print(shell, "Interface: %s", interface);
    shell_print(shell, "Pin: %d", pin);
    shell_print(shell, "I2C Address: 0x%x", i2c_address);
    shell_print(shell, "ID: %d", id);
    shell_print(shell, "Registers: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
                registers[0], registers[1], registers[2], registers[3], registers[4]);

    // Add your device configuration code here

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_devices,
    SHELL_CMD_ARG(devices, NULL, "Configure device\nUsage: devices -i interface -p pin -a i2c_address -id id -r {registers 0 to 4 values}", cmd_devices, 11, 3),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(config, &sub_devices, "Device configuration commands", NULL);
#endif

