#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/uart.h>
#include "includes/i2c_sensors.h"

#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include "includes/flash_utils.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/smf.h>
#include <zephyr/logging/log.h>
#include "includes/utils.h"
#include "includes/BLE_conf.h"

//LOG_MODULE_REGISTER(alog);

/////////////////////////////////////////////////////////////INITS
static struct nvs_fs fs;

#define NVS_PARTITION		storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)






static uint8_t rx_buf[10] = {0}; //A buffer to store incoming UART data
bool recieved = false;


#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};
/////////////////////////////////////////////////////////////////
const struct device *uart1 = DEVICE_DT_GET(DT_NODELABEL(uart0));
const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

sensor *first, *second, *third;



static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type) {
	
	case UART_TX_DONE:

		break;

	case UART_TX_ABORTED:
		// do something
		break;
		
	case UART_RX_RDY:
		recieved = true;
		break;

	case UART_RX_BUF_REQUEST:
		break;

	case UART_RX_BUF_RELEASED:
		break;
		
	case UART_RX_DISABLED:
		uart_rx_enable(dev, rx_buf, sizeof(rx_buf), 100);
		break;

	case UART_RX_STOPPED:
		break;
		
	default:
		break;
	}
}







// Forward declaration of state table
static const struct smf_state app_states[];

// List of application states
enum app_state {
    STATE_INIT,
    STATE_READ_SENSORS,
    STATE_WRITE_TO_FLASH,
    STATE_SEND_BLUETOOTH,
    STATE_SLEEP,
    NUM_STATES
};

// User defined object
struct app_object {
    // This must be first
    struct smf_ctx ctx;

    // Other state specific data can be added here
} app_obj;

// Function prototypes for state entry, run, and exit functions
static void state_init_entry(void *o);
static void state_init_run(void *o);
static void state_read_sensors_run(void *o);
static void state_write_to_flash_run(void *o);
static void state_send_bluetooth_run(void *o);
static void state_sleep_entry(void *o);

// State entry, run, and exit functions

sensor *third;

struct flash_pages_info info;
uint8_t write_buff = 0;
uint8_t rtc_data[7] = {0};

static const bt_addr_t custom_mac = {
    .val = { 0xDB, 0xEA, 0xFD, 0xEE, 0xAD, 0xDE }
};

static void state_init_entry(void *o) {
    k_sleep(K_MSEC(1000));

	int err;
	bt_id_create(&custom_mac, NULL);

    fs.flash_device = NVS_PARTITION_DEVICE;
	if (!device_is_ready(fs.flash_device)) {
		printk("Flash device %s is not ready\n", fs.flash_device->name);
		return ;
	}
	fs.offset = NVS_PARTITION_OFFSET;
	int rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	if (rc) {
		printk("Unable to get page info\n");
		return ;
	}
	fs.sector_size = info.size;
	fs.sector_count = 3U;

	rc = nvs_mount(&fs);
	if (rc) {
		printk("Flash Init failed\n");
		return ;
	}

	sensor_config config_struct_gpio = {
			.adress=0,
			.type = GPIO,
			.num_registers=-1,
			.reg_addr={0},
			._id = 5,
			.channel = adc_channels,
			.value_size = sizeof(int16_t),
			.Pin=AIN1
	};
	third = configure_sensor(&config_struct_gpio);


	sensor_config config_struct_i2c = {
			.adress=0x68,
			.type = I2C,
			.num_registers=2,
			.reg_addr={0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			._id = 0,
			.channel = NULL,
			.value_size = sizeof(uint8_t)
	};
	sensor *first = configure_sensor(&config_struct_i2c);

		
    while(!device_is_ready(i2c_dev))
    {
        printk("device not ready \n");
        k_sleep(K_MSEC(100));
    }

	
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}



	bt_ready();
}

static void state_init_run(void *o) {
    // Transition to the next state
	k_sleep(K_SECONDS(1));
    smf_set_state(SMF_CTX(o), &app_states[STATE_READ_SENSORS]);
}

static void state_read_sensors_run(void *o) {
    int err;
	// Read sensor data
    // Transition to the next state
	read_sensor(i2c_dev, third);
	printk("value from GPIO sensor: %d\n", *( (int32_t *)third->values));
	write_data_fs(&fs, third);
	k_sleep(K_SECONDS(1));

    smf_set_state(SMF_CTX(o), &app_states[STATE_WRITE_TO_FLASH]);
}

static void state_write_to_flash_run(void *o) {
    // Write sensor data to flash
    // Transition to the next state
	//printk("hello from flash run state\n");
	if(connected_check && notif_enabled) smf_set_state(SMF_CTX(o), &app_states[STATE_SEND_BLUETOOTH]);
	else smf_set_state(SMF_CTX(o), &app_states[STATE_SLEEP]);
}

static void state_send_bluetooth_run(void *o) {
    // Send data through Bluetooth
    // Transition to the next state
	
	int err = 0;
	int i =0;
	struct flash_entery *data;

	
	size_t required_ram = third->sucessful_read * sizeof(struct flash_entery);


	data = malloc(required_ram);
		
		if (data == NULL) {
			printk("Failed to allocate memory\n");
			// Handle the error condition
		} else {
			// Use the allocated memory
			printk("\tsuccessful reads for 3rd sensor: %d\n", third->sucessful_read);
			read_history_into(&fs, third, data);
			for (int i = 0; i < third->sucessful_read; i++) {
				
				printk("data values: %d\n", data[i].value);
				// Send data through BLE connection
				int err = bt_gatt_notify(NULL, &sensor_svc.attrs[1], &data[i].value, sizeof(data[i].value));
				uint32_t timestamp = (data[i].timestamp.day << 24) | (data[i].timestamp.hour << 16) | (data[i].timestamp.min << 8) | data[i].timestamp.sec;
				err = bt_gatt_notify(NULL, &sensor_svc.attrs[5], &timestamp, sizeof(uint32_t));
				if (err < 0) {
					printk("Failed to send data over BLE connection (err %d)\n", err);
				}
				// Delete flash entries with the id third->id
				err = delete_entries_by_id(&fs, third->config._id);
				if (err) {
					printk("Failed to delete flash entries (err %d)\n", err);
				}
			}
			free(data);
			}
	
    smf_set_state(SMF_CTX(o), &app_states[STATE_SLEEP]);
}

static void state_sleep_entry(void *o) {
    // Enter sleep mode
	k_sleep(K_SECONDS(2));
    // Implement logic to wake up from sleep
    // Transition to the initial state to repeat the cycle
    smf_set_state(SMF_CTX(o), &app_states[STATE_READ_SENSORS]);
}

// Populate state table
static const struct smf_state app_states[] = {
    [STATE_INIT] = SMF_CREATE_STATE(state_init_entry, state_init_run, NULL),
    [STATE_READ_SENSORS] = SMF_CREATE_STATE(NULL, state_read_sensors_run, NULL),
    [STATE_WRITE_TO_FLASH] = SMF_CREATE_STATE(NULL, state_write_to_flash_run, NULL),
    [STATE_SEND_BLUETOOTH] = SMF_CREATE_STATE(NULL, state_send_bluetooth_run, NULL),
    [STATE_SLEEP] = SMF_CREATE_STATE(state_sleep_entry, NULL, NULL),
};

int main(void) {
    int32_t ret;

    // Set initial state
    smf_set_initial(SMF_CTX(&app_obj), &app_states[STATE_INIT]);

    // Run the state machine
    while(1) {
        // State machine terminates if a non-zero value is returned
        ret = smf_run_state(SMF_CTX(&app_obj));
        if (ret) {
            // Handle return code and terminate state machine if needed
            break;
        }
		k_sleep(K_SECONDS(2));
    }

    return 0;
}
