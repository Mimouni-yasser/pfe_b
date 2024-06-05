#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/uart.h>
#include "includes/i2c_sensors.h"
#include <zephyr/sys/util.h>
#include <zephyr/pm/device.h>

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

#define CHUNK_SIZE 10 //this is the number of bytes to send at a time, it is useed to prevent running out of RAM and subsequent crash/failure to send.

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

sensor *first, *second, *third, *forth;

sensor *list[4];










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


struct flash_pages_info info;
uint8_t write_buff = 0;
uint8_t rtc_data[7] = {0};


static bt_addr_le_t custom_mac = {
	.a = (bt_addr_t) {0x01, 0x02, 0x03, 0x04, 0x05, 0x06},
	.type = BT_ADDR_LE_RANDOM
};

static void state_init_entry(void *o) {

	sensor_svc = (struct bt_gatt_service) BT_GATT_SERVICE(
		svc_config
	);

    int reg_err = bt_gatt_service_register(&sensor_svc);
	printk("%d", reg_err);


	k_sleep(K_MSEC(1000));



	int err;

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


	sensor_config config_second_gpio = {
			.adress=0,
			.type = GPIO,
			.num_registers=-1,
			.reg_addr={0},
			._id = 5,
			.channel = &adc_channels[4],
			.value_size = sizeof(int16_t),
			.Pin=AIN1
	};
	second = configure_sensor(&config_second_gpio);

	sensor_config config_forth_gpio = {
			.adress=0,
			.type = GPIO,
			.num_registers=-1,
			.reg_addr={0},
			._id = 5,
			.channel = &adc_channels[3],
			.value_size = sizeof(int16_t),
			.Pin=AIN1
	};

	forth = configure_sensor(&config_forth_gpio);

	sensor_config config_struct_i2c = {
			.adress=0x68,
			.type = I2C,
			.num_registers=2,
			.reg_addr={0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			._id = 0,
			.channel = NULL,
			.value_size = sizeof(uint8_t)
	};
	first = configure_sensor(&config_struct_i2c);

		
    while(!device_is_ready(i2c_dev))
    {
        printk("device not ready \n");
        k_sleep(K_MSEC(100));
    }

	
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}

	bt_id_create(&custom_mac, NULL);

	bt_ready();

	list[0] = first;
	list[1] = second;
	list[2] = third;
	list[3] = forth;

	// Transition to the next state
	smf_set_state(SMF_CTX(o), &app_states[STATE_READ_SENSORS]);
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
	err = read_sensor(i2c_dev, third);
	if(err){
		printk("Failed to read sensor data\n");
		smf_set_state(SMF_CTX(o), &app_states[STATE_SLEEP]);
		return;
	}
	printk("value from GPIO sensor: %d\n", *( (int32_t *)third->values));
	write_data_fs(&fs, third);
	k_sleep(K_SECONDS(1));

	err = read_sensor(i2c_dev, first);
	write_data_fs(&fs, first);
	k_sleep(K_SECONDS(1));

	err = read_sensor(i2c_dev, second);
	write_data_fs(&fs, second);
	k_sleep(K_SECONDS(1));

	err = read_sensor(i2c_dev, forth);
	write_data_fs(&fs, forth);
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

	for(uint8_t i = 0; i<4; i++ ){
    size_t total_reads = list[i]->sucessful_read;
    size_t chunks = (total_reads + CHUNK_SIZE - 1) / CHUNK_SIZE;
    struct flash_entery *data = malloc(CHUNK_SIZE * sizeof(struct flash_entery));

    if (data == NULL) {
        printk("Failed to allocate memory\n");
        smf_set_state(SMF_CTX(o), &app_states[STATE_SLEEP]);
        return;
    }

    for (size_t chunk_idx = 0; chunk_idx < chunks; chunk_idx++) {
        // Calculate the number of entries to read in this chunk
		

        size_t entries_to_read = CHUNK_SIZE;
        if (chunk_idx == chunks - 1 && total_reads % CHUNK_SIZE != 0) {
            entries_to_read = total_reads % CHUNK_SIZE;
        }

        // Read the chunk from flash
        read_history_into(&fs, list[i], data, chunk_idx * CHUNK_SIZE, entries_to_read);

        // Send each entry in the chunk over BLE
			for (size_t i = 0; i < entries_to_read; i++) {
				printk("Data values: %d\n", data[i].value);
				int err = bt_gatt_notify(NULL, &sensor_svc.attrs[1], &data[i], sizeof(struct flash_entery));
				if (err < 0) {
					printk("Failed to send data over BLE connection (err %d)\n", err);
					smf_set_state(SMF_CTX(o), &app_states[STATE_SLEEP]);
					free(data);
					return;
				}
			}
		}
		
    free(data);
    list[i]->sucessful_read = 0;
		//delete_entries_with_id(&fs, third->config._id);
    }
    smf_set_state(SMF_CTX(o), &app_states[STATE_SLEEP]);
}

static void state_sleep_entry(void *o) {

	k_sleep(K_SECONDS(3));
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

struct bt_gatt_service config_serv = BT_GATT_SERVICE(svc_config);

int main(void) {
    int32_t ret;

	NRF_P1->PIN_CNF[4] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
						(GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
						(GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
						(GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
						(GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

		k_sleep(K_SECONDS(1));
	if(NRF_P1->IN & (1 << 4))
	{
		int err;
		
		bt_gatt_service_register(&config_serv);
		if(err)
			printk("error during registring of the service");
		printk("entering configuration mode\n");
			//accept configuration from BLE
	
		k_sleep(K_SECONDS(1));
		err = bt_enable(NULL);
		if (err) {
			printk("Bluetooth init failed (err %d)\n", err);
			return;
		}
		k_sleep(K_SECONDS(1));

		


		struct bt_le_adv_param adv_param = {
			.options = BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME,
			.interval_min = BT_GAP_ADV_SLOW_INT_MIN,
			.interval_max = BT_GAP_ADV_SLOW_INT_MAX,
			.id = BT_ID_DEFAULT,
			.sid = 0,
			.peer = NULL,
		};



		err = bt_le_adv_start(&adv_param, ad_conf, ARRAY_SIZE(ad), NULL, 0);
		if (err) {
			printk("Advertising failed to start (err %d)\n", err);
			return;
		}
		k_sleep(K_SECONDS(1));

		printk("configuration ongoing");

		return 0;
	}

    //Set initial state
    smf_set_initial(SMF_CTX(&app_obj), &app_states[STATE_INIT]);

    // Run the state machine
    while(1) {
        // State machine terminates if a non-zero value is returned
        ret = smf_run_state(SMF_CTX(&app_obj));
        if (ret) {
            // Handle return code and terminate state machine if needed
            break;
        }
    }

    return 0;
}
