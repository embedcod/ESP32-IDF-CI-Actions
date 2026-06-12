/***************************************************************************************************************************************
∎ PROJECT: Water Quality Monitor
∎ SOFTWARE: Engineer Eric Mulwa
	 ≈ EMAIL: mulwaericbsc@gmail.com
	 ≈ PHONE: 0796456877
	 ≈ DATE:  31/07/2025
	 ≈ UPDATED: 04/08/2025
	  
	 ⇉ OWNERSHIP: Jack MacLeans
	 ⇉ LICENSE:   Proprietary - All rights reserved

***************************************************************************************************************************************/

#include <stdio.h>
#include <esp_system.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_adc/adc_oneshot.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <esp_system.h>
#include "esp_mac.h"
#include <time.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/uart.h"
#include <esp_system.h>
#include <esp_sleep.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_attr.h"
#include "mqtt.h"
#include "esp_err.h"

/*******************************************************************************************************************************
DEFINITION: PUBLISHER RING BUFFER
**********************************/
// PUBLISHER RING BUFFER
#define MAX_COMMANDS  	  30   		  // Maximum commands in ring buffer
#define MAX_SLAVES        30   	      // Max expected sensors per scan
#define MAX_DATA_POINTS   MAX_SLAVES  // One entry per sensor per read cycle
typedef struct { 
	int     Sensor_Address;
	float   Sensor_Value;
} SlaveData;
int Battery_Capacity = 100;
// Array to store slave data
SlaveData ringBuffer[MAX_DATA_POINTS];
int writeIndex = 0;
int dataCount = 0;           // Number of valid entries in the buffer
bool bufferReady = false;    // Flag to trigger publishing task
// Temporary slave data storage while reading
SlaveData temp_slave_data;

/*******************************************************************************************************************************/

/*******************************************************************************************************************************
DEFINITION: SUBSCRIBER - SLAVES_WRITE RTOS QUEUE
***********************************************/
SemaphoreHandle_t xRS485_Mutex;
#define MAX_COMMANDS 30

/*******************************************************************************************************************************/

/*******************************************************************************************************************************/
// Pin definitions
#define BUTTON_PIN 18
#define LED_PIN 12
#define RSTX_PIN 22 // RS-485 TX
#define RSRX_PIN 21 // RS-485 RX
#define RI_PIN 32
#define BUZZER_PIN 19
#define POWER_PIN 15
#define SENSE_PIN 14
#define RS485_EN 13

/*******************************************************************************************************************************/

/*******************************************************************************************************************************/
// Button Definitions
#define DEBOUNCE_TIME_MS 300      
#define MULTI_PRESS_DELAY_MS 4000  
#define MIN_PRESS_INTERVAL_MS 300 
/*******************************************************************************************************************************/

/*******************************************************************************************************************************/
// Global variables
#define TAG "RS485"
volatile int buttonPressCount = 0;  
volatile bool interruptFlag = false;
TimerHandle_t pressTimer;      
static uint32_t storedChipId = 0;
RTC_DATA_ATTR int PowerOn = 0;
RTC_DATA_ATTR int firstboot = 0;
unsigned long lastHeartbeatTime = 0;
/*******************************************************************************************************************************/

/*******************************************************************************************************************************
WATCHDOG CONSTANTS
******************/
#define WATCHDOG_TIMEOUT_MS 120000  // 2 minutes
//TimerHandle_t watchdogTimer; 
TimerHandle_t watchdogCore0;
TimerHandle_t watchdogCore1;
/******************************************************************************************************************************/

/*******************************************************************************************************************************
INIT: PWM LED
********************************/
// PWM Configuration Constants
#define LEDC_TIMER            LEDC_TIMER_0
#define LEDC_MODE             LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_BUZZER   LEDC_CHANNEL_0
#define LEDC_CHANNEL_LED      LEDC_CHANNEL_1
#define LEDC_DUTY_RES         LEDC_TIMER_13_BIT 
#define LEDC_FREQUENCY        3000    

// Duty Cycle Constants for LED
#define DUTY_5_PERCENT        (410)    
#define DUTY_10_PERCENT       (819)
#define DUTY_50_PERCENT       (4095)
#define DUTY_25_PERCENT       (2047) 
#define DUTY_20_PERCENT       (1638) 
#define DUTY_100_PERCENT      (8191)
#define FADE_STEP             (50)
#define FADE_DELAY_MS         (10) 
/*******************************************************************************************************************************/


/*******************************************************************************************************************************
INIT: VOLTAGE MONITOR
********************************/
// Battery Voltage Monitor Variables and Thresholds
const float MIN_VOLTAGE = 2;
const float MAX_VOLTAGE = 4.19;                                  
const gpio_num_t analogPin = GPIO_NUM_35;      
const float R1 = 2700.0;   
const float R2 = 10000.0; 
const float Vref = 3.3;   
int batteryLevel = 0;   
adc_oneshot_unit_handle_t adc_handle;    
/******************************************************************************************************************************/


/*******************************************************************************************************************************
DEFINITIONS: UART INITIALIZATIONS
********************************/
// UART1 Initialization (RS-485)
#define UART_PORT_NUM      UART_NUM_2
#define TXD_PIN            GPIO_NUM_21
#define RXD_PIN            GPIO_NUM_22
#define RTS_PIN            UART_PIN_NO_CHANGE
#define CTS_PIN            UART_PIN_NO_CHANGE
#define BUFF_SIZE           256
#define BAUD_RATE          9600
#define READ_TIMEOUT_MS    1000

#define RS485_MIN_ADDRESS 0
#define RS485_MAX_ADDRESS 5
#define RS485_BASE_ADDRESS 5 // Default base for assigning new addresses
int32_t next_available_address = RS485_BASE_ADDRESS;
volatile bool provision = false;

// UART2 Definitions (MQTTs)
#define PUBLISH_INTERVAL (2 * 60 * 1000) 
char APN[32] = "safaricomiot"; 
unsigned long lastPublishTime = 0; 
/*******************************************************************************************************************************/

/***************************************************************************************************************************************
FUNCTIONS: PROTOTYPES
**********************/
void setupWatchdogs();
void resetWatchdog(int core_id);
void watchdogCallback(TimerHandle_t xTimer);
void setupButton();
void initializeTimer();
void buttonPressHandler();
void pressTimerCallback(TimerHandle_t xTimer);
void handleButtonPress();
void initializeSystem();
void ReadSlaves();
void powerOnSequence();
void resetDevice();
void shutdownDevice();
void provisioning();
// Asterisk Separator
void printAsteriskSeparator(int length);
// VOLTAGE MONITOR FUNCTIONS
void setup_pins();
float readBatteryVoltage();
void updateBatteryLevel();
void checkLowBatteryAndShutdown();
void battery_monitor();
// PWM LED AND BUZZER FUNCTIONS
void setupled();
void error(int index);
void heartbeat();
void ledfade(int index, int delay);
void flashled(int index, int duration, int delay);
// DEVICE POWER DOWN
void powerdown();
// CHIP MAC {Serial Number}
uint32_t getChipId();
void getChipIdString(char *deviceSerial, size_t size);
// RS-485 Functions
void rs485_uart_init();
void nvs_init();
void store_slave_data();
char* create_json_payloadi(int index);
uint16_t calculate_crc(uint8_t *data, uint8_t length);
int send_read_ph_command(uint8_t address, uint16_t register_addr, uint16_t register_count);
int scan_rs485_bus(uint8_t start_addr, uint8_t end_addr, uint8_t *found_addresses, int max_found);
esp_err_t change_sensor_address(uint8_t old_addr, uint8_t new_addr);
float receive_ph_value();
void read_ph_sensor();
void provision_rs485_sensor();
// NVS
int32_t load_next_address_from_nvs();
void save_next_address_to_nvs(int32_t address);
/*******************************************************************************************************************************/
// RTOS Tasks
void create_queue_mutex();
void mqtts_task(void *arg);
void read_task(void *param);
void write_task(void *param);
/*******************************************************************************************************************************/


/*******************************************************************************************************************************
FUNCTION: BUTTON INTERRUPT HANDLER
*********************************/
static void IRAM_ATTR button_isr_handler(void* arg) {
    static unsigned long lastInterruptTime = 0;
    static unsigned long lastPressTime = 0;
    unsigned long interruptTime = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;

    if (interruptTime - lastInterruptTime > DEBOUNCE_TIME_MS) {
        if (interruptTime - lastPressTime > MIN_PRESS_INTERVAL_MS) {
            buttonPressCount++;
            xTimerResetFromISR(pressTimer, NULL);
            lastPressTime = interruptTime;
        }
    }
    lastInterruptTime = interruptTime;
}

void handleButtonPress() { 
        if (interruptFlag) {
            switch (buttonPressCount) {
                case 1:
                    flashled(6, 200, 200);
                    //resetDevice();
                    provision = true;
                    vTaskDelay(pdMS_TO_TICKS(100));
                    break;
                case 2:
                    flashled(4, 2000, 500);
                    shutdownDevice();
                    vTaskDelay(pdMS_TO_TICKS(200));
                    break;
                default: // long press to provision
                    ledfade(6, 10);
                    provision = true;
                    vTaskDelay(pdMS_TO_TICKS(200));           
                    break;
            }
            interruptFlag = false;  
            buttonPressCount = 0;  
        }
	}
/*******************************************************************************************************************************/


/*******************************************************************************************************************************
FUNCTION: MAIN APPLICATION
**************************/
void app_main() {
	powerOnSequence();
    initializeSystem();
    heartbeat();
    resetWatchdog(0);
    resetWatchdog(1);
    
	// MQTTs Task pinned to core 0
    xTaskCreatePinnedToCore(mqtts_task, "mqtts_task", 4096, NULL, 3, NULL, 0);	    
	// RS485 Read & Write Tasks pinned to core 1
    xTaskCreatePinnedToCore(write_task, "write_task", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(read_task, "read_task", 4096, NULL, 2, NULL, 1);
}
/*******************************************************************************************************************************/


/*******************************************************************************************************************************
RTOS: TASKS
**************************/
void write_task(void *param) {
    while (1) {
		heartbeat(); // In write task only
		resetWatchdog(1);
		vTaskDelay(pdMS_TO_TICKS(100));
		  if (interruptFlag) {
	          handleButtonPress();
	      }
		if (provision) {
            if (xSemaphoreTake(xRS485_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
				printf("Acquired RS485 Bus Mutex, proceeding with sensor provisioning...\n");
                heartbeat();
                resetWatchdog(1);				
				provisioning();
                heartbeat();
                resetWatchdog(1);
                provision = false;
                vTaskDelay(pdMS_TO_TICKS(100));
                xSemaphoreGive(xRS485_Mutex); // Release mutex after processing

            } else {
		            ESP_LOGW(TAG, "Failed to acquire RS485 mutex, waiting...");
		        }
		  }
          heartbeat();
          resetWatchdog(1);
        // Yield CPU to lower-priority tasks with better latency
        vTaskDelay(pdMS_TO_TICKS(100));  
    }
}

void read_task(void *param) {
    while (1) {
		resetWatchdog(1);
		if (!bufferReady) { // check if its time to populate the publisher buffer
				printf("Publisher buffer empty, reading data from the sensors...\n");
		         // Try to take the mutex without blocking
		        if (xSemaphoreTake(xRS485_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
					printf("Acquired RS485 Bus Mutex, proceeding to read data...\n");
		            ReadSlaves();
		            resetWatchdog(1);
		            // Give back the mutex immediately
		            xSemaphoreGive(xRS485_Mutex);
		        } else {
		            ESP_LOGW(TAG, "Failed to acquire RS485 mutex for reading, waiting...");
		        }
		    } 
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void mqtts_task(void *arg) {
    resetWatchdog(0);
	if (!mqtt_client()) {
		error(3);
	    ESP_LOGE(TAG_GSM, "mqtt_subclient execution failed!");
	    powerdown_modem();
	    esp_restart();
    } else {
        resetWatchdog(0);
	    printf("Entering MQTT RPC URC Handler loop...\n");
	    vTaskDelay(pdMS_TO_TICKS(100));
	    while (1) {
        resetWatchdog(0);
        // Check for incoming messages (non-blocking)
        check_mqtt_urc();   
        vTaskDelay(pdMS_TO_TICKS(100));
    
	    unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
	    if (firstboot == 0) { 
			firstboot = 1;
			vTaskDelay(pdMS_TO_TICKS(100));
		    resetWatchdog(0);
			if (bufferReady) {
				check_mqtt_link_status();
				printf("Publisher buffer ready, emptying....\n");
		 		if (!mqtt_pubclient()) {
					error(3);
				    ESP_LOGE(TAG_GSM, "mqtt_pubclient failed!\n");
			    } else {	
				    printf("mqtt_pubclient executed successfully!\n");
		            resetWatchdog(0);
			    }
		        lastPublishTime = currentTime; 
			    vTaskDelay(pdMS_TO_TICKS(20));  
			    check_mqtt_link_status(); // mostly active with the dummypub running
			    vTaskDelay(pdMS_TO_TICKS(20));     					
			}     
		 } else if ((currentTime - lastPublishTime) >= PUBLISH_INTERVAL) {
		    resetWatchdog(0);
			if (bufferReady) {
				check_mqtt_link_status();
				printf("Publisher buffer ready, emptying....\n");
		 		if (!mqtt_pubclient()) {
					error(3);
				    ESP_LOGE(TAG_GSM, "mqtt_pubclient failed!\n");
			    } else {	
				    printf("mqtt_pubclient executed successfully!\n");
		            resetWatchdog(0);
			    }
		        lastPublishTime = currentTime; 
			    vTaskDelay(pdMS_TO_TICKS(20));  
			    check_mqtt_link_status(); // mostly active with the dummypub running
			    vTaskDelay(pdMS_TO_TICKS(20));     					
			}   
   		 }   
	    }
    }
} 


/************************************************************************************************************************************** 
FUNCTIONS: INITIALIZATION
************************/

 /**
 * @brief Timer callback for processing button presses
 */
void pressTimerCallback(TimerHandle_t xTimer) {
    interruptFlag = true;               // Set flag to process button press in main loop
}

/**
 * @brief Initialize Button with Interrupt
 */
void setupButton() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;   
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ONLY;
    gpio_config(&io_conf);
    // Install ISR service and add button interrupt
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);
}

/**
 * @brief Initialize Timer for handling multi-press delay
 */
void initializeTimer() {
    pressTimer = xTimerCreate("PressTimer", MULTI_PRESS_DELAY_MS / portTICK_PERIOD_MS, pdFALSE, NULL, pressTimerCallback);
}

/**
 * @brief Get chip ID based on the MAC address
 */
uint32_t getChipId() {
    if (storedChipId == 0) {
        uint8_t mac[6];
        esp_base_mac_addr_get(mac); 

        for (int i = 0; i < 6; i++) {
            storedChipId |= ((uint32_t)mac[i] << (8 * i));
        }
    }
    return storedChipId;
}

/**
 * @brief Get the chip ID as a string
 */
void getChipIdString(char *deviceSerial, size_t size) {
    uint32_t chipId = getChipId();
    snprintf(deviceSerial, size, "%" PRIu32, chipId);  
    printf("Device Serial Number (Chip ID): %s\n", deviceSerial); 
}

// JSON data creation for publisher
char* create_json_payloadi(int index) { 
    if (index >= MAX_DATA_POINTS) return NULL; // Out of bounds check
    printf("Creating flat JSON data...\n");
    char prefix[16];
    snprintf(prefix, sizeof(prefix), "S%d", index + 1);
    cJSON *root = cJSON_CreateObject();
    char key[64];
    cJSON_AddNumberToObject(root, "Battery", Battery_Capacity);
    snprintf(key, sizeof(key), "%sAddress", prefix);
    cJSON_AddNumberToObject(root, key, ringBuffer[index].Sensor_Address);
    snprintf(key, sizeof(key), "%sValue", prefix);
    cJSON_AddNumberToObject(root, key, ringBuffer[index].Sensor_Value);
    char deviceSerial[12];
    getChipIdString(deviceSerial, sizeof(deviceSerial));
    cJSON_AddStringToObject(root, "Serial Number", deviceSerial);
    char *jsonString = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return jsonString;
}

// Store slave data in the struct
void store_slave_data(int slave_address) {
    if (slave_address < MAX_DATA_POINTS) { 
        ringBuffer[slave_address].Sensor_Address               	= temp_slave_data.Sensor_Address;
        ringBuffer[slave_address].Sensor_Value                  = temp_slave_data.Sensor_Value;
        //writeIndex++;
        dataCount++;
    }
}

/*******************************************************************************************************************************/


/*******************************************************************************************************************************
FUNCTION: WATCHDOG SETUP
************************/
void setupWatchdogs() {
    watchdogCore0 = xTimerCreate("WDT_Core0", pdMS_TO_TICKS(WATCHDOG_TIMEOUT_MS), pdFALSE, (void *)0, watchdogCallback);
    watchdogCore1 = xTimerCreate("WDT_Core1", pdMS_TO_TICKS(WATCHDOG_TIMEOUT_MS), pdFALSE, (void *)1, watchdogCallback);

    if (watchdogCore0 == NULL || watchdogCore1 == NULL) {
        printf("Failed to create one or both watchdog timers\n");
    }
}

void resetWatchdog(int core_id) {
    TimerHandle_t target = (core_id == 0) ? watchdogCore0 : watchdogCore1;

    if (xTimerReset(target, 0) != pdPASS) {
        printf("Failed to reset watchdog for core %d\n", core_id);
    }
}

void watchdogCallback(TimerHandle_t xTimer) {
    int core_id = (int)pvTimerGetTimerID(xTimer);
    printf("Watchdog for core %d expired! Restarting system...\n", core_id);

    // Optional: log system state or dump error report before restart
    powerdown_modem(); // Power down Quectel before reset
    esp_restart();
}



/**
 * @brief Print asterisk separator in debug messages
 */
void printAsteriskSeparator(int length) {
    char separator[length + 1]; 
    for (int i = 0; i < length; i++) {
        separator[i] = '*'; 
    }
    separator[length] = '\0'; 
    printf("\n");
    printf("%s", separator);
    printf("\n");
}

/*******************************************************************************************************************************/


/*******************************************************************************************************************************
FUNCTIONS: SENSOR FUNCTIONS
********************************/



/**
 * @brief Initialize GPIOs and ADC
 */
void setup_pins() {
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1
    };
    adc_oneshot_new_unit(&unit_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t channel_cfg = {
        .atten = ADC_ATTEN_DB_12,                             
        .bitwidth = ADC_BITWIDTH_12             
    };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_7, &channel_cfg); 
}

/**
 * @brief Read the battery voltage
 */ 
float readBatteryVoltage() {
	esp_rom_gpio_pad_select_gpio(SENSE_PIN);                       
    gpio_set_direction(SENSE_PIN, GPIO_MODE_OUTPUT);   
    gpio_set_level(SENSE_PIN, 1);
	vTaskDelay(pdMS_TO_TICKS(1000));   
   
	int adcValue;
	adc_oneshot_read(adc_handle, ADC_CHANNEL_7, &adcValue);
	float voltage = (adcValue / 4095.0) * Vref;         
	voltage = voltage * ((R1 + R2) / R2);

	esp_rom_gpio_pad_select_gpio(SENSE_PIN);                       
    gpio_set_direction(SENSE_PIN, GPIO_MODE_OUTPUT); 
    gpio_set_level(SENSE_PIN, 0);         
    printf("Read Voltage: %.2f V\n", voltage);
    return voltage;
}

/**
 * @brief Update battery level based on voltage
 */
void updateBatteryLevel() {
    float voltage = readBatteryVoltage();
    if (voltage <= MIN_VOLTAGE) {
        batteryLevel = 0;
    } else if (voltage >= MAX_VOLTAGE) {
        batteryLevel = 100;
    } else {
        float voltageRange = MAX_VOLTAGE - MIN_VOLTAGE;
        float percentagePerVolt = 100.0 / voltageRange;
        batteryLevel = ((voltage - MIN_VOLTAGE) * percentagePerVolt) / 5 * 5;
    }
    Battery_Capacity = batteryLevel;
    printf("Battery Capacity: %d%%\n", batteryLevel);
}

/**
 * @brief Check for low battery and shutdown if voltage is below minimum
 */
void checkLowBatteryAndShutdown() {
	printf("Checking if low battery and shutdown...\n");
    float voltage = readBatteryVoltage();                     

    if (voltage < MIN_VOLTAGE) {
        printf("Battery voltage critically low. Shutting down...\n");
       vTaskDelay(pdMS_TO_TICKS(1000));    
        powerdown();
        //while (voltage < MIN_VOLTAGE) { TODO
		//	vTaskDelay(pdMS_TO_TICKS(500));
		//}
    }
}

/**
 * @brief Power Off
 */
void powerdown() {
		powerdown_modem();
	    esp_rom_gpio_pad_select_gpio(POWER_PIN);
		gpio_set_direction(POWER_PIN, GPIO_MODE_OUTPUT);
		gpio_set_level(POWER_PIN, 1);	
	    vTaskDelay(pdMS_TO_TICKS(1000)); 
}

/**
 * @brief Main battery monitoring task
 */
void battery_monitor() {
	printf("Executing battery monitoring task...\n");
            updateBatteryLevel();                              
            checkLowBatteryAndShutdown();    
}

/////////////////////////
// PWM LED Functions  ///
/////////////////////////

// Configure PWM for LED
void setupled() {
	printf("setting led as PWM...\n");
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,  
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_LED,
        .duty       = 0,
        .gpio_num   = LED_PIN,
        .speed_mode = LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER
    };
    ledc_channel_config(&ledc_channel);
}

// Heartbeat
void heartbeat() {
    unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if ((currentTime - lastHeartbeatTime) >= 5000) {  // 5000 ms = 5 seconds
        flashled(1, 200, 100); 
        lastHeartbeatTime = currentTime;
    }
}

// Error indicator
void error(int index) {
    flashled(index, 200, 500);
}

// LED flash index times at 100% duty cycle
void flashled(int index, int duration, int delay) {
    for (int i = 0; i < index; i++) {  
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LED, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LED);
        vTaskDelay(pdMS_TO_TICKS(duration));
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LED, DUTY_100_PERCENT);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LED);
        vTaskDelay(pdMS_TO_TICKS(delay));
    }
}

// Smoothly transition LED brightness from 0% to 100% and back to 0% index times
void ledfade(int index, int delay) {
    for (int i = 0; i < index; i++) {  
        for (int duty = 0; duty <= DUTY_100_PERCENT; duty += FADE_STEP) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LED, duty);     
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LED);
            vTaskDelay(pdMS_TO_TICKS(delay));
        }
        for (int duty = DUTY_100_PERCENT; duty >= 0; duty -= FADE_STEP) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LED, duty);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LED);
            vTaskDelay(pdMS_TO_TICKS(delay));
        }
    }
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LED, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LED);
}


void create_queue_mutex() {
    // Create mutex
    xRS485_Mutex = xSemaphoreCreateMutex();
    if (xRS485_Mutex == NULL) {
		error(3);
        ESP_LOGE(TAG, "Failed to create RS485 mutex");
    }
}

/**
 * @brief Initialize NVS
 */
void nvs_init() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    vTaskDelay(pdMS_TO_TICKS(200));   
}

/**
 * @brief Initialize UART for RS-485 communication
 */
void rs485_uart_init() {
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUFF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, RTS_PIN, CTS_PIN));
    ESP_ERROR_CHECK(uart_set_mode(UART_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX));
    
    ESP_LOGI(TAG, "UART initialized for RS485 communication");
}

/**
 * @brief Calculate Modbus RTU CRC16
 */
uint16_t calculate_crc(uint8_t *data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    
    for (uint8_t pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)data[pos];
        
        for (uint8_t i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}


/**
 * @brief Send Modbus RTU read command
 * @param address Modbus device address
 * @param register_addr Starting register address
 * @param register_count Number of registers to read
 * @return Number of bytes sent
 */
int send_read_command(uint8_t address, uint16_t register_addr, uint16_t register_count) {
    uint8_t command[8];
    
    // Build Modbus RTU command
    command[0] = address;                // Slave address
    command[1] = 0x03;                   // Function code (read holding registers)
    command[2] = (register_addr >> 8) & 0xFF; // Starting address high byte
    command[3] = register_addr & 0xFF;    // Starting address low byte
    command[4] = (register_count >> 8) & 0xFF; // Quantity high byte
    command[5] = register_count & 0xFF;   // Quantity low byte
    
    // Calculate CRC
    uint16_t crc = calculate_crc(command, 6);
    command[6] = crc & 0xFF;             // CRC low byte
    command[7] = (crc >> 8) & 0xFF;      // CRC high byte
    
    // Send command
    return uart_write_bytes(UART_PORT_NUM, (const char*)command, sizeof(command));
}

/**
 * @brief Receive and parse Modbus response
 * @param expected_address Expected device address in response
 * @param[out] raw_value Pointer to store the raw 16-bit register value
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t receive_modbus_response(uint8_t expected_address, uint16_t *raw_value) {
    uint8_t data[BUFF_SIZE] = {0};
    int len = uart_read_bytes(UART_PORT_NUM, data, BUFF_SIZE - 1, pdMS_TO_TICKS(READ_TIMEOUT_MS));
    
    if (len <= 0) {
        ESP_LOGE(TAG, "No response from sensor");
        return ESP_FAIL;
    }
    
    // Print raw response for debugging
    char buffer[3 * len + 1];
    for (int i = 0; i < len; i++) {
        sprintf(buffer + 3*i, "%02X ", data[i]);
    }
    ESP_LOGI(TAG, "Raw response (%d bytes): %s", len, buffer);
    
    // Validate response length
    if (len != 7) {  // Expected: [Addr][Func][ByteCount][DataHi][DataLo][CRC Lo][CRC Hi]
        ESP_LOGE(TAG, "Unexpected response length: %d", len);
        return ESP_FAIL;
    }
    if (len < 2) {  // Expected: [Addr][Func][ByteCount][DataHi][DataLo][CRC Lo][CRC Hi]
        ESP_LOGE(TAG, "Unexpected response length: %d", len);
        return ESP_FAIL;
    }
    
    // Verify address matches
    if (data[0] != expected_address) {
        ESP_LOGE(TAG, "Address mismatch (expected %02X, got %02X)", expected_address, data[0]);
        return ESP_FAIL;
    }
    
    // Verify CRC
    uint16_t received_crc = (data[len-1] << 8) | data[len-2];
    uint16_t calculated_crc = calculate_crc(data, len-2);
    
    if (received_crc != calculated_crc) {
        ESP_LOGE(TAG, "CRC error (Rx: %04X, Calc: %04X)", received_crc, calculated_crc);
        return ESP_FAIL;
    }
    
    // Extract raw value (bytes 3 and 4)
    *raw_value = (data[3] << 8) | data[4];
    return ESP_OK;
}

/**
 * @brief Read pH value from sensor at given address via Modbus
 * @param address Modbus address of the sensor
 * @return pH value on success, -1.0 on error
 */
float read_sensor(uint8_t address) {
    const uint16_t PH_REGISTER_ADDR = 0x0000;   // Register for pH value
    const uint16_t REG_COUNT = 0x0001;          // Reading one register

    // Clear UART RX buffer
    uint8_t flush;
    while (uart_read_bytes(UART_PORT_NUM, &flush, 1, pdMS_TO_TICKS(10)) > 0);

    // Send Modbus FC03 Read Holding Register command
    int sent = send_read_command(address, PH_REGISTER_ADDR, REG_COUNT);
    if (sent != 8) {
        ESP_LOGE(TAG, "Failed to send read command to address %02X", address);
        return -1.0f;
    }
    uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(200)); // Give sensor time to respond

    // Receive and parse response
    uint16_t raw_value = 0;
    if (receive_modbus_response(address, &raw_value) == ESP_OK) {
        float ph_value = raw_value / 100.0f;
        ESP_LOGI(TAG, "Sensor [%02X] pH: %.2f", address, ph_value);
        return ph_value;
    } else {
        ESP_LOGW(TAG, "No valid response from sensor [%02X]", address);
        return -1.0f;
    }
}

/**
 * @brief Scan RS485 bus for active devices
 * @param start_addr Starting address to scan (typically 0x01)
 * @param end_addr Ending address to scan (typically 0xF7)
 * @param found_addresses Array to store found addresses
 * @param max_found Maximum number of addresses to store
 * @return Number of devices found
 */
int scan_rs485_bus(uint8_t start_addr, uint8_t end_addr, uint8_t *found_addresses, int max_found) {
    int found_count = 0;
    //uint16_t dummy_value;
    
    ESP_LOGI(TAG, "Starting RS485 bus scan from %02X to %02X", start_addr, end_addr);
    
    for (uint8_t addr = start_addr; addr <= end_addr && found_count < max_found; addr++) {
        // Try reading a register (address 0x0000 which is commonly available)
        if (read_sensor(addr) >= 0) {
            found_addresses[found_count++] = addr;
            ESP_LOGI(TAG, "Found device at address %02X", addr);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Short delay between probes
    }
    
    ESP_LOGI(TAG, "Scan complete. Found %d devices", found_count);
    return found_count;
}

/**
 * @brief Change Modbus device address
 * @param old_addr Current device address (must be <5)
 * @param new_addr New device address (must be >=5 and <=247)
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t change_sensor_address(uint8_t old_addr, uint8_t new_addr) {
    // Validate addresses 
    if (old_addr >= 5) {
        ESP_LOGE(TAG, "Old address must be <5 (got %02X)", old_addr);
        return ESP_FAIL;
    }
    if (new_addr < 5 || new_addr > 247) {
        ESP_LOGE(TAG, "New address must be >=5 and <=247 (got %02X)", new_addr);
        return ESP_FAIL;
    }
    // Modbus command to change address (function code 0x06 - write single register)
    uint8_t command[8] = {
        old_addr,       // Current device address
        0x06,          // Function code (write single register)
        0x00, 0x00,    // Register address for device address TODO
        0x00, new_addr, // New address value
        0x00, 0x00     // Placeholder for CRC (calculated below)
    };   
    // Calculate CRC for the command
    uint16_t crc = calculate_crc(command, 6);
    command[6] = crc & 0xFF;
    command[7] = (crc >> 8) & 0xFF;
    // Clear RX buffer
    uint8_t dummy;
    while (uart_read_bytes(UART_PORT_NUM, &dummy, 1, pdMS_TO_TICKS(10)) > 0) {}
    // Send command
    ESP_LOGI(TAG, "Sending address change command: %02X -> %02X", old_addr, new_addr);
    uart_write_bytes(UART_PORT_NUM, (const char*)command, sizeof(command));
    uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(100));  
    // Wait for response
    vTaskDelay(pdMS_TO_TICKS(200)); 
    // Read and verify response
    uint8_t response[8];
    int len = uart_read_bytes(UART_PORT_NUM, response, sizeof(response), pdMS_TO_TICKS(200));   
    /*if (len != 8) {
        ESP_LOGE(TAG, "Invalid response length: %d", len);
        return ESP_FAIL;
    }*/  
	if (len < 1) {
	    ESP_LOGE(TAG, "Response too short: %d", len);
	    return ESP_FAIL;
	}
       
    /*/ Verify response matches what we sent (echo back)
    if (memcmp(command, response, 6) != 0) {
        ESP_LOGE(TAG, "Response doesn't match command");
        return ESP_FAIL;
    }   
    // Verify CRC
    uint16_t received_crc = (response[7] << 8) | response[6];
    uint16_t calculated_crc = calculate_crc(response, 6);
    if (received_crc != calculated_crc) {
        ESP_LOGE(TAG, "CRC error (Received: %04X, Calculated: %04X)", received_crc, calculated_crc);
        return ESP_FAIL;
    }*/   
    ESP_LOGI(TAG, "Address changed successfully");
    return ESP_OK;
}

/**
 * @brief Load the next available sensor address from NVS.
 *        If not found, uses RS485_BASE_ADDRESS.
 */
int32_t load_next_address_from_nvs() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);

    if (err == ESP_OK) {
        err = nvs_get_i32(nvs_handle, "next_address", &next_available_address);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Loaded next available address: %ld", next_available_address);
        } else {
            ESP_LOGW(TAG, "No stored address found, using base: %d", RS485_BASE_ADDRESS);
            next_available_address = RS485_BASE_ADDRESS;
        }
        nvs_close(nvs_handle);
    } else {
        ESP_LOGE(TAG, "Failed to open NVS in read mode");
    }

    return next_available_address;
}

/**
 * @brief Save the next available sensor address to NVS.
 */
void save_next_address_to_nvs(int32_t address) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);

    if (err == ESP_OK) {
        err = nvs_set_i32(nvs_handle, "next_address", address);
        if (err == ESP_OK) {
            nvs_commit(nvs_handle);
            ESP_LOGI(TAG, "Saved next available address: %ld", address);
        } else {
            ESP_LOGE(TAG, "Failed to write next address to NVS!");
        }
        nvs_close(nvs_handle);
    } else {
        ESP_LOGE(TAG, "Failed to open NVS in write mode");
    }
}

/**
 * @brief Scan RS485 bus and provision new sensor if exactly one sensor found with address < 5
 */
void provision_rs485_sensor() {
    uint8_t detected_devices[10];
    int detected_count = scan_rs485_bus(RS485_MIN_ADDRESS, RS485_MAX_ADDRESS, detected_devices, 10);
    int low_address_count = 0;
    uint8_t candidate_address = 0;
    // Count devices with address < 5
    for (int i = 0; i < detected_count; i++) {
        if (detected_devices[i] < RS485_BASE_ADDRESS) {
            candidate_address = detected_devices[i];
            low_address_count++;
        }
    }
    if (low_address_count == 0) {
        ESP_LOGI(TAG, "No unprovisioned sensor found.");
        return;
    } else if (low_address_count > 1) {
        ESP_LOGW(TAG, "Multiple sensors with address < %d found. Provision aborted.", RS485_BASE_ADDRESS);
        return;
    }
    // Load next address from NVS
    int32_t new_address = load_next_address_from_nvs() + 1;
    if (change_sensor_address(candidate_address, new_address) == ESP_OK) {
        ESP_LOGI(TAG, "Changed sensor address from 0x%02X to 0x%02lX", candidate_address, new_address);
        save_next_address_to_nvs(new_address);
        flashled(12, 100, 100);
        /*/ Optional delay before verifying
        vTaskDelay(pdMS_TO_TICKS(100));
        if (read_sensor(new_address) >= 0) {
            ESP_LOGI(TAG, "Sensor at new address 0x%02lX responded successfully.", new_address);
            save_next_address_to_nvs(new_address);
        } else {
            ESP_LOGE(TAG, "Sensor did not respond at new address 0x%02lX. Provision failed.", new_address);
        }*/
    } else {
        ESP_LOGE(TAG, "Failed to change sensor address from 0x%02X to 0x%02lX", candidate_address, new_address);
    }
}
/*******************************************************************************************************************************/

/**********************************************************************************************************************************
FUNCTIONS: MAIN APPLICATION 
**************************/
void powerOnSequence() { 
              printAsteriskSeparator(100);
			  printf("System Firmware Started\n");
    		  setupWatchdogs();
    		  vTaskDelay(pdMS_TO_TICKS(100));
			  printAsteriskSeparator(100);
			  printf("Executing Power-On Sequence...\n"); 
			  vTaskDelay(pdMS_TO_TICKS(100));
//------------------------------------------------------------------------------------------------			  
			  //Initialize PWM led
			  setupled(); 
			  nvs_init();
    		  create_queue_mutex();
			  vTaskDelay(pdMS_TO_TICKS(1000));		  
//------------------------------------------------------------------------------------------------			
 if (PowerOn == 0) {
              ledfade(2, 10);
			  PowerOn = 1;
			  //save_next_address_to_nvs(RS485_BASE_ADDRESS);
              vTaskDelay(pdMS_TO_TICKS(100));
 } else if (PowerOn == 1){}	 	
//------------------------------------------------------------------------------------------------			 
			  vTaskDelay(pdMS_TO_TICKS(2000));
			  printf("Power-On Sequence Executed!\n"); 
			  printAsteriskSeparator(100);
			  }
void initializeSystem() { 
			  printAsteriskSeparator(100);
			  printf("Initializing System...\n"); 
//------------------------------------------------------------------------------------------------	
			  // Initialize Interrupt Button
			  setupButton();
			  initializeTimer();
//------------------------------------------------------------------------------------------------				  
			  // Initialize Voltage Monitor
			  setup_pins();
			  rs485_uart_init();
//------------------------------------------------------------------------------------------------				  		
			  vTaskDelay(pdMS_TO_TICKS(3000));
			  printAsteriskSeparator(100);
			  }

void ReadSlaves() { 
	  printAsteriskSeparator(100);	  
		load_next_address_from_nvs();
		for (int i = RS485_BASE_ADDRESS + 1; i <= next_available_address; i++) { 
		//for (int i = 1; i <= 7; i++) {
            float value = read_sensor(i);
		    if (value > 0.0f) {
		        printf("Sensor %02X Value: %.2f\n", i, value);
		        temp_slave_data.Sensor_Value = value;
		    } else {
		        // Error or invalid data
		        printf("Failed to read sensor %02X, setting value = 0.00\n", i);
		        temp_slave_data.Sensor_Value = 0.00f;  // Force 0.00 on failure
		    }
			  temp_slave_data.Sensor_Address = i;
			  store_slave_data(i - (RS485_BASE_ADDRESS + 1));
			  printf("Read slave No: %02X\n", i);
			  vTaskDelay(pdMS_TO_TICKS(10));
			  memset(&temp_slave_data, 0, sizeof(temp_slave_data));            
              vTaskDelay(pdMS_TO_TICKS(100)); // Short delay between sensors  
		}
		bufferReady = true;
		vTaskDelay(pdMS_TO_TICKS(10));	
			  printAsteriskSeparator(100);	
		    }
		    
/*******************************************************************************************************************************/ 


/**************************************************************************************************************************************
FUNCTIONS: BUTTON INTERRUPT
**************************/ 
void resetDevice() { 
                printAsteriskSeparator(100);
                printf("Resetting Device...\n");
                vTaskDelay(pdMS_TO_TICKS(100));
                esp_restart();
                printAsteriskSeparator(100);
                 }
	
void shutdownDevice() { 
                printAsteriskSeparator(100);
                printf("Shutting Down Device...\n");
                powerdown();               
                printAsteriskSeparator(100);
				 }

void provisioning() { 
                printAsteriskSeparator(100);
                printf("Provisioning new RS485 sensor...\n");
                provision_rs485_sensor();              
                printAsteriskSeparator(100);
				 }
/************************************************************************************************************************************** 
PROJECT: WATER QUALITY MONITOR 					OWNER: JACK MACLEANS				DEVELOPER: ERIC MULWA
***************************************************************************************************************************************/