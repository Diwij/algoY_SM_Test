#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "espnow_example.h"
#include "driver/dac.h"
#include "state_machine.h"
#include "esp_random.h"

#define DAC_EXAMPLE_CHANNEL_1 				    0

/*
 * New Params
 */
#define GPIO_CTRL_RELAY_1_OUTPUT				16
#define GPIO_CTRL_RELAY_2_OUTPUT				17
#define GPIO_CTRL_RELAY_3_OUTPUT				12
#define GPIO_CTRL_RELAY_4_OUTPUT				14

#define GPIO_POLARITY_POS_1_INPUT				34
#define GPIO_POLARITY_POS_2_INPUT				35

#define GPIO_LED_1_OUTPUT						25
#define GPIO_LED_2_OUTPUT						26
#define GPIO_LED_3_OUTPUT						27

#define GPIO_VOLTAGE_SENSE_INPUT				32

/*
 * Old Params
 */

//#define GPIO_CHGEN_PIN_OUTPUT					27
//#define GPIO_PG_PIN_INPUT						36
//#define GPIO_STAT1_PIN_INPUT					32
//#define GPIO_STAT2_PIN_INPUT					33
//#define GPIO_DRONE_PRESENT_PIN_INPUT			4

//#define GPIO_JTAG_TMS_PIN						14
//#define GPIO_JTAG_TDI_PIN						12
//#define GPIO_JTAG_TCK_PIN						13
//#define GPIO_JTAG_TDO_PIN						15


#define GPIO_OUTPUT_PIN_SEL			((1ULL<<GPIO_CTRL_RELAY_1_OUTPUT) | (1ULL<<GPIO_CTRL_RELAY_2_OUTPUT) | (1ULL<<GPIO_CTRL_RELAY_3_OUTPUT) | (1ULL<<GPIO_CTRL_RELAY_4_OUTPUT))
#define GPIO_INPUT_PIN_SEL 			((1ULL<<GPIO_POLARITY_POS_1_INPUT) | (1ULL<<GPIO_POLARITY_POS_2_INPUT))
#define GPIO_INPUT_VOLTAGE_SEL 		((1ULL<<GPIO_VOLTAGE_SENSE_INPUT))
#define GPIO_OUTPUT_LED_SEL 		((1ULL<<GPIO_LED_1_OUTPUT) | (1ULL<<GPIO_LED_2_OUTPUT) | (1ULL<<GPIO_LED_3_OUTPUT))

bool Charge_Required_Flag1 = false;   // Indication from battery voltage
bool Charge_Required_Flag2 = false;   // Indication from co-controller
bool Landed_Idle_Flag = false;  	  // Indication of Landed Idle from co-controller

float VOLTAGE = 0;
float CURRENT = 0;
uint8_t Drone_Presence = 0;
uint8_t drone_state = 0;

#define ADC1_CHANNEL_VOLTAGE    		ADC_CHANNEL_6
#define ADC1_CHANNEL_CURRENT			ADC_CHANNEL_7
#define NO_OF_SAMPLES					64

static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static TaskHandle_t task_1hz_handler;
static TaskHandle_t task_10hz_handler;
static TaskHandle_t task_20hz_handler;
static TaskHandle_t task_50hz_handler;

static uint16_t drone_id = 1000;
static uint16_t drone_handshake_data;
//static uint8_t drone_current_pose;
//static uint8_t drone_next_pose[5];
static uint16_t drone_soc = 0;
static uint16_t battery_polarity = 0;
static uint16_t charger_capabilities = 10;
static uint16_t cell_configuration = 0;
static uint16_t required_charging_profile = 0;
static uint16_t drone_state_change_command = 0;
static uint16_t charge_enable = 0;
static uint16_t error_flag = 0;
static uint8_t ack=0;
//static uint8_t v_fb = 0;
static uint16_t v_bat = 0;
static uint16_t current_bat = 0;
static uint8_t task_count = 0;
static uint8_t infinite_loop = 0;
static uint16_t cell_config;
static float Vout_base = 0;
static float Vout_req = 0;
static float DeltaV = 0;
static float Vdac = 0;
static float Vref = 2.1; // in Volts
static float R1= 100, R2= 670, Rdac= 100;  // in Kohm
static uint8_t VDAC;
#define CONFIG_ESPNOW_WIFI_MODE_STATION 1
#define CONFIG_ESPNOW_PMK 				"pmk1234567890123"
#define CONFIG_ESPNOW_LMK 				"lmk1234567890123"
#define CONFIG_ESPNOW_CHANNEL			1
#define CONFIG_ESPNOW_SEND_COUNT		100
#define CONFIG_ESPNOW_SEND_DELAY 		1000
#define CONFIG_ESPNOW_SEND_LEN 			20

bool CHARGING_COMPLETE_FLAG = false;
bool LANDED_ON_PAD_FLAG = false;

static const char *TAG = "espnow_example";

static QueueHandle_t s_example_espnow_queue;

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };

static void example_espnow_deinit(example_espnow_send_param_t *send_param);

/* State Machine*/

#define SYS_OK (int8_t)(0)
#define SYS_ERR (int8_t)(-1)
#define SYS_FAIL (int8_t)(-2)

uint8_t ctr = 0;
uint8_t charge_flag;
TsStateMachine_t sm;
TsBmsStateMachine_Ext *bms_sm_ext;
//TsStateMachine_flags_t bms_flags;
//TsChargeTerminationFlags_t charge_flags;
static void sm_state_unknown_entry(TsStateMachine_t *const sm);
static void sm_state_unknown_main(TsStateMachine_t *const sm);

static void sm_state_Waiting_entry(TsStateMachine_t *const sm);
static void sm_state_Waiting_main(TsStateMachine_t *const sm);
static void sm_state_Waiting_exit(TsStateMachine_t *const sm);

static void sm_state_DroneLanded_entry(TsStateMachine_t *const sm);
static void sm_state_DroneLanded_main(TsStateMachine_t *const sm);
static void sm_state_DroneLanded_exit(TsStateMachine_t *const sm);

static void sm_state_ReadyForCharge_entry(TsStateMachine_t *const sm);
static void sm_state_ReadyForCharge_main(TsStateMachine_t *const sm);
static void sm_state_ReadyForCharge_exit(TsStateMachine_t *const sm);

static void sm_state_Charging_entry(TsStateMachine_t *const sm);
static void sm_state_Charging_main(TsStateMachine_t *const sm);
static void sm_state_Charging_exit(TsStateMachine_t *const sm);

static void sm_state_EndOfCharge_entry(TsStateMachine_t *const sm);
static void sm_state_EndOfCharge_main(TsStateMachine_t *const sm);
static void sm_state_EndOfCharge_exit(TsStateMachine_t *const sm);

static void sm_state_LandIdle_entry(TsStateMachine_t *const sm);
static void sm_state_LandIdle_main(TsStateMachine_t *const sm);
static void sm_state_LandIdle_exit(TsStateMachine_t *const sm);

static void sm_state_LandShort_entry(TsStateMachine_t *const sm);
static void sm_state_LandShort_main(TsStateMachine_t *const sm);
static void sm_state_LandShort_exit(TsStateMachine_t *const sm);

static void app_bms_inhibitions_handler(void);
static int16_t app_bms_state_transition_handler(TeBmsState_t state);




//---------------------------------------------------------------------------------------------
// Static variables

static const TsBmsStateModule_t bms_sm[eBmsMaxStates] =
{
    {.state = eBmsState_Unknown,
         .callback = {  sm_state_unknown_entry,       sm_state_unknown_main,      NULL                    }},
    {.state = eBmsState_Waiting,
         .callback = {  sm_state_Waiting_entry,         sm_state_Waiting_main,        sm_state_Waiting_exit     }},
    {.state = eBmsState_DroneLanded,
         .callback = {  sm_state_DroneLanded_entry,         sm_state_DroneLanded_main,        sm_state_DroneLanded_exit     }},
    {.state = eBmsState_ReadyForCharge,
         .callback = {  sm_state_ReadyForCharge_entry,        sm_state_ReadyForCharge_main,       sm_state_ReadyForCharge_exit    }},
    {.state = eBmsState_Charging,
	     .callback = {  sm_state_Charging_entry,          sm_state_Charging_main,         sm_state_Charging_exit      }},
	{.state = eBmsState_EndOfCharge,
	     .callback = {  sm_state_EndOfCharge_entry,         sm_state_EndOfCharge_main,     sm_state_EndOfCharge_exit     }},
	{.state = eBmsState_LandIdle,
	     .callback = {  sm_state_LandIdle_entry,    sm_state_LandIdle_main,     sm_state_LandIdle_exit    }},
    {.state = eBmsState_LandShort,
         .callback = {  sm_state_LandShort_entry,     sm_state_LandShort_main,      sm_state_LandShort_exit     }}};

//---------------------------------------------------------------------------------------------
// Global Functions

int16_t app_bms_state_machine_init(const TsBmsStateMachine_Ext *pBms_sm_ext)
{
    int16_t exit_code = SYS_ERR;
    sm.Valid_Charger_Setting_Flag = false;
    sm.Charge_Required_Flag2 = false;
    sm.Charging_Complete_Flag = false;
    sm.Landed_Idle_Flag = false;

    do
    {
        if(NULL == pBms_sm_ext)
        {
            break;
        }
        sm.curr_state = eBmsState_Waiting;
        if(SYS_OK != app_bms_state_transition_handler(sm.curr_state))
        {
            break;
        }
        exit_code = SYS_OK;
    }while(false);

    return exit_code;
}



void app_bms_sm_tick(void)
{
    TeBmsState_t bms_state = eBmsState_Unknown;
    // Check for inhibitions
 //   app_bms_inhibitions_handler();

    // TODO: Fault checks
    switch(sm.request)
    {
    case eBmsStateRequest_None:
    	bms_state = eBmsState_Unknown;
    	break;
    case eBmsStateRequest_LandShort:
    	bms_state = eBmsState_LandShort;
    	break;
    case eBmsStateRequest_Waiting:
    	bms_state = eBmsState_Waiting;
    	break;
    case eBmsStateRequest_DroneLanded:
    	bms_state = eBmsState_DroneLanded;
    	break;
    case eBmsStateRequest_ReadyForCharge:
    	bms_state = eBmsState_ReadyForCharge;
    	break;
    case eBmsStateRequest_Charging:
    	bms_state = eBmsState_Charging;
    	break;
    case eBmsStateRequest_EndOfCharge:
        bms_state = eBmsState_EndOfCharge;
        break;
    case eBmsStateRequest_LandIdle:
        bms_state = eBmsState_LandIdle;
        break;
    default:
    	bms_state = eBmsState_Unknown;
    	break;
    }


    if(sm.curr_state != bms_state)
    {
        // Call State transition handler only when the state is changed
        app_bms_state_transition_handler(bms_state);
    }
    else
    {
        // Call the main state function
        if(bms_sm[sm.curr_state].callback.main != NULL)
        {
            bms_sm[sm.curr_state].callback.main(&sm);
        }
    }

}





// ----------------------------------------------------------------
/* Inhibitions Handler */
//static void app_bms_inhibitions_handler(void)
//{
//    switch(sm.request)
//    {
//    case eBmsStateRequest_None:
//        break;
//
//    case eBmsStateRequest_Drive:
//        if(sm.curr_state != eBmsState_Drive)
//        {
//            if(sm.inhibit_drive || sm.inhibit_drive_entry)
//            {
//                sm.request = eBmsStateRequest_None;
//            }
//        }
//        else
//        {
//            if(sm.inhibit_drive)
//            {
//                sm.request = eBmsStateRequest_None;
//            }
//        }
//        break;
//
//    case eBmsStateRequest_Charge:
//        if(sm.inhibit_charge || sm.inhibit_charge_entry)
//        {
//            sm.request = eBmsStateRequest_None;
//        }
//        break;
//
//    case eBmsStateRequest_Sleep:
//        break;
//    case eBmsStateRequest_Idle:
//    	break;
//    }
//
//}

/* BMS State Transition Handler */
static int16_t app_bms_state_transition_handler(TeBmsState_t state)
{
    int16_t exit_code = SYS_ERR;

    // 1.Exit current state
    if (NULL != bms_sm[sm.curr_state].callback.exit)
    {
        bms_sm[sm.curr_state].callback.exit(&sm);
    }

    // 2. Update State Transition
    sm.prev_state = sm.curr_state;
    sm.curr_state = state;

    // 3. Enter New State
    if (NULL != bms_sm[sm.curr_state].callback.entry)
    {
        bms_sm[sm.curr_state].callback.entry(&sm);
        exit_code = SYS_OK;
    }
    else
    {
        exit_code = SYS_FAIL;
    }

    return exit_code;
}

void app_set_bms_state_request(void)
{

	TeBmsStateRequest_t state_request = eBmsStateRequest_None;

	if (Drone_Presence == 0){
		state_request = eBmsStateRequest_Waiting;
		drone_state = 1;
	}else if ((Drone_Presence == 1 || Drone_Presence == 2) && current_bat < 0x01FF)
		{
		if(sm.curr_state == eBmsState_Charging || sm.curr_state ==eBmsState_EndOfCharge){
			state_request = eBmsStateRequest_EndOfCharge;
			drone_state = 6;
		}else if (sm.Valid_Charger_Setting_Flag == true){
			state_request = eBmsStateRequest_ReadyForCharge;
			drone_state = 4;
		}
		else{
			state_request = eBmsStateRequest_DroneLanded;
			drone_state = 2;
		}

	}else if ((Drone_Presence == 1 || Drone_Presence == 2) && current_bat > 0x01FF){
		state_request = eBmsStateRequest_Charging;
		drone_state = 5;
	}else{
		state_request = eBmsStateRequest_None;
	}

	sm.request = state_request;
}




/* WiFi should start before using ESPNOW */
static void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    example_espnow_event_t evt;
    example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_example_espnow_queue, &evt, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void example_espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    example_espnow_event_t evt;
    example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;

    if (xQueueSend(s_example_espnow_queue, &evt, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

void received_noncyclic(uint8_t specifier, uint16_t data){
	if(specifier == DRONE_HANDSHAKE_DATA)
	    drone_handshake_data = data;
	else if(specifier == DRONE_ID)
		drone_id = data;
	else if(specifier == DRONE_SOC)
		drone_soc = data;
	else if(specifier == DRONE_STATE)
		drone_state = data;
	else if(specifier == BATTERY_POLARITY)
		battery_polarity = data;
	else if(specifier == CHARGER_CAPABILITIES)
		charger_capabilities = data;
	else if(specifier == CELL_CONFIGURATION)
		cell_configuration = data;
	else if(specifier == REQUIRED_CHARGING_PROFILE)
		required_charging_profile = data;
	else if(specifier == DRONESTATE_CHANGE_COMMAND)
		drone_state_change_command = data;
	else if(specifier == CHARGE_ENABLE)
		charge_enable = data;
	else if(specifier == BATTERY_VOLTAGE)
		v_bat = data;
	else if(specifier == ERROR_FLAG)
		error_flag = data;
	else if(specifier == ACK)
		ack = data;

}
/* Parse received ESPNOW data. */
int example_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, int *magic)
{  // printf("Testing: %02x",*(data+10));
    if(*(data+10)==(uint8_t)2){ //Received non cyclic data

	example_espnow_data_non_cyclic_t *buf = (example_espnow_data_non_cyclic_t *)data;
    uint16_t crc, crc_cal = 0;
//    example_espnow_noncyclic_payload_t *dat = (example_espnow_noncyclic_payload_t *)buf->payload;

//    uint8_t *y = (uint8_t *)dat;

    received_noncyclic(buf->specifier, buf->data);

//    printf("Received Data:");
//	printf("%02x",buf->payload_ID);
//	printf("%02x",buf->specifier);
//	printf("%02x",buf->data);
//	printf("%02x",buf->ack);
//    printf("\n");


    if (data_len < sizeof(example_espnow_data_non_cyclic_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        return buf->type;
    }
  }
    else if(*(data+10)==(uint8_t)1 && *(data+11)==(uint8_t)1){ // Received cyclic charging data
    	example_espnow_data_cyclic_charging_t *buf = (example_espnow_data_cyclic_charging_t *)data;
        uint16_t crc, crc_cal = 0;

//        v_fb = buf->Vfb;
        v_bat = buf->Vbat;
        drone_state = buf->dronestate;
        drone_soc = buf->SOC;
        ack =buf->ack;
        current_bat = buf->current_bat;

//        printf("Received Data:");
//    	printf("%02x",buf->payload_ID);
//    	printf("%02x",buf->specifier);
//    	printf("%02x",buf->ack);
// //   	printf("%02x",buf->Vfb);
//    	printf("%02x",buf->SOC);
//    	printf("%02x",buf->Vbat);
//    	printf("%02x",buf->dronestate);
//    	printf("%02x",buf->current_bat);
//
//        printf("\n");

        if (data_len < sizeof(example_espnow_data_cyclic_charging_t)) {
            ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
            return -1;
        }

        *state = buf->state;
        *seq = buf->seq_num;
        *magic = buf->magic;
        crc = buf->crc;
        buf->crc = 0;
        crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

        if (crc_cal == crc) {
            return buf->type;
        }
    }
    else if(*(data+10)==(uint8_t)1 && *(data+11)==(uint8_t)2){  // Received cyclic swarm data
    	example_espnow_data_cyclic_swarm_t *buf = (example_espnow_data_cyclic_swarm_t *)data;
        uint16_t crc, crc_cal = 0;

//        printf("Received Data:");
//    	printf("%02x",buf->payload_ID);
//    	printf("%02x",buf->specifier);
//    	printf("%02x",buf->ack);
//    	printf("%02x",buf->curr_pose);
//        for(int i=0; i<5; i++){
//        	printf("%02x", buf->next_poses[i]);
//        }
//        printf("\n");

        if (data_len < sizeof(example_espnow_data_cyclic_charging_t)) {
            ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
            return -1;
        }

        *state = buf->state;
        *seq = buf->seq_num;
        *magic = buf->magic;
        crc = buf->crc;
        buf->crc = 0;
        crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

        if (crc_cal == crc) {
            return buf->type;
        }
    }
    else{
    	example_espnow_data_broadcast_t *buf = (example_espnow_data_broadcast_t *)data;
        uint16_t crc, crc_cal = 0;

//        printf("Received Data:");
//    	printf("%02x",buf->payload_ID);
//    	printf("%02x",buf->specifier);
//    	printf("%02x",buf->ack);
//        printf("%02x", buf->drone_id);
//        printf("\n");

        if (data_len < sizeof(example_espnow_data_broadcast_t)) {
            ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
            return -1;
        }

        *state = buf->state;
        *seq = buf->seq_num;
        *magic = buf->magic;
        crc = buf->crc;
        buf->crc = 0;
        crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

        if (crc_cal == crc) {
            return buf->type;
        }
    }
    return -1;
}

/*
 * Preparing non_cyclic_payload to be sent
 */
void non_cyclic_payload(example_espnow_send_param_t *send_param,example_espnow_data_non_cyclic_t *buf, uint8_t payload_id, uint8_t specifier){
    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    buf->payload_ID=payload_id;
    buf->specifier=specifier;
    buf->ack=0;

    if(specifier == DRONE_HANDSHAKE_DATA)
    	    buf->data = drone_handshake_data;
    	else if(specifier == DRONE_ID)
    		buf->data = drone_id;
    	else if(specifier == DRONE_SOC)
    		buf->data = drone_soc;
    	else if(specifier == DRONE_STATE)
    		buf->data = drone_state;
    	else if(specifier == BATTERY_POLARITY)
    		buf->data = battery_polarity;
    	else if(specifier == CHARGER_CAPABILITIES)
    		buf->data = charger_capabilities;
    	else if(specifier == CELL_CONFIGURATION)
    		buf->data = cell_configuration;
    	else if(specifier == REQUIRED_CHARGING_PROFILE)
    		buf->data = required_charging_profile;
    	else if(specifier == DRONESTATE_CHANGE_COMMAND)
    		buf->data = drone_state_change_command;
    	else if(specifier == CHARGE_ENABLE)
    		buf->data = 1;
    	else if(specifier == ERROR_FLAG)
    		buf->data = error_flag;
    	else if(specifier == ACK)
    		buf->data = 1;
    	else
    		buf->data = 0;
}

/*
 * Preparing cyclic_payload_charging to be sent
 */
void cyclic_payload_charging(example_espnow_send_param_t *send_param,example_espnow_data_cyclic_charging_t *buf, uint8_t payload_id,uint8_t specifier){
    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    buf->payload_ID=payload_id;
    buf->specifier=specifier;
    buf->ack=0;
//    buf->Vfb=20;
    buf->SOC=30;
    buf->Vbat=40;
    buf->dronestate=50;
}

/*
 * Preparing cyclic_payload_swarm to be sent
 */
void cyclic_payload_swarm(example_espnow_send_param_t *send_param,example_espnow_data_cyclic_swarm_t *buf, uint8_t payload_id,uint8_t specifier){
    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    buf->payload_ID=payload_id;
    buf->specifier= specifier;
    buf->ack=0;
    buf->curr_pose=255;
    for(int i=0; i<5; i++){
    	buf->next_poses[i]=255;
    }

}


/* Prepare ESPNOW data to be sent. */
void example_espnow_data_prepare(example_espnow_send_param_t *send_param, uint8_t payload_id, uint8_t specifier)
{

  if (payload_id == 2 || payload_id == 4){// Non-cyclic payload
	example_espnow_data_non_cyclic_t *buf = (example_espnow_data_non_cyclic_t *)send_param->buffer;
//    example_espnow_noncyclic_payload_t *dat = (example_espnow_noncyclic_payload_t *)buf->payload;
    assert(send_param->len >= sizeof(example_espnow_data_non_cyclic_t));

    non_cyclic_payload(send_param,buf,payload_id,specifier);

    /* Fill all remaining bytes after the data with random values */
    esp_fill_random(buf->random, send_param->len - sizeof(example_espnow_data_non_cyclic_t));

    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);

  }

  else if (payload_id == 1 || payload_id == 3){ // Cyclic payload
	  if(specifier == 1){ // Charging cyclic payload
		example_espnow_data_cyclic_charging_t *buf = (example_espnow_data_cyclic_charging_t *)send_param->buffer;
	    assert(send_param->len >= sizeof(example_espnow_data_cyclic_charging_t));

        cyclic_payload_charging(send_param, buf, payload_id, specifier);

	    /* Fill all remaining bytes after the data with random values */
	    esp_fill_random(buf->random, send_param->len - sizeof(example_espnow_data_cyclic_charging_t));

	    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);

	  }else if(specifier == 2){ // Swarm cyclic payload
			example_espnow_data_cyclic_swarm_t *buf = (example_espnow_data_cyclic_swarm_t *)send_param->buffer;
		    assert(send_param->len >= sizeof(example_espnow_data_cyclic_swarm_t));

	        cyclic_payload_swarm(send_param, buf, payload_id, specifier);

		    /* Fill all remaining bytes after the data with random values */
		    esp_fill_random(buf->random, send_param->len - sizeof(example_espnow_data_cyclic_swarm_t));

		    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);

	  }
	  else{
		  /*
		   * Acknowledgement
		   */
		  example_espnow_data_ack_t *buf = (example_espnow_data_ack_t *)send_param->buffer;
		  assert(send_param->len >= sizeof(example_espnow_data_ack_t));
		  buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
		  buf->state = send_param->state;
		  buf->seq_num = s_example_espnow_seq[buf->type]++;
		  buf->crc = 0;
		  buf->magic = send_param->magic;
		  buf->payload_ID=payload_id;
		  buf->specifier= specifier;
		  buf->ack=255;
		  buf->drone_id=drone_id;
		  /* Fill all remaining bytes after the data with random values */
		  esp_fill_random(buf->random, send_param->len - sizeof(example_espnow_data_ack_t));
          buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);


	  }

  }
  else{
	  example_espnow_data_broadcast_t *buf = (example_espnow_data_broadcast_t *)send_param->buffer;
	  assert(send_param->len >= sizeof(example_espnow_data_ack_t));
	  buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
	  buf->state = send_param->state;
	  buf->seq_num = s_example_espnow_seq[buf->type]++;
	  buf->crc = 0;
	  buf->magic = send_param->magic;
	  buf->payload_ID=payload_id;
	  buf->specifier= specifier;
	  buf->ack=0;
	  buf->drone_id=drone_id;

	  /* Fill all remaining bytes after the data with random values */
	  esp_fill_random(buf->random, send_param->len - sizeof(example_espnow_data_ack_t));
      buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);



  }
}



static void example_espnow_send_receive(void *pvParameter)
{
    example_espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    int recv_magic = 0;
    bool is_broadcast = false;
    int ret;
//    printf("\n1\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    /* Start sending broadcast ESPNOW data. */
    example_espnow_send_param_t *send_param = (example_espnow_send_param_t *)pvParameter;
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        example_espnow_deinit(send_param);
        vTaskDelete(NULL);
    }

    while (xQueueReceive(s_example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case EXAMPLE_ESPNOW_SEND_CB:
            {
//            	printf("\n2\n");
                example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
//                is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);

//                ESP_LOGD(TAG, "Send data to "MACSTR", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

//                if (is_broadcast && (send_param->broadcast == false)) {
//                    break;
//                }
//


                /* Delay a while before sending the next data. */
                if (send_param->delay > 0) {
                    vTaskDelay(send_param->delay/portTICK_PERIOD_MS);
                }

//                ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(send_cb->mac_addr));

                break;
            }
            case EXAMPLE_ESPNOW_RECV_CB:
            {
//            	printf("3\n");
                example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

                ret = example_espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
                free(recv_cb->data);
                if (ret == EXAMPLE_ESPNOW_DATA_BROADCAST) {
//                    ESP_LOGI(TAG, "Receive %dth broadcast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    /* If MAC address does not exist in peer list, add it to peer list. */
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
                        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                        if (peer == NULL) {
                            ESP_LOGE(TAG, "Malloc peer information fail");
                            example_espnow_deinit(send_param);
                            vTaskDelete(NULL);
                        }
                        memset(peer, 0, sizeof(esp_now_peer_info_t));
                        peer->channel = CONFIG_ESPNOW_CHANNEL;
                        peer->ifidx = ESPNOW_WIFI_IF;
                        peer->encrypt = true;
                        memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                        memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        ESP_ERROR_CHECK( esp_now_add_peer(peer) );
                        free(peer);
                    }

                  esp_now_peer_num_t *num = malloc(sizeof(esp_now_peer_num_t));

                    if (esp_now_get_peer_num(num) != ESP_OK) {
                           ESP_LOGE(TAG, "Peer error");
                           vTaskDelete(NULL);
                          }
                           else {
//                                  printf("Num of Peer:%d \n",num->total_num);
                                }
                    free(num);
                }
                else if (ret == EXAMPLE_ESPNOW_DATA_UNICAST) {
//                    ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    /* If receive unicast ESPNOW data, also stop sending broadcast ESPNOW data. */
                    send_param->broadcast = false;
                }
                else {
                    //ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                }
                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
        printf("espnow\n");
    }
}


void Drop_in_contact_resistance(){
    float Vter = 0;
    printf("adc1_init...\n");
    adc1_config_width(width);
    adc1_config_channel_atten( ADC1_CHANNEL_CURRENT, ADC_ATTEN_DB_11 );
    adc1_config_channel_atten( ADC1_CHANNEL_VOLTAGE, ADC_ATTEN_DB_11 );

    printf("start conversion.\n");
   	int read_raw=0;
    for (int i=0; i < NO_OF_SAMPLES; i++){
       	int raw;
       	raw= adc1_get_raw( ADC1_CHANNEL_VOLTAGE);
       	read_raw +=raw;
    }
   	read_raw/=NO_OF_SAMPLES;
    read_raw = read_raw + 95;
   	Vter = ((float)read_raw/4095)*3.3*10;
   	printf("Vter:%f\n",Vter);
    printf("Vadc:%f\n",(float)read_raw/4095);
   	DeltaV = Vter - (v_bat/1000);
   	printf("\nDeltaV :%f\n", DeltaV);
}

void required_DAC_output(){

	if(cell_configuration == 3){
		cell_config = 3;
		Vout_base  = 12.6;
	}
	else if(cell_configuration == 4){
		Vout_base = 16.8;
	cell_config = 4;
	}
	else if(cell_configuration == 5){
		Vout_base = 21;
	cell_config = 5;
	}
	else if(cell_configuration == 6){
		Vout_base = 25.2;
	cell_config = 6;
	}
	else{
		Vout_base = 16.8;
	    cell_config = 4;
	}

	Drop_in_contact_resistance();

	Vout_req = Vout_base + DeltaV;

//	Vout_req = Vref*(1+(R2/R1)) - (Vdac-Vref) * (R2/Rdac);

	Vdac = Vref + (Vref*(1+R2/R1) - Vout_req) * (Rdac/R2);

	VDAC = (Vdac/3.3)*255;
	printf("VDAC:%d\n",VDAC);

}

/*
 * Drone state is determined by drone_state variable
 *
 * waiting for the drone to land = 1
 * Ready for Charge = 4
 * Charging = 5
 * End of Charge = 6
 * Land Idle = 7
 * Landed on short = 8
 */


static void task_1hz(){
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 500;

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount ();
	for( ;; )
	{
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
		printf("task_1hz\n");

		// Perform action here.
	}
}

static void task_10hz(){
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 200;


	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount ();
	for( ;; )
	{
		// Perform action here.


		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

	}
}

static void task_20hz(){
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 800;

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount ();
	for( ;; )
	{

		// Perform action here.
		printf("task_20hz\n");
		if(gpio_get_level(GPIO_POLARITY_POS_1_INPUT) == 1)
		{
			Drone_Presence = 1;
		}
		else if(gpio_get_level(GPIO_POLARITY_POS_2_INPUT) == 1)
		{
			Drone_Presence = 2;
		}
		else
		{
			Drone_Presence = 0;
		}
		printf("Drone Presence:%d\n",Drone_Presence);
		app_set_bms_state_request();
		app_bms_sm_tick();
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}

static void task_50hz(void *pvParameter){
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100;

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount ();
	for( ;; )
	{
		// Perform action here.
		printf("task_50hz\n");
		example_espnow_send_receive(pvParameter);

	}
}


// ------------------ WAITING STATE ------------------------------
/* Waiting Entry */
static void sm_state_Waiting_entry(TsStateMachine_t *const sm)
{
	sm->send_param = malloc(sizeof(example_espnow_send_param_t));
	memset(sm->send_param, 0, sizeof(example_espnow_send_param_t));
	if (sm->send_param == NULL) {
		ESP_LOGE(TAG, "Malloc send parameter fail");
		vSemaphoreDelete(s_example_espnow_queue);
		esp_now_deinit();
	}
	sm->send_param->unicast = false;
	sm->send_param->broadcast = true;
	sm->send_param->state = 0;
	sm->send_param->magic = esp_random();
	sm->send_param->count = CONFIG_ESPNOW_SEND_COUNT;
	sm->send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
	sm->send_param->len = CONFIG_ESPNOW_SEND_LEN;
	sm->send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
	if (sm->send_param->buffer == NULL) {
		ESP_LOGE(TAG, "Malloc send buffer fail");
		free(sm->send_param);
		vSemaphoreDelete(s_example_espnow_queue);
		esp_now_deinit();
	}
	memcpy(sm->send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);

	sm_state_Waiting_main(sm);
}

/* Waiting Main */
static void sm_state_Waiting_main(TsStateMachine_t *const sm)
{
	if (ack == 1){
		//Open Dock
		printf("Drone Incoming, Opening Dock!");
	}
	else{
		printf("Waiting for Drone!");
	}
}

/* Waiting Exit */
static void sm_state_Waiting_exit(TsStateMachine_t *const sm)
{
	//close Dock
   free(sm->send_param);
}

// ------------------ DroneLanded STATE ------------------------------
/* DroneLanded entry */
static void sm_state_DroneLanded_entry(TsStateMachine_t *const sm)
{
	sm->send_param = malloc(sizeof(example_espnow_send_param_t));
	memset(sm->send_param, 0, sizeof(example_espnow_send_param_t));
	if (sm->send_param == NULL) {
		ESP_LOGE(TAG, "Malloc send parameter fail");
		vSemaphoreDelete(s_example_espnow_queue);
		esp_now_deinit();
	}
	sm->send_param->unicast = false;
	sm->send_param->broadcast = true;
	sm->send_param->state = 0;
	sm->send_param->magic = esp_random();
	sm->send_param->count = CONFIG_ESPNOW_SEND_COUNT;
	sm->send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
	sm->send_param->len = CONFIG_ESPNOW_SEND_LEN;
	sm->send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
	if (sm->send_param->buffer == NULL) {
		ESP_LOGE(TAG, "Malloc send buffer fail");
		free(sm->send_param);
		vSemaphoreDelete(s_example_espnow_queue);
		esp_now_deinit();
	}
	memcpy(sm->send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
	vTaskDelay(2000/portTICK_PERIOD_MS);
	printf("Drone ID: %d\n",drone_id);
	vTaskDelay( 200/portTICK_PERIOD_MS);
	printf("Battery vtg: %f\n",(float)v_bat/1000);
	vTaskDelay( 200/portTICK_PERIOD_MS);
	printf("SOC: %d\n",drone_soc);
	vTaskDelay( 200/portTICK_PERIOD_MS);
	printf("Cell config: %d S\n",cell_configuration);
    sm_state_DroneLanded_main(sm);
}

/* DroneLanded Main */
static void sm_state_DroneLanded_main(TsStateMachine_t *const sm)
{
//	example_espnow_data_prepare(sm->send_param,NONCYCLIC_DRONE,CHARGER_CAPABILITIES);
//	if (esp_now_send(sm->send_param->dest_mac, sm->send_param->buffer, sm->send_param->len) != ESP_OK) {
//		ESP_LOGE(TAG, "Send error");
//		example_espnow_deinit(sm->send_param);
//		vTaskDelete(NULL);
//	}
//	if (required_charging_profile == 10){
//		sm->Valid_Charger_Setting_Flag = 1;
//	}
	if(Drone_Presence == 1)
	{
		gpio_set_level(GPIO_CTRL_RELAY_1_OUTPUT, 1);
		gpio_set_level(GPIO_CTRL_RELAY_4_OUTPUT, 1);
	}
	else if(Drone_Presence == 2)
	{
		gpio_set_level(GPIO_CTRL_RELAY_2_OUTPUT, 1);
		gpio_set_level(GPIO_CTRL_RELAY_3_OUTPUT, 1);
	}
}

/* DroneLanded exit */
static void sm_state_DroneLanded_exit(TsStateMachine_t *const sm)
{
	free(sm->send_param);
}

// ------------------ ReadyForCharge STATE ------------------------------
/* ReadyForCharge entry */
static void sm_state_ReadyForCharge_entry(TsStateMachine_t *const sm)
{
	sm->send_param = malloc(sizeof(example_espnow_send_param_t));
	memset(sm->send_param, 0, sizeof(example_espnow_send_param_t));
	if (sm->send_param == NULL) {
		ESP_LOGE(TAG, "Malloc send parameter fail");
		vSemaphoreDelete(s_example_espnow_queue);
		esp_now_deinit();
	}
	sm->send_param->unicast = false;
	sm->send_param->broadcast = true;
	sm->send_param->state = 0;
	sm->send_param->magic = esp_random();
	sm->send_param->count = CONFIG_ESPNOW_SEND_COUNT;
	sm->send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
	sm->send_param->len = CONFIG_ESPNOW_SEND_LEN;
	sm->send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
	if (sm->send_param->buffer == NULL) {
		ESP_LOGE(TAG, "Malloc send buffer fail");
		free(sm->send_param);
		vSemaphoreDelete(s_example_espnow_queue);
		esp_now_deinit();
	}
	memcpy(sm->send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
	//CV set
	//CC set
    sm_state_ReadyForCharge_main(sm);
}

/* ReadyForCharge main */
static void sm_state_ReadyForCharge_main(TsStateMachine_t *const sm)
{
	//Turn ON Charger
	if(Drone_Presence == 1)
	{
		gpio_set_level(GPIO_CTRL_RELAY_1_OUTPUT, 1);
		gpio_set_level(GPIO_CTRL_RELAY_4_OUTPUT, 1);
	}
	else if(Drone_Presence == 2)
	{
		gpio_set_level(GPIO_CTRL_RELAY_2_OUTPUT, 1);
		gpio_set_level(GPIO_CTRL_RELAY_3_OUTPUT, 1);
	}
	printf("starting charging...\n");

}

/* ReadyForCharge exit */
static void sm_state_ReadyForCharge_exit(TsStateMachine_t *const sm)
{
	free(sm->send_param);
}

// ------------------ Charging STATE ------------------------------
/* Charging entry */
static void sm_state_Charging_entry(TsStateMachine_t *const sm)
{
	sm->send_param = malloc(sizeof(example_espnow_send_param_t));
	memset(sm->send_param, 0, sizeof(example_espnow_send_param_t));
	if (sm->send_param == NULL) {
		ESP_LOGE(TAG, "Malloc send parameter fail");
		vSemaphoreDelete(s_example_espnow_queue);
		esp_now_deinit();
	}
	sm->send_param->unicast = false;
	sm->send_param->broadcast = true;
	sm->send_param->state = 0;
	sm->send_param->magic = esp_random();
	sm->send_param->count = CONFIG_ESPNOW_SEND_COUNT;
	sm->send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
	sm->send_param->len = CONFIG_ESPNOW_SEND_LEN;
	sm->send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
	if (sm->send_param->buffer == NULL) {
		ESP_LOGE(TAG, "Malloc send buffer fail");
		free(sm->send_param);
		vSemaphoreDelete(s_example_espnow_queue);
		esp_now_deinit();
	}
	memcpy(sm->send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
	sm_state_Charging_main(sm);

}

/* Charging main */
static void sm_state_Charging_main(TsStateMachine_t *const sm)
{
	//Turn ON charger
	//Print data
	if(Drone_Presence == 1)
	{
		gpio_set_level(GPIO_CTRL_RELAY_1_OUTPUT, 1);
		gpio_set_level(GPIO_CTRL_RELAY_4_OUTPUT, 1);
	}
	else if(Drone_Presence == 2)
	{
		gpio_set_level(GPIO_CTRL_RELAY_2_OUTPUT, 1);
		gpio_set_level(GPIO_CTRL_RELAY_3_OUTPUT, 1);
	}
	vTaskDelay( 200/portTICK_PERIOD_MS);
	printf("Battery vtg: %f\n",(float)v_bat/1000);
	vTaskDelay( 200/portTICK_PERIOD_MS);
	printf("Battery cur: %f\n",(float)current_bat/1000);

}



/* Charging exit */
static void sm_state_Charging_exit(TsStateMachine_t *const sm)
{
	//Charger OFF
	gpio_set_level(GPIO_OUTPUT_PIN_SEL,0);
	free(sm->send_param);
}

// ------------------ EndOfCharge STATE ------------------------------
/* EndOfCharge entry */
static void sm_state_EndOfCharge_entry(TsStateMachine_t *const sm)
{
	sm->send_param = malloc(sizeof(example_espnow_send_param_t));
	memset(sm->send_param, 0, sizeof(example_espnow_send_param_t));
	if (sm->send_param == NULL) {
		ESP_LOGE(TAG, "Malloc send parameter fail");
		vSemaphoreDelete(s_example_espnow_queue);
		esp_now_deinit();
	}
	sm->send_param->unicast = false;
	sm->send_param->broadcast = true;
	sm->send_param->state = 0;
	sm->send_param->magic = esp_random();
	sm->send_param->count = CONFIG_ESPNOW_SEND_COUNT;
	sm->send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
	sm->send_param->len = CONFIG_ESPNOW_SEND_LEN;
	sm->send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
	if (sm->send_param->buffer == NULL) {
		ESP_LOGE(TAG, "Malloc send buffer fail");
		free(sm->send_param);
		vSemaphoreDelete(s_example_espnow_queue);
		esp_now_deinit();
	}
	memcpy(sm->send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);

	gpio_set_level(GPIO_OUTPUT_PIN_SEL,0);

    sm_state_EndOfCharge_main(sm);

}

/* EndOfCharge main */
static void sm_state_EndOfCharge_main(TsStateMachine_t *const sm)
{
	gpio_set_level(GPIO_OUTPUT_PIN_SEL,0);

	if(gpio_get_level(GPIO_INPUT_PIN_SEL) == 0){
		LANDED_ON_PAD_FLAG = false;
	}else
		LANDED_ON_PAD_FLAG = true;


	if(LANDED_ON_PAD_FLAG == true){
		printf("\n Charging completed, waiting for the drone to fly\n");
	}
	else {
		printf("\n Drone take off done!\n");

}
}



/* EndOfCharge exit */
static void sm_state_EndOfCharge_exit(TsStateMachine_t *const sm)
{
	free(sm->send_param);
}

// ------------------ LandIdle STATE ------------------------------
/* LandIdle entry */
static void sm_state_LandIdle_entry(TsStateMachine_t *const sm)
{
	sm->send_param = malloc(sizeof(example_espnow_send_param_t));
	memset(sm->send_param, 0, sizeof(example_espnow_send_param_t));
	if (sm->send_param == NULL) {
		ESP_LOGE(TAG, "Malloc send parameter fail");
		vSemaphoreDelete(s_example_espnow_queue);
		esp_now_deinit();
	}
	sm->send_param->unicast = false;
	sm->send_param->broadcast = true;
	sm->send_param->state = 0;
	sm->send_param->magic = esp_random();
	sm->send_param->count = CONFIG_ESPNOW_SEND_COUNT;
	sm->send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
	sm->send_param->len = CONFIG_ESPNOW_SEND_LEN;
	sm->send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
	if (sm->send_param->buffer == NULL) {
		ESP_LOGE(TAG, "Malloc send buffer fail");
		free(sm->send_param);
		vSemaphoreDelete(s_example_espnow_queue);
		esp_now_deinit();
	}
	memcpy(sm->send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    sm_state_LandIdle_main(sm);

}

/* LandIdle main */
static void sm_state_LandIdle_main(TsStateMachine_t *const sm)
{

}

/* LandIdle exit */
static void sm_state_LandIdle_exit(TsStateMachine_t *const sm)
{
	free(sm->send_param);
}

// ------------------ LandShort STATE ------------------------------
/* LandShort entry */
static void sm_state_LandShort_entry(TsStateMachine_t *const sm)
{
	sm->send_param = malloc(sizeof(example_espnow_send_param_t));
	memset(sm->send_param, 0, sizeof(example_espnow_send_param_t));
	if (sm->send_param == NULL) {
		ESP_LOGE(TAG, "Malloc send parameter fail");
		vSemaphoreDelete(s_example_espnow_queue);
		esp_now_deinit();
	}
	sm->send_param->unicast = false;
	sm->send_param->broadcast = true;
	sm->send_param->state = 0;
	sm->send_param->magic = esp_random();
	sm->send_param->count = CONFIG_ESPNOW_SEND_COUNT;
	sm->send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
	sm->send_param->len = CONFIG_ESPNOW_SEND_LEN;
	sm->send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
	if (sm->send_param->buffer == NULL) {
		ESP_LOGE(TAG, "Malloc send buffer fail");
		free(sm->send_param);
		vSemaphoreDelete(s_example_espnow_queue);
		esp_now_deinit();
	}
	memcpy(sm->send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    sm_state_LandShort_main(sm);

}

/* LandShort main */
static void sm_state_LandShort_main(TsStateMachine_t *const sm)
{

}

/* LandShort exit */
static void sm_state_LandShort_exit(TsStateMachine_t *const sm)
{
	free(sm->send_param);
}

// ------------------ UNKNOWN STATE ------------------------------
/* Unknown entry */
static void sm_state_unknown_entry(TsStateMachine_t *const sm)
{
    sm_state_unknown_main(sm);
}

/* Unknown main */
static void sm_state_unknown_main(TsStateMachine_t *const sm)
{

}

static esp_err_t example_espnow_init(void)
{
    example_espnow_send_param_t *send_param;

    s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (s_example_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }


    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) ); //callback function for sending espnow data
    ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) ); // callback function for receiving espnow data

    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    send_param->unicast = false;
    send_param->broadcast = true;
    send_param->state = 0;
    send_param->magic = esp_random();
    send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = CONFIG_ESPNOW_SEND_LEN;
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    example_espnow_data_prepare(send_param,5,1);


    esp_now_peer_num_t *num = malloc(sizeof(esp_now_peer_num_t));

      if (esp_now_get_peer_num(num) != ESP_OK) {
             ESP_LOGE(TAG, "Peer error");
             vTaskDelete(NULL);
            }
             else {
                    printf("Num of Peer after adding broadcast peer:%d",num->total_num);
                  }
      free(num);


      xTaskCreatePinnedToCore(task_50hz, "task_50hz", 2048, send_param,10,&task_50hz_handler,0);

    return ESP_OK;
}

static void example_espnow_deinit(example_espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(s_example_espnow_queue);
    esp_now_deinit();
}


void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    esp_err_t r;

    gpio_num_t dac_gpio_num_1;

    r = dac_pad_get_io_num( DAC_EXAMPLE_CHANNEL_1, &dac_gpio_num_1 );
    assert( r == ESP_OK );

    printf("DAC channel %d @ GPIO %d.\n",DAC_EXAMPLE_CHANNEL_1 + 1, dac_gpio_num_1 );

    dac_output_enable( DAC_EXAMPLE_CHANNEL_1 );

    example_wifi_init();
    example_espnow_init();

//	printf("1....\n");
	gpio_config_t io_config;

	//disable the interrupt
	io_config.intr_type = GPIO_INTR_DISABLE;
	//set as output mode
	io_config.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pin that we want to set
	io_config.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable the pull down
	io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable the pull up
	io_config.pull_up_en = GPIO_PULLUP_DISABLE;
//	printf("2....\n");
    gpio_config(&io_config);
//    printf("3....\n");
	//set as input mode
	io_config.mode = GPIO_MODE_INPUT;
    //bit mask of the pin that we want to set
	io_config.pin_bit_mask = GPIO_INPUT_PIN_SEL;
//	printf("4....\n");
    gpio_config(&io_config);
//printf("5....\n");
//    adc_voltage_queue = xQueueCreate(5,sizeof(int));
//    adc_current_queue = xQueueCreate(5,sizeof(int));

    xTaskCreatePinnedToCore(task_1hz,"task_1hz",2048,NULL,5,&task_1hz_handler,1);
    xTaskCreatePinnedToCore(task_10hz,"task_10hz",2048,NULL,6,&task_10hz_handler,1);
    xTaskCreatePinnedToCore(task_20hz,"task_20hz",2048,NULL,10,&task_20hz_handler,1);
    if(task_1hz_handler == NULL || task_10hz_handler==NULL || task_20hz_handler== NULL){
    	printf("\n Task creation failed!\n");

    }
}

