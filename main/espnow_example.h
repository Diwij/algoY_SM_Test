/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef ESPNOW_EXAMPLE_H
#define ESPNOW_EXAMPLE_H

/* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

#define ESPNOW_QUEUE_SIZE           6

/*
 * SPECIFIER FOR SENDING NON-CYCLIC PAYLOAD
 */
#define DRONE_HANDSHAKE_DATA		1
#define DRONE_ID					2
#define DRONE_CURRENT_POSE			3
#define DRONE_NEXT_POSE				4
#define DRONE_SOC					5
#define DRONE_STATE					6
#define BATTERY_POLARITY			7
#define CHARGER_CAPABILITIES		8
#define CELL_CONFIGURATION			9
#define REQUIRED_CHARGING_PROFILE	10
#define DRONESTATE_CHANGE_COMMAND	11
#define CHARGE_ENABLE				12
#define BATTERY_VOLTAGE				13
#define ERROR_FLAG					14
#define ACK							15

/*
 * PAYLOAD_ID DEFINITION
 */
enum{
	BROADCAST,
	CYCLIC_DRONE,
	NONCYCLIC_DRONE,
	CYCLIC_CHARGER,
	NONCYCLIC_CHARGER,
};


#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

typedef enum {
    EXAMPLE_ESPNOW_SEND_CB,
    EXAMPLE_ESPNOW_RECV_CB,
} example_espnow_event_id_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} example_espnow_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} example_espnow_event_recv_cb_t;

typedef union {
    example_espnow_event_send_cb_t send_cb;
    example_espnow_event_recv_cb_t recv_cb;
} example_espnow_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct {
    example_espnow_event_id_t id;
    example_espnow_event_info_t info;
} example_espnow_event_t;

enum {
    EXAMPLE_ESPNOW_DATA_BROADCAST,
    EXAMPLE_ESPNOW_DATA_UNICAST,
    EXAMPLE_ESPNOW_DATA_MAX,
};

/* User defined field of ESPNOW Non Cyclic data in this example. */
typedef struct {
    uint8_t type;                         //Broadcast or unicast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num;                     //Sequence number of ESPNOW data.
    uint16_t crc;                         //CRC16 value of ESPNOW data.
    uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
	uint8_t payload_ID;       			 // 11 for Non-cyclic payload
	uint8_t specifier;					 //	Command corresponding to the type of data
	uint8_t ack;						 // ACK byte
	uint16_t data;						 // actual data
    uint8_t random[];
} __attribute__((packed)) example_espnow_data_non_cyclic_t;

/* User defined field of ESPNOW Cyclic charging data in this example. */
typedef struct {
    uint8_t type;                         //Broadcast or unicast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num;                     //Sequence number of ESPNOW data.
    uint16_t crc;                         //CRC16 value of ESPNOW data.
    uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
	uint8_t payload_ID;       			 // 11 for Non-cyclic payload
    uint8_t specifier;                   // 1 for charging cyclic payload, 3 for ACK
	uint8_t ack;						 // ACK byte
//    uint8_t Vfb;					     //	Feedback voltage data
	uint8_t SOC;						 // SOC data
	uint16_t Vbat;						 // Drone End battery terminal voltage data
	uint8_t dronestate;					 // State of the drone FSM
	uint16_t current_bat;				 // Current of the battery pack
	uint8_t random[];
} __attribute__((packed)) example_espnow_data_cyclic_charging_t;


/* User defined field of ESPNOW Cyclic swarm data in this example. */
typedef struct {
    uint8_t type;                         //Broadcast or unicast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num;                     //Sequence number of ESPNOW data.
    uint16_t crc;                         //CRC16 value of ESPNOW data.
    uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
	uint8_t payload_ID;       			 // 11 for Non-cyclic payload
    uint8_t specifier;                   // 2 for swarm cyclic payload, 3 for ACK
	uint8_t ack;						 // ACK byte
    uint8_t curr_pose;
    uint8_t next_poses[5];
	uint8_t random[];
} __attribute__((packed)) example_espnow_data_cyclic_swarm_t;


/* User defined field of ESPNOW ACK data in this example. */
typedef struct {
    uint8_t type;                         //Broadcast or unicast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num;                     //Sequence number of ESPNOW data.
    uint16_t crc;                         //CRC16 value of ESPNOW data.
    uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
	uint8_t payload_ID;       			 // 11 for Non-cyclic payload
    uint8_t specifier;                   // 2 for swarm cyclic payload, 3 for ACK
    uint8_t ack;                         //ACK byte
    uint8_t drone_id;
    uint8_t random[];
} __attribute__((packed)) example_espnow_data_ack_t;

/*
 * User defined field of ESPNOW Broadcast data in this example
 */
typedef struct {
    uint8_t type;                         //Broadcast or unicast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num;                     //Sequence number of ESPNOW data.
    uint16_t crc;                         //CRC16 value of ESPNOW data.
    uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
	uint8_t payload_ID;       			 // 11 for Non-cyclic payload
    uint8_t specifier;                   // 2 for swarm cyclic payload, 3 for ACK
    uint8_t ack;                         //ACK byte
    uint8_t drone_id;
    uint8_t random[];
} __attribute__((packed)) example_espnow_data_broadcast_t;



/* Parameters of sending ESPNOW data. */
typedef struct {
    bool unicast;                         //Send unicast ESPNOW data.
    bool broadcast;                       //Send broadcast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint16_t count;                       //Total count of unicast ESPNOW data to be sent.
    uint16_t delay;                       //Delay between sending two ESPNOW data, unit: ms.
    int len;                              //Length of ESPNOW data to be sent, unit: byte.
    uint8_t *buffer;                      //Buffer pointing to ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   //MAC address of destination device.
} example_espnow_send_param_t;

/*Non-cyclic payload*/
typedef struct{
	uint8_t payload_ID;       			 // 11 for Non-cyclic payload
	uint8_t specifier;					 //	Command corresponding to the type of data
	uint8_t data;						 // actual data
	uint8_t ack;						 // ACK byte

} __attribute__((packed)) example_espnow_noncyclic_payload_t;


/*Cyclic payload*/
typedef struct{
	uint8_t payload_ID;       			 // 11 for Non-cyclic payload
	uint8_t Vfb;					     //	Feedback voltage data
	uint8_t SOC;						 // SOC data
	uint8_t Vbat;						 // Drone End battery terminal voltage data
	uint8_t dronestate;					 // State of the drone FSM
	uint8_t ack;						 // ACK byte
} __attribute__((packed)) example_espnow_cyclic_payload_t;



#endif
