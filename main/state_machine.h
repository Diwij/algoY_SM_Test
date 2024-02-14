/*
 * state_machine.h
 *
 *  Created on: Dec 27, 2023
 *      Author: ankit
 */

#ifndef MAIN_STATE_MACHINE_H_
#define MAIN_STATE_MACHINE_H_

#include "espnow_example.h"
#define FAULT_THRESHOLD 3

typedef enum
{
    eBmsState_Unknown = 0,
    eBmsState_Waiting,
    eBmsState_DroneLanded,
    eBmsState_ReadyForCharge,
    eBmsState_Charging,
    eBmsState_EndOfCharge,
	eBmsState_LandIdle,
	eBmsState_LandShort,
	eBmsMaxStates
} TeBmsState_t;

typedef enum
{
    eBmsStateRequest_None = 0,
    eBmsStateRequest_Waiting,
    eBmsStateRequest_DroneLanded,
	eBmsStateRequest_ReadyForCharge,
	eBmsStateRequest_Charging,
	eBmsStateRequest_EndOfCharge,
    eBmsStateRequest_LandIdle,
	eBmsStateRequest_LandShort
} TeBmsStateRequest_t;

typedef struct
{
    void (*entry)();
    void (*main)();
    void (*exit)();
} TsBmsStateCallback_t;

typedef struct
{
    TeBmsState_t state;
    TsBmsStateCallback_t callback;
} TsBmsStateModule_t;

typedef struct
{
    int16_t (*command_connect)(void);
    int16_t (*command_disconnect)(void);

    int16_t (*set_balance_enable)(bool enable);

    int16_t (*bios_request_sleep)(void);
    bool (*bios_sleep_requested)(void);
    bool (*initial_sleep_entry_condition_met)(void);
}TsBmsStateMachine_Ext;

typedef struct
{
    bool Valid_Charger_Setting_Flag;
    bool Charge_Required_Flag2;
    bool Landed_Idle_Flag;
    bool Charging_Complete_Flag;
    bool fault_set;
    TsBmsStateMachine_Ext bms_sm_ext;
    TeBmsStateRequest_t request;
    TeBmsState_t prev_state;
    TeBmsState_t curr_state;
    TeBmsState_t next_state;
    TsBmsStateModule_t sm_module;
    example_espnow_send_param_t *send_param;
} TsStateMachine_t;

//typedef struct
//{
//	bool Charge_Required_Flag1;  // Indication from battery voltage
//	bool Charge_Required_Flag2;   // Indication from co-controller
//	bool Landed_Idle_Flag;  	  // Indication of Landed Idle from co-controller
//	bool Charging_Complete_Flag;
//} TsStateMachine_flags_t;

typedef enum
{
    eCharge_VCT=0,
    eCharge_CCC,
    eCharge_IC
} TeChargeTerminationFlags_t;

extern TsStateMachine_t sm;

//extern TsStateMachine_flags_t bms_flags;




#endif /* MAIN_STATE_MACHINE_H_ */
