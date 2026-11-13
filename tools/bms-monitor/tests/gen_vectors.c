/*
 * Generates tests/vectors.json: battery messages serialised by the firmware's
 * own Nunavut headers (inc/UAVCAN), with the values that went in. The Python
 * decoder in bms_monitor.py is tested against these bytes.
 *
 *   gcc -std=c11 -I../../../inc/UAVCAN gen_vectors.c -lm -o gen_vectors
 *   ./gen_vectors > vectors.json
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define NUNAVUT_ASSERT(x) ((void)0)
/* src/uavcan.c limits the cell array to six entries in the same way. */
#define reg_drone_service_battery_Status_0_2_cell_voltages_ARRAY_CAPACITY_ 6U
#include "nunavut/support/serialization.h"
#include "reg/drone/physics/electricity/SourceTs_0_1.h"
#include "reg/drone/service/battery/Status_0_2.h"
#include "reg/drone/service/battery/Parameters_0_3.h"
#include "legacy/equipment/power/BatteryInfo_1_0.h"
#include "uavcan/_register/Access_1_0.h"
#include "uavcan/node/ExecuteCommand_1_1.h"

static void hex(const char *key, const uint8_t *buffer, size_t size, const char *tail)
{
    printf("    \"%s\": \"", key);
    for (size_t i = 0; i < size; i++) printf("%02x", buffer[i]);
    printf("\"%s\n", tail);
}

int main(void)
{
    uint8_t buffer[600];
    size_t size;
    printf("{\n");

    /* legacy BatteryInfo, as BatteryInfoToTransmitBuffer() fills it */
    legacy_equipment_power_BatteryInfo_1_0 info;
    memset(&info, 0, sizeof(info));
    info.temperature = 21.25f;
    info.voltage = 15.5f;
    info.current = -2.75f;
    info.average_power_10sec = -42.5f;
    info.remaining_capacity_wh = 96.5f;
    info.full_charge_capacity_wh = 144.0f;
    info.hours_to_full_charge = 1.5f;
    info.status_flags = 1U | 8U | 256U;
    info.state_of_health_pct = 93;
    info.state_of_charge_pct = 67;
    info.state_of_charge_pct_stdev = 0;
    info.battery_id = 5;
    info.model_instance_id = 123456789UL;
    const char *name = "SOSUB pod";
    info.model_name.count = strlen(name);
    memcpy(info.model_name.elements, name, info.model_name.count);
    size = sizeof(buffer);
    if (legacy_equipment_power_BatteryInfo_1_0_serialize_(&info, buffer, &size)) return 1;
    printf("  \"info\": {\n");
    hex("payload", buffer, size, ",");
    printf("    \"expect\": {\"temperatureC\": 21.25, \"voltage\": 15.5, \"current\": -2.75, "
           "\"averagePower10sW\": -42.5, \"remainingWh\": 96.5, \"fullWh\": 144.0, \"hoursToFull\": 1.5, "
           "\"statusFlags\": 265, \"stateOfHealthPct\": 93, \"stateOfChargePct\": 67, "
           "\"stateOfChargeStdevPct\": 0, \"batteryId\": 5, \"modelInstanceId\": 123456789, "
           "\"modelName\": \"SOSUB pod\"}\n  },\n");

    /* battery Status, as BatteryStatusToTransmitBuffer() fills it (272.15 offset included) */
    reg_drone_service_battery_Status_0_2 status;
    memset(&status, 0, sizeof(status));
    status.heartbeat.readiness.value = 3;
    status.heartbeat.health.value = 0;
    status.temperature_min_max[0].kelvin = 20.5f + 272.15f;
    status.temperature_min_max[1].kelvin = 27.25f + 272.15f;
    status.available_charge.coulomb = 6.5f * 3600.0f;
    status._error.value = 0;
    status.cell_voltages.count = 4;
    status.cell_voltages.elements[0] = 3.875f;
    status.cell_voltages.elements[1] = 3.8125f;
    status.cell_voltages.elements[2] = 3.75f;
    status.cell_voltages.elements[3] = 3.9375f;
    size = sizeof(buffer);
    if (reg_drone_service_battery_Status_0_2_serialize_(&status, buffer, &size)) return 2;
    printf("  \"status\": {\n");
    hex("payload", buffer, size, ",");
    printf("    \"expect\": {\"readiness\": 3, \"health\": 0, \"temperatureMinC\": 20.5, "
           "\"temperatureMaxC\": 27.25, \"availableChargeC\": 23400.0, \"availableChargeAh\": 6.5, "
           "\"error\": 0, \"cellVoltages\": [3.875, 3.8125, 3.75, 3.9375]}\n  },\n");

    /* energy source */
    reg_drone_physics_electricity_SourceTs_0_1 source;
    memset(&source, 0, sizeof(source));
    source.timestamp.microsecond = 987654321012ULL;
    source.value.power.current.ampere = -2.75f;
    source.value.power.voltage.volt = 15.5f;
    source.value.energy.joule = 347400.0f;
    source.value.full_energy.joule = 518400.0f;
    size = sizeof(buffer);
    if (reg_drone_physics_electricity_SourceTs_0_1_serialize_(&source, buffer, &size)) return 3;
    printf("  \"source\": {\n");
    hex("payload", buffer, size, ",");
    printf("    \"expect\": {\"timestampUs\": 987654321012, \"current\": -2.75, \"voltage\": 15.5, "
           "\"energyJ\": 347400.0, \"fullEnergyJ\": 518400.0, \"energyWh\": 96.5, \"fullEnergyWh\": 144.0}\n  },\n");

    /* battery Parameters */
    reg_drone_service_battery_Parameters_0_3 params;
    memset(&params, 0, sizeof(params));
    params.unique_id = 0x1122334455667788ULL;
    params.mass.kilogram = 1.75f;
    params.design_capacity.coulomb = 10.0f * 3600.0f;
    params.design_cell_voltage_min_max[0].volt = 3.0f;
    params.design_cell_voltage_min_max[1].volt = 4.25f;
    params.discharge_current.ampere = 30.0f;
    params.discharge_current_burst.ampere = 60.0f;
    params.charge_current.ampere = 4.5f;
    params.charge_current_fast.ampere = 9.0f;
    params.charge_termination_threshold.ampere = 0.125f;
    params.charge_voltage.volt = 4.25f;
    params.cycle_count = 321;
    params.state_of_health_pct = 93;
    params.technology.value = 110;
    params.nominal_voltage.volt = 14.5f;
    size = sizeof(buffer);
    if (reg_drone_service_battery_Parameters_0_3_serialize_(&params, buffer, &size)) return 4;
    printf("  \"params\": {\n");
    hex("payload", buffer, size, ",");
    printf("    \"expect\": {\"uniqueId\": 1234605616436508552, \"massKg\": 1.75, \"designCapacityC\": 36000.0, "
           "\"designCapacityAh\": 10.0, \"cellVoltageMin\": 3.0, \"cellVoltageMax\": 4.25, "
           "\"dischargeCurrent\": 30.0, \"dischargeCurrentBurst\": 60.0, \"chargeCurrent\": 4.5, "
           "\"chargeCurrentFast\": 9.0, \"chargeTerminationThreshold\": 0.125, \"chargeVoltage\": 4.25, "
           "\"cycleCount\": 321, \"stateOfHealthPct\": 93, \"technology\": 110, \"nominalVoltage\": 14.5}\n  },\n");

    /* register reply: bms_fault_code as get_bms_fault_code() builds it (natural32, one element) */
    uavcan_register_Access_Response_1_0 response;
    uavcan_register_Access_Response_1_0_initialize_(&response);
    response.value._tag_ = 9;
    response.value.natural32.value.count = 1;
    response.value.natural32.value.elements[0] = 0x00210480UL;
    size = sizeof(buffer);
    if (uavcan_register_Access_Response_1_0_serialize_(&response, buffer, &size)) return 5;
    printf("  \"faultCodeResponse\": {\n");
    hex("payload", buffer, size, ",");
    printf("    \"expect\": {\"tag\": 9, \"values\": [2163840]}\n  },\n");

    /* register reply: a subject ID as get_battery_info_port_id() builds it (natural16) */
    uavcan_register_Access_Response_1_0_initialize_(&response);
    response.value._tag_ = 10;
    response.value.natural16.value.count = 1;
    response.value.natural16.value.elements[0] = 4099;
    size = sizeof(buffer);
    if (uavcan_register_Access_Response_1_0_serialize_(&response, buffer, &size)) return 6;
    printf("  \"portResponse\": {\n");
    hex("payload", buffer, size, ",");
    printf("    \"expect\": {\"tag\": 10, \"values\": [4099]}\n  },\n");

    /* register read request, as a correct client must send it (empty value) */
    uavcan_register_Access_Request_1_0 request;
    uavcan_register_Access_Request_1_0_initialize_(&request);
    const char *reg = "bms_fault_code";
    request.name.name.count = strlen(reg);
    memcpy(request.name.name.elements, reg, request.name.name.count);
    uavcan_register_Value_1_0_select_empty_(&request.value);
    size = sizeof(buffer);
    if (uavcan_register_Access_Request_1_0_serialize_(&request, buffer, &size)) return 7;
    printf("  \"faultCodeRequest\": {\n");
    hex("payload", buffer, size, ",");
    printf("    \"name\": \"bms_fault_code\"\n  }\n}\n");
    return 0;
}
