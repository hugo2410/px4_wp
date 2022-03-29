// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/VehicleStatusFlags.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/vehicle_status_flags__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__VehicleStatusFlags__init(px4_msgs__msg__VehicleStatusFlags * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // calibration_enabled
  // system_sensors_initialized
  // system_hotplug_timeout
  // auto_mission_available
  // angular_velocity_valid
  // attitude_valid
  // local_altitude_valid
  // local_position_valid
  // local_velocity_valid
  // global_position_valid
  // gps_position_valid
  // home_position_valid
  // power_input_valid
  // battery_healthy
  // escs_error
  // escs_failure
  // position_reliant_on_gps
  // position_reliant_on_optical_flow
  // position_reliant_on_vision_position
  // dead_reckoning
  // circuit_breaker_engaged_power_check
  // circuit_breaker_engaged_airspd_check
  // circuit_breaker_engaged_enginefailure_check
  // circuit_breaker_flight_termination_disabled
  // circuit_breaker_engaged_usb_check
  // circuit_breaker_engaged_posfailure_check
  // circuit_breaker_vtol_fw_arming_check
  // offboard_control_signal_lost
  // rc_signal_found_once
  // rc_calibration_in_progress
  // rc_calibration_valid
  // vtol_transition_failure
  // usb_connected
  // sd_card_detected_once
  // avoidance_system_required
  // avoidance_system_valid
  // parachute_system_present
  // parachute_system_healthy
  return true;
}

void
px4_msgs__msg__VehicleStatusFlags__fini(px4_msgs__msg__VehicleStatusFlags * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // calibration_enabled
  // system_sensors_initialized
  // system_hotplug_timeout
  // auto_mission_available
  // angular_velocity_valid
  // attitude_valid
  // local_altitude_valid
  // local_position_valid
  // local_velocity_valid
  // global_position_valid
  // gps_position_valid
  // home_position_valid
  // power_input_valid
  // battery_healthy
  // escs_error
  // escs_failure
  // position_reliant_on_gps
  // position_reliant_on_optical_flow
  // position_reliant_on_vision_position
  // dead_reckoning
  // circuit_breaker_engaged_power_check
  // circuit_breaker_engaged_airspd_check
  // circuit_breaker_engaged_enginefailure_check
  // circuit_breaker_flight_termination_disabled
  // circuit_breaker_engaged_usb_check
  // circuit_breaker_engaged_posfailure_check
  // circuit_breaker_vtol_fw_arming_check
  // offboard_control_signal_lost
  // rc_signal_found_once
  // rc_calibration_in_progress
  // rc_calibration_valid
  // vtol_transition_failure
  // usb_connected
  // sd_card_detected_once
  // avoidance_system_required
  // avoidance_system_valid
  // parachute_system_present
  // parachute_system_healthy
}

px4_msgs__msg__VehicleStatusFlags *
px4_msgs__msg__VehicleStatusFlags__create()
{
  px4_msgs__msg__VehicleStatusFlags * msg = (px4_msgs__msg__VehicleStatusFlags *)malloc(sizeof(px4_msgs__msg__VehicleStatusFlags));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__VehicleStatusFlags));
  bool success = px4_msgs__msg__VehicleStatusFlags__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__VehicleStatusFlags__destroy(px4_msgs__msg__VehicleStatusFlags * msg)
{
  if (msg) {
    px4_msgs__msg__VehicleStatusFlags__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__VehicleStatusFlags__Sequence__init(px4_msgs__msg__VehicleStatusFlags__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__VehicleStatusFlags * data = NULL;
  if (size) {
    data = (px4_msgs__msg__VehicleStatusFlags *)calloc(size, sizeof(px4_msgs__msg__VehicleStatusFlags));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__VehicleStatusFlags__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__VehicleStatusFlags__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
px4_msgs__msg__VehicleStatusFlags__Sequence__fini(px4_msgs__msg__VehicleStatusFlags__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__VehicleStatusFlags__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

px4_msgs__msg__VehicleStatusFlags__Sequence *
px4_msgs__msg__VehicleStatusFlags__Sequence__create(size_t size)
{
  px4_msgs__msg__VehicleStatusFlags__Sequence * array = (px4_msgs__msg__VehicleStatusFlags__Sequence *)malloc(sizeof(px4_msgs__msg__VehicleStatusFlags__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__VehicleStatusFlags__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__VehicleStatusFlags__Sequence__destroy(px4_msgs__msg__VehicleStatusFlags__Sequence * array)
{
  if (array) {
    px4_msgs__msg__VehicleStatusFlags__Sequence__fini(array);
  }
  free(array);
}
