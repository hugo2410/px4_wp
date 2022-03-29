// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/SensorGnssRelative.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/sensor_gnss_relative__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__SensorGnssRelative__init(px4_msgs__msg__SensorGnssRelative * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // timestamp_sample
  // device_id
  // time_utc_usec
  // reference_station_id
  // position
  // position_accuracy
  // heading
  // heading_accuracy
  // position_length
  // accuracy_length
  // gnss_fix_ok
  // differential_solution
  // relative_position_valid
  // carrier_solution_floating
  // carrier_solution_fixed
  // moving_base_mode
  // reference_position_miss
  // reference_observations_miss
  // heading_valid
  // relative_position_normalized
  return true;
}

void
px4_msgs__msg__SensorGnssRelative__fini(px4_msgs__msg__SensorGnssRelative * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // timestamp_sample
  // device_id
  // time_utc_usec
  // reference_station_id
  // position
  // position_accuracy
  // heading
  // heading_accuracy
  // position_length
  // accuracy_length
  // gnss_fix_ok
  // differential_solution
  // relative_position_valid
  // carrier_solution_floating
  // carrier_solution_fixed
  // moving_base_mode
  // reference_position_miss
  // reference_observations_miss
  // heading_valid
  // relative_position_normalized
}

px4_msgs__msg__SensorGnssRelative *
px4_msgs__msg__SensorGnssRelative__create()
{
  px4_msgs__msg__SensorGnssRelative * msg = (px4_msgs__msg__SensorGnssRelative *)malloc(sizeof(px4_msgs__msg__SensorGnssRelative));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__SensorGnssRelative));
  bool success = px4_msgs__msg__SensorGnssRelative__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__SensorGnssRelative__destroy(px4_msgs__msg__SensorGnssRelative * msg)
{
  if (msg) {
    px4_msgs__msg__SensorGnssRelative__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__SensorGnssRelative__Sequence__init(px4_msgs__msg__SensorGnssRelative__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__SensorGnssRelative * data = NULL;
  if (size) {
    data = (px4_msgs__msg__SensorGnssRelative *)calloc(size, sizeof(px4_msgs__msg__SensorGnssRelative));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__SensorGnssRelative__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__SensorGnssRelative__fini(&data[i - 1]);
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
px4_msgs__msg__SensorGnssRelative__Sequence__fini(px4_msgs__msg__SensorGnssRelative__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__SensorGnssRelative__fini(&array->data[i]);
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

px4_msgs__msg__SensorGnssRelative__Sequence *
px4_msgs__msg__SensorGnssRelative__Sequence__create(size_t size)
{
  px4_msgs__msg__SensorGnssRelative__Sequence * array = (px4_msgs__msg__SensorGnssRelative__Sequence *)malloc(sizeof(px4_msgs__msg__SensorGnssRelative__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__SensorGnssRelative__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__SensorGnssRelative__Sequence__destroy(px4_msgs__msg__SensorGnssRelative__Sequence * array)
{
  if (array) {
    px4_msgs__msg__SensorGnssRelative__Sequence__fini(array);
  }
  free(array);
}
