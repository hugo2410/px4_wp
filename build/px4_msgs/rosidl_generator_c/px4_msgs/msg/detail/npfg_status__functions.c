// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/NpfgStatus.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/npfg_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__NpfgStatus__init(px4_msgs__msg__NpfgStatus * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // wind_est_valid
  // lat_accel
  // lat_accel_ff
  // bearing_feas
  // bearing_feas_on_track
  // signed_track_error
  // track_error_bound
  // airspeed_ref
  // bearing
  // heading_ref
  // min_ground_speed_ref
  // adapted_period
  // p_gain
  // time_const
  return true;
}

void
px4_msgs__msg__NpfgStatus__fini(px4_msgs__msg__NpfgStatus * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // wind_est_valid
  // lat_accel
  // lat_accel_ff
  // bearing_feas
  // bearing_feas_on_track
  // signed_track_error
  // track_error_bound
  // airspeed_ref
  // bearing
  // heading_ref
  // min_ground_speed_ref
  // adapted_period
  // p_gain
  // time_const
}

px4_msgs__msg__NpfgStatus *
px4_msgs__msg__NpfgStatus__create()
{
  px4_msgs__msg__NpfgStatus * msg = (px4_msgs__msg__NpfgStatus *)malloc(sizeof(px4_msgs__msg__NpfgStatus));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__NpfgStatus));
  bool success = px4_msgs__msg__NpfgStatus__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__NpfgStatus__destroy(px4_msgs__msg__NpfgStatus * msg)
{
  if (msg) {
    px4_msgs__msg__NpfgStatus__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__NpfgStatus__Sequence__init(px4_msgs__msg__NpfgStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__NpfgStatus * data = NULL;
  if (size) {
    data = (px4_msgs__msg__NpfgStatus *)calloc(size, sizeof(px4_msgs__msg__NpfgStatus));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__NpfgStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__NpfgStatus__fini(&data[i - 1]);
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
px4_msgs__msg__NpfgStatus__Sequence__fini(px4_msgs__msg__NpfgStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__NpfgStatus__fini(&array->data[i]);
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

px4_msgs__msg__NpfgStatus__Sequence *
px4_msgs__msg__NpfgStatus__Sequence__create(size_t size)
{
  px4_msgs__msg__NpfgStatus__Sequence * array = (px4_msgs__msg__NpfgStatus__Sequence *)malloc(sizeof(px4_msgs__msg__NpfgStatus__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__NpfgStatus__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__NpfgStatus__Sequence__destroy(px4_msgs__msg__NpfgStatus__Sequence * array)
{
  if (array) {
    px4_msgs__msg__NpfgStatus__Sequence__fini(array);
  }
  free(array);
}
