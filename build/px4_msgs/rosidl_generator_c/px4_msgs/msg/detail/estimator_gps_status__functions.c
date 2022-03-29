// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/EstimatorGpsStatus.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/estimator_gps_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__EstimatorGpsStatus__init(px4_msgs__msg__EstimatorGpsStatus * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // timestamp_sample
  // checks_passed
  // check_fail_gps_fix
  // check_fail_min_sat_count
  // check_fail_max_pdop
  // check_fail_max_horz_err
  // check_fail_max_vert_err
  // check_fail_max_spd_err
  // check_fail_max_horz_drift
  // check_fail_max_vert_drift
  // check_fail_max_horz_spd_err
  // check_fail_max_vert_spd_err
  // position_drift_rate_horizontal_m_s
  // position_drift_rate_vertical_m_s
  // filtered_horizontal_speed_m_s
  return true;
}

void
px4_msgs__msg__EstimatorGpsStatus__fini(px4_msgs__msg__EstimatorGpsStatus * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // timestamp_sample
  // checks_passed
  // check_fail_gps_fix
  // check_fail_min_sat_count
  // check_fail_max_pdop
  // check_fail_max_horz_err
  // check_fail_max_vert_err
  // check_fail_max_spd_err
  // check_fail_max_horz_drift
  // check_fail_max_vert_drift
  // check_fail_max_horz_spd_err
  // check_fail_max_vert_spd_err
  // position_drift_rate_horizontal_m_s
  // position_drift_rate_vertical_m_s
  // filtered_horizontal_speed_m_s
}

px4_msgs__msg__EstimatorGpsStatus *
px4_msgs__msg__EstimatorGpsStatus__create()
{
  px4_msgs__msg__EstimatorGpsStatus * msg = (px4_msgs__msg__EstimatorGpsStatus *)malloc(sizeof(px4_msgs__msg__EstimatorGpsStatus));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__EstimatorGpsStatus));
  bool success = px4_msgs__msg__EstimatorGpsStatus__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__EstimatorGpsStatus__destroy(px4_msgs__msg__EstimatorGpsStatus * msg)
{
  if (msg) {
    px4_msgs__msg__EstimatorGpsStatus__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__EstimatorGpsStatus__Sequence__init(px4_msgs__msg__EstimatorGpsStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__EstimatorGpsStatus * data = NULL;
  if (size) {
    data = (px4_msgs__msg__EstimatorGpsStatus *)calloc(size, sizeof(px4_msgs__msg__EstimatorGpsStatus));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__EstimatorGpsStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__EstimatorGpsStatus__fini(&data[i - 1]);
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
px4_msgs__msg__EstimatorGpsStatus__Sequence__fini(px4_msgs__msg__EstimatorGpsStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__EstimatorGpsStatus__fini(&array->data[i]);
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

px4_msgs__msg__EstimatorGpsStatus__Sequence *
px4_msgs__msg__EstimatorGpsStatus__Sequence__create(size_t size)
{
  px4_msgs__msg__EstimatorGpsStatus__Sequence * array = (px4_msgs__msg__EstimatorGpsStatus__Sequence *)malloc(sizeof(px4_msgs__msg__EstimatorGpsStatus__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__EstimatorGpsStatus__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__EstimatorGpsStatus__Sequence__destroy(px4_msgs__msg__EstimatorGpsStatus__Sequence * array)
{
  if (array) {
    px4_msgs__msg__EstimatorGpsStatus__Sequence__fini(array);
  }
  free(array);
}
