// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/GimbalV1Command.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/gimbal_v1_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__GimbalV1Command__init(px4_msgs__msg__GimbalV1Command * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // param1
  // param2
  // param3
  // param4
  // param5
  // param6
  // param7
  // command
  // target_system
  // target_component
  // source_system
  // source_component
  // confirmation
  // from_external
  return true;
}

void
px4_msgs__msg__GimbalV1Command__fini(px4_msgs__msg__GimbalV1Command * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // param1
  // param2
  // param3
  // param4
  // param5
  // param6
  // param7
  // command
  // target_system
  // target_component
  // source_system
  // source_component
  // confirmation
  // from_external
}

px4_msgs__msg__GimbalV1Command *
px4_msgs__msg__GimbalV1Command__create()
{
  px4_msgs__msg__GimbalV1Command * msg = (px4_msgs__msg__GimbalV1Command *)malloc(sizeof(px4_msgs__msg__GimbalV1Command));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__GimbalV1Command));
  bool success = px4_msgs__msg__GimbalV1Command__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__GimbalV1Command__destroy(px4_msgs__msg__GimbalV1Command * msg)
{
  if (msg) {
    px4_msgs__msg__GimbalV1Command__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__GimbalV1Command__Sequence__init(px4_msgs__msg__GimbalV1Command__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__GimbalV1Command * data = NULL;
  if (size) {
    data = (px4_msgs__msg__GimbalV1Command *)calloc(size, sizeof(px4_msgs__msg__GimbalV1Command));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__GimbalV1Command__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__GimbalV1Command__fini(&data[i - 1]);
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
px4_msgs__msg__GimbalV1Command__Sequence__fini(px4_msgs__msg__GimbalV1Command__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__GimbalV1Command__fini(&array->data[i]);
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

px4_msgs__msg__GimbalV1Command__Sequence *
px4_msgs__msg__GimbalV1Command__Sequence__create(size_t size)
{
  px4_msgs__msg__GimbalV1Command__Sequence * array = (px4_msgs__msg__GimbalV1Command__Sequence *)malloc(sizeof(px4_msgs__msg__GimbalV1Command__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__GimbalV1Command__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__GimbalV1Command__Sequence__destroy(px4_msgs__msg__GimbalV1Command__Sequence * array)
{
  if (array) {
    px4_msgs__msg__GimbalV1Command__Sequence__fini(array);
  }
  free(array);
}
