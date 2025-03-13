// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from nav2_dynamic_msgs:msg/ObstacleArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "nav2_dynamic_msgs/msg/detail/obstacle_array__rosidl_typesupport_introspection_c.h"
#include "nav2_dynamic_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "nav2_dynamic_msgs/msg/detail/obstacle_array__functions.h"
#include "nav2_dynamic_msgs/msg/detail/obstacle_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `obstacles`
#include "nav2_dynamic_msgs/msg/obstacle.h"
// Member `obstacles`
#include "nav2_dynamic_msgs/msg/detail/obstacle__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  nav2_dynamic_msgs__msg__ObstacleArray__init(message_memory);
}

void nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_fini_function(void * message_memory)
{
  nav2_dynamic_msgs__msg__ObstacleArray__fini(message_memory);
}

size_t nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__size_function__ObstacleArray__obstacles(
  const void * untyped_member)
{
  const nav2_dynamic_msgs__msg__Obstacle__Sequence * member =
    (const nav2_dynamic_msgs__msg__Obstacle__Sequence *)(untyped_member);
  return member->size;
}

const void * nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__get_const_function__ObstacleArray__obstacles(
  const void * untyped_member, size_t index)
{
  const nav2_dynamic_msgs__msg__Obstacle__Sequence * member =
    (const nav2_dynamic_msgs__msg__Obstacle__Sequence *)(untyped_member);
  return &member->data[index];
}

void * nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__get_function__ObstacleArray__obstacles(
  void * untyped_member, size_t index)
{
  nav2_dynamic_msgs__msg__Obstacle__Sequence * member =
    (nav2_dynamic_msgs__msg__Obstacle__Sequence *)(untyped_member);
  return &member->data[index];
}

void nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__fetch_function__ObstacleArray__obstacles(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const nav2_dynamic_msgs__msg__Obstacle * item =
    ((const nav2_dynamic_msgs__msg__Obstacle *)
    nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__get_const_function__ObstacleArray__obstacles(untyped_member, index));
  nav2_dynamic_msgs__msg__Obstacle * value =
    (nav2_dynamic_msgs__msg__Obstacle *)(untyped_value);
  *value = *item;
}

void nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__assign_function__ObstacleArray__obstacles(
  void * untyped_member, size_t index, const void * untyped_value)
{
  nav2_dynamic_msgs__msg__Obstacle * item =
    ((nav2_dynamic_msgs__msg__Obstacle *)
    nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__get_function__ObstacleArray__obstacles(untyped_member, index));
  const nav2_dynamic_msgs__msg__Obstacle * value =
    (const nav2_dynamic_msgs__msg__Obstacle *)(untyped_value);
  *item = *value;
}

bool nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__resize_function__ObstacleArray__obstacles(
  void * untyped_member, size_t size)
{
  nav2_dynamic_msgs__msg__Obstacle__Sequence * member =
    (nav2_dynamic_msgs__msg__Obstacle__Sequence *)(untyped_member);
  nav2_dynamic_msgs__msg__Obstacle__Sequence__fini(member);
  return nav2_dynamic_msgs__msg__Obstacle__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nav2_dynamic_msgs__msg__ObstacleArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "obstacles",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nav2_dynamic_msgs__msg__ObstacleArray, obstacles),  // bytes offset in struct
    NULL,  // default value
    nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__size_function__ObstacleArray__obstacles,  // size() function pointer
    nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__get_const_function__ObstacleArray__obstacles,  // get_const(index) function pointer
    nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__get_function__ObstacleArray__obstacles,  // get(index) function pointer
    nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__fetch_function__ObstacleArray__obstacles,  // fetch(index, &value) function pointer
    nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__assign_function__ObstacleArray__obstacles,  // assign(index, value) function pointer
    nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__resize_function__ObstacleArray__obstacles  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_message_members = {
  "nav2_dynamic_msgs__msg",  // message namespace
  "ObstacleArray",  // message name
  2,  // number of fields
  sizeof(nav2_dynamic_msgs__msg__ObstacleArray),
  nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_message_member_array,  // message members
  nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_init_function,  // function to initialize message memory (memory has to be allocated)
  nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_message_type_support_handle = {
  0,
  &nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_nav2_dynamic_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nav2_dynamic_msgs, msg, ObstacleArray)() {
  nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nav2_dynamic_msgs, msg, Obstacle)();
  if (!nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_message_type_support_handle.typesupport_identifier) {
    nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &nav2_dynamic_msgs__msg__ObstacleArray__rosidl_typesupport_introspection_c__ObstacleArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
