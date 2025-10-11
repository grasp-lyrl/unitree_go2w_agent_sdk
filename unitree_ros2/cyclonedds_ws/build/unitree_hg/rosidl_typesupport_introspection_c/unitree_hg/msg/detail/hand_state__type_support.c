// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from unitree_hg:msg/HandState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "unitree_hg/msg/detail/hand_state__rosidl_typesupport_introspection_c.h"
#include "unitree_hg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "unitree_hg/msg/detail/hand_state__functions.h"
#include "unitree_hg/msg/detail/hand_state__struct.h"


// Include directives for member types
// Member `motor_state`
#include "unitree_hg/msg/motor_state.h"
// Member `motor_state`
#include "unitree_hg/msg/detail/motor_state__rosidl_typesupport_introspection_c.h"
// Member `imu_state`
#include "unitree_hg/msg/imu_state.h"
// Member `imu_state`
#include "unitree_hg/msg/detail/imu_state__rosidl_typesupport_introspection_c.h"
// Member `press_sensor_state`
#include "unitree_hg/msg/press_sensor_state.h"
// Member `press_sensor_state`
#include "unitree_hg/msg/detail/press_sensor_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void HandState__rosidl_typesupport_introspection_c__HandState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  unitree_hg__msg__HandState__init(message_memory);
}

void HandState__rosidl_typesupport_introspection_c__HandState_fini_function(void * message_memory)
{
  unitree_hg__msg__HandState__fini(message_memory);
}

size_t HandState__rosidl_typesupport_introspection_c__size_function__MotorState__motor_state(
  const void * untyped_member)
{
  const unitree_hg__msg__MotorState__Sequence * member =
    (const unitree_hg__msg__MotorState__Sequence *)(untyped_member);
  return member->size;
}

const void * HandState__rosidl_typesupport_introspection_c__get_const_function__MotorState__motor_state(
  const void * untyped_member, size_t index)
{
  const unitree_hg__msg__MotorState__Sequence * member =
    (const unitree_hg__msg__MotorState__Sequence *)(untyped_member);
  return &member->data[index];
}

void * HandState__rosidl_typesupport_introspection_c__get_function__MotorState__motor_state(
  void * untyped_member, size_t index)
{
  unitree_hg__msg__MotorState__Sequence * member =
    (unitree_hg__msg__MotorState__Sequence *)(untyped_member);
  return &member->data[index];
}

bool HandState__rosidl_typesupport_introspection_c__resize_function__MotorState__motor_state(
  void * untyped_member, size_t size)
{
  unitree_hg__msg__MotorState__Sequence * member =
    (unitree_hg__msg__MotorState__Sequence *)(untyped_member);
  unitree_hg__msg__MotorState__Sequence__fini(member);
  return unitree_hg__msg__MotorState__Sequence__init(member, size);
}

size_t HandState__rosidl_typesupport_introspection_c__size_function__PressSensorState__press_sensor_state(
  const void * untyped_member)
{
  const unitree_hg__msg__PressSensorState__Sequence * member =
    (const unitree_hg__msg__PressSensorState__Sequence *)(untyped_member);
  return member->size;
}

const void * HandState__rosidl_typesupport_introspection_c__get_const_function__PressSensorState__press_sensor_state(
  const void * untyped_member, size_t index)
{
  const unitree_hg__msg__PressSensorState__Sequence * member =
    (const unitree_hg__msg__PressSensorState__Sequence *)(untyped_member);
  return &member->data[index];
}

void * HandState__rosidl_typesupport_introspection_c__get_function__PressSensorState__press_sensor_state(
  void * untyped_member, size_t index)
{
  unitree_hg__msg__PressSensorState__Sequence * member =
    (unitree_hg__msg__PressSensorState__Sequence *)(untyped_member);
  return &member->data[index];
}

bool HandState__rosidl_typesupport_introspection_c__resize_function__PressSensorState__press_sensor_state(
  void * untyped_member, size_t size)
{
  unitree_hg__msg__PressSensorState__Sequence * member =
    (unitree_hg__msg__PressSensorState__Sequence *)(untyped_member);
  unitree_hg__msg__PressSensorState__Sequence__fini(member);
  return unitree_hg__msg__PressSensorState__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember HandState__rosidl_typesupport_introspection_c__HandState_message_member_array[6] = {
  {
    "motor_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg__msg__HandState, motor_state),  // bytes offset in struct
    NULL,  // default value
    HandState__rosidl_typesupport_introspection_c__size_function__MotorState__motor_state,  // size() function pointer
    HandState__rosidl_typesupport_introspection_c__get_const_function__MotorState__motor_state,  // get_const(index) function pointer
    HandState__rosidl_typesupport_introspection_c__get_function__MotorState__motor_state,  // get(index) function pointer
    HandState__rosidl_typesupport_introspection_c__resize_function__MotorState__motor_state  // resize(index) function pointer
  },
  {
    "imu_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg__msg__HandState, imu_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "press_sensor_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg__msg__HandState, press_sensor_state),  // bytes offset in struct
    NULL,  // default value
    HandState__rosidl_typesupport_introspection_c__size_function__PressSensorState__press_sensor_state,  // size() function pointer
    HandState__rosidl_typesupport_introspection_c__get_const_function__PressSensorState__press_sensor_state,  // get_const(index) function pointer
    HandState__rosidl_typesupport_introspection_c__get_function__PressSensorState__press_sensor_state,  // get(index) function pointer
    HandState__rosidl_typesupport_introspection_c__resize_function__PressSensorState__press_sensor_state  // resize(index) function pointer
  },
  {
    "power_v",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg__msg__HandState, power_v),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "power_a",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg__msg__HandState, power_a),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reserve",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(unitree_hg__msg__HandState, reserve),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers HandState__rosidl_typesupport_introspection_c__HandState_message_members = {
  "unitree_hg__msg",  // message namespace
  "HandState",  // message name
  6,  // number of fields
  sizeof(unitree_hg__msg__HandState),
  HandState__rosidl_typesupport_introspection_c__HandState_message_member_array,  // message members
  HandState__rosidl_typesupport_introspection_c__HandState_init_function,  // function to initialize message memory (memory has to be allocated)
  HandState__rosidl_typesupport_introspection_c__HandState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t HandState__rosidl_typesupport_introspection_c__HandState_message_type_support_handle = {
  0,
  &HandState__rosidl_typesupport_introspection_c__HandState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_unitree_hg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unitree_hg, msg, HandState)() {
  HandState__rosidl_typesupport_introspection_c__HandState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unitree_hg, msg, MotorState)();
  HandState__rosidl_typesupport_introspection_c__HandState_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unitree_hg, msg, IMUState)();
  HandState__rosidl_typesupport_introspection_c__HandState_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unitree_hg, msg, PressSensorState)();
  if (!HandState__rosidl_typesupport_introspection_c__HandState_message_type_support_handle.typesupport_identifier) {
    HandState__rosidl_typesupport_introspection_c__HandState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &HandState__rosidl_typesupport_introspection_c__HandState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
