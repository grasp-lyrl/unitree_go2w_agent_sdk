// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from unitree_hg:msg/MainBoardState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "unitree_hg/msg/detail/main_board_state__rosidl_typesupport_introspection_c.h"
#include "unitree_hg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "unitree_hg/msg/detail/main_board_state__functions.h"
#include "unitree_hg/msg/detail/main_board_state__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void MainBoardState__rosidl_typesupport_introspection_c__MainBoardState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  unitree_hg__msg__MainBoardState__init(message_memory);
}

void MainBoardState__rosidl_typesupport_introspection_c__MainBoardState_fini_function(void * message_memory)
{
  unitree_hg__msg__MainBoardState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MainBoardState__rosidl_typesupport_introspection_c__MainBoardState_message_member_array[4] = {
  {
    "fan_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(unitree_hg__msg__MainBoardState, fan_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "temperature",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(unitree_hg__msg__MainBoardState, temperature),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(unitree_hg__msg__MainBoardState, value),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(unitree_hg__msg__MainBoardState, state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MainBoardState__rosidl_typesupport_introspection_c__MainBoardState_message_members = {
  "unitree_hg__msg",  // message namespace
  "MainBoardState",  // message name
  4,  // number of fields
  sizeof(unitree_hg__msg__MainBoardState),
  MainBoardState__rosidl_typesupport_introspection_c__MainBoardState_message_member_array,  // message members
  MainBoardState__rosidl_typesupport_introspection_c__MainBoardState_init_function,  // function to initialize message memory (memory has to be allocated)
  MainBoardState__rosidl_typesupport_introspection_c__MainBoardState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MainBoardState__rosidl_typesupport_introspection_c__MainBoardState_message_type_support_handle = {
  0,
  &MainBoardState__rosidl_typesupport_introspection_c__MainBoardState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_unitree_hg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unitree_hg, msg, MainBoardState)() {
  if (!MainBoardState__rosidl_typesupport_introspection_c__MainBoardState_message_type_support_handle.typesupport_identifier) {
    MainBoardState__rosidl_typesupport_introspection_c__MainBoardState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MainBoardState__rosidl_typesupport_introspection_c__MainBoardState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
