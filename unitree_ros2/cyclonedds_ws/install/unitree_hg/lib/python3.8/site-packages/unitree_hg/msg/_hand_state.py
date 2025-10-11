# generated from rosidl_generator_py/resource/_idl.py.em
# with input from unitree_hg:msg/HandState.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'reserve'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_HandState(type):
    """Metaclass of message 'HandState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('unitree_hg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'unitree_hg.msg.HandState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__hand_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__hand_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__hand_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__hand_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__hand_state

            from unitree_hg.msg import IMUState
            if IMUState.__class__._TYPE_SUPPORT is None:
                IMUState.__class__.__import_type_support__()

            from unitree_hg.msg import MotorState
            if MotorState.__class__._TYPE_SUPPORT is None:
                MotorState.__class__.__import_type_support__()

            from unitree_hg.msg import PressSensorState
            if PressSensorState.__class__._TYPE_SUPPORT is None:
                PressSensorState.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class HandState(metaclass=Metaclass_HandState):
    """Message class 'HandState'."""

    __slots__ = [
        '_motor_state',
        '_imu_state',
        '_press_sensor_state',
        '_power_v',
        '_power_a',
        '_reserve',
    ]

    _fields_and_field_types = {
        'motor_state': 'sequence<unitree_hg/MotorState>',
        'imu_state': 'unitree_hg/IMUState',
        'press_sensor_state': 'sequence<unitree_hg/PressSensorState>',
        'power_v': 'float',
        'power_a': 'float',
        'reserve': 'uint32[2]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['unitree_hg', 'msg'], 'MotorState')),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['unitree_hg', 'msg'], 'IMUState'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['unitree_hg', 'msg'], 'PressSensorState')),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint32'), 2),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.motor_state = kwargs.get('motor_state', [])
        from unitree_hg.msg import IMUState
        self.imu_state = kwargs.get('imu_state', IMUState())
        self.press_sensor_state = kwargs.get('press_sensor_state', [])
        self.power_v = kwargs.get('power_v', float())
        self.power_a = kwargs.get('power_a', float())
        if 'reserve' not in kwargs:
            self.reserve = numpy.zeros(2, dtype=numpy.uint32)
        else:
            self.reserve = numpy.array(kwargs.get('reserve'), dtype=numpy.uint32)
            assert self.reserve.shape == (2, )

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.motor_state != other.motor_state:
            return False
        if self.imu_state != other.imu_state:
            return False
        if self.press_sensor_state != other.press_sensor_state:
            return False
        if self.power_v != other.power_v:
            return False
        if self.power_a != other.power_a:
            return False
        if all(self.reserve != other.reserve):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def motor_state(self):
        """Message field 'motor_state'."""
        return self._motor_state

    @motor_state.setter
    def motor_state(self, value):
        if __debug__:
            from unitree_hg.msg import MotorState
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, MotorState) for v in value) and
                 True), \
                "The 'motor_state' field must be a set or sequence and each value of type 'MotorState'"
        self._motor_state = value

    @property
    def imu_state(self):
        """Message field 'imu_state'."""
        return self._imu_state

    @imu_state.setter
    def imu_state(self, value):
        if __debug__:
            from unitree_hg.msg import IMUState
            assert \
                isinstance(value, IMUState), \
                "The 'imu_state' field must be a sub message of type 'IMUState'"
        self._imu_state = value

    @property
    def press_sensor_state(self):
        """Message field 'press_sensor_state'."""
        return self._press_sensor_state

    @press_sensor_state.setter
    def press_sensor_state(self, value):
        if __debug__:
            from unitree_hg.msg import PressSensorState
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, PressSensorState) for v in value) and
                 True), \
                "The 'press_sensor_state' field must be a set or sequence and each value of type 'PressSensorState'"
        self._press_sensor_state = value

    @property
    def power_v(self):
        """Message field 'power_v'."""
        return self._power_v

    @power_v.setter
    def power_v(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'power_v' field must be of type 'float'"
        self._power_v = value

    @property
    def power_a(self):
        """Message field 'power_a'."""
        return self._power_a

    @power_a.setter
    def power_a(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'power_a' field must be of type 'float'"
        self._power_a = value

    @property
    def reserve(self):
        """Message field 'reserve'."""
        return self._reserve

    @reserve.setter
    def reserve(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint32, \
                "The 'reserve' numpy.ndarray() must have the dtype of 'numpy.uint32'"
            assert value.size == 2, \
                "The 'reserve' numpy.ndarray() must have a size of 2"
            self._reserve = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 2 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 4294967296 for val in value)), \
                "The 'reserve' field must be a set or sequence with length 2 and each value of type 'int' and each unsigned integer in [0, 4294967295]"
        self._reserve = numpy.array(value, dtype=numpy.uint32)
