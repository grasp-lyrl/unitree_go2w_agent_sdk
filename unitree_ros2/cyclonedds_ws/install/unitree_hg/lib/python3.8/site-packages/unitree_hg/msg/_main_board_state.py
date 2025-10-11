# generated from rosidl_generator_py/resource/_idl.py.em
# with input from unitree_hg:msg/MainBoardState.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'fan_state'
# Member 'temperature'
# Member 'value'
# Member 'state'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MainBoardState(type):
    """Metaclass of message 'MainBoardState'."""

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
                'unitree_hg.msg.MainBoardState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__main_board_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__main_board_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__main_board_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__main_board_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__main_board_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MainBoardState(metaclass=Metaclass_MainBoardState):
    """Message class 'MainBoardState'."""

    __slots__ = [
        '_fan_state',
        '_temperature',
        '_value',
        '_state',
    ]

    _fields_and_field_types = {
        'fan_state': 'uint16[6]',
        'temperature': 'int16[6]',
        'value': 'float[6]',
        'state': 'uint32[6]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint16'), 6),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int16'), 6),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 6),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint32'), 6),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        if 'fan_state' not in kwargs:
            self.fan_state = numpy.zeros(6, dtype=numpy.uint16)
        else:
            self.fan_state = numpy.array(kwargs.get('fan_state'), dtype=numpy.uint16)
            assert self.fan_state.shape == (6, )
        if 'temperature' not in kwargs:
            self.temperature = numpy.zeros(6, dtype=numpy.int16)
        else:
            self.temperature = numpy.array(kwargs.get('temperature'), dtype=numpy.int16)
            assert self.temperature.shape == (6, )
        if 'value' not in kwargs:
            self.value = numpy.zeros(6, dtype=numpy.float32)
        else:
            self.value = numpy.array(kwargs.get('value'), dtype=numpy.float32)
            assert self.value.shape == (6, )
        if 'state' not in kwargs:
            self.state = numpy.zeros(6, dtype=numpy.uint32)
        else:
            self.state = numpy.array(kwargs.get('state'), dtype=numpy.uint32)
            assert self.state.shape == (6, )

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
        if all(self.fan_state != other.fan_state):
            return False
        if all(self.temperature != other.temperature):
            return False
        if all(self.value != other.value):
            return False
        if all(self.state != other.state):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def fan_state(self):
        """Message field 'fan_state'."""
        return self._fan_state

    @fan_state.setter
    def fan_state(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint16, \
                "The 'fan_state' numpy.ndarray() must have the dtype of 'numpy.uint16'"
            assert value.size == 6, \
                "The 'fan_state' numpy.ndarray() must have a size of 6"
            self._fan_state = value
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
                 len(value) == 6 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'fan_state' field must be a set or sequence with length 6 and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._fan_state = numpy.array(value, dtype=numpy.uint16)

    @property
    def temperature(self):
        """Message field 'temperature'."""
        return self._temperature

    @temperature.setter
    def temperature(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int16, \
                "The 'temperature' numpy.ndarray() must have the dtype of 'numpy.int16'"
            assert value.size == 6, \
                "The 'temperature' numpy.ndarray() must have a size of 6"
            self._temperature = value
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
                 len(value) == 6 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -32768 and val < 32768 for val in value)), \
                "The 'temperature' field must be a set or sequence with length 6 and each value of type 'int' and each integer in [-32768, 32767]"
        self._temperature = numpy.array(value, dtype=numpy.int16)

    @property
    def value(self):
        """Message field 'value'."""
        return self._value

    @value.setter
    def value(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'value' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 6, \
                "The 'value' numpy.ndarray() must have a size of 6"
            self._value = value
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
                 len(value) == 6 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'value' field must be a set or sequence with length 6 and each value of type 'float'"
        self._value = numpy.array(value, dtype=numpy.float32)

    @property
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint32, \
                "The 'state' numpy.ndarray() must have the dtype of 'numpy.uint32'"
            assert value.size == 6, \
                "The 'state' numpy.ndarray() must have a size of 6"
            self._state = value
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
                 len(value) == 6 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 4294967296 for val in value)), \
                "The 'state' field must be a set or sequence with length 6 and each value of type 'int' and each unsigned integer in [0, 4294967295]"
        self._state = numpy.array(value, dtype=numpy.uint32)
