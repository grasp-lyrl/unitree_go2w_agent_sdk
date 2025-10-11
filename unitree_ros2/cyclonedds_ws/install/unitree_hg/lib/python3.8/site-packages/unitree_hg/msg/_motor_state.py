# generated from rosidl_generator_py/resource/_idl.py.em
# with input from unitree_hg:msg/MotorState.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'temperature'
# Member 'sensor'
# Member 'reserve'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MotorState(type):
    """Metaclass of message 'MotorState'."""

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
                'unitree_hg.msg.MotorState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__motor_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__motor_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__motor_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__motor_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__motor_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MotorState(metaclass=Metaclass_MotorState):
    """Message class 'MotorState'."""

    __slots__ = [
        '_mode',
        '_q',
        '_dq',
        '_ddq',
        '_tau_est',
        '_temperature',
        '_vol',
        '_sensor',
        '_motorstate',
        '_reserve',
    ]

    _fields_and_field_types = {
        'mode': 'uint8',
        'q': 'float',
        'dq': 'float',
        'ddq': 'float',
        'tau_est': 'float',
        'temperature': 'int16[2]',
        'vol': 'float',
        'sensor': 'uint32[2]',
        'motorstate': 'uint32',
        'reserve': 'uint32[4]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int16'), 2),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint32'), 2),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint32'), 4),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.mode = kwargs.get('mode', int())
        self.q = kwargs.get('q', float())
        self.dq = kwargs.get('dq', float())
        self.ddq = kwargs.get('ddq', float())
        self.tau_est = kwargs.get('tau_est', float())
        if 'temperature' not in kwargs:
            self.temperature = numpy.zeros(2, dtype=numpy.int16)
        else:
            self.temperature = numpy.array(kwargs.get('temperature'), dtype=numpy.int16)
            assert self.temperature.shape == (2, )
        self.vol = kwargs.get('vol', float())
        if 'sensor' not in kwargs:
            self.sensor = numpy.zeros(2, dtype=numpy.uint32)
        else:
            self.sensor = numpy.array(kwargs.get('sensor'), dtype=numpy.uint32)
            assert self.sensor.shape == (2, )
        self.motorstate = kwargs.get('motorstate', int())
        if 'reserve' not in kwargs:
            self.reserve = numpy.zeros(4, dtype=numpy.uint32)
        else:
            self.reserve = numpy.array(kwargs.get('reserve'), dtype=numpy.uint32)
            assert self.reserve.shape == (4, )

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
        if self.mode != other.mode:
            return False
        if self.q != other.q:
            return False
        if self.dq != other.dq:
            return False
        if self.ddq != other.ddq:
            return False
        if self.tau_est != other.tau_est:
            return False
        if all(self.temperature != other.temperature):
            return False
        if self.vol != other.vol:
            return False
        if all(self.sensor != other.sensor):
            return False
        if self.motorstate != other.motorstate:
            return False
        if all(self.reserve != other.reserve):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'mode' field must be an unsigned integer in [0, 255]"
        self._mode = value

    @property
    def q(self):
        """Message field 'q'."""
        return self._q

    @q.setter
    def q(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'q' field must be of type 'float'"
        self._q = value

    @property
    def dq(self):
        """Message field 'dq'."""
        return self._dq

    @dq.setter
    def dq(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'dq' field must be of type 'float'"
        self._dq = value

    @property
    def ddq(self):
        """Message field 'ddq'."""
        return self._ddq

    @ddq.setter
    def ddq(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ddq' field must be of type 'float'"
        self._ddq = value

    @property
    def tau_est(self):
        """Message field 'tau_est'."""
        return self._tau_est

    @tau_est.setter
    def tau_est(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tau_est' field must be of type 'float'"
        self._tau_est = value

    @property
    def temperature(self):
        """Message field 'temperature'."""
        return self._temperature

    @temperature.setter
    def temperature(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int16, \
                "The 'temperature' numpy.ndarray() must have the dtype of 'numpy.int16'"
            assert value.size == 2, \
                "The 'temperature' numpy.ndarray() must have a size of 2"
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
                 len(value) == 2 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -32768 and val < 32768 for val in value)), \
                "The 'temperature' field must be a set or sequence with length 2 and each value of type 'int' and each integer in [-32768, 32767]"
        self._temperature = numpy.array(value, dtype=numpy.int16)

    @property
    def vol(self):
        """Message field 'vol'."""
        return self._vol

    @vol.setter
    def vol(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vol' field must be of type 'float'"
        self._vol = value

    @property
    def sensor(self):
        """Message field 'sensor'."""
        return self._sensor

    @sensor.setter
    def sensor(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint32, \
                "The 'sensor' numpy.ndarray() must have the dtype of 'numpy.uint32'"
            assert value.size == 2, \
                "The 'sensor' numpy.ndarray() must have a size of 2"
            self._sensor = value
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
                "The 'sensor' field must be a set or sequence with length 2 and each value of type 'int' and each unsigned integer in [0, 4294967295]"
        self._sensor = numpy.array(value, dtype=numpy.uint32)

    @property
    def motorstate(self):
        """Message field 'motorstate'."""
        return self._motorstate

    @motorstate.setter
    def motorstate(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'motorstate' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'motorstate' field must be an unsigned integer in [0, 4294967295]"
        self._motorstate = value

    @property
    def reserve(self):
        """Message field 'reserve'."""
        return self._reserve

    @reserve.setter
    def reserve(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint32, \
                "The 'reserve' numpy.ndarray() must have the dtype of 'numpy.uint32'"
            assert value.size == 4, \
                "The 'reserve' numpy.ndarray() must have a size of 4"
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
                 len(value) == 4 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 4294967296 for val in value)), \
                "The 'reserve' field must be a set or sequence with length 4 and each value of type 'int' and each unsigned integer in [0, 4294967295]"
        self._reserve = numpy.array(value, dtype=numpy.uint32)
