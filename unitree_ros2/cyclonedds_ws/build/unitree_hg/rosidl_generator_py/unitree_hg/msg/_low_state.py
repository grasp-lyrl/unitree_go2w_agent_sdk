# generated from rosidl_generator_py/resource/_idl.py.em
# with input from unitree_hg:msg/LowState.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'version'
# Member 'wireless_remote'
# Member 'reserve'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LowState(type):
    """Metaclass of message 'LowState'."""

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
                'unitree_hg.msg.LowState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__low_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__low_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__low_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__low_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__low_state

            from unitree_hg.msg import IMUState
            if IMUState.__class__._TYPE_SUPPORT is None:
                IMUState.__class__.__import_type_support__()

            from unitree_hg.msg import MotorState
            if MotorState.__class__._TYPE_SUPPORT is None:
                MotorState.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LowState(metaclass=Metaclass_LowState):
    """Message class 'LowState'."""

    __slots__ = [
        '_version',
        '_mode_pr',
        '_mode_machine',
        '_tick',
        '_imu_state',
        '_motor_state',
        '_wireless_remote',
        '_reserve',
        '_crc',
    ]

    _fields_and_field_types = {
        'version': 'uint32[2]',
        'mode_pr': 'uint8',
        'mode_machine': 'uint8',
        'tick': 'uint32',
        'imu_state': 'unitree_hg/IMUState',
        'motor_state': 'unitree_hg/MotorState[35]',
        'wireless_remote': 'uint8[40]',
        'reserve': 'uint32[4]',
        'crc': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint32'), 2),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['unitree_hg', 'msg'], 'IMUState'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.NamespacedType(['unitree_hg', 'msg'], 'MotorState'), 35),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint8'), 40),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint32'), 4),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        if 'version' not in kwargs:
            self.version = numpy.zeros(2, dtype=numpy.uint32)
        else:
            self.version = numpy.array(kwargs.get('version'), dtype=numpy.uint32)
            assert self.version.shape == (2, )
        self.mode_pr = kwargs.get('mode_pr', int())
        self.mode_machine = kwargs.get('mode_machine', int())
        self.tick = kwargs.get('tick', int())
        from unitree_hg.msg import IMUState
        self.imu_state = kwargs.get('imu_state', IMUState())
        from unitree_hg.msg import MotorState
        self.motor_state = kwargs.get(
            'motor_state',
            [MotorState() for x in range(35)]
        )
        if 'wireless_remote' not in kwargs:
            self.wireless_remote = numpy.zeros(40, dtype=numpy.uint8)
        else:
            self.wireless_remote = numpy.array(kwargs.get('wireless_remote'), dtype=numpy.uint8)
            assert self.wireless_remote.shape == (40, )
        if 'reserve' not in kwargs:
            self.reserve = numpy.zeros(4, dtype=numpy.uint32)
        else:
            self.reserve = numpy.array(kwargs.get('reserve'), dtype=numpy.uint32)
            assert self.reserve.shape == (4, )
        self.crc = kwargs.get('crc', int())

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
        if all(self.version != other.version):
            return False
        if self.mode_pr != other.mode_pr:
            return False
        if self.mode_machine != other.mode_machine:
            return False
        if self.tick != other.tick:
            return False
        if self.imu_state != other.imu_state:
            return False
        if self.motor_state != other.motor_state:
            return False
        if all(self.wireless_remote != other.wireless_remote):
            return False
        if all(self.reserve != other.reserve):
            return False
        if self.crc != other.crc:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def version(self):
        """Message field 'version'."""
        return self._version

    @version.setter
    def version(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint32, \
                "The 'version' numpy.ndarray() must have the dtype of 'numpy.uint32'"
            assert value.size == 2, \
                "The 'version' numpy.ndarray() must have a size of 2"
            self._version = value
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
                "The 'version' field must be a set or sequence with length 2 and each value of type 'int' and each unsigned integer in [0, 4294967295]"
        self._version = numpy.array(value, dtype=numpy.uint32)

    @property
    def mode_pr(self):
        """Message field 'mode_pr'."""
        return self._mode_pr

    @mode_pr.setter
    def mode_pr(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode_pr' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'mode_pr' field must be an unsigned integer in [0, 255]"
        self._mode_pr = value

    @property
    def mode_machine(self):
        """Message field 'mode_machine'."""
        return self._mode_machine

    @mode_machine.setter
    def mode_machine(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode_machine' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'mode_machine' field must be an unsigned integer in [0, 255]"
        self._mode_machine = value

    @property
    def tick(self):
        """Message field 'tick'."""
        return self._tick

    @tick.setter
    def tick(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'tick' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'tick' field must be an unsigned integer in [0, 4294967295]"
        self._tick = value

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
                 len(value) == 35 and
                 all(isinstance(v, MotorState) for v in value) and
                 True), \
                "The 'motor_state' field must be a set or sequence with length 35 and each value of type 'MotorState'"
        self._motor_state = value

    @property
    def wireless_remote(self):
        """Message field 'wireless_remote'."""
        return self._wireless_remote

    @wireless_remote.setter
    def wireless_remote(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint8, \
                "The 'wireless_remote' numpy.ndarray() must have the dtype of 'numpy.uint8'"
            assert value.size == 40, \
                "The 'wireless_remote' numpy.ndarray() must have a size of 40"
            self._wireless_remote = value
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
                 len(value) == 40 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'wireless_remote' field must be a set or sequence with length 40 and each value of type 'int' and each unsigned integer in [0, 255]"
        self._wireless_remote = numpy.array(value, dtype=numpy.uint8)

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

    @property
    def crc(self):
        """Message field 'crc'."""
        return self._crc

    @crc.setter
    def crc(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'crc' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'crc' field must be an unsigned integer in [0, 4294967295]"
        self._crc = value
