# generated from rosidl_generator_py/resource/_idl.py.em
# with input from unitree_hg:msg/LowCmd.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'reserve'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LowCmd(type):
    """Metaclass of message 'LowCmd'."""

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
                'unitree_hg.msg.LowCmd')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__low_cmd
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__low_cmd
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__low_cmd
            cls._TYPE_SUPPORT = module.type_support_msg__msg__low_cmd
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__low_cmd

            from unitree_hg.msg import MotorCmd
            if MotorCmd.__class__._TYPE_SUPPORT is None:
                MotorCmd.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LowCmd(metaclass=Metaclass_LowCmd):
    """Message class 'LowCmd'."""

    __slots__ = [
        '_mode_pr',
        '_mode_machine',
        '_motor_cmd',
        '_reserve',
        '_crc',
    ]

    _fields_and_field_types = {
        'mode_pr': 'uint8',
        'mode_machine': 'uint8',
        'motor_cmd': 'unitree_hg/MotorCmd[35]',
        'reserve': 'uint32[4]',
        'crc': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.NamespacedType(['unitree_hg', 'msg'], 'MotorCmd'), 35),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint32'), 4),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.mode_pr = kwargs.get('mode_pr', int())
        self.mode_machine = kwargs.get('mode_machine', int())
        from unitree_hg.msg import MotorCmd
        self.motor_cmd = kwargs.get(
            'motor_cmd',
            [MotorCmd() for x in range(35)]
        )
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
        if self.mode_pr != other.mode_pr:
            return False
        if self.mode_machine != other.mode_machine:
            return False
        if self.motor_cmd != other.motor_cmd:
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
    def motor_cmd(self):
        """Message field 'motor_cmd'."""
        return self._motor_cmd

    @motor_cmd.setter
    def motor_cmd(self, value):
        if __debug__:
            from unitree_hg.msg import MotorCmd
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
                 all(isinstance(v, MotorCmd) for v in value) and
                 True), \
                "The 'motor_cmd' field must be a set or sequence with length 35 and each value of type 'MotorCmd'"
        self._motor_cmd = value

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
