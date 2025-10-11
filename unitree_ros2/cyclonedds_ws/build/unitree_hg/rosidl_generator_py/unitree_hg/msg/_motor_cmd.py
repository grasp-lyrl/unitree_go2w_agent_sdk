# generated from rosidl_generator_py/resource/_idl.py.em
# with input from unitree_hg:msg/MotorCmd.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MotorCmd(type):
    """Metaclass of message 'MotorCmd'."""

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
                'unitree_hg.msg.MotorCmd')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__motor_cmd
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__motor_cmd
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__motor_cmd
            cls._TYPE_SUPPORT = module.type_support_msg__msg__motor_cmd
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__motor_cmd

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MotorCmd(metaclass=Metaclass_MotorCmd):
    """Message class 'MotorCmd'."""

    __slots__ = [
        '_mode',
        '_q',
        '_dq',
        '_tau',
        '_kp',
        '_kd',
        '_reserve',
    ]

    _fields_and_field_types = {
        'mode': 'uint8',
        'q': 'float',
        'dq': 'float',
        'tau': 'float',
        'kp': 'float',
        'kd': 'float',
        'reserve': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.mode = kwargs.get('mode', int())
        self.q = kwargs.get('q', float())
        self.dq = kwargs.get('dq', float())
        self.tau = kwargs.get('tau', float())
        self.kp = kwargs.get('kp', float())
        self.kd = kwargs.get('kd', float())
        self.reserve = kwargs.get('reserve', int())

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
        if self.tau != other.tau:
            return False
        if self.kp != other.kp:
            return False
        if self.kd != other.kd:
            return False
        if self.reserve != other.reserve:
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
    def tau(self):
        """Message field 'tau'."""
        return self._tau

    @tau.setter
    def tau(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tau' field must be of type 'float'"
        self._tau = value

    @property
    def kp(self):
        """Message field 'kp'."""
        return self._kp

    @kp.setter
    def kp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kp' field must be of type 'float'"
        self._kp = value

    @property
    def kd(self):
        """Message field 'kd'."""
        return self._kd

    @kd.setter
    def kd(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kd' field must be of type 'float'"
        self._kd = value

    @property
    def reserve(self):
        """Message field 'reserve'."""
        return self._reserve

    @reserve.setter
    def reserve(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'reserve' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'reserve' field must be an unsigned integer in [0, 4294967295]"
        self._reserve = value
