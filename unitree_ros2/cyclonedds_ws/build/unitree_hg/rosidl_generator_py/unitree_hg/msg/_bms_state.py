# generated from rosidl_generator_py/resource/_idl.py.em
# with input from unitree_hg:msg/BmsState.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'cell_vol'
# Member 'bmsvoltage'
# Member 'temperature'
# Member 'bmsstate'
# Member 'reserve'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BmsState(type):
    """Metaclass of message 'BmsState'."""

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
                'unitree_hg.msg.BmsState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__bms_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__bms_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__bms_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__bms_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__bms_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class BmsState(metaclass=Metaclass_BmsState):
    """Message class 'BmsState'."""

    __slots__ = [
        '_version_high',
        '_version_low',
        '_fn',
        '_cell_vol',
        '_bmsvoltage',
        '_current',
        '_soc',
        '_soh',
        '_temperature',
        '_cycle',
        '_manufacturer_date',
        '_bmsstate',
        '_reserve',
    ]

    _fields_and_field_types = {
        'version_high': 'uint8',
        'version_low': 'uint8',
        'fn': 'uint8',
        'cell_vol': 'uint16[40]',
        'bmsvoltage': 'uint32[3]',
        'current': 'int32',
        'soc': 'uint8',
        'soh': 'uint8',
        'temperature': 'int16[12]',
        'cycle': 'uint16',
        'manufacturer_date': 'uint16',
        'bmsstate': 'uint32[5]',
        'reserve': 'uint32[3]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint16'), 40),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint32'), 3),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int16'), 12),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint32'), 5),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint32'), 3),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.version_high = kwargs.get('version_high', int())
        self.version_low = kwargs.get('version_low', int())
        self.fn = kwargs.get('fn', int())
        if 'cell_vol' not in kwargs:
            self.cell_vol = numpy.zeros(40, dtype=numpy.uint16)
        else:
            self.cell_vol = numpy.array(kwargs.get('cell_vol'), dtype=numpy.uint16)
            assert self.cell_vol.shape == (40, )
        if 'bmsvoltage' not in kwargs:
            self.bmsvoltage = numpy.zeros(3, dtype=numpy.uint32)
        else:
            self.bmsvoltage = numpy.array(kwargs.get('bmsvoltage'), dtype=numpy.uint32)
            assert self.bmsvoltage.shape == (3, )
        self.current = kwargs.get('current', int())
        self.soc = kwargs.get('soc', int())
        self.soh = kwargs.get('soh', int())
        if 'temperature' not in kwargs:
            self.temperature = numpy.zeros(12, dtype=numpy.int16)
        else:
            self.temperature = numpy.array(kwargs.get('temperature'), dtype=numpy.int16)
            assert self.temperature.shape == (12, )
        self.cycle = kwargs.get('cycle', int())
        self.manufacturer_date = kwargs.get('manufacturer_date', int())
        if 'bmsstate' not in kwargs:
            self.bmsstate = numpy.zeros(5, dtype=numpy.uint32)
        else:
            self.bmsstate = numpy.array(kwargs.get('bmsstate'), dtype=numpy.uint32)
            assert self.bmsstate.shape == (5, )
        if 'reserve' not in kwargs:
            self.reserve = numpy.zeros(3, dtype=numpy.uint32)
        else:
            self.reserve = numpy.array(kwargs.get('reserve'), dtype=numpy.uint32)
            assert self.reserve.shape == (3, )

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
        if self.version_high != other.version_high:
            return False
        if self.version_low != other.version_low:
            return False
        if self.fn != other.fn:
            return False
        if all(self.cell_vol != other.cell_vol):
            return False
        if all(self.bmsvoltage != other.bmsvoltage):
            return False
        if self.current != other.current:
            return False
        if self.soc != other.soc:
            return False
        if self.soh != other.soh:
            return False
        if all(self.temperature != other.temperature):
            return False
        if self.cycle != other.cycle:
            return False
        if self.manufacturer_date != other.manufacturer_date:
            return False
        if all(self.bmsstate != other.bmsstate):
            return False
        if all(self.reserve != other.reserve):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def version_high(self):
        """Message field 'version_high'."""
        return self._version_high

    @version_high.setter
    def version_high(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'version_high' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'version_high' field must be an unsigned integer in [0, 255]"
        self._version_high = value

    @property
    def version_low(self):
        """Message field 'version_low'."""
        return self._version_low

    @version_low.setter
    def version_low(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'version_low' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'version_low' field must be an unsigned integer in [0, 255]"
        self._version_low = value

    @property
    def fn(self):
        """Message field 'fn'."""
        return self._fn

    @fn.setter
    def fn(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'fn' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'fn' field must be an unsigned integer in [0, 255]"
        self._fn = value

    @property
    def cell_vol(self):
        """Message field 'cell_vol'."""
        return self._cell_vol

    @cell_vol.setter
    def cell_vol(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint16, \
                "The 'cell_vol' numpy.ndarray() must have the dtype of 'numpy.uint16'"
            assert value.size == 40, \
                "The 'cell_vol' numpy.ndarray() must have a size of 40"
            self._cell_vol = value
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
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'cell_vol' field must be a set or sequence with length 40 and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._cell_vol = numpy.array(value, dtype=numpy.uint16)

    @property
    def bmsvoltage(self):
        """Message field 'bmsvoltage'."""
        return self._bmsvoltage

    @bmsvoltage.setter
    def bmsvoltage(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint32, \
                "The 'bmsvoltage' numpy.ndarray() must have the dtype of 'numpy.uint32'"
            assert value.size == 3, \
                "The 'bmsvoltage' numpy.ndarray() must have a size of 3"
            self._bmsvoltage = value
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
                 len(value) == 3 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 4294967296 for val in value)), \
                "The 'bmsvoltage' field must be a set or sequence with length 3 and each value of type 'int' and each unsigned integer in [0, 4294967295]"
        self._bmsvoltage = numpy.array(value, dtype=numpy.uint32)

    @property
    def current(self):
        """Message field 'current'."""
        return self._current

    @current.setter
    def current(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'current' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'current' field must be an integer in [-2147483648, 2147483647]"
        self._current = value

    @property
    def soc(self):
        """Message field 'soc'."""
        return self._soc

    @soc.setter
    def soc(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'soc' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'soc' field must be an unsigned integer in [0, 255]"
        self._soc = value

    @property
    def soh(self):
        """Message field 'soh'."""
        return self._soh

    @soh.setter
    def soh(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'soh' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'soh' field must be an unsigned integer in [0, 255]"
        self._soh = value

    @property
    def temperature(self):
        """Message field 'temperature'."""
        return self._temperature

    @temperature.setter
    def temperature(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int16, \
                "The 'temperature' numpy.ndarray() must have the dtype of 'numpy.int16'"
            assert value.size == 12, \
                "The 'temperature' numpy.ndarray() must have a size of 12"
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
                 len(value) == 12 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -32768 and val < 32768 for val in value)), \
                "The 'temperature' field must be a set or sequence with length 12 and each value of type 'int' and each integer in [-32768, 32767]"
        self._temperature = numpy.array(value, dtype=numpy.int16)

    @property
    def cycle(self):
        """Message field 'cycle'."""
        return self._cycle

    @cycle.setter
    def cycle(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cycle' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'cycle' field must be an unsigned integer in [0, 65535]"
        self._cycle = value

    @property
    def manufacturer_date(self):
        """Message field 'manufacturer_date'."""
        return self._manufacturer_date

    @manufacturer_date.setter
    def manufacturer_date(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'manufacturer_date' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'manufacturer_date' field must be an unsigned integer in [0, 65535]"
        self._manufacturer_date = value

    @property
    def bmsstate(self):
        """Message field 'bmsstate'."""
        return self._bmsstate

    @bmsstate.setter
    def bmsstate(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint32, \
                "The 'bmsstate' numpy.ndarray() must have the dtype of 'numpy.uint32'"
            assert value.size == 5, \
                "The 'bmsstate' numpy.ndarray() must have a size of 5"
            self._bmsstate = value
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
                 len(value) == 5 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 4294967296 for val in value)), \
                "The 'bmsstate' field must be a set or sequence with length 5 and each value of type 'int' and each unsigned integer in [0, 4294967295]"
        self._bmsstate = numpy.array(value, dtype=numpy.uint32)

    @property
    def reserve(self):
        """Message field 'reserve'."""
        return self._reserve

    @reserve.setter
    def reserve(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint32, \
                "The 'reserve' numpy.ndarray() must have the dtype of 'numpy.uint32'"
            assert value.size == 3, \
                "The 'reserve' numpy.ndarray() must have a size of 3"
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
                 len(value) == 3 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 4294967296 for val in value)), \
                "The 'reserve' field must be a set or sequence with length 3 and each value of type 'int' and each unsigned integer in [0, 4294967295]"
        self._reserve = numpy.array(value, dtype=numpy.uint32)
