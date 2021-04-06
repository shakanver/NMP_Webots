# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from webots_ros/field_set_colorRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class field_set_colorRequest(genpy.Message):
  _md5sum = "ae309348d6fdd3fa0e43829b8bba7c9c"
  _type = "webots_ros/field_set_colorRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """uint64 field
int32 index
std_msgs/ColorRGBA value

================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
"""
  __slots__ = ['field','index','value']
  _slot_types = ['uint64','int32','std_msgs/ColorRGBA']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       field,index,value

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(field_set_colorRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.field is None:
        self.field = 0
      if self.index is None:
        self.index = 0
      if self.value is None:
        self.value = std_msgs.msg.ColorRGBA()
    else:
      self.field = 0
      self.index = 0
      self.value = std_msgs.msg.ColorRGBA()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_Qi4f().pack(_x.field, _x.index, _x.value.r, _x.value.g, _x.value.b, _x.value.a))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.value is None:
        self.value = std_msgs.msg.ColorRGBA()
      end = 0
      _x = self
      start = end
      end += 28
      (_x.field, _x.index, _x.value.r, _x.value.g, _x.value.b, _x.value.a,) = _get_struct_Qi4f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_Qi4f().pack(_x.field, _x.index, _x.value.r, _x.value.g, _x.value.b, _x.value.a))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.value is None:
        self.value = std_msgs.msg.ColorRGBA()
      end = 0
      _x = self
      start = end
      end += 28
      (_x.field, _x.index, _x.value.r, _x.value.g, _x.value.b, _x.value.a,) = _get_struct_Qi4f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_Qi4f = None
def _get_struct_Qi4f():
    global _struct_Qi4f
    if _struct_Qi4f is None:
        _struct_Qi4f = struct.Struct("<Qi4f")
    return _struct_Qi4f
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from webots_ros/field_set_colorResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class field_set_colorResponse(genpy.Message):
  _md5sum = "3c2bcf2ff0894cb3058b1bf4c8c4175a"
  _type = "webots_ros/field_set_colorResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int32 success

"""
  __slots__ = ['success']
  _slot_types = ['int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       success

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(field_set_colorResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.success is None:
        self.success = 0
    else:
      self.success = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self.success
      buff.write(_get_struct_i().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 4
      (self.success,) = _get_struct_i().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.success
      buff.write(_get_struct_i().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 4
      (self.success,) = _get_struct_i().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_i = None
def _get_struct_i():
    global _struct_i
    if _struct_i is None:
        _struct_i = struct.Struct("<i")
    return _struct_i
class field_set_color(object):
  _type          = 'webots_ros/field_set_color'
  _md5sum = 'a58a56eaaf940e418b6c8321888e9929'
  _request_class  = field_set_colorRequest
  _response_class = field_set_colorResponse
