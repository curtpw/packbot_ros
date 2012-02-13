"""autogenerated by genmsg_py from Misc.msg. Do not edit."""
import roslib.message
import struct


class Misc(roslib.message.Message):
  _md5sum = "fe37317c2a99014a4cdc2dae9b111890"
  _type = "packbot_ros/Misc"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# this message type is to store miscellaneous packbot parametes
# in this case,
# 	1) status of the attack light
#		2) drive camera tilt degrees
# attack light
bool light
# drive camera tilt angle in degrees
# unit - degrees, integer value
int32 camera_tilt

"""
  __slots__ = ['light','camera_tilt']
  _slot_types = ['bool','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       light,camera_tilt
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(Misc, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.light is None:
        self.light = False
      if self.camera_tilt is None:
        self.camera_tilt = 0
    else:
      self.light = False
      self.camera_tilt = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_Bi.pack(_x.light, _x.camera_tilt))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      _x = self
      start = end
      end += 5
      (_x.light, _x.camera_tilt,) = _struct_Bi.unpack(str[start:end])
      self.light = bool(self.light)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_Bi.pack(_x.light, _x.camera_tilt))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 5
      (_x.light, _x.camera_tilt,) = _struct_Bi.unpack(str[start:end])
      self.light = bool(self.light)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_Bi = struct.Struct("<Bi")
