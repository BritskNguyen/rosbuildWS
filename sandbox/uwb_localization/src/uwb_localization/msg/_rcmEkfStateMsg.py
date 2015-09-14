"""autogenerated by genpy from uwb_localization/rcmEkfStateMsg.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy

class rcmEkfStateMsg(genpy.Message):
  _md5sum = "f0206a0ffc3c6ed82339f59621096a35"
  _type = "uwb_localization/rcmEkfStateMsg"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """time time_stamp
float64 x
float64 y
float64 z
float64 dx
float64 dy
float64 dz
float64[36] covariance

"""
  __slots__ = ['time_stamp','x','y','z','dx','dy','dz','covariance']
  _slot_types = ['time','float64','float64','float64','float64','float64','float64','float64[36]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       time_stamp,x,y,z,dx,dy,dz,covariance

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(rcmEkfStateMsg, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.time_stamp is None:
        self.time_stamp = genpy.Time()
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.z is None:
        self.z = 0.
      if self.dx is None:
        self.dx = 0.
      if self.dy is None:
        self.dy = 0.
      if self.dz is None:
        self.dz = 0.
      if self.covariance is None:
        self.covariance = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
    else:
      self.time_stamp = genpy.Time()
      self.x = 0.
      self.y = 0.
      self.z = 0.
      self.dx = 0.
      self.dy = 0.
      self.dz = 0.
      self.covariance = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]

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
      buff.write(_struct_2I6d.pack(_x.time_stamp.secs, _x.time_stamp.nsecs, _x.x, _x.y, _x.z, _x.dx, _x.dy, _x.dz))
      buff.write(_struct_36d.pack(*self.covariance))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.time_stamp is None:
        self.time_stamp = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 56
      (_x.time_stamp.secs, _x.time_stamp.nsecs, _x.x, _x.y, _x.z, _x.dx, _x.dy, _x.dz,) = _struct_2I6d.unpack(str[start:end])
      start = end
      end += 288
      self.covariance = _struct_36d.unpack(str[start:end])
      self.time_stamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_2I6d.pack(_x.time_stamp.secs, _x.time_stamp.nsecs, _x.x, _x.y, _x.z, _x.dx, _x.dy, _x.dz))
      buff.write(self.covariance.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.time_stamp is None:
        self.time_stamp = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 56
      (_x.time_stamp.secs, _x.time_stamp.nsecs, _x.x, _x.y, _x.z, _x.dx, _x.dy, _x.dz,) = _struct_2I6d.unpack(str[start:end])
      start = end
      end += 288
      self.covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=36)
      self.time_stamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2I6d = struct.Struct("<2I6d")
_struct_36d = struct.Struct("<36d")
