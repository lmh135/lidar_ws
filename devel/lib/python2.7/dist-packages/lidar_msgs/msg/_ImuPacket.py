# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from lidar_msgs/ImuPacket.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class ImuPacket(genpy.Message):
  _md5sum = "a8ec3aca7e5b627e23bc444ef30e8bcd"
  _type = "lidar_msgs/ImuPacket"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float32[3] imu_gyro
float32[3] imu_accel
float32[3] imu_magn
float32[3] imu_euler
uint64 timestamp
uint64 id_num
"""
  __slots__ = ['imu_gyro','imu_accel','imu_magn','imu_euler','timestamp','id_num']
  _slot_types = ['float32[3]','float32[3]','float32[3]','float32[3]','uint64','uint64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       imu_gyro,imu_accel,imu_magn,imu_euler,timestamp,id_num

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ImuPacket, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.imu_gyro is None:
        self.imu_gyro = [0.] * 3
      if self.imu_accel is None:
        self.imu_accel = [0.] * 3
      if self.imu_magn is None:
        self.imu_magn = [0.] * 3
      if self.imu_euler is None:
        self.imu_euler = [0.] * 3
      if self.timestamp is None:
        self.timestamp = 0
      if self.id_num is None:
        self.id_num = 0
    else:
      self.imu_gyro = [0.] * 3
      self.imu_accel = [0.] * 3
      self.imu_magn = [0.] * 3
      self.imu_euler = [0.] * 3
      self.timestamp = 0
      self.id_num = 0

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
      buff.write(_get_struct_3f().pack(*self.imu_gyro))
      buff.write(_get_struct_3f().pack(*self.imu_accel))
      buff.write(_get_struct_3f().pack(*self.imu_magn))
      buff.write(_get_struct_3f().pack(*self.imu_euler))
      _x = self
      buff.write(_get_struct_2Q().pack(_x.timestamp, _x.id_num))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 12
      self.imu_gyro = _get_struct_3f().unpack(str[start:end])
      start = end
      end += 12
      self.imu_accel = _get_struct_3f().unpack(str[start:end])
      start = end
      end += 12
      self.imu_magn = _get_struct_3f().unpack(str[start:end])
      start = end
      end += 12
      self.imu_euler = _get_struct_3f().unpack(str[start:end])
      _x = self
      start = end
      end += 16
      (_x.timestamp, _x.id_num,) = _get_struct_2Q().unpack(str[start:end])
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
      buff.write(self.imu_gyro.tostring())
      buff.write(self.imu_accel.tostring())
      buff.write(self.imu_magn.tostring())
      buff.write(self.imu_euler.tostring())
      _x = self
      buff.write(_get_struct_2Q().pack(_x.timestamp, _x.id_num))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 12
      self.imu_gyro = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
      start = end
      end += 12
      self.imu_accel = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
      start = end
      end += 12
      self.imu_magn = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
      start = end
      end += 12
      self.imu_euler = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
      _x = self
      start = end
      end += 16
      (_x.timestamp, _x.id_num,) = _get_struct_2Q().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3f = None
def _get_struct_3f():
    global _struct_3f
    if _struct_3f is None:
        _struct_3f = struct.Struct("<3f")
    return _struct_3f
_struct_2Q = None
def _get_struct_2Q():
    global _struct_2Q
    if _struct_2Q is None:
        _struct_2Q = struct.Struct("<2Q")
    return _struct_2Q
