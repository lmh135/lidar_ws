// Generated by gencpp from file lidar_msgs/RecvPacket.msg
// DO NOT EDIT!


#ifndef LIDAR_MSGS_MESSAGE_RECVPACKET_H
#define LIDAR_MSGS_MESSAGE_RECVPACKET_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <lidar_msgs/LaserPacket.h>
#include <lidar_msgs/ImuPacket.h>
#include <lidar_msgs/GPSPacket.h>

namespace lidar_msgs
{
template <class ContainerAllocator>
struct RecvPacket_
{
  typedef RecvPacket_<ContainerAllocator> Type;

  RecvPacket_()
    : laserpkt()
    , imupkt()
    , gpspkt()  {
    }
  RecvPacket_(const ContainerAllocator& _alloc)
    : laserpkt(_alloc)
    , imupkt(_alloc)
    , gpspkt(_alloc)  {
  (void)_alloc;
    }



   typedef  ::lidar_msgs::LaserPacket_<ContainerAllocator>  _laserpkt_type;
  _laserpkt_type laserpkt;

   typedef  ::lidar_msgs::ImuPacket_<ContainerAllocator>  _imupkt_type;
  _imupkt_type imupkt;

   typedef  ::lidar_msgs::GPSPacket_<ContainerAllocator>  _gpspkt_type;
  _gpspkt_type gpspkt;





  typedef boost::shared_ptr< ::lidar_msgs::RecvPacket_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lidar_msgs::RecvPacket_<ContainerAllocator> const> ConstPtr;

}; // struct RecvPacket_

typedef ::lidar_msgs::RecvPacket_<std::allocator<void> > RecvPacket;

typedef boost::shared_ptr< ::lidar_msgs::RecvPacket > RecvPacketPtr;
typedef boost::shared_ptr< ::lidar_msgs::RecvPacket const> RecvPacketConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lidar_msgs::RecvPacket_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lidar_msgs::RecvPacket_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace lidar_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'lidar_msgs': ['/home/longmen/lidar_ws/src/lidar_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::lidar_msgs::RecvPacket_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lidar_msgs::RecvPacket_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lidar_msgs::RecvPacket_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lidar_msgs::RecvPacket_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_msgs::RecvPacket_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_msgs::RecvPacket_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lidar_msgs::RecvPacket_<ContainerAllocator> >
{
  static const char* value()
  {
    return "78587254adf0ff3522b94defe9027af8";
  }

  static const char* value(const ::lidar_msgs::RecvPacket_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x78587254adf0ff35ULL;
  static const uint64_t static_value2 = 0x22b94defe9027af8ULL;
};

template<class ContainerAllocator>
struct DataType< ::lidar_msgs::RecvPacket_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lidar_msgs/RecvPacket";
  }

  static const char* value(const ::lidar_msgs::RecvPacket_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lidar_msgs::RecvPacket_<ContainerAllocator> >
{
  static const char* value()
  {
    return "LaserPacket laserpkt\n\
ImuPacket imupkt\n\
GPSPacket gpspkt\n\
\n\
================================================================================\n\
MSG: lidar_msgs/LaserPacket\n\
float32[12] Azimuth\n\
float32[192] distance\n\
uint64 timestamp\n\
uint64 id_num\n\
\n\
================================================================================\n\
MSG: lidar_msgs/ImuPacket\n\
float32[3] imu_gyro\n\
float32[3] imu_accel\n\
float32[3] imu_magn\n\
float32[3] imu_euler\n\
uint64 timestamp\n\
uint64 id_num\n\
\n\
================================================================================\n\
MSG: lidar_msgs/GPSPacket\n\
float64[3] ned_xyz\n\
float64[3] ned_uvw\n\
float64 heading\n\
uint8 gps_sats\n\
uint8 gps_mod\n\
uint64 timestamp\n\
uint64 id_num\n\
";
  }

  static const char* value(const ::lidar_msgs::RecvPacket_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lidar_msgs::RecvPacket_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.laserpkt);
      stream.next(m.imupkt);
      stream.next(m.gpspkt);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RecvPacket_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lidar_msgs::RecvPacket_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lidar_msgs::RecvPacket_<ContainerAllocator>& v)
  {
    s << indent << "laserpkt: ";
    s << std::endl;
    Printer< ::lidar_msgs::LaserPacket_<ContainerAllocator> >::stream(s, indent + "  ", v.laserpkt);
    s << indent << "imupkt: ";
    s << std::endl;
    Printer< ::lidar_msgs::ImuPacket_<ContainerAllocator> >::stream(s, indent + "  ", v.imupkt);
    s << indent << "gpspkt: ";
    s << std::endl;
    Printer< ::lidar_msgs::GPSPacket_<ContainerAllocator> >::stream(s, indent + "  ", v.gpspkt);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LIDAR_MSGS_MESSAGE_RECVPACKET_H
