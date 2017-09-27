// Generated by gencpp from file ece470_ur3_driver/positions.msg
// DO NOT EDIT!


#ifndef ECE470_UR3_DRIVER_MESSAGE_POSITIONS_H
#define ECE470_UR3_DRIVER_MESSAGE_POSITIONS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ece470_ur3_driver
{
template <class ContainerAllocator>
struct positions_
{
  typedef positions_<ContainerAllocator> Type;

  positions_()
    : position()
    , velocity()
    , grip(0)
    , isReady(false)
    , state()
    , pending(false)  {
    }
  positions_(const ContainerAllocator& _alloc)
    : position(_alloc)
    , velocity(_alloc)
    , grip(0)
    , isReady(false)
    , state(_alloc)
    , pending(false)  {
    }



   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _position_type;
  _position_type position;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _velocity_type;
  _velocity_type velocity;

   typedef int32_t _grip_type;
  _grip_type grip;

   typedef uint8_t _isReady_type;
  _isReady_type isReady;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _state_type;
  _state_type state;

   typedef uint8_t _pending_type;
  _pending_type pending;




  typedef boost::shared_ptr< ::ece470_ur3_driver::positions_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ece470_ur3_driver::positions_<ContainerAllocator> const> ConstPtr;

}; // struct positions_

typedef ::ece470_ur3_driver::positions_<std::allocator<void> > positions;

typedef boost::shared_ptr< ::ece470_ur3_driver::positions > positionsPtr;
typedef boost::shared_ptr< ::ece470_ur3_driver::positions const> positionsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ece470_ur3_driver::positions_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ece470_ur3_driver::positions_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ece470_ur3_driver

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'ece470_ur3_driver': ['/home/youbot/catkin_rkaushi2/src/drivers/ece470_ur3_driver/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ece470_ur3_driver::positions_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ece470_ur3_driver::positions_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ece470_ur3_driver::positions_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ece470_ur3_driver::positions_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ece470_ur3_driver::positions_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ece470_ur3_driver::positions_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ece470_ur3_driver::positions_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bf2455dc51ac9b75c8e422304ce3ab30";
  }

  static const char* value(const ::ece470_ur3_driver::positions_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbf2455dc51ac9b75ULL;
  static const uint64_t static_value2 = 0xc8e422304ce3ab30ULL;
};

template<class ContainerAllocator>
struct DataType< ::ece470_ur3_driver::positions_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ece470_ur3_driver/positions";
  }

  static const char* value(const ::ece470_ur3_driver::positions_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ece470_ur3_driver::positions_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[] position\n\
float64[] velocity\n\
int32 grip\n\
bool isReady\n\
string state\n\
bool pending\n\
";
  }

  static const char* value(const ::ece470_ur3_driver::positions_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ece470_ur3_driver::positions_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.position);
      stream.next(m.velocity);
      stream.next(m.grip);
      stream.next(m.isReady);
      stream.next(m.state);
      stream.next(m.pending);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct positions_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ece470_ur3_driver::positions_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ece470_ur3_driver::positions_<ContainerAllocator>& v)
  {
    s << indent << "position[]" << std::endl;
    for (size_t i = 0; i < v.position.size(); ++i)
    {
      s << indent << "  position[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.position[i]);
    }
    s << indent << "velocity[]" << std::endl;
    for (size_t i = 0; i < v.velocity.size(); ++i)
    {
      s << indent << "  velocity[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.velocity[i]);
    }
    s << indent << "grip: ";
    Printer<int32_t>::stream(s, indent + "  ", v.grip);
    s << indent << "isReady: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.isReady);
    s << indent << "state: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.state);
    s << indent << "pending: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.pending);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ECE470_UR3_DRIVER_MESSAGE_POSITIONS_H
