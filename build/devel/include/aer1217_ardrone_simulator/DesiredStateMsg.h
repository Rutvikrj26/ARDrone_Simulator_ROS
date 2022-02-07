// Generated by gencpp from file aer1217_ardrone_simulator/DesiredStateMsg.msg
// DO NOT EDIT!


#ifndef AER1217_ARDRONE_SIMULATOR_MESSAGE_DESIREDSTATEMSG_H
#define AER1217_ARDRONE_SIMULATOR_MESSAGE_DESIREDSTATEMSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace aer1217_ardrone_simulator
{
template <class ContainerAllocator>
struct DesiredStateMsg_
{
  typedef DesiredStateMsg_<ContainerAllocator> Type;

  DesiredStateMsg_()
    : x()
    , x_dot()
    , yaw(0.0)
    , yaw_rate(0.0)  {
      x.assign(0.0);

      x_dot.assign(0.0);
  }
  DesiredStateMsg_(const ContainerAllocator& _alloc)
    : x()
    , x_dot()
    , yaw(0.0)
    , yaw_rate(0.0)  {
  (void)_alloc;
      x.assign(0.0);

      x_dot.assign(0.0);
  }



   typedef boost::array<double, 3>  _x_type;
  _x_type x;

   typedef boost::array<double, 3>  _x_dot_type;
  _x_dot_type x_dot;

   typedef double _yaw_type;
  _yaw_type yaw;

   typedef double _yaw_rate_type;
  _yaw_rate_type yaw_rate;





  typedef boost::shared_ptr< ::aer1217_ardrone_simulator::DesiredStateMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::aer1217_ardrone_simulator::DesiredStateMsg_<ContainerAllocator> const> ConstPtr;

}; // struct DesiredStateMsg_

typedef ::aer1217_ardrone_simulator::DesiredStateMsg_<std::allocator<void> > DesiredStateMsg;

typedef boost::shared_ptr< ::aer1217_ardrone_simulator::DesiredStateMsg > DesiredStateMsgPtr;
typedef boost::shared_ptr< ::aer1217_ardrone_simulator::DesiredStateMsg const> DesiredStateMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::aer1217_ardrone_simulator::DesiredStateMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::aer1217_ardrone_simulator::DesiredStateMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace aer1217_ardrone_simulator

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'aer1217_ardrone_simulator': ['/home/aer1217/aer1217/labs/src/aer1217_ardrone_simulator/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::aer1217_ardrone_simulator::DesiredStateMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::aer1217_ardrone_simulator::DesiredStateMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::aer1217_ardrone_simulator::DesiredStateMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::aer1217_ardrone_simulator::DesiredStateMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::aer1217_ardrone_simulator::DesiredStateMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::aer1217_ardrone_simulator::DesiredStateMsg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::aer1217_ardrone_simulator::DesiredStateMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cc61550db4d69f891417c4905802aea9";
  }

  static const char* value(const ::aer1217_ardrone_simulator::DesiredStateMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcc61550db4d69f89ULL;
  static const uint64_t static_value2 = 0x1417c4905802aea9ULL;
};

template<class ContainerAllocator>
struct DataType< ::aer1217_ardrone_simulator::DesiredStateMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "aer1217_ardrone_simulator/DesiredStateMsg";
  }

  static const char* value(const ::aer1217_ardrone_simulator::DesiredStateMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::aer1217_ardrone_simulator::DesiredStateMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# MotorCommands\n\
#\n\
# Rikky Duivenvoorden 2017-01-30 -- For use in AER1217 Winter 2017\n\
#\n\
# Data communicates the motor commands in RPM as a four element array in the\n\
# following order [front_left, front_right, rear_left, rear_right]\n\
\n\
float64[3] x\n\
float64[3] x_dot\n\
float64 yaw\n\
float64 yaw_rate\n\
";
  }

  static const char* value(const ::aer1217_ardrone_simulator::DesiredStateMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::aer1217_ardrone_simulator::DesiredStateMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.x_dot);
      stream.next(m.yaw);
      stream.next(m.yaw_rate);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DesiredStateMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::aer1217_ardrone_simulator::DesiredStateMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::aer1217_ardrone_simulator::DesiredStateMsg_<ContainerAllocator>& v)
  {
    s << indent << "x[]" << std::endl;
    for (size_t i = 0; i < v.x.size(); ++i)
    {
      s << indent << "  x[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.x[i]);
    }
    s << indent << "x_dot[]" << std::endl;
    for (size_t i = 0; i < v.x_dot.size(); ++i)
    {
      s << indent << "  x_dot[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.x_dot[i]);
    }
    s << indent << "yaw: ";
    Printer<double>::stream(s, indent + "  ", v.yaw);
    s << indent << "yaw_rate: ";
    Printer<double>::stream(s, indent + "  ", v.yaw_rate);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AER1217_ARDRONE_SIMULATOR_MESSAGE_DESIREDSTATEMSG_H
