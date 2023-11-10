// Generated by gencpp from file robotics/JointIntollerance.msg
// DO NOT EDIT!


#ifndef ROBOTICS_MESSAGE_JOINTINTOLLERANCE_H
#define ROBOTICS_MESSAGE_JOINTINTOLLERANCE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robotics
{
template <class ContainerAllocator>
struct JointIntollerance_
{
  typedef JointIntollerance_<ContainerAllocator> Type;

  JointIntollerance_()
    : name()
    , position(0.0)
    , velocity(0.0)
    , acceleration(0.0)  {
    }
  JointIntollerance_(const ContainerAllocator& _alloc)
    : name(_alloc)
    , position(0.0)
    , velocity(0.0)
    , acceleration(0.0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;

   typedef double _position_type;
  _position_type position;

   typedef double _velocity_type;
  _velocity_type velocity;

   typedef double _acceleration_type;
  _acceleration_type acceleration;





  typedef boost::shared_ptr< ::robotics::JointIntollerance_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robotics::JointIntollerance_<ContainerAllocator> const> ConstPtr;

}; // struct JointIntollerance_

typedef ::robotics::JointIntollerance_<std::allocator<void> > JointIntollerance;

typedef boost::shared_ptr< ::robotics::JointIntollerance > JointIntollerancePtr;
typedef boost::shared_ptr< ::robotics::JointIntollerance const> JointIntolleranceConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robotics::JointIntollerance_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robotics::JointIntollerance_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::robotics::JointIntollerance_<ContainerAllocator1> & lhs, const ::robotics::JointIntollerance_<ContainerAllocator2> & rhs)
{
  return lhs.name == rhs.name &&
    lhs.position == rhs.position &&
    lhs.velocity == rhs.velocity &&
    lhs.acceleration == rhs.acceleration;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::robotics::JointIntollerance_<ContainerAllocator1> & lhs, const ::robotics::JointIntollerance_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace robotics

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::robotics::JointIntollerance_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotics::JointIntollerance_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotics::JointIntollerance_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotics::JointIntollerance_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotics::JointIntollerance_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotics::JointIntollerance_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robotics::JointIntollerance_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f544fe9c16cf04547e135dd6063ff5be";
  }

  static const char* value(const ::robotics::JointIntollerance_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf544fe9c16cf0454ULL;
  static const uint64_t static_value2 = 0x7e135dd6063ff5beULL;
};

template<class ContainerAllocator>
struct DataType< ::robotics::JointIntollerance_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robotics/JointIntollerance";
  }

  static const char* value(const ::robotics::JointIntollerance_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robotics::JointIntollerance_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string name\n"
"float64 position \n"
"float64 velocity\n"
"float64 acceleration\n"
;
  }

  static const char* value(const ::robotics::JointIntollerance_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robotics::JointIntollerance_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
      stream.next(m.position);
      stream.next(m.velocity);
      stream.next(m.acceleration);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct JointIntollerance_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robotics::JointIntollerance_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robotics::JointIntollerance_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "position: ";
    Printer<double>::stream(s, indent + "  ", v.position);
    s << indent << "velocity: ";
    Printer<double>::stream(s, indent + "  ", v.velocity);
    s << indent << "acceleration: ";
    Printer<double>::stream(s, indent + "  ", v.acceleration);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOTICS_MESSAGE_JOINTINTOLLERANCE_H
