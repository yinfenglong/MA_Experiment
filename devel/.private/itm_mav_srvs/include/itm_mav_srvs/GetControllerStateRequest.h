// Generated by gencpp from file itm_mav_srvs/GetControllerStateRequest.msg
// DO NOT EDIT!


#ifndef ITM_MAV_SRVS_MESSAGE_GETCONTROLLERSTATEREQUEST_H
#define ITM_MAV_SRVS_MESSAGE_GETCONTROLLERSTATEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace itm_mav_srvs
{
template <class ContainerAllocator>
struct GetControllerStateRequest_
{
  typedef GetControllerStateRequest_<ContainerAllocator> Type;

  GetControllerStateRequest_()
    : robot_name()
    , command_id(0)  {
    }
  GetControllerStateRequest_(const ContainerAllocator& _alloc)
    : robot_name(_alloc)
    , command_id(0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _robot_name_type;
  _robot_name_type robot_name;

   typedef uint8_t _command_id_type;
  _command_id_type command_id;





  typedef boost::shared_ptr< ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetControllerStateRequest_

typedef ::itm_mav_srvs::GetControllerStateRequest_<std::allocator<void> > GetControllerStateRequest;

typedef boost::shared_ptr< ::itm_mav_srvs::GetControllerStateRequest > GetControllerStateRequestPtr;
typedef boost::shared_ptr< ::itm_mav_srvs::GetControllerStateRequest const> GetControllerStateRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator1> & lhs, const ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator2> & rhs)
{
  return lhs.robot_name == rhs.robot_name &&
    lhs.command_id == rhs.command_id;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator1> & lhs, const ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace itm_mav_srvs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d8c6063b046aafbed6fda5050bbf6cf4";
  }

  static const char* value(const ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd8c6063b046aafbeULL;
  static const uint64_t static_value2 = 0xd6fda5050bbf6cf4ULL;
};

template<class ContainerAllocator>
struct DataType< ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "itm_mav_srvs/GetControllerStateRequest";
  }

  static const char* value(const ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string robot_name\n"
"uint8 command_id\n"
;
  }

  static const char* value(const ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.robot_name);
      stream.next(m.command_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetControllerStateRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::itm_mav_srvs::GetControllerStateRequest_<ContainerAllocator>& v)
  {
    s << indent << "robot_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.robot_name);
    s << indent << "command_id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.command_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ITM_MAV_SRVS_MESSAGE_GETCONTROLLERSTATEREQUEST_H