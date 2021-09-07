// Generated by gencpp from file itm_mav_srvs/SetModeRequest.msg
// DO NOT EDIT!


#ifndef ITM_MAV_SRVS_MESSAGE_SETMODEREQUEST_H
#define ITM_MAV_SRVS_MESSAGE_SETMODEREQUEST_H


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
struct SetModeRequest_
{
  typedef SetModeRequest_<ContainerAllocator> Type;

  SetModeRequest_()
    : mode(0)  {
    }
  SetModeRequest_(const ContainerAllocator& _alloc)
    : mode(0)  {
  (void)_alloc;
    }



   typedef uint8_t _mode_type;
  _mode_type mode;





  typedef boost::shared_ptr< ::itm_mav_srvs::SetModeRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::itm_mav_srvs::SetModeRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetModeRequest_

typedef ::itm_mav_srvs::SetModeRequest_<std::allocator<void> > SetModeRequest;

typedef boost::shared_ptr< ::itm_mav_srvs::SetModeRequest > SetModeRequestPtr;
typedef boost::shared_ptr< ::itm_mav_srvs::SetModeRequest const> SetModeRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::itm_mav_srvs::SetModeRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::itm_mav_srvs::SetModeRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::itm_mav_srvs::SetModeRequest_<ContainerAllocator1> & lhs, const ::itm_mav_srvs::SetModeRequest_<ContainerAllocator2> & rhs)
{
  return lhs.mode == rhs.mode;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::itm_mav_srvs::SetModeRequest_<ContainerAllocator1> & lhs, const ::itm_mav_srvs::SetModeRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace itm_mav_srvs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::itm_mav_srvs::SetModeRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::itm_mav_srvs::SetModeRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::itm_mav_srvs::SetModeRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::itm_mav_srvs::SetModeRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::itm_mav_srvs::SetModeRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::itm_mav_srvs::SetModeRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::itm_mav_srvs::SetModeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "89b81386720be1cd0ce7a3953fcd1b19";
  }

  static const char* value(const ::itm_mav_srvs::SetModeRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x89b81386720be1cdULL;
  static const uint64_t static_value2 = 0x0ce7a3953fcd1b19ULL;
};

template<class ContainerAllocator>
struct DataType< ::itm_mav_srvs::SetModeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "itm_mav_srvs/SetModeRequest";
  }

  static const char* value(const ::itm_mav_srvs::SetModeRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::itm_mav_srvs::SetModeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 mode\n"
;
  }

  static const char* value(const ::itm_mav_srvs::SetModeRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::itm_mav_srvs::SetModeRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mode);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetModeRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::itm_mav_srvs::SetModeRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::itm_mav_srvs::SetModeRequest_<ContainerAllocator>& v)
  {
    s << indent << "mode: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.mode);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ITM_MAV_SRVS_MESSAGE_SETMODEREQUEST_H
