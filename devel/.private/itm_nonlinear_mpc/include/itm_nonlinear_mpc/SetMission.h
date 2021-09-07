// Generated by gencpp from file itm_nonlinear_mpc/SetMission.msg
// DO NOT EDIT!


#ifndef ITM_NONLINEAR_MPC_MESSAGE_SETMISSION_H
#define ITM_NONLINEAR_MPC_MESSAGE_SETMISSION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace itm_nonlinear_mpc
{
template <class ContainerAllocator>
struct SetMission_
{
  typedef SetMission_<ContainerAllocator> Type;

  SetMission_()
    : header()
    , command_idx(0)
    , mission_mode(0)  {
    }
  SetMission_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , command_idx(0)
    , mission_mode(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint32_t _command_idx_type;
  _command_idx_type command_idx;

   typedef uint8_t _mission_mode_type;
  _mission_mode_type mission_mode;





  typedef boost::shared_ptr< ::itm_nonlinear_mpc::SetMission_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::itm_nonlinear_mpc::SetMission_<ContainerAllocator> const> ConstPtr;

}; // struct SetMission_

typedef ::itm_nonlinear_mpc::SetMission_<std::allocator<void> > SetMission;

typedef boost::shared_ptr< ::itm_nonlinear_mpc::SetMission > SetMissionPtr;
typedef boost::shared_ptr< ::itm_nonlinear_mpc::SetMission const> SetMissionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::itm_nonlinear_mpc::SetMission_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::itm_nonlinear_mpc::SetMission_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::itm_nonlinear_mpc::SetMission_<ContainerAllocator1> & lhs, const ::itm_nonlinear_mpc::SetMission_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.command_idx == rhs.command_idx &&
    lhs.mission_mode == rhs.mission_mode;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::itm_nonlinear_mpc::SetMission_<ContainerAllocator1> & lhs, const ::itm_nonlinear_mpc::SetMission_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace itm_nonlinear_mpc

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::itm_nonlinear_mpc::SetMission_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::itm_nonlinear_mpc::SetMission_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::itm_nonlinear_mpc::SetMission_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::itm_nonlinear_mpc::SetMission_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::itm_nonlinear_mpc::SetMission_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::itm_nonlinear_mpc::SetMission_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::itm_nonlinear_mpc::SetMission_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1631998827fd12d678dc74adb693f0b9";
  }

  static const char* value(const ::itm_nonlinear_mpc::SetMission_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1631998827fd12d6ULL;
  static const uint64_t static_value2 = 0x78dc74adb693f0b9ULL;
};

template<class ContainerAllocator>
struct DataType< ::itm_nonlinear_mpc::SetMission_<ContainerAllocator> >
{
  static const char* value()
  {
    return "itm_nonlinear_mpc/SetMission";
  }

  static const char* value(const ::itm_nonlinear_mpc::SetMission_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::itm_nonlinear_mpc::SetMission_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"uint32 command_idx\n"
"uint8 mission_mode\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::itm_nonlinear_mpc::SetMission_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::itm_nonlinear_mpc::SetMission_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.command_idx);
      stream.next(m.mission_mode);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetMission_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::itm_nonlinear_mpc::SetMission_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::itm_nonlinear_mpc::SetMission_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "command_idx: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.command_idx);
    s << indent << "mission_mode: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.mission_mode);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ITM_NONLINEAR_MPC_MESSAGE_SETMISSION_H
