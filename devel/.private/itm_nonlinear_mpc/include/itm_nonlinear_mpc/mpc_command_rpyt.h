// Generated by gencpp from file itm_nonlinear_mpc/mpc_command_rpyt.msg
// DO NOT EDIT!


#ifndef ITM_NONLINEAR_MPC_MESSAGE_MPC_COMMAND_RPYT_H
#define ITM_NONLINEAR_MPC_MESSAGE_MPC_COMMAND_RPYT_H


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
struct mpc_command_rpyt_
{
  typedef mpc_command_rpyt_<ContainerAllocator> Type;

  mpc_command_rpyt_()
    : header()
    , roll_ref(0.0)
    , pitch_ref(0.0)
    , yaw_rate_cmd(0.0)
    , thrust_ref(0.0)  {
    }
  mpc_command_rpyt_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , roll_ref(0.0)
    , pitch_ref(0.0)
    , yaw_rate_cmd(0.0)
    , thrust_ref(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _roll_ref_type;
  _roll_ref_type roll_ref;

   typedef double _pitch_ref_type;
  _pitch_ref_type pitch_ref;

   typedef double _yaw_rate_cmd_type;
  _yaw_rate_cmd_type yaw_rate_cmd;

   typedef double _thrust_ref_type;
  _thrust_ref_type thrust_ref;





  typedef boost::shared_ptr< ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator> const> ConstPtr;

}; // struct mpc_command_rpyt_

typedef ::itm_nonlinear_mpc::mpc_command_rpyt_<std::allocator<void> > mpc_command_rpyt;

typedef boost::shared_ptr< ::itm_nonlinear_mpc::mpc_command_rpyt > mpc_command_rpytPtr;
typedef boost::shared_ptr< ::itm_nonlinear_mpc::mpc_command_rpyt const> mpc_command_rpytConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator1> & lhs, const ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.roll_ref == rhs.roll_ref &&
    lhs.pitch_ref == rhs.pitch_ref &&
    lhs.yaw_rate_cmd == rhs.yaw_rate_cmd &&
    lhs.thrust_ref == rhs.thrust_ref;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator1> & lhs, const ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace itm_nonlinear_mpc

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3a1aaed29b0fec0f986f12a3290ec8b8";
  }

  static const char* value(const ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3a1aaed29b0fec0fULL;
  static const uint64_t static_value2 = 0x986f12a3290ec8b8ULL;
};

template<class ContainerAllocator>
struct DataType< ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "itm_nonlinear_mpc/mpc_command_rpyt";
  }

  static const char* value(const ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"float64 roll_ref\n"
"float64 pitch_ref\n"
"float64 yaw_rate_cmd\n"
"float64 thrust_ref\n"
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

  static const char* value(const ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.roll_ref);
      stream.next(m.pitch_ref);
      stream.next(m.yaw_rate_cmd);
      stream.next(m.thrust_ref);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct mpc_command_rpyt_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::itm_nonlinear_mpc::mpc_command_rpyt_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "roll_ref: ";
    Printer<double>::stream(s, indent + "  ", v.roll_ref);
    s << indent << "pitch_ref: ";
    Printer<double>::stream(s, indent + "  ", v.pitch_ref);
    s << indent << "yaw_rate_cmd: ";
    Printer<double>::stream(s, indent + "  ", v.yaw_rate_cmd);
    s << indent << "thrust_ref: ";
    Printer<double>::stream(s, indent + "  ", v.thrust_ref);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ITM_NONLINEAR_MPC_MESSAGE_MPC_COMMAND_RPYT_H