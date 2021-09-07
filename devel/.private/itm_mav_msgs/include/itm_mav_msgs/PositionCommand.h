// Generated by gencpp from file itm_mav_msgs/PositionCommand.msg
// DO NOT EDIT!


#ifndef ITM_MAV_MSGS_MESSAGE_POSITIONCOMMAND_H
#define ITM_MAV_MSGS_MESSAGE_POSITIONCOMMAND_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>

namespace itm_mav_msgs
{
template <class ContainerAllocator>
struct PositionCommand_
{
  typedef PositionCommand_<ContainerAllocator> Type;

  PositionCommand_()
    : header()
    , position()
    , velocity()
    , acceleration()
    , jerk()
    , snap()
    , orientation()
    , attitude_rate()
    , thrust(0.0)
    , heading(0.0)
    , heading_rate(0.0)
    , heading_acceleration(0.0)
    , heading_jerk(0.0)
    , disable_position_gains(false)
    , disable_antiwindups(false)
    , use_position_horizontal(0)
    , use_position_vertical(0)
    , use_velocity_horizontal(0)
    , use_velocity_vertical(0)
    , use_acceleration(0)
    , use_jerk(0)
    , use_snap(0)
    , use_attitude_rate(0)
    , use_heading(0)
    , use_heading_rate(0)
    , use_heading_acceleration(0)
    , use_heading_jerk(0)
    , use_orientation(0)
    , use_thrust(0)  {
    }
  PositionCommand_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , position(_alloc)
    , velocity(_alloc)
    , acceleration(_alloc)
    , jerk(_alloc)
    , snap(_alloc)
    , orientation(_alloc)
    , attitude_rate(_alloc)
    , thrust(0.0)
    , heading(0.0)
    , heading_rate(0.0)
    , heading_acceleration(0.0)
    , heading_jerk(0.0)
    , disable_position_gains(false)
    , disable_antiwindups(false)
    , use_position_horizontal(0)
    , use_position_vertical(0)
    , use_velocity_horizontal(0)
    , use_velocity_vertical(0)
    , use_acceleration(0)
    , use_jerk(0)
    , use_snap(0)
    , use_attitude_rate(0)
    , use_heading(0)
    , use_heading_rate(0)
    , use_heading_acceleration(0)
    , use_heading_jerk(0)
    , use_orientation(0)
    , use_thrust(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _position_type;
  _position_type position;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _velocity_type;
  _velocity_type velocity;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _acceleration_type;
  _acceleration_type acceleration;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _jerk_type;
  _jerk_type jerk;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _snap_type;
  _snap_type snap;

   typedef  ::geometry_msgs::Quaternion_<ContainerAllocator>  _orientation_type;
  _orientation_type orientation;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _attitude_rate_type;
  _attitude_rate_type attitude_rate;

   typedef double _thrust_type;
  _thrust_type thrust;

   typedef double _heading_type;
  _heading_type heading;

   typedef double _heading_rate_type;
  _heading_rate_type heading_rate;

   typedef double _heading_acceleration_type;
  _heading_acceleration_type heading_acceleration;

   typedef double _heading_jerk_type;
  _heading_jerk_type heading_jerk;

   typedef uint8_t _disable_position_gains_type;
  _disable_position_gains_type disable_position_gains;

   typedef uint8_t _disable_antiwindups_type;
  _disable_antiwindups_type disable_antiwindups;

   typedef uint8_t _use_position_horizontal_type;
  _use_position_horizontal_type use_position_horizontal;

   typedef uint8_t _use_position_vertical_type;
  _use_position_vertical_type use_position_vertical;

   typedef uint8_t _use_velocity_horizontal_type;
  _use_velocity_horizontal_type use_velocity_horizontal;

   typedef uint8_t _use_velocity_vertical_type;
  _use_velocity_vertical_type use_velocity_vertical;

   typedef uint8_t _use_acceleration_type;
  _use_acceleration_type use_acceleration;

   typedef uint8_t _use_jerk_type;
  _use_jerk_type use_jerk;

   typedef uint8_t _use_snap_type;
  _use_snap_type use_snap;

   typedef uint8_t _use_attitude_rate_type;
  _use_attitude_rate_type use_attitude_rate;

   typedef uint8_t _use_heading_type;
  _use_heading_type use_heading;

   typedef uint8_t _use_heading_rate_type;
  _use_heading_rate_type use_heading_rate;

   typedef uint8_t _use_heading_acceleration_type;
  _use_heading_acceleration_type use_heading_acceleration;

   typedef uint8_t _use_heading_jerk_type;
  _use_heading_jerk_type use_heading_jerk;

   typedef uint8_t _use_orientation_type;
  _use_orientation_type use_orientation;

   typedef uint8_t _use_thrust_type;
  _use_thrust_type use_thrust;





  typedef boost::shared_ptr< ::itm_mav_msgs::PositionCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::itm_mav_msgs::PositionCommand_<ContainerAllocator> const> ConstPtr;

}; // struct PositionCommand_

typedef ::itm_mav_msgs::PositionCommand_<std::allocator<void> > PositionCommand;

typedef boost::shared_ptr< ::itm_mav_msgs::PositionCommand > PositionCommandPtr;
typedef boost::shared_ptr< ::itm_mav_msgs::PositionCommand const> PositionCommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::itm_mav_msgs::PositionCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::itm_mav_msgs::PositionCommand_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::itm_mav_msgs::PositionCommand_<ContainerAllocator1> & lhs, const ::itm_mav_msgs::PositionCommand_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.position == rhs.position &&
    lhs.velocity == rhs.velocity &&
    lhs.acceleration == rhs.acceleration &&
    lhs.jerk == rhs.jerk &&
    lhs.snap == rhs.snap &&
    lhs.orientation == rhs.orientation &&
    lhs.attitude_rate == rhs.attitude_rate &&
    lhs.thrust == rhs.thrust &&
    lhs.heading == rhs.heading &&
    lhs.heading_rate == rhs.heading_rate &&
    lhs.heading_acceleration == rhs.heading_acceleration &&
    lhs.heading_jerk == rhs.heading_jerk &&
    lhs.disable_position_gains == rhs.disable_position_gains &&
    lhs.disable_antiwindups == rhs.disable_antiwindups &&
    lhs.use_position_horizontal == rhs.use_position_horizontal &&
    lhs.use_position_vertical == rhs.use_position_vertical &&
    lhs.use_velocity_horizontal == rhs.use_velocity_horizontal &&
    lhs.use_velocity_vertical == rhs.use_velocity_vertical &&
    lhs.use_acceleration == rhs.use_acceleration &&
    lhs.use_jerk == rhs.use_jerk &&
    lhs.use_snap == rhs.use_snap &&
    lhs.use_attitude_rate == rhs.use_attitude_rate &&
    lhs.use_heading == rhs.use_heading &&
    lhs.use_heading_rate == rhs.use_heading_rate &&
    lhs.use_heading_acceleration == rhs.use_heading_acceleration &&
    lhs.use_heading_jerk == rhs.use_heading_jerk &&
    lhs.use_orientation == rhs.use_orientation &&
    lhs.use_thrust == rhs.use_thrust;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::itm_mav_msgs::PositionCommand_<ContainerAllocator1> & lhs, const ::itm_mav_msgs::PositionCommand_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace itm_mav_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::itm_mav_msgs::PositionCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::itm_mav_msgs::PositionCommand_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::itm_mav_msgs::PositionCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::itm_mav_msgs::PositionCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::itm_mav_msgs::PositionCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::itm_mav_msgs::PositionCommand_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::itm_mav_msgs::PositionCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4d8e95e3ee792c1a5ce3afe2d9f2396a";
  }

  static const char* value(const ::itm_mav_msgs::PositionCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4d8e95e3ee792c1aULL;
  static const uint64_t static_value2 = 0x5ce3afe2d9f2396aULL;
};

template<class ContainerAllocator>
struct DataType< ::itm_mav_msgs::PositionCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "itm_mav_msgs/PositionCommand";
  }

  static const char* value(const ::itm_mav_msgs::PositionCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::itm_mav_msgs::PositionCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This represents the output of the currently active Tracker (mrs_uav_manager::Tracker).\n"
"# This message is returned from a tracker by means of the update() function, called by the mrs_uav_manager::ControlManager.\n"
"\n"
"std_msgs/Header header\n"
"\n"
"# The desired position.\n"
"geometry_msgs/Point position\n"
"\n"
"# The desired velocity.\n"
"geometry_msgs/Vector3 velocity\n"
"\n"
"# The desired acceleration.\n"
"geometry_msgs/Vector3 acceleration\n"
"\n"
"# The desired jerk.\n"
"geometry_msgs/Vector3 jerk\n"
"\n"
"# The desired snap.\n"
"geometry_msgs/Vector3 snap\n"
"\n"
"# The desired orientation and attitude rate.\n"
"# It is mutually exclusive to heading+heading_rate.\n"
"geometry_msgs/Quaternion orientation\n"
"geometry_msgs/Point attitude_rate\n"
"\n"
"# when used, it will override the thrust output of the controller\n"
"float64 thrust\n"
"\n"
"# The desired heading and heading rate.\n"
"# It is mutually exclusive to orientation+attitude_rate.\n"
"float64 heading\n"
"float64 heading_rate\n"
"float64 heading_acceleration\n"
"float64 heading_jerk\n"
"\n"
"# Whether the controller should mute its position feedback.\n"
"# Useful, e.g., during takeoff or in situations where sudden control\n"
"# error is expected but not immediately required to be fixed.\n"
"bool disable_position_gains\n"
"\n"
"# Whether the controller should disable its antiwindups\n"
"bool disable_antiwindups\n"
"\n"
"# Flags defining whether the XY or Z position reference was filled in.\n"
"# If not, the controllers should pay no attention to it and to the corresponding control error.\n"
"uint8 use_position_horizontal\n"
"uint8 use_position_vertical\n"
"\n"
"# Flags defining whether the XY or Z velocity reference was filled in.\n"
"# If not, the controllers should pay no attention to it and to the corresponding control error.\n"
"uint8 use_velocity_horizontal\n"
"uint8 use_velocity_vertical\n"
"\n"
"# Flags defining which references were filled in.\n"
"# If not, the controllers should pay no attention to them and to the corresponding control errors.\n"
"uint8 use_acceleration\n"
"uint8 use_jerk\n"
"uint8 use_snap\n"
"uint8 use_attitude_rate\n"
"uint8 use_heading\n"
"uint8 use_heading_rate\n"
"uint8 use_heading_acceleration\n"
"uint8 use_heading_jerk\n"
"uint8 use_orientation\n"
"uint8 use_thrust\n"
"\n"
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
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::itm_mav_msgs::PositionCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::itm_mav_msgs::PositionCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.position);
      stream.next(m.velocity);
      stream.next(m.acceleration);
      stream.next(m.jerk);
      stream.next(m.snap);
      stream.next(m.orientation);
      stream.next(m.attitude_rate);
      stream.next(m.thrust);
      stream.next(m.heading);
      stream.next(m.heading_rate);
      stream.next(m.heading_acceleration);
      stream.next(m.heading_jerk);
      stream.next(m.disable_position_gains);
      stream.next(m.disable_antiwindups);
      stream.next(m.use_position_horizontal);
      stream.next(m.use_position_vertical);
      stream.next(m.use_velocity_horizontal);
      stream.next(m.use_velocity_vertical);
      stream.next(m.use_acceleration);
      stream.next(m.use_jerk);
      stream.next(m.use_snap);
      stream.next(m.use_attitude_rate);
      stream.next(m.use_heading);
      stream.next(m.use_heading_rate);
      stream.next(m.use_heading_acceleration);
      stream.next(m.use_heading_jerk);
      stream.next(m.use_orientation);
      stream.next(m.use_thrust);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PositionCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::itm_mav_msgs::PositionCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::itm_mav_msgs::PositionCommand_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "position: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.position);
    s << indent << "velocity: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity);
    s << indent << "acceleration: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.acceleration);
    s << indent << "jerk: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.jerk);
    s << indent << "snap: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.snap);
    s << indent << "orientation: ";
    s << std::endl;
    Printer< ::geometry_msgs::Quaternion_<ContainerAllocator> >::stream(s, indent + "  ", v.orientation);
    s << indent << "attitude_rate: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.attitude_rate);
    s << indent << "thrust: ";
    Printer<double>::stream(s, indent + "  ", v.thrust);
    s << indent << "heading: ";
    Printer<double>::stream(s, indent + "  ", v.heading);
    s << indent << "heading_rate: ";
    Printer<double>::stream(s, indent + "  ", v.heading_rate);
    s << indent << "heading_acceleration: ";
    Printer<double>::stream(s, indent + "  ", v.heading_acceleration);
    s << indent << "heading_jerk: ";
    Printer<double>::stream(s, indent + "  ", v.heading_jerk);
    s << indent << "disable_position_gains: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.disable_position_gains);
    s << indent << "disable_antiwindups: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.disable_antiwindups);
    s << indent << "use_position_horizontal: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_position_horizontal);
    s << indent << "use_position_vertical: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_position_vertical);
    s << indent << "use_velocity_horizontal: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_velocity_horizontal);
    s << indent << "use_velocity_vertical: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_velocity_vertical);
    s << indent << "use_acceleration: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_acceleration);
    s << indent << "use_jerk: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_jerk);
    s << indent << "use_snap: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_snap);
    s << indent << "use_attitude_rate: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_attitude_rate);
    s << indent << "use_heading: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_heading);
    s << indent << "use_heading_rate: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_heading_rate);
    s << indent << "use_heading_acceleration: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_heading_acceleration);
    s << indent << "use_heading_jerk: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_heading_jerk);
    s << indent << "use_orientation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_orientation);
    s << indent << "use_thrust: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_thrust);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ITM_MAV_MSGS_MESSAGE_POSITIONCOMMAND_H