// Auto-generated. Do not edit!

// (in-package itm_mav_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PositionCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.position = null;
      this.velocity = null;
      this.acceleration = null;
      this.jerk = null;
      this.snap = null;
      this.orientation = null;
      this.attitude_rate = null;
      this.thrust = null;
      this.heading = null;
      this.heading_rate = null;
      this.heading_acceleration = null;
      this.heading_jerk = null;
      this.disable_position_gains = null;
      this.disable_antiwindups = null;
      this.use_position_horizontal = null;
      this.use_position_vertical = null;
      this.use_velocity_horizontal = null;
      this.use_velocity_vertical = null;
      this.use_acceleration = null;
      this.use_jerk = null;
      this.use_snap = null;
      this.use_attitude_rate = null;
      this.use_heading = null;
      this.use_heading_rate = null;
      this.use_heading_acceleration = null;
      this.use_heading_jerk = null;
      this.use_orientation = null;
      this.use_thrust = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('acceleration')) {
        this.acceleration = initObj.acceleration
      }
      else {
        this.acceleration = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('jerk')) {
        this.jerk = initObj.jerk
      }
      else {
        this.jerk = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('snap')) {
        this.snap = initObj.snap
      }
      else {
        this.snap = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('orientation')) {
        this.orientation = initObj.orientation
      }
      else {
        this.orientation = new geometry_msgs.msg.Quaternion();
      }
      if (initObj.hasOwnProperty('attitude_rate')) {
        this.attitude_rate = initObj.attitude_rate
      }
      else {
        this.attitude_rate = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('thrust')) {
        this.thrust = initObj.thrust
      }
      else {
        this.thrust = 0.0;
      }
      if (initObj.hasOwnProperty('heading')) {
        this.heading = initObj.heading
      }
      else {
        this.heading = 0.0;
      }
      if (initObj.hasOwnProperty('heading_rate')) {
        this.heading_rate = initObj.heading_rate
      }
      else {
        this.heading_rate = 0.0;
      }
      if (initObj.hasOwnProperty('heading_acceleration')) {
        this.heading_acceleration = initObj.heading_acceleration
      }
      else {
        this.heading_acceleration = 0.0;
      }
      if (initObj.hasOwnProperty('heading_jerk')) {
        this.heading_jerk = initObj.heading_jerk
      }
      else {
        this.heading_jerk = 0.0;
      }
      if (initObj.hasOwnProperty('disable_position_gains')) {
        this.disable_position_gains = initObj.disable_position_gains
      }
      else {
        this.disable_position_gains = false;
      }
      if (initObj.hasOwnProperty('disable_antiwindups')) {
        this.disable_antiwindups = initObj.disable_antiwindups
      }
      else {
        this.disable_antiwindups = false;
      }
      if (initObj.hasOwnProperty('use_position_horizontal')) {
        this.use_position_horizontal = initObj.use_position_horizontal
      }
      else {
        this.use_position_horizontal = 0;
      }
      if (initObj.hasOwnProperty('use_position_vertical')) {
        this.use_position_vertical = initObj.use_position_vertical
      }
      else {
        this.use_position_vertical = 0;
      }
      if (initObj.hasOwnProperty('use_velocity_horizontal')) {
        this.use_velocity_horizontal = initObj.use_velocity_horizontal
      }
      else {
        this.use_velocity_horizontal = 0;
      }
      if (initObj.hasOwnProperty('use_velocity_vertical')) {
        this.use_velocity_vertical = initObj.use_velocity_vertical
      }
      else {
        this.use_velocity_vertical = 0;
      }
      if (initObj.hasOwnProperty('use_acceleration')) {
        this.use_acceleration = initObj.use_acceleration
      }
      else {
        this.use_acceleration = 0;
      }
      if (initObj.hasOwnProperty('use_jerk')) {
        this.use_jerk = initObj.use_jerk
      }
      else {
        this.use_jerk = 0;
      }
      if (initObj.hasOwnProperty('use_snap')) {
        this.use_snap = initObj.use_snap
      }
      else {
        this.use_snap = 0;
      }
      if (initObj.hasOwnProperty('use_attitude_rate')) {
        this.use_attitude_rate = initObj.use_attitude_rate
      }
      else {
        this.use_attitude_rate = 0;
      }
      if (initObj.hasOwnProperty('use_heading')) {
        this.use_heading = initObj.use_heading
      }
      else {
        this.use_heading = 0;
      }
      if (initObj.hasOwnProperty('use_heading_rate')) {
        this.use_heading_rate = initObj.use_heading_rate
      }
      else {
        this.use_heading_rate = 0;
      }
      if (initObj.hasOwnProperty('use_heading_acceleration')) {
        this.use_heading_acceleration = initObj.use_heading_acceleration
      }
      else {
        this.use_heading_acceleration = 0;
      }
      if (initObj.hasOwnProperty('use_heading_jerk')) {
        this.use_heading_jerk = initObj.use_heading_jerk
      }
      else {
        this.use_heading_jerk = 0;
      }
      if (initObj.hasOwnProperty('use_orientation')) {
        this.use_orientation = initObj.use_orientation
      }
      else {
        this.use_orientation = 0;
      }
      if (initObj.hasOwnProperty('use_thrust')) {
        this.use_thrust = initObj.use_thrust
      }
      else {
        this.use_thrust = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PositionCommand
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.velocity, buffer, bufferOffset);
    // Serialize message field [acceleration]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.acceleration, buffer, bufferOffset);
    // Serialize message field [jerk]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.jerk, buffer, bufferOffset);
    // Serialize message field [snap]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.snap, buffer, bufferOffset);
    // Serialize message field [orientation]
    bufferOffset = geometry_msgs.msg.Quaternion.serialize(obj.orientation, buffer, bufferOffset);
    // Serialize message field [attitude_rate]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.attitude_rate, buffer, bufferOffset);
    // Serialize message field [thrust]
    bufferOffset = _serializer.float64(obj.thrust, buffer, bufferOffset);
    // Serialize message field [heading]
    bufferOffset = _serializer.float64(obj.heading, buffer, bufferOffset);
    // Serialize message field [heading_rate]
    bufferOffset = _serializer.float64(obj.heading_rate, buffer, bufferOffset);
    // Serialize message field [heading_acceleration]
    bufferOffset = _serializer.float64(obj.heading_acceleration, buffer, bufferOffset);
    // Serialize message field [heading_jerk]
    bufferOffset = _serializer.float64(obj.heading_jerk, buffer, bufferOffset);
    // Serialize message field [disable_position_gains]
    bufferOffset = _serializer.bool(obj.disable_position_gains, buffer, bufferOffset);
    // Serialize message field [disable_antiwindups]
    bufferOffset = _serializer.bool(obj.disable_antiwindups, buffer, bufferOffset);
    // Serialize message field [use_position_horizontal]
    bufferOffset = _serializer.uint8(obj.use_position_horizontal, buffer, bufferOffset);
    // Serialize message field [use_position_vertical]
    bufferOffset = _serializer.uint8(obj.use_position_vertical, buffer, bufferOffset);
    // Serialize message field [use_velocity_horizontal]
    bufferOffset = _serializer.uint8(obj.use_velocity_horizontal, buffer, bufferOffset);
    // Serialize message field [use_velocity_vertical]
    bufferOffset = _serializer.uint8(obj.use_velocity_vertical, buffer, bufferOffset);
    // Serialize message field [use_acceleration]
    bufferOffset = _serializer.uint8(obj.use_acceleration, buffer, bufferOffset);
    // Serialize message field [use_jerk]
    bufferOffset = _serializer.uint8(obj.use_jerk, buffer, bufferOffset);
    // Serialize message field [use_snap]
    bufferOffset = _serializer.uint8(obj.use_snap, buffer, bufferOffset);
    // Serialize message field [use_attitude_rate]
    bufferOffset = _serializer.uint8(obj.use_attitude_rate, buffer, bufferOffset);
    // Serialize message field [use_heading]
    bufferOffset = _serializer.uint8(obj.use_heading, buffer, bufferOffset);
    // Serialize message field [use_heading_rate]
    bufferOffset = _serializer.uint8(obj.use_heading_rate, buffer, bufferOffset);
    // Serialize message field [use_heading_acceleration]
    bufferOffset = _serializer.uint8(obj.use_heading_acceleration, buffer, bufferOffset);
    // Serialize message field [use_heading_jerk]
    bufferOffset = _serializer.uint8(obj.use_heading_jerk, buffer, bufferOffset);
    // Serialize message field [use_orientation]
    bufferOffset = _serializer.uint8(obj.use_orientation, buffer, bufferOffset);
    // Serialize message field [use_thrust]
    bufferOffset = _serializer.uint8(obj.use_thrust, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PositionCommand
    let len;
    let data = new PositionCommand(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [acceleration]
    data.acceleration = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [jerk]
    data.jerk = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [snap]
    data.snap = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [orientation]
    data.orientation = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset);
    // Deserialize message field [attitude_rate]
    data.attitude_rate = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [thrust]
    data.thrust = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [heading]
    data.heading = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [heading_rate]
    data.heading_rate = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [heading_acceleration]
    data.heading_acceleration = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [heading_jerk]
    data.heading_jerk = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [disable_position_gains]
    data.disable_position_gains = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [disable_antiwindups]
    data.disable_antiwindups = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [use_position_horizontal]
    data.use_position_horizontal = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [use_position_vertical]
    data.use_position_vertical = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [use_velocity_horizontal]
    data.use_velocity_horizontal = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [use_velocity_vertical]
    data.use_velocity_vertical = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [use_acceleration]
    data.use_acceleration = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [use_jerk]
    data.use_jerk = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [use_snap]
    data.use_snap = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [use_attitude_rate]
    data.use_attitude_rate = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [use_heading]
    data.use_heading = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [use_heading_rate]
    data.use_heading_rate = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [use_heading_acceleration]
    data.use_heading_acceleration = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [use_heading_jerk]
    data.use_heading_jerk = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [use_orientation]
    data.use_orientation = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [use_thrust]
    data.use_thrust = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 232;
  }

  static datatype() {
    // Returns string type for a message object
    return 'itm_mav_msgs/PositionCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4d8e95e3ee792c1a5ce3afe2d9f2396a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This represents the output of the currently active Tracker (mrs_uav_manager::Tracker).
    # This message is returned from a tracker by means of the update() function, called by the mrs_uav_manager::ControlManager.
    
    std_msgs/Header header
    
    # The desired position.
    geometry_msgs/Point position
    
    # The desired velocity.
    geometry_msgs/Vector3 velocity
    
    # The desired acceleration.
    geometry_msgs/Vector3 acceleration
    
    # The desired jerk.
    geometry_msgs/Vector3 jerk
    
    # The desired snap.
    geometry_msgs/Vector3 snap
    
    # The desired orientation and attitude rate.
    # It is mutually exclusive to heading+heading_rate.
    geometry_msgs/Quaternion orientation
    geometry_msgs/Point attitude_rate
    
    # when used, it will override the thrust output of the controller
    float64 thrust
    
    # The desired heading and heading rate.
    # It is mutually exclusive to orientation+attitude_rate.
    float64 heading
    float64 heading_rate
    float64 heading_acceleration
    float64 heading_jerk
    
    # Whether the controller should mute its position feedback.
    # Useful, e.g., during takeoff or in situations where sudden control
    # error is expected but not immediately required to be fixed.
    bool disable_position_gains
    
    # Whether the controller should disable its antiwindups
    bool disable_antiwindups
    
    # Flags defining whether the XY or Z position reference was filled in.
    # If not, the controllers should pay no attention to it and to the corresponding control error.
    uint8 use_position_horizontal
    uint8 use_position_vertical
    
    # Flags defining whether the XY or Z velocity reference was filled in.
    # If not, the controllers should pay no attention to it and to the corresponding control error.
    uint8 use_velocity_horizontal
    uint8 use_velocity_vertical
    
    # Flags defining which references were filled in.
    # If not, the controllers should pay no attention to them and to the corresponding control errors.
    uint8 use_acceleration
    uint8 use_jerk
    uint8 use_snap
    uint8 use_attitude_rate
    uint8 use_heading
    uint8 use_heading_rate
    uint8 use_heading_acceleration
    uint8 use_heading_jerk
    uint8 use_orientation
    uint8 use_thrust
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PositionCommand(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Point.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Point()
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = geometry_msgs.msg.Vector3.Resolve(msg.velocity)
    }
    else {
      resolved.velocity = new geometry_msgs.msg.Vector3()
    }

    if (msg.acceleration !== undefined) {
      resolved.acceleration = geometry_msgs.msg.Vector3.Resolve(msg.acceleration)
    }
    else {
      resolved.acceleration = new geometry_msgs.msg.Vector3()
    }

    if (msg.jerk !== undefined) {
      resolved.jerk = geometry_msgs.msg.Vector3.Resolve(msg.jerk)
    }
    else {
      resolved.jerk = new geometry_msgs.msg.Vector3()
    }

    if (msg.snap !== undefined) {
      resolved.snap = geometry_msgs.msg.Vector3.Resolve(msg.snap)
    }
    else {
      resolved.snap = new geometry_msgs.msg.Vector3()
    }

    if (msg.orientation !== undefined) {
      resolved.orientation = geometry_msgs.msg.Quaternion.Resolve(msg.orientation)
    }
    else {
      resolved.orientation = new geometry_msgs.msg.Quaternion()
    }

    if (msg.attitude_rate !== undefined) {
      resolved.attitude_rate = geometry_msgs.msg.Point.Resolve(msg.attitude_rate)
    }
    else {
      resolved.attitude_rate = new geometry_msgs.msg.Point()
    }

    if (msg.thrust !== undefined) {
      resolved.thrust = msg.thrust;
    }
    else {
      resolved.thrust = 0.0
    }

    if (msg.heading !== undefined) {
      resolved.heading = msg.heading;
    }
    else {
      resolved.heading = 0.0
    }

    if (msg.heading_rate !== undefined) {
      resolved.heading_rate = msg.heading_rate;
    }
    else {
      resolved.heading_rate = 0.0
    }

    if (msg.heading_acceleration !== undefined) {
      resolved.heading_acceleration = msg.heading_acceleration;
    }
    else {
      resolved.heading_acceleration = 0.0
    }

    if (msg.heading_jerk !== undefined) {
      resolved.heading_jerk = msg.heading_jerk;
    }
    else {
      resolved.heading_jerk = 0.0
    }

    if (msg.disable_position_gains !== undefined) {
      resolved.disable_position_gains = msg.disable_position_gains;
    }
    else {
      resolved.disable_position_gains = false
    }

    if (msg.disable_antiwindups !== undefined) {
      resolved.disable_antiwindups = msg.disable_antiwindups;
    }
    else {
      resolved.disable_antiwindups = false
    }

    if (msg.use_position_horizontal !== undefined) {
      resolved.use_position_horizontal = msg.use_position_horizontal;
    }
    else {
      resolved.use_position_horizontal = 0
    }

    if (msg.use_position_vertical !== undefined) {
      resolved.use_position_vertical = msg.use_position_vertical;
    }
    else {
      resolved.use_position_vertical = 0
    }

    if (msg.use_velocity_horizontal !== undefined) {
      resolved.use_velocity_horizontal = msg.use_velocity_horizontal;
    }
    else {
      resolved.use_velocity_horizontal = 0
    }

    if (msg.use_velocity_vertical !== undefined) {
      resolved.use_velocity_vertical = msg.use_velocity_vertical;
    }
    else {
      resolved.use_velocity_vertical = 0
    }

    if (msg.use_acceleration !== undefined) {
      resolved.use_acceleration = msg.use_acceleration;
    }
    else {
      resolved.use_acceleration = 0
    }

    if (msg.use_jerk !== undefined) {
      resolved.use_jerk = msg.use_jerk;
    }
    else {
      resolved.use_jerk = 0
    }

    if (msg.use_snap !== undefined) {
      resolved.use_snap = msg.use_snap;
    }
    else {
      resolved.use_snap = 0
    }

    if (msg.use_attitude_rate !== undefined) {
      resolved.use_attitude_rate = msg.use_attitude_rate;
    }
    else {
      resolved.use_attitude_rate = 0
    }

    if (msg.use_heading !== undefined) {
      resolved.use_heading = msg.use_heading;
    }
    else {
      resolved.use_heading = 0
    }

    if (msg.use_heading_rate !== undefined) {
      resolved.use_heading_rate = msg.use_heading_rate;
    }
    else {
      resolved.use_heading_rate = 0
    }

    if (msg.use_heading_acceleration !== undefined) {
      resolved.use_heading_acceleration = msg.use_heading_acceleration;
    }
    else {
      resolved.use_heading_acceleration = 0
    }

    if (msg.use_heading_jerk !== undefined) {
      resolved.use_heading_jerk = msg.use_heading_jerk;
    }
    else {
      resolved.use_heading_jerk = 0
    }

    if (msg.use_orientation !== undefined) {
      resolved.use_orientation = msg.use_orientation;
    }
    else {
      resolved.use_orientation = 0
    }

    if (msg.use_thrust !== undefined) {
      resolved.use_thrust = msg.use_thrust;
    }
    else {
      resolved.use_thrust = 0
    }

    return resolved;
    }
};

module.exports = PositionCommand;
