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

class AttitudeCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.controller = null;
      this.ramping_up = null;
      this.attitude = null;
      this.attitude_rate = null;
      this.desired_acceleration = null;
      this.thrust = null;
      this.mass_difference = null;
      this.total_mass = null;
      this.disturbance_wx_w = null;
      this.disturbance_wy_w = null;
      this.disturbance_bx_w = null;
      this.disturbance_by_w = null;
      this.disturbance_bx_b = null;
      this.disturbance_by_b = null;
      this.controller_enforcing_constraints = null;
      this.horizontal_speed_constraint = null;
      this.horizontal_acc_constraint = null;
      this.vertical_asc_speed_constraint = null;
      this.vertical_asc_acc_constraint = null;
      this.vertical_desc_speed_constraint = null;
      this.vertical_desc_acc_constraint = null;
      this.mode_mask = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('controller')) {
        this.controller = initObj.controller
      }
      else {
        this.controller = '';
      }
      if (initObj.hasOwnProperty('ramping_up')) {
        this.ramping_up = initObj.ramping_up
      }
      else {
        this.ramping_up = false;
      }
      if (initObj.hasOwnProperty('attitude')) {
        this.attitude = initObj.attitude
      }
      else {
        this.attitude = new geometry_msgs.msg.Quaternion();
      }
      if (initObj.hasOwnProperty('attitude_rate')) {
        this.attitude_rate = initObj.attitude_rate
      }
      else {
        this.attitude_rate = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('desired_acceleration')) {
        this.desired_acceleration = initObj.desired_acceleration
      }
      else {
        this.desired_acceleration = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('thrust')) {
        this.thrust = initObj.thrust
      }
      else {
        this.thrust = 0.0;
      }
      if (initObj.hasOwnProperty('mass_difference')) {
        this.mass_difference = initObj.mass_difference
      }
      else {
        this.mass_difference = 0.0;
      }
      if (initObj.hasOwnProperty('total_mass')) {
        this.total_mass = initObj.total_mass
      }
      else {
        this.total_mass = 0.0;
      }
      if (initObj.hasOwnProperty('disturbance_wx_w')) {
        this.disturbance_wx_w = initObj.disturbance_wx_w
      }
      else {
        this.disturbance_wx_w = 0.0;
      }
      if (initObj.hasOwnProperty('disturbance_wy_w')) {
        this.disturbance_wy_w = initObj.disturbance_wy_w
      }
      else {
        this.disturbance_wy_w = 0.0;
      }
      if (initObj.hasOwnProperty('disturbance_bx_w')) {
        this.disturbance_bx_w = initObj.disturbance_bx_w
      }
      else {
        this.disturbance_bx_w = 0.0;
      }
      if (initObj.hasOwnProperty('disturbance_by_w')) {
        this.disturbance_by_w = initObj.disturbance_by_w
      }
      else {
        this.disturbance_by_w = 0.0;
      }
      if (initObj.hasOwnProperty('disturbance_bx_b')) {
        this.disturbance_bx_b = initObj.disturbance_bx_b
      }
      else {
        this.disturbance_bx_b = 0.0;
      }
      if (initObj.hasOwnProperty('disturbance_by_b')) {
        this.disturbance_by_b = initObj.disturbance_by_b
      }
      else {
        this.disturbance_by_b = 0.0;
      }
      if (initObj.hasOwnProperty('controller_enforcing_constraints')) {
        this.controller_enforcing_constraints = initObj.controller_enforcing_constraints
      }
      else {
        this.controller_enforcing_constraints = false;
      }
      if (initObj.hasOwnProperty('horizontal_speed_constraint')) {
        this.horizontal_speed_constraint = initObj.horizontal_speed_constraint
      }
      else {
        this.horizontal_speed_constraint = 0.0;
      }
      if (initObj.hasOwnProperty('horizontal_acc_constraint')) {
        this.horizontal_acc_constraint = initObj.horizontal_acc_constraint
      }
      else {
        this.horizontal_acc_constraint = 0.0;
      }
      if (initObj.hasOwnProperty('vertical_asc_speed_constraint')) {
        this.vertical_asc_speed_constraint = initObj.vertical_asc_speed_constraint
      }
      else {
        this.vertical_asc_speed_constraint = 0.0;
      }
      if (initObj.hasOwnProperty('vertical_asc_acc_constraint')) {
        this.vertical_asc_acc_constraint = initObj.vertical_asc_acc_constraint
      }
      else {
        this.vertical_asc_acc_constraint = 0.0;
      }
      if (initObj.hasOwnProperty('vertical_desc_speed_constraint')) {
        this.vertical_desc_speed_constraint = initObj.vertical_desc_speed_constraint
      }
      else {
        this.vertical_desc_speed_constraint = 0.0;
      }
      if (initObj.hasOwnProperty('vertical_desc_acc_constraint')) {
        this.vertical_desc_acc_constraint = initObj.vertical_desc_acc_constraint
      }
      else {
        this.vertical_desc_acc_constraint = 0.0;
      }
      if (initObj.hasOwnProperty('mode_mask')) {
        this.mode_mask = initObj.mode_mask
      }
      else {
        this.mode_mask = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AttitudeCommand
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [controller]
    bufferOffset = _serializer.string(obj.controller, buffer, bufferOffset);
    // Serialize message field [ramping_up]
    bufferOffset = _serializer.bool(obj.ramping_up, buffer, bufferOffset);
    // Serialize message field [attitude]
    bufferOffset = geometry_msgs.msg.Quaternion.serialize(obj.attitude, buffer, bufferOffset);
    // Serialize message field [attitude_rate]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.attitude_rate, buffer, bufferOffset);
    // Serialize message field [desired_acceleration]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.desired_acceleration, buffer, bufferOffset);
    // Serialize message field [thrust]
    bufferOffset = _serializer.float64(obj.thrust, buffer, bufferOffset);
    // Serialize message field [mass_difference]
    bufferOffset = _serializer.float64(obj.mass_difference, buffer, bufferOffset);
    // Serialize message field [total_mass]
    bufferOffset = _serializer.float64(obj.total_mass, buffer, bufferOffset);
    // Serialize message field [disturbance_wx_w]
    bufferOffset = _serializer.float64(obj.disturbance_wx_w, buffer, bufferOffset);
    // Serialize message field [disturbance_wy_w]
    bufferOffset = _serializer.float64(obj.disturbance_wy_w, buffer, bufferOffset);
    // Serialize message field [disturbance_bx_w]
    bufferOffset = _serializer.float64(obj.disturbance_bx_w, buffer, bufferOffset);
    // Serialize message field [disturbance_by_w]
    bufferOffset = _serializer.float64(obj.disturbance_by_w, buffer, bufferOffset);
    // Serialize message field [disturbance_bx_b]
    bufferOffset = _serializer.float64(obj.disturbance_bx_b, buffer, bufferOffset);
    // Serialize message field [disturbance_by_b]
    bufferOffset = _serializer.float64(obj.disturbance_by_b, buffer, bufferOffset);
    // Serialize message field [controller_enforcing_constraints]
    bufferOffset = _serializer.bool(obj.controller_enforcing_constraints, buffer, bufferOffset);
    // Serialize message field [horizontal_speed_constraint]
    bufferOffset = _serializer.float64(obj.horizontal_speed_constraint, buffer, bufferOffset);
    // Serialize message field [horizontal_acc_constraint]
    bufferOffset = _serializer.float64(obj.horizontal_acc_constraint, buffer, bufferOffset);
    // Serialize message field [vertical_asc_speed_constraint]
    bufferOffset = _serializer.float64(obj.vertical_asc_speed_constraint, buffer, bufferOffset);
    // Serialize message field [vertical_asc_acc_constraint]
    bufferOffset = _serializer.float64(obj.vertical_asc_acc_constraint, buffer, bufferOffset);
    // Serialize message field [vertical_desc_speed_constraint]
    bufferOffset = _serializer.float64(obj.vertical_desc_speed_constraint, buffer, bufferOffset);
    // Serialize message field [vertical_desc_acc_constraint]
    bufferOffset = _serializer.float64(obj.vertical_desc_acc_constraint, buffer, bufferOffset);
    // Serialize message field [mode_mask]
    bufferOffset = _serializer.uint8(obj.mode_mask, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AttitudeCommand
    let len;
    let data = new AttitudeCommand(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [controller]
    data.controller = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [ramping_up]
    data.ramping_up = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [attitude]
    data.attitude = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset);
    // Deserialize message field [attitude_rate]
    data.attitude_rate = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [desired_acceleration]
    data.desired_acceleration = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [thrust]
    data.thrust = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mass_difference]
    data.mass_difference = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [total_mass]
    data.total_mass = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [disturbance_wx_w]
    data.disturbance_wx_w = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [disturbance_wy_w]
    data.disturbance_wy_w = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [disturbance_bx_w]
    data.disturbance_bx_w = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [disturbance_by_w]
    data.disturbance_by_w = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [disturbance_bx_b]
    data.disturbance_bx_b = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [disturbance_by_b]
    data.disturbance_by_b = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [controller_enforcing_constraints]
    data.controller_enforcing_constraints = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [horizontal_speed_constraint]
    data.horizontal_speed_constraint = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [horizontal_acc_constraint]
    data.horizontal_acc_constraint = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vertical_asc_speed_constraint]
    data.vertical_asc_speed_constraint = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vertical_asc_acc_constraint]
    data.vertical_asc_acc_constraint = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vertical_desc_speed_constraint]
    data.vertical_desc_speed_constraint = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vertical_desc_acc_constraint]
    data.vertical_desc_acc_constraint = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mode_mask]
    data.mode_mask = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.controller.length;
    return length + 207;
  }

  static datatype() {
    // Returns string type for a message object
    return 'itm_mav_msgs/AttitudeCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ba99a1fcefbbc4c8eb8328bcdd1d674c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This represents an output of a UAV feedback controller (mrs_uav_manager::Controller).
    # This message is returned from a controller by means of the update() function, called by the mrs_uav_manager::ControlManager.
    
    # fork from CTU work
    
    std_msgs/Header header
    
    # The name of the controller (implementation-wise).
    # Beware, multiple instances of the same controller can be running.
    # The ControlManagerDiagnostics message contains the name given them
    # by the ControlManager.
    string controller
    
    # True if the controller is in the initial rampup phase (just after activation).
    bool ramping_up
    
    # The desired orientation produced by the controller.
    # This field should be filled every time.
    geometry_msgs/Quaternion attitude
    
    # The desired attitude rate produced by the controller.
    # This field is optional.
    geometry_msgs/Point attitude_rate
    
    # Desired acceleration produced by the controller.
    # This field is mandatory if flying with "mrs_odometry".
    # The desired acceleration should be without calculate without
    # compensation of external forces and disturbances, e.g., without
    # the compensation for the gravity vector, wind and internal UAV biases.
    geometry_msgs/Point desired_acceleration
    
    # The desired thrust, [0, 1].
    float64 thrust
    
    # The estimated mass difference from the nominal mass.
    float64 mass_difference
    
    # Total estimated UAV mass.
    float64 total_mass
    
    # World-frame disturbances expressed in the world frame.
    float64 disturbance_wx_w
    float64 disturbance_wy_w
    
    # Body-frame (fcu_untilted) disturbances expressed in the world frame.
    float64 disturbance_bx_w
    float64 disturbance_by_w
    
    # Body-frame (fcu_untilted) disturbances expressed in the body frame (fcu_untilted).
    float64 disturbance_bx_b
    float64 disturbance_by_b
    
    # The controller can enforce dynamics constraints over the trackers.
    # This can be used when flying with a controller that is limited to
    # some maximum speed and acceleration.
    bool controller_enforcing_constraints
    float64 horizontal_speed_constraint
    float64 horizontal_acc_constraint
    float64 vertical_asc_speed_constraint
    float64 vertical_asc_acc_constraint
    float64 vertical_desc_speed_constraint
    float64 vertical_desc_acc_constraint
    
    # Defines what output should be used, whether the attitude
    # or the attitude rate.
    uint8 mode_mask
    uint8 MODE_ATTITUDE=1
    uint8 MODE_ATTITUDE_RATE=2
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
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AttitudeCommand(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.controller !== undefined) {
      resolved.controller = msg.controller;
    }
    else {
      resolved.controller = ''
    }

    if (msg.ramping_up !== undefined) {
      resolved.ramping_up = msg.ramping_up;
    }
    else {
      resolved.ramping_up = false
    }

    if (msg.attitude !== undefined) {
      resolved.attitude = geometry_msgs.msg.Quaternion.Resolve(msg.attitude)
    }
    else {
      resolved.attitude = new geometry_msgs.msg.Quaternion()
    }

    if (msg.attitude_rate !== undefined) {
      resolved.attitude_rate = geometry_msgs.msg.Point.Resolve(msg.attitude_rate)
    }
    else {
      resolved.attitude_rate = new geometry_msgs.msg.Point()
    }

    if (msg.desired_acceleration !== undefined) {
      resolved.desired_acceleration = geometry_msgs.msg.Point.Resolve(msg.desired_acceleration)
    }
    else {
      resolved.desired_acceleration = new geometry_msgs.msg.Point()
    }

    if (msg.thrust !== undefined) {
      resolved.thrust = msg.thrust;
    }
    else {
      resolved.thrust = 0.0
    }

    if (msg.mass_difference !== undefined) {
      resolved.mass_difference = msg.mass_difference;
    }
    else {
      resolved.mass_difference = 0.0
    }

    if (msg.total_mass !== undefined) {
      resolved.total_mass = msg.total_mass;
    }
    else {
      resolved.total_mass = 0.0
    }

    if (msg.disturbance_wx_w !== undefined) {
      resolved.disturbance_wx_w = msg.disturbance_wx_w;
    }
    else {
      resolved.disturbance_wx_w = 0.0
    }

    if (msg.disturbance_wy_w !== undefined) {
      resolved.disturbance_wy_w = msg.disturbance_wy_w;
    }
    else {
      resolved.disturbance_wy_w = 0.0
    }

    if (msg.disturbance_bx_w !== undefined) {
      resolved.disturbance_bx_w = msg.disturbance_bx_w;
    }
    else {
      resolved.disturbance_bx_w = 0.0
    }

    if (msg.disturbance_by_w !== undefined) {
      resolved.disturbance_by_w = msg.disturbance_by_w;
    }
    else {
      resolved.disturbance_by_w = 0.0
    }

    if (msg.disturbance_bx_b !== undefined) {
      resolved.disturbance_bx_b = msg.disturbance_bx_b;
    }
    else {
      resolved.disturbance_bx_b = 0.0
    }

    if (msg.disturbance_by_b !== undefined) {
      resolved.disturbance_by_b = msg.disturbance_by_b;
    }
    else {
      resolved.disturbance_by_b = 0.0
    }

    if (msg.controller_enforcing_constraints !== undefined) {
      resolved.controller_enforcing_constraints = msg.controller_enforcing_constraints;
    }
    else {
      resolved.controller_enforcing_constraints = false
    }

    if (msg.horizontal_speed_constraint !== undefined) {
      resolved.horizontal_speed_constraint = msg.horizontal_speed_constraint;
    }
    else {
      resolved.horizontal_speed_constraint = 0.0
    }

    if (msg.horizontal_acc_constraint !== undefined) {
      resolved.horizontal_acc_constraint = msg.horizontal_acc_constraint;
    }
    else {
      resolved.horizontal_acc_constraint = 0.0
    }

    if (msg.vertical_asc_speed_constraint !== undefined) {
      resolved.vertical_asc_speed_constraint = msg.vertical_asc_speed_constraint;
    }
    else {
      resolved.vertical_asc_speed_constraint = 0.0
    }

    if (msg.vertical_asc_acc_constraint !== undefined) {
      resolved.vertical_asc_acc_constraint = msg.vertical_asc_acc_constraint;
    }
    else {
      resolved.vertical_asc_acc_constraint = 0.0
    }

    if (msg.vertical_desc_speed_constraint !== undefined) {
      resolved.vertical_desc_speed_constraint = msg.vertical_desc_speed_constraint;
    }
    else {
      resolved.vertical_desc_speed_constraint = 0.0
    }

    if (msg.vertical_desc_acc_constraint !== undefined) {
      resolved.vertical_desc_acc_constraint = msg.vertical_desc_acc_constraint;
    }
    else {
      resolved.vertical_desc_acc_constraint = 0.0
    }

    if (msg.mode_mask !== undefined) {
      resolved.mode_mask = msg.mode_mask;
    }
    else {
      resolved.mode_mask = 0
    }

    return resolved;
    }
};

// Constants for message
AttitudeCommand.Constants = {
  MODE_ATTITUDE: 1,
  MODE_ATTITUDE_RATE: 2,
}

module.exports = AttitudeCommand;
