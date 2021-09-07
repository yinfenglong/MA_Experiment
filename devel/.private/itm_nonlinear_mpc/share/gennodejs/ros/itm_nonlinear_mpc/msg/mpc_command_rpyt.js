// Auto-generated. Do not edit!

// (in-package itm_nonlinear_mpc.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class mpc_command_rpyt {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.roll_ref = null;
      this.pitch_ref = null;
      this.yaw_rate_cmd = null;
      this.thrust_ref = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('roll_ref')) {
        this.roll_ref = initObj.roll_ref
      }
      else {
        this.roll_ref = 0.0;
      }
      if (initObj.hasOwnProperty('pitch_ref')) {
        this.pitch_ref = initObj.pitch_ref
      }
      else {
        this.pitch_ref = 0.0;
      }
      if (initObj.hasOwnProperty('yaw_rate_cmd')) {
        this.yaw_rate_cmd = initObj.yaw_rate_cmd
      }
      else {
        this.yaw_rate_cmd = 0.0;
      }
      if (initObj.hasOwnProperty('thrust_ref')) {
        this.thrust_ref = initObj.thrust_ref
      }
      else {
        this.thrust_ref = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type mpc_command_rpyt
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [roll_ref]
    bufferOffset = _serializer.float64(obj.roll_ref, buffer, bufferOffset);
    // Serialize message field [pitch_ref]
    bufferOffset = _serializer.float64(obj.pitch_ref, buffer, bufferOffset);
    // Serialize message field [yaw_rate_cmd]
    bufferOffset = _serializer.float64(obj.yaw_rate_cmd, buffer, bufferOffset);
    // Serialize message field [thrust_ref]
    bufferOffset = _serializer.float64(obj.thrust_ref, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type mpc_command_rpyt
    let len;
    let data = new mpc_command_rpyt(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [roll_ref]
    data.roll_ref = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pitch_ref]
    data.pitch_ref = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw_rate_cmd]
    data.yaw_rate_cmd = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [thrust_ref]
    data.thrust_ref = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'itm_nonlinear_mpc/mpc_command_rpyt';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3a1aaed29b0fec0f986f12a3290ec8b8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    float64 roll_ref
    float64 pitch_ref
    float64 yaw_rate_cmd
    float64 thrust_ref
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new mpc_command_rpyt(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.roll_ref !== undefined) {
      resolved.roll_ref = msg.roll_ref;
    }
    else {
      resolved.roll_ref = 0.0
    }

    if (msg.pitch_ref !== undefined) {
      resolved.pitch_ref = msg.pitch_ref;
    }
    else {
      resolved.pitch_ref = 0.0
    }

    if (msg.yaw_rate_cmd !== undefined) {
      resolved.yaw_rate_cmd = msg.yaw_rate_cmd;
    }
    else {
      resolved.yaw_rate_cmd = 0.0
    }

    if (msg.thrust_ref !== undefined) {
      resolved.thrust_ref = msg.thrust_ref;
    }
    else {
      resolved.thrust_ref = 0.0
    }

    return resolved;
    }
};

module.exports = mpc_command_rpyt;
