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

class SetMission {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.command_idx = null;
      this.mission_mode = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('command_idx')) {
        this.command_idx = initObj.command_idx
      }
      else {
        this.command_idx = 0;
      }
      if (initObj.hasOwnProperty('mission_mode')) {
        this.mission_mode = initObj.mission_mode
      }
      else {
        this.mission_mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetMission
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [command_idx]
    bufferOffset = _serializer.uint32(obj.command_idx, buffer, bufferOffset);
    // Serialize message field [mission_mode]
    bufferOffset = _serializer.uint8(obj.mission_mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetMission
    let len;
    let data = new SetMission(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [command_idx]
    data.command_idx = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [mission_mode]
    data.mission_mode = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'itm_nonlinear_mpc/SetMission';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1631998827fd12d678dc74adb693f0b9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    uint32 command_idx
    uint8 mission_mode
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
    const resolved = new SetMission(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.command_idx !== undefined) {
      resolved.command_idx = msg.command_idx;
    }
    else {
      resolved.command_idx = 0
    }

    if (msg.mission_mode !== undefined) {
      resolved.mission_mode = msg.mission_mode;
    }
    else {
      resolved.mission_mode = 0
    }

    return resolved;
    }
};

module.exports = SetMission;
