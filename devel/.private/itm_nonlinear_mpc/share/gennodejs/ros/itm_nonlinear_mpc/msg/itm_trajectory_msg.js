// Auto-generated. Do not edit!

// (in-package itm_nonlinear_mpc.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let itm_trajectory_point = require('./itm_trajectory_point.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class itm_trajectory_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.size = null;
      this.traj = null;
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('size')) {
        this.size = initObj.size
      }
      else {
        this.size = 0;
      }
      if (initObj.hasOwnProperty('traj')) {
        this.traj = initObj.traj
      }
      else {
        this.traj = [];
      }
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type itm_trajectory_msg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = _serializer.uint32(obj.size, buffer, bufferOffset);
    // Serialize message field [traj]
    // Serialize the length for message field [traj]
    bufferOffset = _serializer.uint32(obj.traj.length, buffer, bufferOffset);
    obj.traj.forEach((val) => {
      bufferOffset = itm_trajectory_point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [data]
    bufferOffset = _arraySerializer.float64(obj.data, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type itm_trajectory_msg
    let len;
    let data = new itm_trajectory_msg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [traj]
    // Deserialize array length for message field [traj]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.traj = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.traj[i] = itm_trajectory_point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [data]
    data.data = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 181 * object.traj.length;
    length += 8 * object.data.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'itm_nonlinear_mpc/itm_trajectory_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5fbf4a175c3bba98dd9f6a860e83cf2f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    uint32 size
    itm_nonlinear_mpc/itm_trajectory_point[] traj
    float64[] data
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
    MSG: itm_nonlinear_mpc/itm_trajectory_point
    float64 x
    float64 y
    float64 z
    float64 vx
    float64 vy
    float64 vz
    float64 roll
    float64 pitch
    float64 yaw
    float64[4] q
    float64[2] cube_x
    float64[2] cube_y
    float64[2] cube_z
    float64[2] cube_yaw
    bool fixed
    bool time_known
    int8 derivative
    int8 segment_index
    float64 time_stamp
    bool quaternion_given
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new itm_trajectory_msg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.size !== undefined) {
      resolved.size = msg.size;
    }
    else {
      resolved.size = 0
    }

    if (msg.traj !== undefined) {
      resolved.traj = new Array(msg.traj.length);
      for (let i = 0; i < resolved.traj.length; ++i) {
        resolved.traj[i] = itm_trajectory_point.Resolve(msg.traj[i]);
      }
    }
    else {
      resolved.traj = []
    }

    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = []
    }

    return resolved;
    }
};

module.exports = itm_trajectory_msg;
