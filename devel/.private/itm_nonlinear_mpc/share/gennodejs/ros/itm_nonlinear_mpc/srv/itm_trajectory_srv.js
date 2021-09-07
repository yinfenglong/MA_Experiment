// Auto-generated. Do not edit!

// (in-package itm_nonlinear_mpc.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let itm_trajectory_point = require('../msg/itm_trajectory_point.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class itm_trajectory_srvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.index = null;
      this.traj = null;
    }
    else {
      if (initObj.hasOwnProperty('index')) {
        this.index = initObj.index
      }
      else {
        this.index = 0;
      }
      if (initObj.hasOwnProperty('traj')) {
        this.traj = initObj.traj
      }
      else {
        this.traj = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type itm_trajectory_srvRequest
    // Serialize message field [index]
    bufferOffset = _serializer.uint32(obj.index, buffer, bufferOffset);
    // Serialize message field [traj]
    // Serialize the length for message field [traj]
    bufferOffset = _serializer.uint32(obj.traj.length, buffer, bufferOffset);
    obj.traj.forEach((val) => {
      bufferOffset = itm_trajectory_point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type itm_trajectory_srvRequest
    let len;
    let data = new itm_trajectory_srvRequest(null);
    // Deserialize message field [index]
    data.index = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [traj]
    // Deserialize array length for message field [traj]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.traj = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.traj[i] = itm_trajectory_point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 181 * object.traj.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'itm_nonlinear_mpc/itm_trajectory_srvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8041461697ac94a6fda5d3cf87ef5f2f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 index
    itm_nonlinear_mpc/itm_trajectory_point[] traj
    
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
    const resolved = new itm_trajectory_srvRequest(null);
    if (msg.index !== undefined) {
      resolved.index = msg.index;
    }
    else {
      resolved.index = 0
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

    return resolved;
    }
};

class itm_trajectory_srvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type itm_trajectory_srvResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type itm_trajectory_srvResponse
    let len;
    let data = new itm_trajectory_srvResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'itm_nonlinear_mpc/itm_trajectory_srvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new itm_trajectory_srvResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: itm_trajectory_srvRequest,
  Response: itm_trajectory_srvResponse,
  md5sum() { return '26ff1de4ce8cbafd1f9f9a97392ec44f'; },
  datatype() { return 'itm_nonlinear_mpc/itm_trajectory_srv'; }
};
