// Auto-generated. Do not edit!

// (in-package itm_nonlinear_mpc.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class NavigateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
      this.z = null;
      this.yaw = null;
      this.yaw_rate = null;
      this.speed = null;
      this.frame_id = null;
      this.auto_arm = null;
    }
    else {
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('yaw_rate')) {
        this.yaw_rate = initObj.yaw_rate
      }
      else {
        this.yaw_rate = 0.0;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0.0;
      }
      if (initObj.hasOwnProperty('frame_id')) {
        this.frame_id = initObj.frame_id
      }
      else {
        this.frame_id = '';
      }
      if (initObj.hasOwnProperty('auto_arm')) {
        this.auto_arm = initObj.auto_arm
      }
      else {
        this.auto_arm = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NavigateRequest
    // Serialize message field [x]
    bufferOffset = _serializer.float32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float32(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float32(obj.z, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float32(obj.yaw, buffer, bufferOffset);
    // Serialize message field [yaw_rate]
    bufferOffset = _serializer.float32(obj.yaw_rate, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.float32(obj.speed, buffer, bufferOffset);
    // Serialize message field [frame_id]
    bufferOffset = _serializer.string(obj.frame_id, buffer, bufferOffset);
    // Serialize message field [auto_arm]
    bufferOffset = _serializer.bool(obj.auto_arm, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NavigateRequest
    let len;
    let data = new NavigateRequest(null);
    // Deserialize message field [x]
    data.x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw_rate]
    data.yaw_rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [frame_id]
    data.frame_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [auto_arm]
    data.auto_arm = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.frame_id.length;
    return length + 29;
  }

  static datatype() {
    // Returns string type for a service object
    return 'itm_nonlinear_mpc/NavigateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7695bfffababf12319f0958d40f3d0d8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 x
    float32 y
    float32 z
    float32 yaw
    float32 yaw_rate
    float32 speed
    string frame_id
    bool auto_arm
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NavigateRequest(null);
    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.yaw_rate !== undefined) {
      resolved.yaw_rate = msg.yaw_rate;
    }
    else {
      resolved.yaw_rate = 0.0
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0.0
    }

    if (msg.frame_id !== undefined) {
      resolved.frame_id = msg.frame_id;
    }
    else {
      resolved.frame_id = ''
    }

    if (msg.auto_arm !== undefined) {
      resolved.auto_arm = msg.auto_arm;
    }
    else {
      resolved.auto_arm = false
    }

    return resolved;
    }
};

class NavigateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NavigateResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NavigateResponse
    let len;
    let data = new NavigateResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.message.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'itm_nonlinear_mpc/NavigateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '937c9679a518e3a18d831e57125ea522';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    string message
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NavigateResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: NavigateRequest,
  Response: NavigateResponse,
  md5sum() { return 'd27cf8a87020c28b3d14e06881a86c21'; },
  datatype() { return 'itm_nonlinear_mpc/Navigate'; }
};
