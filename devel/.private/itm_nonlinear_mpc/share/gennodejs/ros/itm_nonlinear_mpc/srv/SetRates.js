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

class SetRatesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pitch_rate = null;
      this.roll_rate = null;
      this.yaw_rate = null;
      this.thrust = null;
      this.auto_arm = null;
    }
    else {
      if (initObj.hasOwnProperty('pitch_rate')) {
        this.pitch_rate = initObj.pitch_rate
      }
      else {
        this.pitch_rate = 0.0;
      }
      if (initObj.hasOwnProperty('roll_rate')) {
        this.roll_rate = initObj.roll_rate
      }
      else {
        this.roll_rate = 0.0;
      }
      if (initObj.hasOwnProperty('yaw_rate')) {
        this.yaw_rate = initObj.yaw_rate
      }
      else {
        this.yaw_rate = 0.0;
      }
      if (initObj.hasOwnProperty('thrust')) {
        this.thrust = initObj.thrust
      }
      else {
        this.thrust = 0.0;
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
    // Serializes a message object of type SetRatesRequest
    // Serialize message field [pitch_rate]
    bufferOffset = _serializer.float32(obj.pitch_rate, buffer, bufferOffset);
    // Serialize message field [roll_rate]
    bufferOffset = _serializer.float32(obj.roll_rate, buffer, bufferOffset);
    // Serialize message field [yaw_rate]
    bufferOffset = _serializer.float32(obj.yaw_rate, buffer, bufferOffset);
    // Serialize message field [thrust]
    bufferOffset = _serializer.float32(obj.thrust, buffer, bufferOffset);
    // Serialize message field [auto_arm]
    bufferOffset = _serializer.bool(obj.auto_arm, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetRatesRequest
    let len;
    let data = new SetRatesRequest(null);
    // Deserialize message field [pitch_rate]
    data.pitch_rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [roll_rate]
    data.roll_rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw_rate]
    data.yaw_rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [thrust]
    data.thrust = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [auto_arm]
    data.auto_arm = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 17;
  }

  static datatype() {
    // Returns string type for a service object
    return 'itm_nonlinear_mpc/SetRatesRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'af3af5eb1fdd1e1bcaa59b13f3881fdb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 pitch_rate
    float32 roll_rate
    float32 yaw_rate
    float32 thrust
    bool auto_arm
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetRatesRequest(null);
    if (msg.pitch_rate !== undefined) {
      resolved.pitch_rate = msg.pitch_rate;
    }
    else {
      resolved.pitch_rate = 0.0
    }

    if (msg.roll_rate !== undefined) {
      resolved.roll_rate = msg.roll_rate;
    }
    else {
      resolved.roll_rate = 0.0
    }

    if (msg.yaw_rate !== undefined) {
      resolved.yaw_rate = msg.yaw_rate;
    }
    else {
      resolved.yaw_rate = 0.0
    }

    if (msg.thrust !== undefined) {
      resolved.thrust = msg.thrust;
    }
    else {
      resolved.thrust = 0.0
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

class SetRatesResponse {
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
    // Serializes a message object of type SetRatesResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetRatesResponse
    let len;
    let data = new SetRatesResponse(null);
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
    return 'itm_nonlinear_mpc/SetRatesResponse';
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
    const resolved = new SetRatesResponse(null);
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
  Request: SetRatesRequest,
  Response: SetRatesResponse,
  md5sum() { return 'a9cc2408dc007c6dd1f503c73d267539'; },
  datatype() { return 'itm_nonlinear_mpc/SetRates'; }
};
