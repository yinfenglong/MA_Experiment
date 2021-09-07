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

class SetLEDEffectRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.effect = null;
      this.r = null;
      this.g = null;
      this.b = null;
    }
    else {
      if (initObj.hasOwnProperty('effect')) {
        this.effect = initObj.effect
      }
      else {
        this.effect = '';
      }
      if (initObj.hasOwnProperty('r')) {
        this.r = initObj.r
      }
      else {
        this.r = 0;
      }
      if (initObj.hasOwnProperty('g')) {
        this.g = initObj.g
      }
      else {
        this.g = 0;
      }
      if (initObj.hasOwnProperty('b')) {
        this.b = initObj.b
      }
      else {
        this.b = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetLEDEffectRequest
    // Serialize message field [effect]
    bufferOffset = _serializer.string(obj.effect, buffer, bufferOffset);
    // Serialize message field [r]
    bufferOffset = _serializer.uint8(obj.r, buffer, bufferOffset);
    // Serialize message field [g]
    bufferOffset = _serializer.uint8(obj.g, buffer, bufferOffset);
    // Serialize message field [b]
    bufferOffset = _serializer.uint8(obj.b, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetLEDEffectRequest
    let len;
    let data = new SetLEDEffectRequest(null);
    // Deserialize message field [effect]
    data.effect = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [r]
    data.r = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [g]
    data.g = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [b]
    data.b = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.effect.length;
    return length + 7;
  }

  static datatype() {
    // Returns string type for a service object
    return 'itm_nonlinear_mpc/SetLEDEffectRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f671ae8175b9ebc5acde7321ebe360c1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string effect
    uint8 r
    uint8 g
    uint8 b
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetLEDEffectRequest(null);
    if (msg.effect !== undefined) {
      resolved.effect = msg.effect;
    }
    else {
      resolved.effect = ''
    }

    if (msg.r !== undefined) {
      resolved.r = msg.r;
    }
    else {
      resolved.r = 0
    }

    if (msg.g !== undefined) {
      resolved.g = msg.g;
    }
    else {
      resolved.g = 0
    }

    if (msg.b !== undefined) {
      resolved.b = msg.b;
    }
    else {
      resolved.b = 0
    }

    return resolved;
    }
};

class SetLEDEffectResponse {
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
    // Serializes a message object of type SetLEDEffectResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetLEDEffectResponse
    let len;
    let data = new SetLEDEffectResponse(null);
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
    return 'itm_nonlinear_mpc/SetLEDEffectResponse';
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
    const resolved = new SetLEDEffectResponse(null);
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
  Request: SetLEDEffectRequest,
  Response: SetLEDEffectResponse,
  md5sum() { return '044f75c927403b22bd59e8dbf871eabd'; },
  datatype() { return 'itm_nonlinear_mpc/SetLEDEffect'; }
};
