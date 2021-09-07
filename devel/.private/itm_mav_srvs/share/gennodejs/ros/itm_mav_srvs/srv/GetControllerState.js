// Auto-generated. Do not edit!

// (in-package itm_mav_srvs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GetControllerStateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_name = null;
      this.command_id = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_name')) {
        this.robot_name = initObj.robot_name
      }
      else {
        this.robot_name = '';
      }
      if (initObj.hasOwnProperty('command_id')) {
        this.command_id = initObj.command_id
      }
      else {
        this.command_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetControllerStateRequest
    // Serialize message field [robot_name]
    bufferOffset = _serializer.string(obj.robot_name, buffer, bufferOffset);
    // Serialize message field [command_id]
    bufferOffset = _serializer.uint8(obj.command_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetControllerStateRequest
    let len;
    let data = new GetControllerStateRequest(null);
    // Deserialize message field [robot_name]
    data.robot_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [command_id]
    data.command_id = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.robot_name.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'itm_mav_srvs/GetControllerStateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd8c6063b046aafbed6fda5050bbf6cf4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string robot_name
    uint8 command_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetControllerStateRequest(null);
    if (msg.robot_name !== undefined) {
      resolved.robot_name = msg.robot_name;
    }
    else {
      resolved.robot_name = ''
    }

    if (msg.command_id !== undefined) {
      resolved.command_id = msg.command_id;
    }
    else {
      resolved.command_id = 0
    }

    return resolved;
    }
};

class GetControllerStateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.connected = null;
    }
    else {
      if (initObj.hasOwnProperty('connected')) {
        this.connected = initObj.connected
      }
      else {
        this.connected = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetControllerStateResponse
    // Serialize message field [connected]
    bufferOffset = _serializer.bool(obj.connected, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetControllerStateResponse
    let len;
    let data = new GetControllerStateResponse(null);
    // Deserialize message field [connected]
    data.connected = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'itm_mav_srvs/GetControllerStateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e0cdaf65159c7f3563426650fb8d3f64';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool connected
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetControllerStateResponse(null);
    if (msg.connected !== undefined) {
      resolved.connected = msg.connected;
    }
    else {
      resolved.connected = false
    }

    return resolved;
    }
};

module.exports = {
  Request: GetControllerStateRequest,
  Response: GetControllerStateResponse,
  md5sum() { return '5ded73ff38c3ac0ab83d4d33c5cfb2fd'; },
  datatype() { return 'itm_mav_srvs/GetControllerState'; }
};
