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

class GetTelemetryRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.frame_id = null;
    }
    else {
      if (initObj.hasOwnProperty('frame_id')) {
        this.frame_id = initObj.frame_id
      }
      else {
        this.frame_id = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetTelemetryRequest
    // Serialize message field [frame_id]
    bufferOffset = _serializer.string(obj.frame_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetTelemetryRequest
    let len;
    let data = new GetTelemetryRequest(null);
    // Deserialize message field [frame_id]
    data.frame_id = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.frame_id.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'itm_nonlinear_mpc/GetTelemetryRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '371cb926969bb965addb80f752c7ca31';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetTelemetryRequest(null);
    if (msg.frame_id !== undefined) {
      resolved.frame_id = msg.frame_id;
    }
    else {
      resolved.frame_id = ''
    }

    return resolved;
    }
};

class GetTelemetryResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.frame_id = null;
      this.connected = null;
      this.armed = null;
      this.mode = null;
      this.x = null;
      this.y = null;
      this.z = null;
      this.lat = null;
      this.lon = null;
      this.alt = null;
      this.vx = null;
      this.vy = null;
      this.vz = null;
      this.pitch = null;
      this.roll = null;
      this.yaw = null;
      this.pitch_rate = null;
      this.roll_rate = null;
      this.yaw_rate = null;
      this.voltage = null;
      this.cell_voltage = null;
    }
    else {
      if (initObj.hasOwnProperty('frame_id')) {
        this.frame_id = initObj.frame_id
      }
      else {
        this.frame_id = '';
      }
      if (initObj.hasOwnProperty('connected')) {
        this.connected = initObj.connected
      }
      else {
        this.connected = false;
      }
      if (initObj.hasOwnProperty('armed')) {
        this.armed = initObj.armed
      }
      else {
        this.armed = false;
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = '';
      }
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
      if (initObj.hasOwnProperty('lat')) {
        this.lat = initObj.lat
      }
      else {
        this.lat = 0.0;
      }
      if (initObj.hasOwnProperty('lon')) {
        this.lon = initObj.lon
      }
      else {
        this.lon = 0.0;
      }
      if (initObj.hasOwnProperty('alt')) {
        this.alt = initObj.alt
      }
      else {
        this.alt = 0.0;
      }
      if (initObj.hasOwnProperty('vx')) {
        this.vx = initObj.vx
      }
      else {
        this.vx = 0.0;
      }
      if (initObj.hasOwnProperty('vy')) {
        this.vy = initObj.vy
      }
      else {
        this.vy = 0.0;
      }
      if (initObj.hasOwnProperty('vz')) {
        this.vz = initObj.vz
      }
      else {
        this.vz = 0.0;
      }
      if (initObj.hasOwnProperty('pitch')) {
        this.pitch = initObj.pitch
      }
      else {
        this.pitch = 0.0;
      }
      if (initObj.hasOwnProperty('roll')) {
        this.roll = initObj.roll
      }
      else {
        this.roll = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
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
      if (initObj.hasOwnProperty('voltage')) {
        this.voltage = initObj.voltage
      }
      else {
        this.voltage = 0.0;
      }
      if (initObj.hasOwnProperty('cell_voltage')) {
        this.cell_voltage = initObj.cell_voltage
      }
      else {
        this.cell_voltage = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetTelemetryResponse
    // Serialize message field [frame_id]
    bufferOffset = _serializer.string(obj.frame_id, buffer, bufferOffset);
    // Serialize message field [connected]
    bufferOffset = _serializer.bool(obj.connected, buffer, bufferOffset);
    // Serialize message field [armed]
    bufferOffset = _serializer.bool(obj.armed, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.string(obj.mode, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float32(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float32(obj.z, buffer, bufferOffset);
    // Serialize message field [lat]
    bufferOffset = _serializer.float64(obj.lat, buffer, bufferOffset);
    // Serialize message field [lon]
    bufferOffset = _serializer.float64(obj.lon, buffer, bufferOffset);
    // Serialize message field [alt]
    bufferOffset = _serializer.float32(obj.alt, buffer, bufferOffset);
    // Serialize message field [vx]
    bufferOffset = _serializer.float32(obj.vx, buffer, bufferOffset);
    // Serialize message field [vy]
    bufferOffset = _serializer.float32(obj.vy, buffer, bufferOffset);
    // Serialize message field [vz]
    bufferOffset = _serializer.float32(obj.vz, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.float32(obj.pitch, buffer, bufferOffset);
    // Serialize message field [roll]
    bufferOffset = _serializer.float32(obj.roll, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float32(obj.yaw, buffer, bufferOffset);
    // Serialize message field [pitch_rate]
    bufferOffset = _serializer.float32(obj.pitch_rate, buffer, bufferOffset);
    // Serialize message field [roll_rate]
    bufferOffset = _serializer.float32(obj.roll_rate, buffer, bufferOffset);
    // Serialize message field [yaw_rate]
    bufferOffset = _serializer.float32(obj.yaw_rate, buffer, bufferOffset);
    // Serialize message field [voltage]
    bufferOffset = _serializer.float32(obj.voltage, buffer, bufferOffset);
    // Serialize message field [cell_voltage]
    bufferOffset = _serializer.float32(obj.cell_voltage, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetTelemetryResponse
    let len;
    let data = new GetTelemetryResponse(null);
    // Deserialize message field [frame_id]
    data.frame_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [connected]
    data.connected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [armed]
    data.armed = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lat]
    data.lat = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [lon]
    data.lon = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [alt]
    data.alt = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vx]
    data.vx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vy]
    data.vy = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vz]
    data.vz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [roll]
    data.roll = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch_rate]
    data.pitch_rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [roll_rate]
    data.roll_rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw_rate]
    data.yaw_rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [voltage]
    data.voltage = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cell_voltage]
    data.cell_voltage = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.frame_id.length;
    length += object.mode.length;
    return length + 86;
  }

  static datatype() {
    // Returns string type for a service object
    return 'itm_nonlinear_mpc/GetTelemetryResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '34b7c5985f35b7ea8dc5f87c88ce6c93';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string frame_id
    bool connected
    bool armed
    string mode
    float32 x
    float32 y
    float32 z
    float64 lat
    float64 lon
    float32 alt
    float32 vx
    float32 vy
    float32 vz
    float32 pitch
    float32 roll
    float32 yaw
    float32 pitch_rate
    float32 roll_rate
    float32 yaw_rate
    float32 voltage
    float32 cell_voltage
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetTelemetryResponse(null);
    if (msg.frame_id !== undefined) {
      resolved.frame_id = msg.frame_id;
    }
    else {
      resolved.frame_id = ''
    }

    if (msg.connected !== undefined) {
      resolved.connected = msg.connected;
    }
    else {
      resolved.connected = false
    }

    if (msg.armed !== undefined) {
      resolved.armed = msg.armed;
    }
    else {
      resolved.armed = false
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = ''
    }

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

    if (msg.lat !== undefined) {
      resolved.lat = msg.lat;
    }
    else {
      resolved.lat = 0.0
    }

    if (msg.lon !== undefined) {
      resolved.lon = msg.lon;
    }
    else {
      resolved.lon = 0.0
    }

    if (msg.alt !== undefined) {
      resolved.alt = msg.alt;
    }
    else {
      resolved.alt = 0.0
    }

    if (msg.vx !== undefined) {
      resolved.vx = msg.vx;
    }
    else {
      resolved.vx = 0.0
    }

    if (msg.vy !== undefined) {
      resolved.vy = msg.vy;
    }
    else {
      resolved.vy = 0.0
    }

    if (msg.vz !== undefined) {
      resolved.vz = msg.vz;
    }
    else {
      resolved.vz = 0.0
    }

    if (msg.pitch !== undefined) {
      resolved.pitch = msg.pitch;
    }
    else {
      resolved.pitch = 0.0
    }

    if (msg.roll !== undefined) {
      resolved.roll = msg.roll;
    }
    else {
      resolved.roll = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

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

    if (msg.voltage !== undefined) {
      resolved.voltage = msg.voltage;
    }
    else {
      resolved.voltage = 0.0
    }

    if (msg.cell_voltage !== undefined) {
      resolved.cell_voltage = msg.cell_voltage;
    }
    else {
      resolved.cell_voltage = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: GetTelemetryRequest,
  Response: GetTelemetryResponse,
  md5sum() { return '3f1c3a9f95e1194fe8ebc671ce98ef91'; },
  datatype() { return 'itm_nonlinear_mpc/GetTelemetry'; }
};
