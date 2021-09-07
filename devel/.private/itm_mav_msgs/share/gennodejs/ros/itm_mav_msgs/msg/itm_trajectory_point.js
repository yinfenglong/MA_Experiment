// Auto-generated. Do not edit!

// (in-package itm_mav_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class itm_trajectory_point {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
      this.z = null;
      this.vx = null;
      this.vy = null;
      this.vz = null;
      this.roll = null;
      this.pitch = null;
      this.yaw = null;
      this.roll_des = null;
      this.pitch_des = null;
      this.yaw_des = null;
      this.roll_rate_des = null;
      this.pitch_rate_des = null;
      this.yaw_rate_des = null;
      this.thrust_des = null;
      this.input_given = null;
      this.q = null;
      this.cube_x = null;
      this.cube_y = null;
      this.cube_z = null;
      this.cube_yaw = null;
      this.fixed = null;
      this.time_known = null;
      this.derivative = null;
      this.segment_index = null;
      this.time_stamp = null;
      this.quaternion_given = null;
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
      if (initObj.hasOwnProperty('roll')) {
        this.roll = initObj.roll
      }
      else {
        this.roll = 0.0;
      }
      if (initObj.hasOwnProperty('pitch')) {
        this.pitch = initObj.pitch
      }
      else {
        this.pitch = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('roll_des')) {
        this.roll_des = initObj.roll_des
      }
      else {
        this.roll_des = 0.0;
      }
      if (initObj.hasOwnProperty('pitch_des')) {
        this.pitch_des = initObj.pitch_des
      }
      else {
        this.pitch_des = 0.0;
      }
      if (initObj.hasOwnProperty('yaw_des')) {
        this.yaw_des = initObj.yaw_des
      }
      else {
        this.yaw_des = 0.0;
      }
      if (initObj.hasOwnProperty('roll_rate_des')) {
        this.roll_rate_des = initObj.roll_rate_des
      }
      else {
        this.roll_rate_des = 0.0;
      }
      if (initObj.hasOwnProperty('pitch_rate_des')) {
        this.pitch_rate_des = initObj.pitch_rate_des
      }
      else {
        this.pitch_rate_des = 0.0;
      }
      if (initObj.hasOwnProperty('yaw_rate_des')) {
        this.yaw_rate_des = initObj.yaw_rate_des
      }
      else {
        this.yaw_rate_des = 0.0;
      }
      if (initObj.hasOwnProperty('thrust_des')) {
        this.thrust_des = initObj.thrust_des
      }
      else {
        this.thrust_des = 0.0;
      }
      if (initObj.hasOwnProperty('input_given')) {
        this.input_given = initObj.input_given
      }
      else {
        this.input_given = false;
      }
      if (initObj.hasOwnProperty('q')) {
        this.q = initObj.q
      }
      else {
        this.q = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('cube_x')) {
        this.cube_x = initObj.cube_x
      }
      else {
        this.cube_x = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('cube_y')) {
        this.cube_y = initObj.cube_y
      }
      else {
        this.cube_y = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('cube_z')) {
        this.cube_z = initObj.cube_z
      }
      else {
        this.cube_z = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('cube_yaw')) {
        this.cube_yaw = initObj.cube_yaw
      }
      else {
        this.cube_yaw = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('fixed')) {
        this.fixed = initObj.fixed
      }
      else {
        this.fixed = false;
      }
      if (initObj.hasOwnProperty('time_known')) {
        this.time_known = initObj.time_known
      }
      else {
        this.time_known = false;
      }
      if (initObj.hasOwnProperty('derivative')) {
        this.derivative = initObj.derivative
      }
      else {
        this.derivative = 0;
      }
      if (initObj.hasOwnProperty('segment_index')) {
        this.segment_index = initObj.segment_index
      }
      else {
        this.segment_index = 0;
      }
      if (initObj.hasOwnProperty('time_stamp')) {
        this.time_stamp = initObj.time_stamp
      }
      else {
        this.time_stamp = 0.0;
      }
      if (initObj.hasOwnProperty('quaternion_given')) {
        this.quaternion_given = initObj.quaternion_given
      }
      else {
        this.quaternion_given = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type itm_trajectory_point
    // Serialize message field [x]
    bufferOffset = _serializer.float64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float64(obj.z, buffer, bufferOffset);
    // Serialize message field [vx]
    bufferOffset = _serializer.float64(obj.vx, buffer, bufferOffset);
    // Serialize message field [vy]
    bufferOffset = _serializer.float64(obj.vy, buffer, bufferOffset);
    // Serialize message field [vz]
    bufferOffset = _serializer.float64(obj.vz, buffer, bufferOffset);
    // Serialize message field [roll]
    bufferOffset = _serializer.float64(obj.roll, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.float64(obj.pitch, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float64(obj.yaw, buffer, bufferOffset);
    // Serialize message field [roll_des]
    bufferOffset = _serializer.float64(obj.roll_des, buffer, bufferOffset);
    // Serialize message field [pitch_des]
    bufferOffset = _serializer.float64(obj.pitch_des, buffer, bufferOffset);
    // Serialize message field [yaw_des]
    bufferOffset = _serializer.float64(obj.yaw_des, buffer, bufferOffset);
    // Serialize message field [roll_rate_des]
    bufferOffset = _serializer.float64(obj.roll_rate_des, buffer, bufferOffset);
    // Serialize message field [pitch_rate_des]
    bufferOffset = _serializer.float64(obj.pitch_rate_des, buffer, bufferOffset);
    // Serialize message field [yaw_rate_des]
    bufferOffset = _serializer.float64(obj.yaw_rate_des, buffer, bufferOffset);
    // Serialize message field [thrust_des]
    bufferOffset = _serializer.float64(obj.thrust_des, buffer, bufferOffset);
    // Serialize message field [input_given]
    bufferOffset = _serializer.bool(obj.input_given, buffer, bufferOffset);
    // Check that the constant length array field [q] has the right length
    if (obj.q.length !== 4) {
      throw new Error('Unable to serialize array field q - length must be 4')
    }
    // Serialize message field [q]
    bufferOffset = _arraySerializer.float64(obj.q, buffer, bufferOffset, 4);
    // Check that the constant length array field [cube_x] has the right length
    if (obj.cube_x.length !== 2) {
      throw new Error('Unable to serialize array field cube_x - length must be 2')
    }
    // Serialize message field [cube_x]
    bufferOffset = _arraySerializer.float64(obj.cube_x, buffer, bufferOffset, 2);
    // Check that the constant length array field [cube_y] has the right length
    if (obj.cube_y.length !== 2) {
      throw new Error('Unable to serialize array field cube_y - length must be 2')
    }
    // Serialize message field [cube_y]
    bufferOffset = _arraySerializer.float64(obj.cube_y, buffer, bufferOffset, 2);
    // Check that the constant length array field [cube_z] has the right length
    if (obj.cube_z.length !== 2) {
      throw new Error('Unable to serialize array field cube_z - length must be 2')
    }
    // Serialize message field [cube_z]
    bufferOffset = _arraySerializer.float64(obj.cube_z, buffer, bufferOffset, 2);
    // Check that the constant length array field [cube_yaw] has the right length
    if (obj.cube_yaw.length !== 2) {
      throw new Error('Unable to serialize array field cube_yaw - length must be 2')
    }
    // Serialize message field [cube_yaw]
    bufferOffset = _arraySerializer.float64(obj.cube_yaw, buffer, bufferOffset, 2);
    // Serialize message field [fixed]
    bufferOffset = _serializer.bool(obj.fixed, buffer, bufferOffset);
    // Serialize message field [time_known]
    bufferOffset = _serializer.bool(obj.time_known, buffer, bufferOffset);
    // Serialize message field [derivative]
    bufferOffset = _serializer.int8(obj.derivative, buffer, bufferOffset);
    // Serialize message field [segment_index]
    bufferOffset = _serializer.int8(obj.segment_index, buffer, bufferOffset);
    // Serialize message field [time_stamp]
    bufferOffset = _serializer.float64(obj.time_stamp, buffer, bufferOffset);
    // Serialize message field [quaternion_given]
    bufferOffset = _serializer.bool(obj.quaternion_given, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type itm_trajectory_point
    let len;
    let data = new itm_trajectory_point(null);
    // Deserialize message field [x]
    data.x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vx]
    data.vx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vy]
    data.vy = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vz]
    data.vz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [roll]
    data.roll = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [roll_des]
    data.roll_des = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pitch_des]
    data.pitch_des = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw_des]
    data.yaw_des = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [roll_rate_des]
    data.roll_rate_des = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pitch_rate_des]
    data.pitch_rate_des = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw_rate_des]
    data.yaw_rate_des = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [thrust_des]
    data.thrust_des = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [input_given]
    data.input_given = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [q]
    data.q = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    // Deserialize message field [cube_x]
    data.cube_x = _arrayDeserializer.float64(buffer, bufferOffset, 2)
    // Deserialize message field [cube_y]
    data.cube_y = _arrayDeserializer.float64(buffer, bufferOffset, 2)
    // Deserialize message field [cube_z]
    data.cube_z = _arrayDeserializer.float64(buffer, bufferOffset, 2)
    // Deserialize message field [cube_yaw]
    data.cube_yaw = _arrayDeserializer.float64(buffer, bufferOffset, 2)
    // Deserialize message field [fixed]
    data.fixed = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [time_known]
    data.time_known = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [derivative]
    data.derivative = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [segment_index]
    data.segment_index = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [time_stamp]
    data.time_stamp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [quaternion_given]
    data.quaternion_given = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 238;
  }

  static datatype() {
    // Returns string type for a message object
    return 'itm_mav_msgs/itm_trajectory_point';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '06c29ab4c48d9e597505a69f6a28b27a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 x
    float64 y
    float64 z
    float64 vx
    float64 vy
    float64 vz
    float64 roll
    float64 pitch
    float64 yaw
    float64 roll_des
    float64 pitch_des
    float64 yaw_des
    float64 roll_rate_des
    float64 pitch_rate_des
    float64 yaw_rate_des
    float64 thrust_des
    bool input_given
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
    const resolved = new itm_trajectory_point(null);
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

    if (msg.roll !== undefined) {
      resolved.roll = msg.roll;
    }
    else {
      resolved.roll = 0.0
    }

    if (msg.pitch !== undefined) {
      resolved.pitch = msg.pitch;
    }
    else {
      resolved.pitch = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.roll_des !== undefined) {
      resolved.roll_des = msg.roll_des;
    }
    else {
      resolved.roll_des = 0.0
    }

    if (msg.pitch_des !== undefined) {
      resolved.pitch_des = msg.pitch_des;
    }
    else {
      resolved.pitch_des = 0.0
    }

    if (msg.yaw_des !== undefined) {
      resolved.yaw_des = msg.yaw_des;
    }
    else {
      resolved.yaw_des = 0.0
    }

    if (msg.roll_rate_des !== undefined) {
      resolved.roll_rate_des = msg.roll_rate_des;
    }
    else {
      resolved.roll_rate_des = 0.0
    }

    if (msg.pitch_rate_des !== undefined) {
      resolved.pitch_rate_des = msg.pitch_rate_des;
    }
    else {
      resolved.pitch_rate_des = 0.0
    }

    if (msg.yaw_rate_des !== undefined) {
      resolved.yaw_rate_des = msg.yaw_rate_des;
    }
    else {
      resolved.yaw_rate_des = 0.0
    }

    if (msg.thrust_des !== undefined) {
      resolved.thrust_des = msg.thrust_des;
    }
    else {
      resolved.thrust_des = 0.0
    }

    if (msg.input_given !== undefined) {
      resolved.input_given = msg.input_given;
    }
    else {
      resolved.input_given = false
    }

    if (msg.q !== undefined) {
      resolved.q = msg.q;
    }
    else {
      resolved.q = new Array(4).fill(0)
    }

    if (msg.cube_x !== undefined) {
      resolved.cube_x = msg.cube_x;
    }
    else {
      resolved.cube_x = new Array(2).fill(0)
    }

    if (msg.cube_y !== undefined) {
      resolved.cube_y = msg.cube_y;
    }
    else {
      resolved.cube_y = new Array(2).fill(0)
    }

    if (msg.cube_z !== undefined) {
      resolved.cube_z = msg.cube_z;
    }
    else {
      resolved.cube_z = new Array(2).fill(0)
    }

    if (msg.cube_yaw !== undefined) {
      resolved.cube_yaw = msg.cube_yaw;
    }
    else {
      resolved.cube_yaw = new Array(2).fill(0)
    }

    if (msg.fixed !== undefined) {
      resolved.fixed = msg.fixed;
    }
    else {
      resolved.fixed = false
    }

    if (msg.time_known !== undefined) {
      resolved.time_known = msg.time_known;
    }
    else {
      resolved.time_known = false
    }

    if (msg.derivative !== undefined) {
      resolved.derivative = msg.derivative;
    }
    else {
      resolved.derivative = 0
    }

    if (msg.segment_index !== undefined) {
      resolved.segment_index = msg.segment_index;
    }
    else {
      resolved.segment_index = 0
    }

    if (msg.time_stamp !== undefined) {
      resolved.time_stamp = msg.time_stamp;
    }
    else {
      resolved.time_stamp = 0.0
    }

    if (msg.quaternion_given !== undefined) {
      resolved.quaternion_given = msg.quaternion_given;
    }
    else {
      resolved.quaternion_given = false
    }

    return resolved;
    }
};

module.exports = itm_trajectory_point;
