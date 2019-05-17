// Auto-generated. Do not edit!

// (in-package dozap_second.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Main {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.motor_right = null;
      this.motor_left = null;
      this.rotation_a_right = null;
      this.rotation_a_left = null;
      this.rotation_b_right = null;
      this.rotation_b_left = null;
    }
    else {
      if (initObj.hasOwnProperty('motor_right')) {
        this.motor_right = initObj.motor_right
      }
      else {
        this.motor_right = 0;
      }
      if (initObj.hasOwnProperty('motor_left')) {
        this.motor_left = initObj.motor_left
      }
      else {
        this.motor_left = 0;
      }
      if (initObj.hasOwnProperty('rotation_a_right')) {
        this.rotation_a_right = initObj.rotation_a_right
      }
      else {
        this.rotation_a_right = 0;
      }
      if (initObj.hasOwnProperty('rotation_a_left')) {
        this.rotation_a_left = initObj.rotation_a_left
      }
      else {
        this.rotation_a_left = 0;
      }
      if (initObj.hasOwnProperty('rotation_b_right')) {
        this.rotation_b_right = initObj.rotation_b_right
      }
      else {
        this.rotation_b_right = 0;
      }
      if (initObj.hasOwnProperty('rotation_b_left')) {
        this.rotation_b_left = initObj.rotation_b_left
      }
      else {
        this.rotation_b_left = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Main
    // Serialize message field [motor_right]
    bufferOffset = _serializer.int32(obj.motor_right, buffer, bufferOffset);
    // Serialize message field [motor_left]
    bufferOffset = _serializer.int32(obj.motor_left, buffer, bufferOffset);
    // Serialize message field [rotation_a_right]
    bufferOffset = _serializer.int32(obj.rotation_a_right, buffer, bufferOffset);
    // Serialize message field [rotation_a_left]
    bufferOffset = _serializer.int32(obj.rotation_a_left, buffer, bufferOffset);
    // Serialize message field [rotation_b_right]
    bufferOffset = _serializer.int32(obj.rotation_b_right, buffer, bufferOffset);
    // Serialize message field [rotation_b_left]
    bufferOffset = _serializer.int32(obj.rotation_b_left, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Main
    let len;
    let data = new Main(null);
    // Deserialize message field [motor_right]
    data.motor_right = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [motor_left]
    data.motor_left = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [rotation_a_right]
    data.rotation_a_right = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [rotation_a_left]
    data.rotation_a_left = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [rotation_b_right]
    data.rotation_b_right = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [rotation_b_left]
    data.rotation_b_left = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dozap_second/Main';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '83aace50d71246340a8e7cc2d2789279';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 motor_right
    int32 motor_left
    int32 rotation_a_right
    int32 rotation_a_left
    int32 rotation_b_right
    int32 rotation_b_left
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Main(null);
    if (msg.motor_right !== undefined) {
      resolved.motor_right = msg.motor_right;
    }
    else {
      resolved.motor_right = 0
    }

    if (msg.motor_left !== undefined) {
      resolved.motor_left = msg.motor_left;
    }
    else {
      resolved.motor_left = 0
    }

    if (msg.rotation_a_right !== undefined) {
      resolved.rotation_a_right = msg.rotation_a_right;
    }
    else {
      resolved.rotation_a_right = 0
    }

    if (msg.rotation_a_left !== undefined) {
      resolved.rotation_a_left = msg.rotation_a_left;
    }
    else {
      resolved.rotation_a_left = 0
    }

    if (msg.rotation_b_right !== undefined) {
      resolved.rotation_b_right = msg.rotation_b_right;
    }
    else {
      resolved.rotation_b_right = 0
    }

    if (msg.rotation_b_left !== undefined) {
      resolved.rotation_b_left = msg.rotation_b_left;
    }
    else {
      resolved.rotation_b_left = 0
    }

    return resolved;
    }
};

module.exports = Main;
