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
      this.rotation_right = null;
      this.rotation_left = null;
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
      if (initObj.hasOwnProperty('rotation_right')) {
        this.rotation_right = initObj.rotation_right
      }
      else {
        this.rotation_right = 0;
      }
      if (initObj.hasOwnProperty('rotation_left')) {
        this.rotation_left = initObj.rotation_left
      }
      else {
        this.rotation_left = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Main
    // Serialize message field [motor_right]
    bufferOffset = _serializer.int32(obj.motor_right, buffer, bufferOffset);
    // Serialize message field [motor_left]
    bufferOffset = _serializer.int32(obj.motor_left, buffer, bufferOffset);
    // Serialize message field [rotation_right]
    bufferOffset = _serializer.int32(obj.rotation_right, buffer, bufferOffset);
    // Serialize message field [rotation_left]
    bufferOffset = _serializer.int32(obj.rotation_left, buffer, bufferOffset);
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
    // Deserialize message field [rotation_right]
    data.rotation_right = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [rotation_left]
    data.rotation_left = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dozap_second/Main';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7a83861f39451721eaac1e365039256f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 motor_right
    int32 motor_left
    int32 rotation_right
    int32 rotation_left
    
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

    if (msg.rotation_right !== undefined) {
      resolved.rotation_right = msg.rotation_right;
    }
    else {
      resolved.rotation_right = 0
    }

    if (msg.rotation_left !== undefined) {
      resolved.rotation_left = msg.rotation_left;
    }
    else {
      resolved.rotation_left = 0
    }

    return resolved;
    }
};

module.exports = Main;
