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

class Button {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.move = null;
      this.rotation_right = null;
      this.rotation_left = null;
    }
    else {
      if (initObj.hasOwnProperty('move')) {
        this.move = initObj.move
      }
      else {
        this.move = 0;
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
    // Serializes a message object of type Button
    // Serialize message field [move]
    bufferOffset = _serializer.int32(obj.move, buffer, bufferOffset);
    // Serialize message field [rotation_right]
    bufferOffset = _serializer.int32(obj.rotation_right, buffer, bufferOffset);
    // Serialize message field [rotation_left]
    bufferOffset = _serializer.int32(obj.rotation_left, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Button
    let len;
    let data = new Button(null);
    // Deserialize message field [move]
    data.move = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [rotation_right]
    data.rotation_right = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [rotation_left]
    data.rotation_left = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dozap_second/Button';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'df37f56eb5634ffffccd37fb25f1f170';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 move
    int32 rotation_right
    int32 rotation_left
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Button(null);
    if (msg.move !== undefined) {
      resolved.move = msg.move;
    }
    else {
      resolved.move = 0
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

module.exports = Button;
