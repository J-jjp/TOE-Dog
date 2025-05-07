// Auto-generated. Do not edit!

// (in-package unitree_a1.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MotorData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.legid = null;
      this.motorid = null;
      this.mode = null;
      this.tau = null;
      this.vel = null;
      this.acc = null;
      this.pos = null;
      this.temp = null;
      this.error = null;
    }
    else {
      if (initObj.hasOwnProperty('legid')) {
        this.legid = initObj.legid
      }
      else {
        this.legid = [];
      }
      if (initObj.hasOwnProperty('motorid')) {
        this.motorid = initObj.motorid
      }
      else {
        this.motorid = [];
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = [];
      }
      if (initObj.hasOwnProperty('tau')) {
        this.tau = initObj.tau
      }
      else {
        this.tau = [];
      }
      if (initObj.hasOwnProperty('vel')) {
        this.vel = initObj.vel
      }
      else {
        this.vel = [];
      }
      if (initObj.hasOwnProperty('acc')) {
        this.acc = initObj.acc
      }
      else {
        this.acc = [];
      }
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = [];
      }
      if (initObj.hasOwnProperty('temp')) {
        this.temp = initObj.temp
      }
      else {
        this.temp = [];
      }
      if (initObj.hasOwnProperty('error')) {
        this.error = initObj.error
      }
      else {
        this.error = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorData
    // Serialize message field [legid]
    bufferOffset = _arraySerializer.uint8(obj.legid, buffer, bufferOffset, null);
    // Serialize message field [motorid]
    bufferOffset = _arraySerializer.uint8(obj.motorid, buffer, bufferOffset, null);
    // Serialize message field [mode]
    bufferOffset = _arraySerializer.uint8(obj.mode, buffer, bufferOffset, null);
    // Serialize message field [tau]
    bufferOffset = _arraySerializer.float32(obj.tau, buffer, bufferOffset, null);
    // Serialize message field [vel]
    bufferOffset = _arraySerializer.float32(obj.vel, buffer, bufferOffset, null);
    // Serialize message field [acc]
    bufferOffset = _arraySerializer.float32(obj.acc, buffer, bufferOffset, null);
    // Serialize message field [pos]
    bufferOffset = _arraySerializer.float32(obj.pos, buffer, bufferOffset, null);
    // Serialize message field [temp]
    bufferOffset = _arraySerializer.int8(obj.temp, buffer, bufferOffset, null);
    // Serialize message field [error]
    bufferOffset = _arraySerializer.int8(obj.error, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorData
    let len;
    let data = new MotorData(null);
    // Deserialize message field [legid]
    data.legid = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [motorid]
    data.motorid = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [mode]
    data.mode = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [tau]
    data.tau = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [vel]
    data.vel = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [acc]
    data.acc = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [pos]
    data.pos = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [temp]
    data.temp = _arrayDeserializer.int8(buffer, bufferOffset, null)
    // Deserialize message field [error]
    data.error = _arrayDeserializer.int8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.legid.length;
    length += object.motorid.length;
    length += object.mode.length;
    length += 4 * object.tau.length;
    length += 4 * object.vel.length;
    length += 4 * object.acc.length;
    length += 4 * object.pos.length;
    length += object.temp.length;
    length += object.error.length;
    return length + 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'unitree_a1/MotorData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3b3e3cad6f8f2ba7b0cdd1200569952b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8[] legid
    uint8[] motorid
    uint8[] mode
    float32[] tau
    float32[] vel
    float32[] acc
    float32[] pos
    int8[] temp
    int8[] error
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorData(null);
    if (msg.legid !== undefined) {
      resolved.legid = msg.legid;
    }
    else {
      resolved.legid = []
    }

    if (msg.motorid !== undefined) {
      resolved.motorid = msg.motorid;
    }
    else {
      resolved.motorid = []
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = []
    }

    if (msg.tau !== undefined) {
      resolved.tau = msg.tau;
    }
    else {
      resolved.tau = []
    }

    if (msg.vel !== undefined) {
      resolved.vel = msg.vel;
    }
    else {
      resolved.vel = []
    }

    if (msg.acc !== undefined) {
      resolved.acc = msg.acc;
    }
    else {
      resolved.acc = []
    }

    if (msg.pos !== undefined) {
      resolved.pos = msg.pos;
    }
    else {
      resolved.pos = []
    }

    if (msg.temp !== undefined) {
      resolved.temp = msg.temp;
    }
    else {
      resolved.temp = []
    }

    if (msg.error !== undefined) {
      resolved.error = msg.error;
    }
    else {
      resolved.error = []
    }

    return resolved;
    }
};

module.exports = MotorData;
