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

class MotorCmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.legid = null;
      this.motorid = null;
      this.mode = null;
      this.tau = null;
      this.vel = null;
      this.pos = null;
      this.kp = null;
      this.kd = null;
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
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = [];
      }
      if (initObj.hasOwnProperty('kp')) {
        this.kp = initObj.kp
      }
      else {
        this.kp = [];
      }
      if (initObj.hasOwnProperty('kd')) {
        this.kd = initObj.kd
      }
      else {
        this.kd = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorCmd
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
    // Serialize message field [pos]
    bufferOffset = _arraySerializer.float32(obj.pos, buffer, bufferOffset, null);
    // Serialize message field [kp]
    bufferOffset = _arraySerializer.float32(obj.kp, buffer, bufferOffset, null);
    // Serialize message field [kd]
    bufferOffset = _arraySerializer.float32(obj.kd, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorCmd
    let len;
    let data = new MotorCmd(null);
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
    // Deserialize message field [pos]
    data.pos = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [kp]
    data.kp = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [kd]
    data.kd = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.legid.length;
    length += object.motorid.length;
    length += object.mode.length;
    length += 4 * object.tau.length;
    length += 4 * object.vel.length;
    length += 4 * object.pos.length;
    length += 4 * object.kp.length;
    length += 4 * object.kd.length;
    return length + 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'unitree_a1/MotorCmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8cda35a8825431ce1f5b8477f459672b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8[] legid
    uint8[] motorid
    uint8[] mode
    float32[] tau
    float32[] vel
    float32[] pos
    float32[] kp
    float32[] kd
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorCmd(null);
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

    if (msg.pos !== undefined) {
      resolved.pos = msg.pos;
    }
    else {
      resolved.pos = []
    }

    if (msg.kp !== undefined) {
      resolved.kp = msg.kp;
    }
    else {
      resolved.kp = []
    }

    if (msg.kd !== undefined) {
      resolved.kd = msg.kd;
    }
    else {
      resolved.kd = []
    }

    return resolved;
    }
};

module.exports = MotorCmd;
