// Auto-generated. Do not edit!

// (in-package damiao_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class DmCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_names = null;
      this.pos = null;
      this.vel = null;
      this.kp = null;
      this.kd = null;
      this.tau = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_names')) {
        this.joint_names = initObj.joint_names
      }
      else {
        this.joint_names = [];
      }
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = [];
      }
      if (initObj.hasOwnProperty('vel')) {
        this.vel = initObj.vel
      }
      else {
        this.vel = [];
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
      if (initObj.hasOwnProperty('tau')) {
        this.tau = initObj.tau
      }
      else {
        this.tau = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DmCommand
    // Serialize message field [joint_names]
    bufferOffset = _arraySerializer.string(obj.joint_names, buffer, bufferOffset, null);
    // Serialize message field [pos]
    bufferOffset = _arraySerializer.float64(obj.pos, buffer, bufferOffset, null);
    // Serialize message field [vel]
    bufferOffset = _arraySerializer.float64(obj.vel, buffer, bufferOffset, null);
    // Serialize message field [kp]
    bufferOffset = _arraySerializer.float64(obj.kp, buffer, bufferOffset, null);
    // Serialize message field [kd]
    bufferOffset = _arraySerializer.float64(obj.kd, buffer, bufferOffset, null);
    // Serialize message field [tau]
    bufferOffset = _arraySerializer.float64(obj.tau, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DmCommand
    let len;
    let data = new DmCommand(null);
    // Deserialize message field [joint_names]
    data.joint_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [pos]
    data.pos = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [vel]
    data.vel = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [kp]
    data.kp = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [kd]
    data.kd = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [tau]
    data.tau = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.joint_names.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += 8 * object.pos.length;
    length += 8 * object.vel.length;
    length += 8 * object.kp.length;
    length += 8 * object.kd.length;
    length += 8 * object.tau.length;
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'damiao_msgs/DmCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f2b620aeb6044700d6c7585f10679d8c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] joint_names
    float64[] pos
    float64[] vel
    float64[] kp
    float64[] kd
    float64[] tau
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DmCommand(null);
    if (msg.joint_names !== undefined) {
      resolved.joint_names = msg.joint_names;
    }
    else {
      resolved.joint_names = []
    }

    if (msg.pos !== undefined) {
      resolved.pos = msg.pos;
    }
    else {
      resolved.pos = []
    }

    if (msg.vel !== undefined) {
      resolved.vel = msg.vel;
    }
    else {
      resolved.vel = []
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

    if (msg.tau !== undefined) {
      resolved.tau = msg.tau;
    }
    else {
      resolved.tau = []
    }

    return resolved;
    }
};

module.exports = DmCommand;
