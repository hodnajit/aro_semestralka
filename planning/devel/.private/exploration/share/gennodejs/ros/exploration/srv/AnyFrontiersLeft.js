// Auto-generated. Do not edit!

// (in-package exploration.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class AnyFrontiersLeftRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AnyFrontiersLeftRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AnyFrontiersLeftRequest
    let len;
    let data = new AnyFrontiersLeftRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'exploration/AnyFrontiersLeftRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AnyFrontiersLeftRequest(null);
    return resolved;
    }
};

class AnyFrontiersLeftResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.any_frontiers_left = null;
    }
    else {
      if (initObj.hasOwnProperty('any_frontiers_left')) {
        this.any_frontiers_left = initObj.any_frontiers_left
      }
      else {
        this.any_frontiers_left = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AnyFrontiersLeftResponse
    // Serialize message field [any_frontiers_left]
    bufferOffset = _serializer.bool(obj.any_frontiers_left, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AnyFrontiersLeftResponse
    let len;
    let data = new AnyFrontiersLeftResponse(null);
    // Deserialize message field [any_frontiers_left]
    data.any_frontiers_left = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'exploration/AnyFrontiersLeftResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9c55a8e73f02e61ed1fd7718907efa64';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool any_frontiers_left
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AnyFrontiersLeftResponse(null);
    if (msg.any_frontiers_left !== undefined) {
      resolved.any_frontiers_left = msg.any_frontiers_left;
    }
    else {
      resolved.any_frontiers_left = false
    }

    return resolved;
    }
};

module.exports = {
  Request: AnyFrontiersLeftRequest,
  Response: AnyFrontiersLeftResponse,
  md5sum() { return '9c55a8e73f02e61ed1fd7718907efa64'; },
  datatype() { return 'exploration/AnyFrontiersLeft'; }
};
