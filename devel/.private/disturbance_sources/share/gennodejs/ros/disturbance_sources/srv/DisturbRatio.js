// Auto-generated. Do not edit!

// (in-package disturbance_sources.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class DisturbRatioRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pos_x = null;
      this.pos_y = null;
      this.pos_z = null;
    }
    else {
      if (initObj.hasOwnProperty('pos_x')) {
        this.pos_x = initObj.pos_x
      }
      else {
        this.pos_x = 0.0;
      }
      if (initObj.hasOwnProperty('pos_y')) {
        this.pos_y = initObj.pos_y
      }
      else {
        this.pos_y = 0.0;
      }
      if (initObj.hasOwnProperty('pos_z')) {
        this.pos_z = initObj.pos_z
      }
      else {
        this.pos_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DisturbRatioRequest
    // Serialize message field [pos_x]
    bufferOffset = _serializer.float64(obj.pos_x, buffer, bufferOffset);
    // Serialize message field [pos_y]
    bufferOffset = _serializer.float64(obj.pos_y, buffer, bufferOffset);
    // Serialize message field [pos_z]
    bufferOffset = _serializer.float64(obj.pos_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DisturbRatioRequest
    let len;
    let data = new DisturbRatioRequest(null);
    // Deserialize message field [pos_x]
    data.pos_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pos_y]
    data.pos_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pos_z]
    data.pos_z = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'disturbance_sources/DisturbRatioRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aa385529adbe642710281f4f0434423c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 pos_x
    float64 pos_y
    float64 pos_z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DisturbRatioRequest(null);
    if (msg.pos_x !== undefined) {
      resolved.pos_x = msg.pos_x;
    }
    else {
      resolved.pos_x = 0.0
    }

    if (msg.pos_y !== undefined) {
      resolved.pos_y = msg.pos_y;
    }
    else {
      resolved.pos_y = 0.0
    }

    if (msg.pos_z !== undefined) {
      resolved.pos_z = msg.pos_z;
    }
    else {
      resolved.pos_z = 0.0
    }

    return resolved;
    }
};

class DisturbRatioResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ratio = null;
    }
    else {
      if (initObj.hasOwnProperty('ratio')) {
        this.ratio = initObj.ratio
      }
      else {
        this.ratio = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DisturbRatioResponse
    // Serialize message field [ratio]
    bufferOffset = _serializer.float64(obj.ratio, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DisturbRatioResponse
    let len;
    let data = new DisturbRatioResponse(null);
    // Deserialize message field [ratio]
    data.ratio = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'disturbance_sources/DisturbRatioResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8a9814c2e046da76c15f5a76016fa0c1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 ratio
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DisturbRatioResponse(null);
    if (msg.ratio !== undefined) {
      resolved.ratio = msg.ratio;
    }
    else {
      resolved.ratio = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: DisturbRatioRequest,
  Response: DisturbRatioResponse,
  md5sum() { return '357bf3ef38c1e016d0352525c776ffa2'; },
  datatype() { return 'disturbance_sources/DisturbRatio'; }
};
