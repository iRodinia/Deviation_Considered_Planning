// Auto-generated. Do not edit!

// (in-package reference_governor.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class polyTraj {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.start_planning_time = null;
      this.poly_coefs = null;
      this.poly_duration = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('start_planning_time')) {
        this.start_planning_time = initObj.start_planning_time
      }
      else {
        this.start_planning_time = new std_msgs.msg.Time();
      }
      if (initObj.hasOwnProperty('poly_coefs')) {
        this.poly_coefs = initObj.poly_coefs
      }
      else {
        this.poly_coefs = new std_msgs.msg.Float64MultiArray();
      }
      if (initObj.hasOwnProperty('poly_duration')) {
        this.poly_duration = initObj.poly_duration
      }
      else {
        this.poly_duration = new std_msgs.msg.Float64();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type polyTraj
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [start_planning_time]
    bufferOffset = std_msgs.msg.Time.serialize(obj.start_planning_time, buffer, bufferOffset);
    // Serialize message field [poly_coefs]
    bufferOffset = std_msgs.msg.Float64MultiArray.serialize(obj.poly_coefs, buffer, bufferOffset);
    // Serialize message field [poly_duration]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.poly_duration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type polyTraj
    let len;
    let data = new polyTraj(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [start_planning_time]
    data.start_planning_time = std_msgs.msg.Time.deserialize(buffer, bufferOffset);
    // Deserialize message field [poly_coefs]
    data.poly_coefs = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [poly_duration]
    data.poly_duration = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += std_msgs.msg.Float64MultiArray.getMessageSize(object.poly_coefs);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'reference_governor/polyTraj';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '71bc53ae599621f980c4732c605466da';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    std_msgs/Time start_planning_time
    std_msgs/Float64MultiArray poly_coefs
    std_msgs/Float64 poly_duration
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: std_msgs/Time
    time data
    
    ================================================================================
    MSG: std_msgs/Float64MultiArray
    # Please look at the MultiArrayLayout message definition for
    # documentation on all multiarrays.
    
    MultiArrayLayout  layout        # specification of data layout
    float64[]         data          # array of data
    
    
    ================================================================================
    MSG: std_msgs/MultiArrayLayout
    # The multiarray declares a generic multi-dimensional array of a
    # particular data type.  Dimensions are ordered from outer most
    # to inner most.
    
    MultiArrayDimension[] dim # Array of dimension properties
    uint32 data_offset        # padding elements at front of data
    
    # Accessors should ALWAYS be written in terms of dimension stride
    # and specified outer-most dimension first.
    # 
    # multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
    #
    # A standard, 3-channel 640x480 image with interleaved color channels
    # would be specified as:
    #
    # dim[0].label  = "height"
    # dim[0].size   = 480
    # dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
    # dim[1].label  = "width"
    # dim[1].size   = 640
    # dim[1].stride = 3*640 = 1920
    # dim[2].label  = "channel"
    # dim[2].size   = 3
    # dim[2].stride = 3
    #
    # multiarray(i,j,k) refers to the ith row, jth column, and kth channel.
    
    ================================================================================
    MSG: std_msgs/MultiArrayDimension
    string label   # label of given dimension
    uint32 size    # size of given dimension (in type units)
    uint32 stride  # stride of given dimension
    ================================================================================
    MSG: std_msgs/Float64
    float64 data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new polyTraj(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.start_planning_time !== undefined) {
      resolved.start_planning_time = std_msgs.msg.Time.Resolve(msg.start_planning_time)
    }
    else {
      resolved.start_planning_time = new std_msgs.msg.Time()
    }

    if (msg.poly_coefs !== undefined) {
      resolved.poly_coefs = std_msgs.msg.Float64MultiArray.Resolve(msg.poly_coefs)
    }
    else {
      resolved.poly_coefs = new std_msgs.msg.Float64MultiArray()
    }

    if (msg.poly_duration !== undefined) {
      resolved.poly_duration = std_msgs.msg.Float64.Resolve(msg.poly_duration)
    }
    else {
      resolved.poly_duration = new std_msgs.msg.Float64()
    }

    return resolved;
    }
};

module.exports = polyTraj;
