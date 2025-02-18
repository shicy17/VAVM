// Auto-generated. Do not edit!

// (in-package dvs_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Event = require('./Event.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class EventArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.height = null;
      this.width = null;
      this.events = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0;
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = 0;
      }
      if (initObj.hasOwnProperty('events')) {
        this.events = initObj.events
      }
      else {
        this.events = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EventArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [height]
    bufferOffset = _serializer.uint32(obj.height, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = _serializer.uint32(obj.width, buffer, bufferOffset);
    // Serialize message field [events]
    // Serialize the length for message field [events]
    bufferOffset = _serializer.uint32(obj.events.length, buffer, bufferOffset);
    obj.events.forEach((val) => {
      bufferOffset = Event.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EventArray
    let len;
    let data = new EventArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [height]
    data.height = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [events]
    // Deserialize array length for message field [events]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.events = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.events[i] = Event.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 13 * object.events.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dvs_msgs/EventArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5e8beee5a6c107e504c2e78903c224b8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This message contains an array of events
    # (0, 0) is at top-left corner of image
    #
    
    Header header
    
    uint32 height         # image height, that is, number of rows
    uint32 width          # image width, that is, number of columns
    
    # an array of events
    Event[] events
    
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
    MSG: dvs_msgs/Event
    # A DVS event
    uint16 x
    uint16 y
    time ts
    bool polarity
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EventArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0
    }

    if (msg.width !== undefined) {
      resolved.width = msg.width;
    }
    else {
      resolved.width = 0
    }

    if (msg.events !== undefined) {
      resolved.events = new Array(msg.events.length);
      for (let i = 0; i < resolved.events.length; ++i) {
        resolved.events[i] = Event.Resolve(msg.events[i]);
      }
    }
    else {
      resolved.events = []
    }

    return resolved;
    }
};

module.exports = EventArray;
