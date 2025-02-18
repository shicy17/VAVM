// Auto-generated. Do not edit!

// (in-package dvs_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Event {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
      this.ts = null;
      this.polarity = null;
    }
    else {
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0;
      }
      if (initObj.hasOwnProperty('ts')) {
        this.ts = initObj.ts
      }
      else {
        this.ts = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('polarity')) {
        this.polarity = initObj.polarity
      }
      else {
        this.polarity = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Event
    // Serialize message field [x]
    bufferOffset = _serializer.uint16(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.uint16(obj.y, buffer, bufferOffset);
    // Serialize message field [ts]
    bufferOffset = _serializer.time(obj.ts, buffer, bufferOffset);
    // Serialize message field [polarity]
    bufferOffset = _serializer.bool(obj.polarity, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Event
    let len;
    let data = new Event(null);
    // Deserialize message field [x]
    data.x = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [ts]
    data.ts = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [polarity]
    data.polarity = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dvs_msgs/Event';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '40d339b261445b8ce9a6fef01267cd5d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Event(null);
    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0
    }

    if (msg.ts !== undefined) {
      resolved.ts = msg.ts;
    }
    else {
      resolved.ts = {secs: 0, nsecs: 0}
    }

    if (msg.polarity !== undefined) {
      resolved.polarity = msg.polarity;
    }
    else {
      resolved.polarity = false
    }

    return resolved;
    }
};

module.exports = Event;
