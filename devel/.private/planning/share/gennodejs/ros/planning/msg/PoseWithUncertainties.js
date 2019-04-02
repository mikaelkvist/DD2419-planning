// Auto-generated. Do not edit!

// (in-package planning.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class PoseWithUncertainties {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.position = null;
      this.uncertainties = null;
    }
    else {
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Quaternion();
      }
      if (initObj.hasOwnProperty('uncertainties')) {
        this.uncertainties = initObj.uncertainties
      }
      else {
        this.uncertainties = new geometry_msgs.msg.Quaternion();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PoseWithUncertainties
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Quaternion.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [uncertainties]
    bufferOffset = geometry_msgs.msg.Quaternion.serialize(obj.uncertainties, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PoseWithUncertainties
    let len;
    let data = new PoseWithUncertainties(null);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset);
    // Deserialize message field [uncertainties]
    data.uncertainties = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 64;
  }

  static datatype() {
    // Returns string type for a message object
    return 'planning/PoseWithUncertainties';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '143b13c3f79c2e2058bebcc4baee982a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Quaternion position
    geometry_msgs/Quaternion uncertainties
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PoseWithUncertainties(null);
    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Quaternion.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Quaternion()
    }

    if (msg.uncertainties !== undefined) {
      resolved.uncertainties = geometry_msgs.msg.Quaternion.Resolve(msg.uncertainties)
    }
    else {
      resolved.uncertainties = new geometry_msgs.msg.Quaternion()
    }

    return resolved;
    }
};

module.exports = PoseWithUncertainties;
