// Auto-generated. Do not edit!

// (in-package api.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class State {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Start = null;
      this.AnyQuestions = null;
      this.NoiseLevel = null;
      this.Attentiveness = null;
      this.NoQuestionsLoop = null;
    }
    else {
      if (initObj.hasOwnProperty('Start')) {
        this.Start = initObj.Start
      }
      else {
        this.Start = '';
      }
      if (initObj.hasOwnProperty('AnyQuestions')) {
        this.AnyQuestions = initObj.AnyQuestions
      }
      else {
        this.AnyQuestions = '';
      }
      if (initObj.hasOwnProperty('NoiseLevel')) {
        this.NoiseLevel = initObj.NoiseLevel
      }
      else {
        this.NoiseLevel = '';
      }
      if (initObj.hasOwnProperty('Attentiveness')) {
        this.Attentiveness = initObj.Attentiveness
      }
      else {
        this.Attentiveness = '';
      }
      if (initObj.hasOwnProperty('NoQuestionsLoop')) {
        this.NoQuestionsLoop = initObj.NoQuestionsLoop
      }
      else {
        this.NoQuestionsLoop = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type State
    // Serialize message field [Start]
    bufferOffset = _serializer.string(obj.Start, buffer, bufferOffset);
    // Serialize message field [AnyQuestions]
    bufferOffset = _serializer.string(obj.AnyQuestions, buffer, bufferOffset);
    // Serialize message field [NoiseLevel]
    bufferOffset = _serializer.string(obj.NoiseLevel, buffer, bufferOffset);
    // Serialize message field [Attentiveness]
    bufferOffset = _serializer.string(obj.Attentiveness, buffer, bufferOffset);
    // Serialize message field [NoQuestionsLoop]
    bufferOffset = _serializer.string(obj.NoQuestionsLoop, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type State
    let len;
    let data = new State(null);
    // Deserialize message field [Start]
    data.Start = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [AnyQuestions]
    data.AnyQuestions = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [NoiseLevel]
    data.NoiseLevel = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [Attentiveness]
    data.Attentiveness = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [NoQuestionsLoop]
    data.NoQuestionsLoop = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.Start);
    length += _getByteLength(object.AnyQuestions);
    length += _getByteLength(object.NoiseLevel);
    length += _getByteLength(object.Attentiveness);
    length += _getByteLength(object.NoQuestionsLoop);
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'api/State';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '814c22ab7e9ed8b959e5c73c87910fce';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string Start
    string AnyQuestions 
    string NoiseLevel
    string Attentiveness
    string NoQuestionsLoop
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new State(null);
    if (msg.Start !== undefined) {
      resolved.Start = msg.Start;
    }
    else {
      resolved.Start = ''
    }

    if (msg.AnyQuestions !== undefined) {
      resolved.AnyQuestions = msg.AnyQuestions;
    }
    else {
      resolved.AnyQuestions = ''
    }

    if (msg.NoiseLevel !== undefined) {
      resolved.NoiseLevel = msg.NoiseLevel;
    }
    else {
      resolved.NoiseLevel = ''
    }

    if (msg.Attentiveness !== undefined) {
      resolved.Attentiveness = msg.Attentiveness;
    }
    else {
      resolved.Attentiveness = ''
    }

    if (msg.NoQuestionsLoop !== undefined) {
      resolved.NoQuestionsLoop = msg.NoQuestionsLoop;
    }
    else {
      resolved.NoQuestionsLoop = ''
    }

    return resolved;
    }
};

module.exports = State;
