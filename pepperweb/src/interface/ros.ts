//Definition of functions for interfacing with ROS and Pepper

import ROSLIB from "roslib";
import { Vue } from "vue-class-component";
import { Options } from "vue-class-component";

/** A Common Notification utility class */

// @Options({
//     name:"RosInterface"
// })
export class RosInterface {
  ros: any = null;
  connected = false;
  ws_url: string;
  text_listener: any = null;
  quiz_starter: any = null;
  control_publisher: any = null;
  next_question_publisher: any = null;
  onQuizTriggered: (msg: any) => void;
  onChangeSlide: (msg: any) => void;
  currentSlide = 0;

  constructor(ws_url = "ws://localhost:9000", onQuizTriggered: (msg: any) => void, onChangeSlide: (msg: any) => void) {
    this.ws_url = ws_url;
    this.onQuizTriggered = onQuizTriggered;
    this.onChangeSlide = onChangeSlide;
  }

  connect() {
    this.ros = new ROSLIB.Ros({
      url: this.ws_url,
    });

    this.ros.on("connection", () => {
      this.connected = true;
      console.log("Connected!");
    });

    this.ros.on("error", (error: any) => {
      console.log("Error connecting to websocket server: ", error);
    });

    this.ros.on("close", () => {
      this.connected = false;
      console.log("Connection to websocket server closed.");
    });

    this.quiz_starter = new ROSLIB.Topic({
      ros: this.ros,
      name: "/start_quiz",
      messageType: "std_msgs/String",
    });

    this.control_publisher = new ROSLIB.Topic({
      ros: this.ros,
      name: "/take_control_forwarder",
      messageType: "std_msgs/String",
    });

    this.next_question_publisher = new ROSLIB.Topic({
      ros: this.ros,
      name: "/next_question",
      messageType: "std_msgs/String",
    });
  }

  publishTakeControl(message: ROSLIB.Message): void {
    this.control_publisher.publish(message);
  }

  publishStartQuiz(message: ROSLIB.Message): void {
    this.quiz_starter.publish(message);
  }

  publishNextQuestion(message: ROSLIB.Message): void {
    this.next_question_publisher.publish(message);
  }

  onSlide(msg: any) {
    // this.onChangeSlide(msg);
    this.currentSlide++;
    console.log("Ros slide: ", this.currentSlide);
  }

  disconnect() {
    this.ros.close();
    this.connected = false;
  }
}

export default RosInterface;
