<template>
  <div class="home">
    <img alt="Vue logo" src="../assets/logo.png" />
  </div>
</template>

<script lang="ts">
import { Options, Vue } from "vue-class-component";
import ROSLIB from "roslib";

@Options({})
export default class Home extends Vue {
  ros: any = null;
  connected = false;
  ws_url = "ws://localhost:9000";
  text_listener: any = null;
  text_publisher: any = null;

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

    this.text_listener = new ROSLIB.Topic({
      ros: this.ros,
      name: "/txt_msg",
      messageType: "std_msgs/String",
    });

    this.text_listener.subscribe(function (m: any) {
      console.log(m.data);
    });

    this.text_publisher = new ROSLIB.Topic({
      ros: this.ros,
      name: "/txt_msg_pub",
      messageType: "geometry_msgs/Twist",
    });
    
    this.publishText("I published this message, are you listening?");
  }

  disconnect() {
    this.ros.close();
    this.connected = false;
  }

  publishText(msg: String){
    var data:any = new ROSLIB.Message({
        linear: {
        x: 10,
        y: 0,
        z: 0
      },
      angular: {
        x: 0,
        y: 0,
        z: 11
      }
    });
    this.text_publisher.publish(data);
  }

  mounted() {
    this.connect();
  }
}
</script>
