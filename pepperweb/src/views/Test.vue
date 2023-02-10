<template>
  <div>
    <div class="container">
      <div class="tile">
        <h3>Subscribe</h3>
        <div class="input-field">
          <select :onchange="setSelectedTopic()"  v-model="selectedTopic" id="topicSelect">
            <option @click="setSelectedTopic(sub)" v-for="(sub, index) in subscriptions" :key="index" :value="sub">{{sub}}</option>
          </select>
          <label>Listening to:</label>
        </div>
        <div class="input-field text-deep-purple lighten-1">
          <input id="last_name" type="text" class="validate" />
        </div>
        <a class="waves-effect waves-light btn" @click="addTopic()"><i class="material-icons left">cloud</i>Add topic</a>
        <p :key="streamUpdate">{{currentText}}</p>
      </div>
      <div class="tile">
        <h3>Publish</h3>
           <div class="input-field">
          <select>
            <option v-for="(sub, index) in publishings" :key="index" :value="sub">{{sub}}</option>
          </select>
          <label>Publishing to:</label>
        </div>
        <div class="input-field text-deep-purple lighten-1">
          <input id="last_name" type="text" class="validate" />
        </div>
        <a class="waves-effect waves-light btn" @click="publishText('Quiz over')"><i class="material-icons left">cloud</i>publish</a>
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import { Options, Vue } from "vue-class-component";
import ROSLIB from "roslib";
import M from "materialize-css";

@Options({})
export default class Test extends Vue {
  ros: any = null;
  connected = false;
  ws_url = "ws://localhost:9000";
  text_listener: any = null;
  quiz_listener: any = null;
  slide_listener: any = null;
  control_publisher: any = null;
  subscriptions: Array<String> = ["trigger_quiz", "change_slide"];
  publishings: Array<String> = ["take_control"];
  quiz_input_stream: String = "Data stream:\n";
  slide_input_stream: String = "Data stream:\n";
  streamUpdate: number = 0;
  selectedTopic: string = "trigger_quiz";

  get currentText(){
    if (this.selectedTopic=="trigger_quiz"){
      return this.quiz_input_stream;
    }else if (this.selectedTopic=="change_slide"){
      return this.slide_input_stream;
    }else{
      return ""
    }
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

    this.quiz_listener = new ROSLIB.Topic({
      ros: this.ros,
      name: "/trigger_quiz",
      messageType: "std_msgs/String",
    });
    this.quiz_listener.subscribe(this.onMessageReceived);

    this.slide_listener = new ROSLIB.Topic({
      ros: this.ros,
      name: "/change_slide",
      messageType: "std_msgs/String",
    });
    this.slide_listener.subscribe(this.onMessageReceived);

    this.control_publisher = new ROSLIB.Topic({
      ros: this.ros,
      name: "/take_control",
      messageType: "std_msgs/String",
    });

    //this.publishText("I published this message, are you listening?");
  }

  setSelectedTopic(){
  
  }

  onMessageReceived(msg:any){
    console.log(msg)
     if (this.selectedTopic=="trigger_quiz"){
      this.quiz_input_stream += msg.data +"\n";
    }else if (this.selectedTopic=="change_slide"){
      this.slide_input_stream += msg.data +"\n";
    }
    this.streamUpdate++;
  }


  publishText(msg: String) {
    var data: any = new ROSLIB.Message({data:msg});
    this.control_publisher.publish(data);
  }

  disconnect() {
    this.ros.close();
    this.connected = false;
  }

  

  initialize(){
    document.addEventListener('DOMContentLoaded', function() {
    var elems = document.querySelectorAll('select');
    M.FormSelect.init(elems, {});
  });
  }

  mounted() {
    this.initialize();
    this.connect();
  }
}
</script>

<style>
p {
  white-space: pre;
}
.container {
  margin: 20 50 50 20;
  display: flex;
  flex-direction: row;
  justify-content: space-evenly;
  width: 100%;
}
.tile {
  display: flex;
  flex-direction: column;
  padding: 10 10 10 10;
}
</style>
