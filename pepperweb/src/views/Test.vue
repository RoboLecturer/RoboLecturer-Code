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
        <p :key="streamUpdate" >{{slide_input_stream}}</p>
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
import RosInterface from "../interface/ros";

@Options({})
export default class Test extends Vue {
  rosInterface: RosInterface = new RosInterface("ws://localhost:9000");
  ros: any = null;
  connected = false;
  
  subscriptions: Array<String> = ["trigger_quiz", "change_slide"];
  publishings: Array<String> = ["take_control_forwarder"];
  quiz_input_stream: string = "Data stream:\n";
  slide_input_stream: string = "Data stream:\n";
  streamUpdate: number = 0;
  selectedTopic: string = "change_slide";

  get currentText(){
    return this.slide_input_stream;
    if (this.selectedTopic=="trigger_quiz"){
      return this.quiz_input_stream;
    }else if (this.selectedTopic=="change_slide"){
      return this.slide_input_stream;
    }else{
      return ""
    }
  }

  onQuizTriggered(msg:any){
    console.log("on quiz triggered")
     console.log(msg)
    this.quiz_input_stream += msg.data +"\n";
    this.streamUpdate++;
  }

  onChangeSlide(msg:any){
    console.log("on change slide")
    console.log(msg)
    this.slide_input_stream += msg.data +"\n";
    console.log(this.slide_input_stream)
    this.streamUpdate++;
  }


  publishText(msg: String) {
    var data: ROSLIB.Message = new ROSLIB.Message({data:msg});
    this.rosInterface.publishTakeControl(data);
  }

  setSelectedTopic(){

  }

  initialize(){
    document.addEventListener('DOMContentLoaded', function() {
    var elems = document.querySelectorAll('select');
    M.FormSelect.init(elems, {});
  });
  }

  mounted() {
    this.initialize();
    this.rosInterface.connect();
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
