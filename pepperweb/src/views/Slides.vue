<template>
  <div class="slide">
    <pdf
      src="ICL_EEE_CVPR_Part2.pdf"
      :page="currentPage"
      scale.sync="page-width"
      :resize="true"
      @numpages="setNumPages"
    >
    </pdf>
  </div>
</template>

<script lang="ts">
import { Options, Vue } from "vue-class-component";
import ROSLIB from "roslib";
import M from "materialize-css";
import pdfvuer from "pdfvuer";

Vue.registerHooks(["beforeRouteLeave"]);

@Options({
  components: { pdf: pdfvuer },
})
export default class Slides extends Vue {
  $router: any;
  ros: any = null;
  $cookies: any;
  ws_url = "ws://localhost:9000";
  connected = false;
  text_listener!: ROSLIB.Topic;
  quiz_listener!: ROSLIB.Topic;
  slide_listener!: ROSLIB.Topic;
  control_publisher!: ROSLIB.Topic;

  currentPage = 1;
  numPages = 0;

  setNumPages(pages: number): void {
    this.numPages = pages;
  }

  onQuizTriggered(msg: ROSLIB.Message): void {
    console.log(msg);
    // this.$cookies.set("quizCookie", "this is a global cookie");
    this.$router.push({
      name: "Quiz",
    });
  }

  onChangeSlide(msg: any): void {
    var split = msg.data.split("|");
    var action = split[0];
    var value = Number.parseInt(split[1]);
    console.log(msg.data);
    this.changePage(action, value);
  }

  changePage(action: string, value: number) {
    switch (action) {
      case "increment":
        this.currentPage != this.numPages ? (this.currentPage += 1) : 1;
        break;
      case "decrement":
        this.currentPage > 1 ? (this.currentPage -= 1) : 1;
        break;
      case "goto":
        this.currentPage = value;
        break;
      default:
        break;
    }
    this.$cookies.set("currentSlideNumber", this.currentPage);
  }

  publishText(msg: string): void {
    var data: ROSLIB.Message = new ROSLIB.Message({ data: msg });
    this.ros.publishTakeControl(data);
  }

  connect(): void {
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
    this.quiz_listener.subscribe(this.onQuizTriggered);

    this.slide_listener = new ROSLIB.Topic({
      ros: this.ros,
      name: "/change_slide",
      messageType: "std_msgs/String",
      queue_length: 10,
      throttle_rate: 1,
      reconnect_on_close:true,
    });
    this.slide_listener.subscribe(this.onChangeSlide);

    this.control_publisher = new ROSLIB.Topic({
      ros: this.ros,
      name: "/take_control_forwarder",
      messageType: "std_msgs/String",
    });
  }

  initialize(): void {
    document.addEventListener("DOMContentLoaded", function () {
      var elems = document.querySelectorAll("select");
      M.FormSelect.init(elems, {});
    });
  }

  mounted(): void {
    var pageCookie = this.$cookies.get("currentSlideNumber");
    console.log(pageCookie);
    if (pageCookie) {
      this.currentPage = parseInt(pageCookie);
    }

    this.initialize();
    this.connect();
    document.addEventListener(
      "keyup",
      (event) => {
        const keyName = event.key;
        // As the user releases the Ctrl key, the key is no longer active,
        // so event.ctrlKey is false.
        if (keyName === "ArrowRight") {
          this.changePage("increment", 0);
        } else if (keyName === "ArrowLeft") {
          this.changePage("decrement", 0);
        }
      },
      false
    );
  }

  // beforeRouteLeave() {
  //   console.log("BEFORE RL")

  //   // this.ros.close();
  // }
}
</script>

<style>
.slide {
  /* height:100%;
  width:100%;
  display:flex;
  justify-content: center; */
}

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
