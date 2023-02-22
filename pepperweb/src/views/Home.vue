<template>
  <div>
    <hr class="visualTimer animate" />
    <h1>{{ statusText }}</h1>
    <div class="container">
      <div class="column" style="width: 50%">
        <div class="input-field">
          <input id="last_name" type="text" v-model="username" />
          <label for="last_name">Username</label>
        </div>
        <a class="waves-effect waves-light btn" @click="storeUser"><i class="material-icons left">cloud</i>Join</a>
      </div>
    </div>
    <div class="container">
      <div v-if="questions != null && timerWindowOpen" class="column questions">
        <h1>{{ questions[questionIndex]["prompt"] }}</h1>
        <div
          class="row card"
          v-for="(option, index) in questions[questionIndex]['options']"
          :key="index"
          :style="`background-color:${cardColors[index]}`"
        >
          {{ option }}
        </div>
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import { Options, Vue } from "vue-class-component";
import ROSLIB from "roslib";

Vue.registerHooks(["beforeRouteLeave"]);

@Options({})
export default class Home extends Vue {
  $http: any;
  api_url: any;
  statusText = "Join the lobby";
  ros: any = null;
  connected = false;
  ws_url = "ws://localhost:9000";
  quiz_listener: any = null;
  timer_listener: any = null;
  timerWindowOpen = false;
  questions: any = null;
  questionIndex = 0;
  hrWidth = 0;
  cardColors = ["#ff2400", "#e8b71d", "#1de840", "#335fff", "#dd00f3", "#dd00f3", "#e81d1d", "#e3e81d", "#1ddde8"];
  username = "";

  resetAnimation(): void {
    this.timerWindowOpen = true;
    var y: HTMLElement | null = document.querySelector(".visualTimer");
    this.hrWidth = 0;
    if (y) {
      y.style.width = this.hrWidth + "%";
      y.classList.remove("animate");
      y.classList.add("stopanimation");
    }
    setTimeout(this.startAnimation, 10);
  }

  startAnimation(): void {
    var y: HTMLElement | null = document.querySelector(".visualTimer");
    if (y) {
      this.hrWidth = 100;
      y.classList.remove("stopanimation");
      y.classList.add("animate");
      //y.removeClass("stopanimation").addClass("animate");
      y.style.width = 100 + "%";
    }
  }

  onTimerBegin(): void {
    if (this.questions != null) {
      this.statusText += "6s left";
      this.resetAnimation();
    }
  }

  onQuizTriggered(): void {
    this.statusText = "Quiz begun!";
    this.fetchQuiz();
  }

  async fetchQuiz(): Promise<void> {
    let resp = await this.$http.get(`${this.api_url}/quiz`, {
      headers: {
        "Access-Control-Allow-Origin": "*",
      },
      params: { name: "test" },
    });
    console.log(resp);
    if (resp.status == 200) {
      this.questions = resp.data.questions;
      console.log(this.questions);
    } else {
      this.statusText == "Failed to fetch quiz :(";
    }
  }

  async storeUser(): Promise<void> {
    if (this.username != "") {
      let resp = await this.$http.post(`${this.api_url}/insertUser`, {
        username: this.username,
      });
      if (resp.status == 200) {
        console.log("user stored");
        this.statusText = "Waiting for quiz to start...";
      } else {
        console.log(resp);
      }
    }
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
      name: "/start_quiz",
      messageType: "std_msgs/String",
    });
    this.quiz_listener.subscribe(this.onQuizTriggered);

    this.timer_listener = new ROSLIB.Topic({
      ros: this.ros,
      name: "/start_timer",
      messageType: "std_msgs/String",
    });
    this.timer_listener.subscribe(this.onTimerBegin);
  }

  mounted(): void {
    console.log(this.api_url);
    var y: HTMLElement | null = document.querySelector(".visualTimer");
    if (y) {
      y.style.width = this.hrWidth + "%";
    }

    this.connect();
  }

  beforeRouteLeave(): void {
    this.ros.close();
    this.connected = false;
  }
}
</script>

<style scoped>
:root {
  --time: 10s;
}

.questions {
  width: 100%;
}
.card:hover {
  opacity: 0.5;
}

/* progress bar */
.visualTimer {
  display: block;
  height: 20px;
  background-color: #039be5;
  width: 0%;
  margin: 0;
  border: none;
  z-index: 11;
  position: fixed;
  top: 0px;
}

.visualTimer.animate {
  -webkit-transition: width 8s linear;
  -moz-transition: width 8s linear;
  -ms-transition: width 8s linear;
  -o-transition: width 8s linear;
  transition: width 8s linear;
}
</style>
