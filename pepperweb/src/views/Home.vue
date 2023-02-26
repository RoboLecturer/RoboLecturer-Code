<template>
  <div>
    <hr v-if="joinedLobby" class="visualTimer animate" />
    <h1 v-if="!timerWindowOpen">{{ statusText }}</h1>
    <h1 v-if="quizInitiated">{{ startTimer }}</h1>
    <div class="container" v-if="!joinedLobby && !quizInitiated && !lobbyClosed">
      <div class="column" style="width: 50%">
        <div class="input-field">
          <input id="last_name" type="text" v-model="username" />
          <label for="last_name">Username</label>
        </div>
        <a class="waves-effect waves-light btn" @click="storeUser"><i class="material-icons left">cloud</i>Join</a>
      </div>
    </div>
    <div class="container">
      <div v-if="timerWindowOpen && joinedLobby" class="column questions">
        <h1>{{ questions[questionIndex]["prompt"] }}</h1>
        <div
          class="row card"
          v-for="(option, index) in questions[questionIndex]['options']"
          :key="index"
          :style="`background-color:${cardColors[index]}`"
          @click="captureAnswer(option)"
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
  question_listener: any = null;
  timer_listener: any = null;
  timerWindowOpen = false;
  questions: any = null;
  questionIndex = 0;
  hrWidth = 0;
  cardColors = ["#ff2400", "#e8b71d", "#1de840", "#335fff", "#dd00f3", "#dd00f3", "#e81d1d", "#e3e81d", "#1ddde8"];
  username = "";
  joinedLobby = false;
  quizStarted = false;
  quizInitiated = false;
  startTimer = 5;
  interval = null;
  correctAnswer = "";
  lobbyClosed = false;
  startTimerInterval = 0;

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

  onQuizTriggered(): void {
    clearInterval(this.startTimerInterval);
    if (this.joinedLobby) {
      this.startTimer = 5;
      this.quizInitiated = true;
      this.statusText = "New question in...";
      if (this.questions != null) {
        this.startTimerInterval = setInterval(() => {
          this.startTimer--;
          if (this.startTimer == 0) {
            this.quizStarted = true;
            this.timerWindowOpen = true;
            this.resetAnimation();
            this.quizInitiated = false;
            setTimeout(this.onQuestionFinished, 8000);
          }
        }, 1000);
      }
    } else {
      this.statusText = "Lobby Closed.";
      this.lobbyClosed = true;
    }
  }

  onNextQuestion(): void {
    // clearInterval(this.startTimerInterval);
    if (this.questionIndex != this.questions.length - 1) {
      this.timerWindowOpen = false;
      this.questionIndex++;
      // this.onQuizTriggered();
    }
  }

  onQuestionFinished(): void {
    // this.timerWindowOpen = false;
    this.statusText = "Correct!";
    return;
  }
  onQuizFinished(): void {
    return;
  }

  captureAnswer(answer: string): void {
    if (answer == this.correctAnswer) {
      this.statusText = "Correct answer!";
      //Update points
    } else {
      this.statusText = "Incorrect answer";
      //Update points
    }
  }

  async fetchQuiz(): Promise<void> {
    let resp = await this.$http.get(`${this.api_url}/quiz`, {
      params: { name: "test" },
    });

    if (resp.status == 200) {
      this.questions = resp.data.questions;
      this.correctAnswer = this.questions[this.questionIndex]["answer"];
    } else {
      this.statusText == "Failed to fetch quiz :(";
      console.log(resp);
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
        this.joinedLobby = true;
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

    this.question_listener = new ROSLIB.Topic({
      ros: this.ros,
      name: "/next_question",
      messageType: "std_msgs/String",
    });
    this.question_listener.subscribe(this.onNextQuestion);
  }

  mounted(): void {
    var y: HTMLElement | null = document.querySelector(".visualTimer");
    if (y) {
      y.style.width = this.hrWidth + "%";
    }
    this.connect();
    this.fetchQuiz();
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
  background-color: #22303e;
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
