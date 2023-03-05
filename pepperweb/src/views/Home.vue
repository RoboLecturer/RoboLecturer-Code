<template>
  <div>
    <hr v-if="joinedLobby" class="visualTimer animate" />
    <h1 v-if="!timerWindowOpen">{{ statusText }}</h1>

    <!-- SHOW WHEN NO QUESTION BEING ASKED -->
    <h1 v-if="quizInitiated">{{ startTimer }}</h1>

    <!-- IF TIMER WINDOW OPEN and JOINED LOBBY -->
    <div class="container" v-if="!joinedLobby && !quizInitiated && !lobbyClosed">
      <div class="column" style="width: 50%">
        <div class="input-field">
          <input id="last_name" type="text" v-model="username" />
          <label for="last_name">Username</label>
        </div>
        <a class="waves-effect waves-light btn" @click="storeUser"><i class="material-icons left">cloud</i>Join</a>
      </div>
    </div>
    <!-- IF TIMER WINDOW OPEN and JOINED LOBBY: SHOW QUESTIONS -->
    <div class="container">
      <div v-if="timerWindowOpen && joinedLobby" class="column">
        <h1 class="row s3 l3 m3">{{ questions[questionIndex]["prompt"] }}</h1>
        <div class="options row s9 l9 m9">
          <div
            class="card"
            v-for="(option, index) in questions[questionIndex]['options']"
            :key="index"
            :style="`background-color:${cardColors[index]}`"
            @click="captureAnswer(option)"
          >
            <div class="card-content">{{ option }}</div>
          </div>
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
  quiz_over_listener: any = null;
  timerWindowOpen = false;
  questions: any = null;
  questionIndex = 0;
  hrWidth = 0;
  cardColors = ["#ff2400", "#e8b71d", "#1de840", "#1ddee8", "#dd00f3", "#dd00f3", "#e81d1d", "#e3e81d", "#1ddde8"];
  username = "";
  joinedLobby = false;
  quizStarted = false;
  quizInitiated = false;
  startTimer = 5;
  interval = null;
  lobbyClosed = false;
  startTimerInterval = 0;
  studentId = -1;
  answerIndex = -1;

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
            this.quizInitiated = false;
            this.resetAnimation();
            setTimeout(() => null, 8000);
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
    }
  }

  onQuizOver(): void {
    this.statusText = "Join the lobby";
    this.username = "";
    this.joinedLobby = false;
    this.quizStarted = false;
    this.quizInitiated = false;
    this.timerWindowOpen = false;
    this.lobbyClosed = false;
    this.questionIndex = 0;
  }

  captureAnswer(answer: string): void {
    let index = this.questions[this.questionIndex]["options"].indexOf(answer);
    let correctAnswer = this.questions[this.questionIndex]["answer"];
    this.answerIndex = index;
    var elements = document.querySelectorAll(".card");
    elements.forEach((element, index) => {
      element.classList.add("noHover");
      if (index != this.answerIndex) {
        element.classList.add("unselectedAnswer");
      }
    });
    if (answer == correctAnswer) {
      this.storeResult(true, 100, index);
    } else {
      this.storeResult(false, 100, index);
    }
  }

  async storeResult(correct: boolean, points: number, answerIndex: number): Promise<void> {
    let resp = await this.$http.post(`${this.api_url}/addResult`, {
      StudentId: this.studentId,
      QuestionNumber: this.questionIndex,
      answerIndex: answerIndex,
      isCorrect: correct,
      Points: points,
    });
    if (resp.status != 200) {
      console.log(resp);
    }
    this.getResults();
  }

  async getResults(): Promise<void> {
    let resp = await this.$http.get(`${this.api_url}/getResult`, {
      params: { question_id: this.questionIndex },
    });

    if (resp.status == 200) {
      console.log(resp.data);
    } else {
      console.log(resp);
    }
  }

  async fetchQuiz(): Promise<void> {
    let resp = await this.$http.get(`${this.api_url}/quiz`, {
      params: { name: "test" },
    });

    if (resp.status == 200) {
      this.questions = resp.data.questions;
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
        this.studentId = resp.data.studentId;
        console.log("user stored with id", this.studentId);
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

    this.quiz_over_listener = new ROSLIB.Topic({
      ros: this.ros,
      name: "/take_control_forwarder",
      messageType: "std_msgs/String",
    });
    this.quiz_over_listener.subscribe(this.onQuizOver);
  }

  mounted(): void {
    var y: HTMLElement | null = document.querySelector(".visualTimer");
    if (y) {
      y.style.width = this.hrWidth + "%";
    }
    this.connect();
    this.fetchQuiz();
  }

  // beforeRouteLeave() {
  //   console.log("BEFORE RL Home");
  //   // this.ros.close();
  //   // this.connected = false;
  // }
}
</script>

<style scoped>
.noHover {
  pointer-events: none;
}
.unselectedAnswer {
  background-color: rgba(128, 128, 128, 0.784) !important;
}

.column {
  width: 80%;
}

.card-content {
  font-size: 18px;
}

.options {
  display: flex;
  flex-direction: column;
  justify-content: center;
  padding-top: 30px;
}

.card {
  height: 75px;
}
.card:hover {
  opacity: 0.5;
  font-size: 20px;
}

.questions {
  width: 100%;
  display: flex;
  flex-direction: column;
  justify-content: space-around;
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
