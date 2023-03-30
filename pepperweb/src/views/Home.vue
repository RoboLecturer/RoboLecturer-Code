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
    <div class="container" v-if="!showQuizWinner">
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
    <div class="congrats" v-if="showQuizWinner&&joinedLobby">
      <h1 class="row s3 l3 m3">Congratulations {{ winner }}!</h1>
    </div>
    <div class="usercount" v-if="quizStarted && !showQuizWinner" >
      <h5><i class="material-icons left">quiz</i>{{ questionCountText }}</h5>
    </div>
  </div>
  <div class="correctIncorrectImage" v-if="showUserResult && !showQuizWinner">
    <img class="resultImage" :src="resultImgSrc" alt="" />
    <h5>{{ answeredCorrectly ? "Correct!" : "Incorrect." }}</h5>
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
  ws_url: any;
  ros_ws_url: any;
  statusText = "Join the lobby";
  ros: any = null;
  connected = false;
  quiz_listener: any = null;
  change_slide_listener: any = null;
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
  webSocket: any = null;
  answeredCorrectly: boolean = false;
  showUserResult: boolean = false;
  winner = "";
  showQuizWinner: boolean = false;

  get questionCountText() {
    return this.questionIndex + 1 + " of " + this.questions.length;
  }

  get resultImgSrc() {
    return this.answeredCorrectly ? "assets/correct.png" : "assets/incorrect.png";
  }

  showWinner(){
    console.log(this.winner);
  }

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
    this.showUserResult = false;
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
            setTimeout(() => {
              setTimeout(() => {
                this.showUserResult = true;
              }, 2000);
            }, 8000);
          }
        }, 1000);
      }
    } else {
      this.statusText = "Lobby Closed.";
      this.lobbyClosed = true;
    }
  }

  onChangeSlide(msg: any): void {
    console.log("slide change: ", msg);
  }

  onNextQuestion(): void {
    console.log("Next question");
    if (this.questionIndex != this.questions.length - 1) {
      this.timerWindowOpen = false;
      this.questionIndex++;
    }
  }

  onQuizOver(): void {
    console.log("Quiz over");
    this.statusText = "Join the lobby";
    this.username = "";
    this.joinedLobby = false;
    this.quizStarted = false;
    this.quizInitiated = false;
    this.timerWindowOpen = false;
    this.lobbyClosed = false;
    this.showUserResult = false;
    this.showQuizWinner = false;
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
      this.answeredCorrectly = true;
      this.storeResult(true, 100, index);
    } else {
      this.answeredCorrectly = false;
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
      params: { name: "quiz" },
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
    this.webSocket = new WebSocket(this.ws_url);
    this.webSocket.onmessage = (event: any) => {
      switch (event.data) {
        case "start quiz":
          this.onQuizTriggered();
          break;
        case "next question":
          this.onNextQuestion();
          break;

        case "quiz over":
          this.onQuizOver();
          break;

        default:
          if (event.data.includes("Winner|")) {
            this.winner = event.data.replace("Winner|","");
            this.showQuizWinner = true;
          }
          break;
      }
    };
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
  //   this.ros.close();
  //   this.connected = false;
  // }
}
</script>

<style scoped>
.congrats{
  padding-top:200px;
}
.resultImage {
  width: 60px;
  height: 60px;
}

.correctIncorrectImage {
  display: flex;
  flex-direction: column;
  position: absolute;
  top: 50px;
  right: 30px;
}
.usercount {
  display: flex;
  flex-direction: row;
  position: absolute;
  top: 50px;
  left: 30px;
}
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
