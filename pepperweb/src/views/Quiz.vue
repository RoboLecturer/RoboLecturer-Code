<template>
  <div class="rainbow">
    <hr class="visualTimer animate" />
    <div v-if="quizStarted" class="container column">
      <h1>{{ questions[questionIndex]["prompt"] }}</h1>
    </div>
    <div class="column" v-else>
      <div class="row s3 m3 l3">
        <h1>
          {{ statusText }}
        </h1>
        <h1 v-if="quizInitiated">{{ startTimer }}</h1>
      </div>
      <div v-if="!quizInitiated" class="row s9 m9 l9">
        <p v-for="(user, index) in users" :key="index">{{ user["Username"] }}</p>
      </div>
    </div>

    <div class="navguide">
      <a class="kbc-button">Q</a>
      <a class="kbc-button">S</a>
      <a class="kbc-button">N</a>
    </div>
    <div class="usercount" v-if="users.length > 0">
      <h5><i class="material-icons left">group</i>{{ users.length }}</h5>
    </div>
  </div>
</template>

<script lang="ts">
import RosInterface from "@/interface/ros";
import { Options, Vue } from "vue-class-component";

Vue.registerHooks(["beforeRouteLeave"]);

@Options({})
export default class Quiz extends Vue {
  api_url: any;
  $cookies: any;
  $http: any;
  questions = [];
  quizStarted = false;
  quizInitiated = false;
  questionIndex = 0;
  hrWidth = 0;
  timerWindowOpen = false;
  startTimer = 5;
  startTimerInterval = 0;
  rosInterface: RosInterface = new RosInterface(
    "ws://localhost:9000",
    () => {
      return;
    },
    () => {
      return;
    }
  );
  statusText = "Waiting for users to join...";
  users = [];

  async getUsersInLobby(): Promise<void> {
    let resp = await this.$http.post(`${this.api_url}/getUsers`, {
      quiz_id: 0,
    });
    if (resp.status == 200) {
      this.users = resp.data;
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

  startQuiz(): void {
    clearInterval(this.startTimerInterval);
    this.startTimer = 5;
    this.quizStarted = false;
    this.quizInitiated = true;
    this.rosInterface.publishStartQuiz({ data: "1" }); //starts 5 second timer on all clients
    if (this.questionIndex > 0) {
      this.statusText = "Next question in...";
    } else {
      this.statusText = "Quiz starting in...";
    }
    this.startTimerInterval = setInterval(() => {
      this.startTimer--;
      if (this.startTimer == 0) { //Question timer
        this.resetAnimation();
        this.quizStarted = true;
      }
    }, 1000);
  }

  incrementQuestion(): void {
    if (this.questionIndex != this.questions.length - 1) {
      this.rosInterface.publishNextQuestion({ data: "" });
      this.questionIndex++;
      this.startQuiz();
    } else {
      this.statusText = "Quiz Over!";
    }
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

  mounted(): void {
    this.rosInterface.connect();
    console.log(this.$cookies.get("quizCookie"));
    document.addEventListener(
      "keyup",
      (event) => {
        const keyName = event.key;
        // As the user releases the Ctrl key, the key is no longer active,
        // so event.ctrlKey is false.
        if (keyName === "q") {
          this.rosInterface.publishTakeControl({ data: "Hiya" });
          this.$router.push({
            name: "Slides",
          });
        }
      },
      false
    );
    document.addEventListener(
      "keyup",
      (event) => {
        const keyName = event.key;
        if (keyName === "s") {
          this.startQuiz();
        }
      },
      false
    );
    document.addEventListener(
      "keyup",
      (event) => {
        const keyName = event.key;
        if (keyName === "n") {
          this.incrementQuestion();
        }
      },
      false
    );
    this.fetchQuiz();
    setInterval(this.getUsersInLobby, 2000);
  }

  beforeRouteLeave() {
    this.rosInterface.disconnect();
  }
}
</script>

<style scoped>
.container {
  flex-direction: column;
  height: 100%;
  padding-bottom: 150px;
}
.question {
  /* 2c3e50 */
  color: #22303e;
}
.rainbow {
  height: 100%;
  width: 100%;
  left: 0;
  right: 0;
  top: 0;
  bottom: 0;
  position: absolute;
  background: linear-gradient(
    124deg,
    #ff2400,
    #e81d1d,
    #e8b71d,
    #e3e81d,
    #1de840,
    #1ddde8,
    #4066efbc,
    #dd00f3,
    #dd00f3
  );
  background-size: 1800% 1800%;

  -webkit-animation: rainbow 18s ease infinite;
  -z-animation: rainbow 18s ease infinite;
  -o-animation: rainbow 18s ease infinite;
  animation: rainbow 18s ease infinite;
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

.navguide {
  position: absolute;
  top: 50px;
  right: 50px;
}

.usercount {
  display: flex;
  flex-direction: row;
  position: absolute;
  top: 50px;
  left: 50px;
}

.kbc-button {
  font-size: inherit;
  line-height: inherit;
  margin-bottom: 0.4375rem;
  margin-left: 0.25rem;
  margin-right: 0.25rem;
  margin-top: 0.25rem;
  padding: 0.1rem 0.4rem;
  box-shadow: 0 0 #d9d9d9, 0 0px #d9d9d9, 0 1px #d9d9d9, 0 2px #d9d9d9, 0 3px #d9d9d9, 0 4px #d9d9d9, 0 5px #d9d9d9,
    2px 2.5px 4px #adb5bd, 0 -1px 2.5px #adb5bd;
  background-color: #fff;
  border-color: #e6e6e6;
  color: #343a40;
  -webkit-backface-visibility: hidden;
  backface-visibility: hidden;
  border: 1px solid #e6e6e6;
  border-radius: 0.25rem;
  display: inline-block;
  font-family: SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace;
  font-weight: 400;
  text-align: left;
  transform: translate3d(0, 0, 5px);
  transform-style: preserve-3d;
  transition: all 0.25s cubic-bezier(0.2, 1, 0.2, 1);
}

@-webkit-keyframes rainbow {
  0% {
    background-position: 0% 82%;
  }
  50% {
    background-position: 100% 19%;
  }
  100% {
    background-position: 0% 82%;
  }
}
@-moz-keyframes rainbow {
  0% {
    background-position: 0% 82%;
  }
  50% {
    background-position: 100% 19%;
  }
  100% {
    background-position: 0% 82%;
  }
}
@-o-keyframes rainbow {
  0% {
    background-position: 0% 82%;
  }
  50% {
    background-position: 100% 19%;
  }
  100% {
    background-position: 0% 82%;
  }
}
@keyframes rainbow {
  0% {
    background-position: 0% 82%;
  }
  50% {
    background-position: 100% 19%;
  }
  100% {
    background-position: 0% 82%;
  }
}
</style>
