<template>
  <div class="rainbow">
    <div class="questionContainer">
      <div class="question">
        <h1>
          {{ question }}
        </h1>
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import RosInterface from "@/interface/ros";
import { Options, Vue } from "vue-class-component";

Vue.registerHooks(["beforeRouteLeave"]);

@Options({})
export default class Quiz extends Vue {
  $cookies: any;
  $http: any;
  rosInterface: RosInterface = new RosInterface(
    "ws://localhost:9000",
    () => {
      return;
    },
    () => {
      return;
    }
  );
  question = "What is Pepper robot and how do I communicate with it?";

  async fetchQuiz(): Promise<void> {
    let resp = await this.$http.get("http://127.0.0.1:3000/quiz", {
      headers: {
        "Access-Control-Allow-Origin": "*",
      },
      params: { name: "test" },
    });

    console.log(resp.data);
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
    this.fetchQuiz();
  }

  beforeRouteLeave(){
    this.rosInterface.disconnect();
  }
}
</script>

<style scoped>
.questionContainer {
  height: 100%;
  display: flex;
  justify-content: center;
  flex-direction: column;
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
  background: linear-gradient(124deg, #ff2400, #e81d1d, #e8b71d, #e3e81d, #1de840, #1ddde8, #335fff, #dd00f3, #dd00f3);
  background-size: 1800% 1800%;

  -webkit-animation: rainbow 18s ease infinite;
  -z-animation: rainbow 18s ease infinite;
  -o-animation: rainbow 18s ease infinite;
  animation: rainbow 18s ease infinite;
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
