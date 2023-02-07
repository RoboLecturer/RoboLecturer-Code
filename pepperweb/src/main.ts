import { createApp } from "vue";
import App from "./App.vue";
import router from "./router";
import store from "./store";
import materialize from "materialize-css";


import "materialize-css/dist/css/materialize.min.css";
import "material-icons/iconfont/material-icons.css";

createApp(App).use(store).use(router).mount("#app");
