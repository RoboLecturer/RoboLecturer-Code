import { createApp } from "vue";
import App from "./App.vue";
import router from "./router";
import store from "./store";
import materialize from "materialize-css";
import VueCookies from "vue3-cookies";
import axios from "axios";

import "materialize-css/dist/css/materialize.min.css";
import "material-icons/iconfont/material-icons.css";

const app = createApp(App);
app.use(VueCookies).use(store).use(router).mount("#app");
app.config.globalProperties.$http = axios;
app.config.globalProperties.api_url = "http://127.0.0.1:3000";
