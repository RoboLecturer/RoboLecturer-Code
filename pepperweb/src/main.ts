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
let ip_address = "192.168.0.101"
app.config.globalProperties.ws_url = `ws://${ip_address}:443`
app.config.globalProperties.ros_ws_url = `ws://${ip_address}:9000`
app.config.globalProperties.api_url = `http://${ip_address}:3000`;
app.config.globalProperties.ip_address = ip_address;

