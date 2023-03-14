<template>
  <div>
    <!-- <div id="nav">
      <router-link to="/">Home</router-link> |
      <router-link to="/about">About</router-link>
    </div> -->
    <router-view />
  </div>
</template>

<script lang="ts">
import { Options, Vue } from "vue-class-component";
import M from "materialize-css";

@Options({})
export default class App extends Vue {
  mounted() {
    M.AutoInit(); //initialise materialise css
  }
  async created() {
    document.title = "Roboserver";
  }
}
</script>

<style lang="scss">
// 
// @import "materialize-css/sass/components/color-variables";
// @import "materialize-css/sass/components/variables";
// // $primary-color: color("yellow", "lighten-2") !default;
// $secondary-color: color("purple", "base") !default;
// @import 'materialize-css';


#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
}

#nav {
  padding: 30px;
}

#nav a {
  font-weight: bold;
  color: #2c3e50;
}

#nav a.router-link-exact-active {
  color: #42b983;
}
</style>
