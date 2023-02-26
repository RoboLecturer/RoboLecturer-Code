<template>
  <div class="container"></div>
</template>

<script lang="ts">
import { Options, prop, Vue } from "vue-class-component";
import { Prop, Watch } from "vue-property-decorator";

@Options({})
export default class Question extends Vue {
  @Prop() results!: Array<number>;
}
</script>

<style scoped></style>
