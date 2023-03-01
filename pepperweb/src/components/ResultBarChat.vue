<template>
  <div class="">
    <BarChart :key="renderKey" :chart-data="chartData" />
  </div>
</template>

<script lang="ts">
import { Options, Vue } from "vue-class-component";
import { Prop, Watch } from "vue-property-decorator";

import { Chart, registerables } from "chart.js";
import { BarChart } from "vue-chart-3";
import ChartDataLabels from "chartjs-plugin-datalabels";

Chart.register(...registerables);
// Chart.register(ChartDataLabels);

Chart.defaults.scale.grid.display = false;
Chart.defaults.plugins.tooltip.enabled = false;
Chart.defaults.plugins.legend.display = false;


@Options({
  components: { BarChart },
})
export default class Question extends Vue {
  @Prop() results!: Array<any>;
  @Prop() options!: Array<number>;
  @Prop() correctAnswer!: number;
  @Prop() colors!: number;
  @Prop() leaderboard!: boolean;
  renderKey = 0;

  chartData = {
    labels: this.options,
    datasets: [
      {
        data: this.formattedData,
        backgroundColor: this.colors,
      },
    ],
  };
  

  get formattedData() {
    if (this.leaderboard) {
      return this.results;
    }
    var r: Array<number> = [];
    this.options.forEach((el) => {
      r.push(0);
    });
    this.results.forEach((el) => {
      r[el.answerIndex] = el.count;
    });
    return r;
  }
}
</script>

<style scoped></style>
