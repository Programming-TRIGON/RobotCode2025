<template>
<path :d="points" :class="[status]" />
</template>

<script setup lang="ts">

import type {BranchState} from "../ReefTypes.ts";

defineProps<{
  points: string;
  status: BranchState;
}>();
</script>

<style scoped>
path {
  stroke: rgb(0, 136, 136);
  stroke-miterlimit: 10;
  cursor: pointer;
  transition: opacity 0.3s;
  display:inline;
  stroke-width:0.416776;
  -webkit-tap-highlight-color: transparent;
}

.FREE{
  fill: #f0f0f0;
}

.OCCUPIED{
  fill: #ff0000;
}

.DISABLED{
  fill: #454545;
}

path:hover {
  opacity: 0.7;
}

path:active {
  opacity: 0.5;
}
</style>
