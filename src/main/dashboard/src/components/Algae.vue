<template>
<path :d="points" :class="{present}" />
</template>

<script setup lang="ts">
defineProps<{
  points: string;
  present: boolean;
}>();
</script>

<style scoped>
path {
  stroke: rgb(0, 0, 0);
  stroke-miterlimit: 10;
  cursor: pointer;
  transition: opacity 0.3s;
  display:inline;
  stroke-width:0.416776;
  -webkit-tap-highlight-color: transparent;

  fill: #f0f0f0;
}

.present{
  fill: #006b5f;
}

path:hover {
  opacity: 0.7;
}

path:active {
  opacity: 0.5;
}
</style>
