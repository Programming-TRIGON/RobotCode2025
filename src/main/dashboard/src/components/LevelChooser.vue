<template>
  <div class="container">
    <button v-for="l in LEVELS" @click="requestChange(l)" :class="{selected: l === selected}">
    {{ l }}</button>
  </div>
</template>

<script setup lang="ts">
import { LEVELS, type Level } from '../ReefTypes';

defineProps<{
  selected: Level;
  requestChange: (level: Level) => void;
}>();
</script>

<style scoped>
.container{
  display: flex;
  flex-direction: column;
  margin: 20px;
}

button{
  background-color: #f0f0f0;
  border: none;
  color: black;
  text-align: center;
  text-decoration: none;
  display: inline-block;
  font-size: 40px;
  margin: 4px 2px;
  cursor: pointer;
  border-radius: 12px;
  padding: 10px 24px;
  width: 200px;
  height: 120px;
}

button.selected{
  background-color: #00662a;
  color: white;
}

button:hover{
  background-color: #00451d;
  color: white;
}

button:active{
  background-color: #003c1a;
  color: white;
}

button:focus{
  outline: none;
}
</style>