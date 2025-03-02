<template>
  <div class="container root" :class="{ disconnected: !connected }">
    <h2>Reef Tracker</h2>

    <div class="layout">
      <LevelChooser :selected="settings.targetL.value" :requestChange="l => requestChangeSetting('targetL', l)" class="levels" />
      <Reef v-bind="{ setBranchState, setAlgaeState, reefState, algaeStates }" class="reef" />
      <input type="checkbox" v-model="settings.compromise.value" @change="() => requestChangeSetting('compromise', settings.compromise.value)" />
    </div>
    <span class="latency">Latency: {{ latency }}ms</span>
  </div>
</template>

<script setup lang="ts">
import { ref, reactive, onMounted, type Ref } from 'vue';
import { NT4_Client, type TopicType } from './NT/NT4';
import { BRANCH_STATES, FACES, LEVELS, ReefState, SIDES, type Branch, type BranchState, type Face, type Level, type Side } from './ReefTypes';
import Reef from './components/Reef.vue';
import LevelChooser from './components/LevelChooser.vue';

const connected = ref(false);
const latency = ref(-1);

type SettingConfig = { name: string, type: TopicType, defaultValue: any };

const settingConfigs: SettingConfig[] = [
  {
    name: "compromise",
    type: "boolean",
    defaultValue: "false"
  }
];

settingConfigs.push({
  name: "targetL",
  type: "string",
  defaultValue: "L2"
})





const settings = settingConfigs.reduce((acc, setting) => {
  acc[setting.name] = ref(setting.defaultValue);
  return acc;
}, {} as Record<string, Ref<any>>);
const reefState = reactive<ReefState>(new ReefState());
const algaeStates = reactive<Record<Face, boolean>>(FACES.reduce((acc, face) => {
  acc[face] = false;
  return acc;
}, {} as Record<Face, boolean>));

const nt = new NT4_Client('localhost', "Dashboard", (_topic) => { }, (_topic) => { },
  (topic, _timestamp, value) => {
    if (value === undefined) return;
    if (topic.name.split('/')[3] === "Settings") {
      settings[topic.name.split('/')[4]].value = value;
      return
    }

    const x = topic.name.split('Reef/')[1].split('/');
    const face = x[0] as Face;
    if (x[1] === 'Algae') {
      algaeStates[face] = value;
    } else {
      const level = x[1] as Level;
      const side = x[2] as Side;
      reefState.map[face][level][side] = BRANCH_STATES[value];
    }
  },

  () => { console.log("Connected to NT server on localhost"); connected.value = true; },
  () => { console.log("Disconnected from NT server on localhost"); connected.value = false; },
  (latencyValue) => { latency.value = latencyValue / 1000; }
);


const topics = FACES.flatMap(face => LEVELS.flatMap(level => SIDES.map(side => `Reef/${face}/${level}/${side}`)))
  .concat(FACES.map(face => `Reef/${face}/Algae`))
  .concat(settingConfigs.map(setting => `Settings/${setting.name}`));
nt.subscribe(topics.map(topic => "/Dashboard/robot/" + topic), false);

const getTopicType = (topic: string) => {
  if (topic.split('/').pop() === 'Algae') return "boolean";
  if (topic.split('/')[0] === 'Settings') {
    const setting = settingConfigs.find(setting => setting.name === topic.split('/')[1]);
    if (setting) return setting.type;
  }
  if (topic.split('/')[0] === 'Reef') return "int";
  return "string";
}

topics.forEach((topic) => {
  console.log(`type: ${getTopicType(topic)} for topic: ${topic}`);
  nt.publishTopic("/Dashboard/dashboard/" + topic, getTopicType(topic));
});


onMounted(() => {
  nt.connect();
});

const setBranchState = (branch: Branch, state: BranchState) => {
  nt.addSample("/Dashboard/dashboard/Reef/" + branch.face + "/" + branch.level + "/" + branch.side, BRANCH_STATES.indexOf(state));
}

const setAlgaeState = (face: Face, state: boolean) => {
  nt.addSample("/Dashboard/dashboard/Reef/" + face + "/Algae", state);
};

const requestChangeSetting = (setting: string, value: any) => {
  nt.addSample("/Dashboard/dashboard/Settings/" + setting, value);
}
</script>

<style scoped>
.container.root {
  width: 100%;
  height: 100%;
  padding: 0;
  background-color: #6d6d6d;
  display: flex;
  flex-direction: column;
}

.disconnected {
  background-color: #840000 !important;
}

.layout {
  padding: 0;
  display: flex;
  flex-direction: row;
  align-items: center;
  min-height: 0;
  flex: 1;
}

.latency {
  display: block;
  margin-top: 1rem;
  bottom: 1rem;
  right: 1rem;
  font-size: 0.8rem;
  color: #999;
  position: fixed;
}

.reef{
}

.levels{
  /* margin: 20px; */
}
</style>
