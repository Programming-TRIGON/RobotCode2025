<template>
  <svg viewBox="0 0 264.58333 264.58333">
    <BranchComp v-for="branch in flatReefStateMap" :key="branch.location" :points="getBranchPath(branch.location)"
      @click="toggleBranch(branch)" :status="branch.state" />
    />
    <Algae v-for="face in FACES" :key="face" :points="getAlgaePath(face)" @click="toggleAlgae(face)"
      :present="algaeStates[face]" />
  </svg>
</template>


<script setup lang="ts">
import { computed } from 'vue';
import Algae from './Algae.vue';
import BranchComp from './Branch.vue';
import { BRANCH_STATES, FACES, type Branch, type BranchState, type Face, type Level, type ReefState, type Side } from '../ReefTypes';
const svg = {
  "svg": {
    "namedview": {
      "guide": {
        "_position": "-117.07148,217.65155",
        "_orientation": "0,-1",
        "_id": "guide2",
        "_inkscape:locked": "false",
        "__prefix": "sodipodi"
      },
      "_id": "namedview1",
      "_pagecolor": "#ffffff",
      "_bordercolor": "#000000",
      "_borderopacity": "0.25",
      "_inkscape:showpageshadow": "2",
      "_inkscape:pageopacity": "0.0",
      "_inkscape:pagecheckerboard": "0",
      "_inkscape:deskcolor": "#d1d1d1",
      "_inkscape:document-units": "mm",
      "_showguides": "true",
      "_inkscape:zoom": "1",
      "_inkscape:cx": "500",
      "_inkscape:cy": "500.5",
      "_inkscape:window-width": "1920",
      "_inkscape:window-height": "1001",
      "_inkscape:window-x": "-9",
      "_inkscape:window-y": "-9",
      "_inkscape:window-maximized": "1",
      "_inkscape:current-layer": "layer2",
      "__prefix": "sodipodi"
    },
    "defs": {
      "_id": "defs1"
    },
    "g": [
      {
        "_inkscape:label": "Layer 1",
        "_inkscape:groupmode": "layer",
        "_id": "layer1"
      },
      {
        "path": [
          {
            "_d": "M 33.302877,75.484281 53.059294,86.784884 79.400838,41.159762 66.299312,18.676896 Z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.277851",
            "_id": "path151",
            "_inkscape:label": "2CL"
          },
          {
            "_d": "M 0.30644124,132.29167 H 26.785962 L 53.059294,86.784884 33.302877,75.484281 Z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.277851",
            "_id": "path150",
            "_inkscape:label": "2CR"
          },
          {
            "_d": "M 33.302877,189.09905 53.059294,177.79845 26.785962,132.29167 H 0.30644124 Z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.277851",
            "_id": "path149",
            "_inkscape:label": "2BL"
          },
          {
            "_d": "M 66.298796,245.90644 79.400838,223.42306 53.059294,177.79845 33.302877,189.09905 Z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.277851",
            "_id": "path148",
            "_inkscape:label": "2BR"
          },
          {
            "_d": "M 132.29167,245.90644 V 223.66232 H 79.538814 l -0.137976,-0.23926 -13.102042,22.48338 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.277851",
            "_id": "path147",
            "_inkscape:label": "2AL"
          },
          {
            "_d": "m 198.28402,245.90644 -13.10152,-22.48287 -0.13798,0.23875 h -52.75285 v 22.24412 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.277851",
            "_id": "path146",
            "_inkscape:label": "2AR"
          },
          {
            "_d": "m 231.28046,189.09905 -19.75642,-11.3006 -26.34154,45.62512 13.10152,22.48287 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.277851",
            "_id": "path145",
            "_inkscape:label": "2FL"
          },
          {
            "_d": "m 264.27689,132.29167 h -26.47952 l -26.27333,45.50678 19.75642,11.3006 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.277851",
            "_id": "path144",
            "_inkscape:label": "2FR"
          },
          {
            "_d": "m 211.52404,86.784884 26.27333,45.506786 h 26.47952 L 231.28046,75.484281 Z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.277851",
            "_id": "path143",
            "_inkscape:label": "2EL"
          },
          {
            "_d": "M 211.52404,86.784884 231.28046,75.484281 198.28454,18.676896 185.1825,41.160278 Z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.277851",
            "_id": "path142",
            "_inkscape:label": "2ER"
          },
          {
            "_d": "M 132.29167,18.676896 H 66.299312 l 13.101526,22.482866 0.137976,-0.238746 h 52.752856 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.277851",
            "_id": "path131",
            "_inkscape:label": "2DR"
          },
          {
            "_d": "m 132.29167,18.676896 v 22.24412 h 52.75285 l 0.13798,0.239262 13.10204,-22.483382 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.277851",
            "_id": "path130",
            "_inkscape:label": "2DL"
          },
          {
            "_d": "m 189.6246,165.27105 21.89944,12.5274 26.27333,-45.50678 H 208.6653 Z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.370468",
            "_id": "path171",
            "_inkscape:label": "3FR"
          },
          {
            "_d": "m 170.54876,198.31141 14.63374,25.11216 26.34154,-45.62512 -21.89944,-12.5274 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.370468",
            "_id": "path169",
            "_inkscape:label": "3FL"
          },
          {
            "_d": "m 132.29167,198.43285 v 25.22947 h 52.75285 l 0.13798,-0.23875 -14.63374,-25.11216 -0.0703,0.12144 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.370468",
            "_id": "path167",
            "_inkscape:label": "3AR"
          },
          {
            "_d": "M 26.785962,132.29167 H 55.918034 L 74.958732,99.312284 53.059294,86.784884 Z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.370468",
            "_id": "path163",
            "_inkscape:label": "3CR"
          },
          {
            "_d": "M 53.059294,177.79845 74.958732,165.27105 55.918034,132.29167 H 26.785962 Z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.370468",
            "_id": "path162",
            "_inkscape:label": "3BL"
          },
          {
            "_d": "m 94.03457,198.31141 -19.075838,-33.04036 -21.899438,12.5274 26.341544,45.62461 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.370468",
            "_id": "path161",
            "_inkscape:label": "3BR"
          },
          {
            "_d": "m 94.03457,198.31141 -14.633732,25.11165 0.137976,0.23926 H 132.29167 V 198.43285 H 94.10485 Z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.370468",
            "_id": "path160",
            "_inkscape:label": "3AL"
          },
          {
            "_d": "M 79.400838,41.159762 53.059294,86.784884 74.958732,99.312284 94.03457,66.271924 Z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.370468",
            "_id": "path152",
            "_inkscape:label": "3CL"
          },
          {
            "_d": "m 189.6246,99.312284 19.0407,32.979386 h 29.13207 L 211.52404,86.784884 Z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.370468",
            "_id": "path141",
            "_inkscape:label": "3EL"
          },
          {
            "_d": "M 189.6246,99.312284 211.52404,86.784884 185.1825,41.160278 170.54876,66.271924 Z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.370468",
            "_id": "path140",
            "_inkscape:label": "3ER"
          },
          {
            "_d": "M 132.29167,40.921016 H 79.538814 l -0.137976,0.238746 14.633732,25.112162 0.07028,-0.12144 h 38.18682 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.370468",
            "_id": "path133",
            "_inkscape:label": "3DR"
          },
          {
            "_d": "m 132.29167,40.921016 v 25.229468 h 38.18681 l 0.0703,0.12144 14.63374,-25.111646 -0.13798,-0.239262 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.370468",
            "_id": "path132",
            "_inkscape:label": "3DL"
          },
          {
            "_d": "m 189.6246,165.27105 19.0407,-32.97938 h -30.28601 l -11.52229,19.9564 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.555701",
            "_id": "path170",
            "_inkscape:label": "4FR"
          },
          {
            "_d": "M 170.54876,198.31141 189.6246,165.27105 166.857,152.24807 155.33522,172.20448 Z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.555701",
            "_id": "path168",
            "_inkscape:label": "4FL"
          },
          {
            "_d": "m 132.29167,198.43285 h 38.18681 l 0.0703,-0.12144 -15.21354,-26.10693 h -23.04355 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.555701",
            "_id": "path166",
            "_inkscape:label": "4AR"
          },
          {
            "_d": "M 86.204041,132.29167 H 55.918034 l 19.040698,32.97938 22.767603,-13.02298 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.555701",
            "_id": "path165",
            "_inkscape:label": "4BL"
          },
          {
            "_d": "M 74.958732,99.312284 55.918034,132.29167 h 30.286007 l 11.522294,-19.95641 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.555701",
            "_id": "path164",
            "_inkscape:label": "4CR"
          },
          {
            "_d": "M 109.24811,172.20448 97.726335,152.24807 74.958732,165.27105 94.03457,198.31141 Z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.555701",
            "_id": "path159",
            "_inkscape:label": "4BR"
          },
          {
            "_d": "m 109.24811,172.20448 -15.21354,26.10693 0.07028,0.12144 h 38.18682 v -26.22837 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.555701",
            "_id": "path158",
            "_inkscape:label": "4AL"
          },
          {
            "_d": "M 94.03457,66.271924 74.958732,99.312284 97.726335,112.33526 109.24811,92.378857 Z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.555701",
            "_id": "path153",
            "_inkscape:label": "4CL"
          },
          {
            "_d": "m 166.857,112.33526 11.52229,19.95641 H 208.6653 L 189.6246,99.312284 Z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.555701",
            "_id": "path139",
            "_inkscape:label": "4EL"
          },
          {
            "_d": "M 155.33522,92.378857 166.857,112.33526 189.6246,99.312284 170.54876,66.271924 Z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.555701",
            "_id": "path137",
            "_inkscape:label": "4ER"
          },
          {
            "_d": "M 132.29167,66.150484 H 94.10485 l -0.07028,0.12144 15.21354,26.106933 h 23.04356 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.555701",
            "_id": "path135",
            "_inkscape:label": "4DR"
          },
          {
            "_d": "m 132.29167,66.150484 v 26.228373 h 23.04355 l 15.21354,-26.106933 -0.0703,-0.12144 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.555701",
            "_id": "path134",
            "_inkscape:label": "4DL"
          },
          {
            "_d": "M 132.29167,132.29167 H 86.204041 l 23.044069,39.91281 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.555701",
            "_id": "path157",
            "_inkscape:label": "algb"
          },
          {
            "_d": "m 132.29167,132.29167 -23.04356,39.91281 h 46.08711 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.555701",
            "_id": "path156",
            "_inkscape:label": "alga"
          },
          {
            "_d": "m 132.29167,132.29167 23.04355,39.91281 23.04407,-39.91281 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.555701",
            "_id": "path155",
            "_inkscape:label": "algf"
          },
          {
            "_d": "M 109.24811,92.378857 86.204041,132.29167 h 46.087629 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.555701",
            "_id": "path154",
            "_inkscape:label": "algc"
          },
          {
            "_d": "m 155.33522,92.378857 -23.04355,39.912813 h 46.08762 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.555701",
            "_id": "path138",
            "_inkscape:label": "alge"
          },
          {
            "_d": "m 132.29167,92.378857 h -23.04356 l 23.04356,39.912813 23.04355,-39.912813 z",
            "_style": "fill:none;stroke:#000000;stroke-width:0.555701",
            "_id": "path136",
            "_inkscape:label": "algd"
          }
        ],
        "_inkscape:groupmode": "layer",
        "_id": "layer2",
        "_inkscape:label": "Layer 2"
      }
    ],
    "_xmlns:inkscape": "http://www.inkscape.org/namespaces/inkscape",
    "_xmlns:sodipodi": "http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd",
    "_xmlns": "http://www.w3.org/2000/svg",
    "_xmlns:svg": "http://www.w3.org/2000/svg",
    "_width": "1000",
    "_height": "1000",
    "_viewBox": "0 0 264.58333 264.58333",
    "_version": "1.1",
    "_id": "svg1",
    "_inkscape:version": "1.4 (86a8ad7, 2024-10-11)",
    "_sodipodi:docname": "3LREEF.svg"
  }
}

const props = defineProps<{
  setBranchState: (Branch: Branch, state: BranchState) => void,
  setAlgaeState: (face: Face, state: boolean) => void,
  reefState: ReefState,
  algaeStates: Record<Face, boolean>,
}>();

const getBranchPath = (path: string): string =>
  (svg.svg.g[1].path?.find(p => p['_inkscape:label'] === path)?._d)!;
const getAlgaePath = (face: Face): string =>
  (svg.svg.g[1].path?.find(p => p['_inkscape:label'] === `alg${face}`.toLowerCase())?._d)!;

const toggleBranch = (branch: Branch) => {
  const newState = BRANCH_STATES[(BRANCH_STATES.indexOf(props.reefState.map[branch.face][branch.level][branch.side]) + 1) % BRANCH_STATES.length];
  props.setBranchState(branch, newState);
};

const toggleAlgae = (face: Face) => {
  props.setAlgaeState(face, !props.algaeStates[face]);
}

const flatReefStateMap = computed(() => Object.entries(props.reefState.map).flatMap(([face, levels]): { location: string, state: BranchState, face: Face, level: Level, side: Side }[] => {
  return Object.entries(levels).flatMap(([level, sides]) => {
    return Object.entries(sides).map(([side, state]) => {
      return { location: level[1] + face + side[0], state, face: face as Face, level: level as Level, side: side as Side };
    });
  });
}));
</script>

<style scoped>
svg {
  max-width: 100%;
  max-height: 100%;
}
</style>
