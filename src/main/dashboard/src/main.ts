import { createApp } from 'vue'
import './style.css'
import './style.css'
import App from './App.vue'
import vueDebounce from 'vue-debounce'

createApp(App)
  .directive('debounce', vueDebounce({ lock: true }))
  .mount('#app')
