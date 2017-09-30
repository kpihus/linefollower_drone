import Vue from 'vue'
import App from './App.vue'
import BootstrapVue from 'bootstrap-vue';
import VueNativeSock from 'vue-native-websocket'
import 'bootstrap/dist/css/bootstrap.css';
import 'bootstrap-vue/dist/bootstrap-vue';
import {store} from './store';
import {backend} from './config';


Vue.use(BootstrapVue);
Vue.use(VueNativeSock, backend.uri, {store});

new Vue({
  el: '#app',
  store,
  render: h => h(App)
})
