import Vue from 'vue'
import Vuex from 'vuex'

import mutations from './mutations'

Vue.use(Vuex);

export const store = new Vuex.Store({
  state: {
    connected: false,
    logs: []
  },
  mutations
});
