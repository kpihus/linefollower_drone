import Vue from 'vue'
import Vuex from 'vuex'

Vue.use(Vuex);

const mutations = {
  SOCKET_ONOPEN: (state, status) => {
    console.log('socket connected')
    state.connected = true
  },
  SOCKET_ONCLOSE: (state, status) => {
    console.log('socket disconnected'); // TODO: REMOVE
    state.connected = false
  },
  SOCKET_ONMESSAGE: (state, message) => {
    message = JSON.parse(message.data);
    switch (message.type){
      case 'status':
        state.status = message.payload;
        break;
      default:
        console.error('Unknown message received', message);
        break;
    }
  },
};

const getters = {
  connected: (state) => state.connected,
  status: (state) => state.status
};



export const store = new Vuex.Store({
  state: {
    connected: false,
    logs: [],
    status:{}
  },
  mutations,
  getters
});


