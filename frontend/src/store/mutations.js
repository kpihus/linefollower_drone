export default {
  SOCKET_ONOPEN: (state, status) => {
    console.log('socket connected')
    state.connected = true
  },
  SOCKET_ONCLOSE: (state, status) => {
    console.log('socket disconnected'); // TODO: REMOVE
    state.connected = false
  },
  SOCKET_ONMESSAGE: (state, message) => {
    console.log('message', message); // TODO: REMOVE
  },
}