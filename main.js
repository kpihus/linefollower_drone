const {fork} = require('child_process');
const bunyan = require('bunyan');
const path = require('path');
const WebSocket = require('ws');
const WSServer = WebSocket.Server;
const express = require('express');
const app = express();
const server = require('http').createServer();

const log = bunyan.createLogger({
  name: 'Quad',
});


/*
const vision = fork('./vision');

vision.on('message', (msg) =>{
  switch (msg.type){
    case 'info':
      log.info({info: msg.payload});
      break;

    default:
      console.log('UNKNOWN MESSAGE', msg);
      break;
  }
});

*/


//for groundstation


app.use(express.static(path.join(__dirname, '/frontend')));


var wss = new WSServer({server: server});

const sendWsm = (payload) => {
  wss.clients.forEach(function each(client) {
    if (client.readyState === WebSocket.OPEN) {
      client.send(JSON.stringify(payload));
    }
  });
}

wss.on('connection', function (ws) {
  ws.send(JSON.stringify(process.memoryUsage()), function () { /* ignore errors */
  });
  log.info('Frontend connected');
  ws.on('log', (req) => {
    console.log(req.data);
  });
  ws.on('close', function () {
    log.info('Frontend disconnected');
  });
});

// setInterval(()=>{
//   sendWsm({message: "Hello world"});
// }, 1000);

server.on('request', app);
server.listen(3000, () => {
  log.info('App is listeing on port 3000')
});

const fccomm = fork('./fccomm');
fccomm.on('message', function (msg) {
  // console.log('CHILD MESSAGE', msg);
  switch (msg.type) {
    case 'info':
      log.info({info: msg.payload});
      break;

    case 'status':
      log.info({status: msg.payload});
      sendWsm({
        type: 'status',
        payload: msg.payload
      });
      break;
    case 'atti_r':
      sendWsm({
        type: 'atti_r',
        payload: msg.payload
      });
      break;
    case 'motors':
      sendWsm({
        type: 'motors',
        payload: msg.payload
      });
      break;

    default:
      console.log('UNKNOWN MESSAGE', msg);
      break;
  }
});



