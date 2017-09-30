const {fork} = require('child_process');
const bunyan = require('bunyan');
const path = require('path');
var WSServer = require('ws').Server;
const express = require('express');
const app = express();
var server = require('http').createServer();

const log = bunyan.createLogger({name: 'Quad'});


const fccomm = fork('./fccomm');
fccomm.on('message', function(msg){
  console.log('CHILD MESSAGE', msg);
  switch (msg.type){
    case 'info':
      log.info({info: msg.payload});
      break;

    default:
      console.log('UNKNOWN MESSAGE', msg);
      break;
  }
});

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


//for groundstation
app.use(express.static(path.join(__dirname, '/frontend')));


var wss = new WSServer({server: server});

wss.on('connection', function (ws) {
  ws.send(JSON.stringify(process.memoryUsage()), function () { /* ignore errors */ });
  log.info('Frontend connected');
  ws.on('close', function () {
    log.info('Frontend disconnected');
    clearInterval(id);
  });
});

server.on('request', app);
server.listen(3000, ()=>{
  log.info('App is listeing on port 3000')
});

