const {fork} = require('child_process');
const bunyan = require('bunyan');
const express = require('express');
const app = express();
const expressWs = require('express-ws')(app);

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

app.ws('/', function(ws, req) {
  ws.on('message', function(msg) {
    console.log(msg);
  });
});

const socketInstances = expressWs.get('/');

const sendMessage = (data) =>{
  socketInstances.clients.forEach(client => {
    client.send(data);
  })
};

app.use(express.static('frontend'));
app.listen(3000, ()=>{
  log.info('App is listeing on port 3000')
});

