const {fork} = require('child_process');
const bunyan = require('bunyan');
const express = require('express');
const app = express();

app.use(express.static('public'));

const fccomm = fork('./fccomm');


const log = bunyan.createLogger({name: 'Quad'});




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


app.listen(3000, ()=>{
  log.info('App is listeing on port 3000')
});