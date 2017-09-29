process.send({type: 'info', payload:'Starting communicator'});
const SerialPort = require('serialport');
const mavlink = require('../apm/mavlink');


var connection = new SerialPort('/dev/ttyACM0', {
  baudRate: 115200
});

process.on('message', msg =>{
  console.log('Message from parent', msg);
});


const EventEmitter = require('events');

class QEmitter extends EventEmitter {}

const QE = new QEmitter();

mavlinkParser = new MAVLink();

QE.on('armed', ()=>{
  console.log('Motors armed')
});


connection.on('data', function(data) {
  mavlinkParser.parseBuffer(data);
});

// Attach an event handler for any valid MAVLink message
mavlinkParser.on('message', function(message) {
  console.log('Got a message of any type!');
  console.log(message);
});

// Attach an event handler for a specific MAVLink message
// mavlinkParser.on('HEARTBEAT', function(message) {
//   console.log('Got a heartbeat message!');
//   console.log(message); // message is a HEARTBEAT message
// });

mavlinkParser.on('RC_CHANNELS_RAW', function(message) {
  const ch8 = message.chan8_raw;
  console.log('CH8', ch8); // message is a HEARTBEAT message
  if(ch8>1600){
    QE.emit('start')
  }else if(ch8<1400){
    QE.emit('stop')
  }
});
//
mavlinkParser.on('COMMAND_ACK', function(message) {
  console.log('COMMAND', message.command, 'RESULT', message.result); // message is a HEARTBEAT message
  switch(message.command){
    case 400:
      //ARM DISARM
      switch (message.result){
        case 0:
          //Armed
          QE.emit('armed');
          break;
      }
      break;
  }
});

QE.on('start', ()=>{
  console.log('START ACTION');
  armMotors(1);
});

QE.on('stop', ()=>{
  console.log('STOP ACTION');
  // armMotors(0);
});






setTimeout(function(){
  let request = new mavlink.messages.request_data_stream(1, 1, mavlink.MAV_DATA_STREAM_ALL, 1, 1);
  let p = new Buffer(request.pack(mavlinkParser));
  connection.write(p);
}, 10000);

const armMotors = (action)=>{
  let request = new mavlink.messages.command_long(1,1, mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 1);
  let p = new Buffer(request.pack(mavlinkParser));
  connection.write(p);
}

