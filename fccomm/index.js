process.send({type: 'info', payload: 'Starting communicator'});
const SerialPort = require('serialport');
const observe = require('observe');
const EventEmitter = require('events');

const MAVLink = require('../apm/mavlink');

const modes = {
  'INIT': {next: 'READY'},
  'READY': {next: 'ARMING'},
  'ARMING': {next: 'ARMED'},
  'ARMED': {next: 'TAKE_OFF'},
  'TAKE_OFF': {next: 'MISSION'},
  'MISSION': {next: 'FINISH'},
  'FINISH': {next: 'LAND'},
  'LAND': {next: null}
};

const params = {
  status: {
    armed: false,
    mission_started: false,
    last_heartbeat: new Date().getTime(),
    battery_voltage: 0,
    current_mode: 'N/A'
  },
  atti_c: { //Calculated attitude
    roll: 0,
    pitch: 0,
    yaw: 0,
  },
  atti_r: { //Real attitude
    roll: 0,
    pitch: 0,
    yaw: 0,
  }
};

const state = observe(params);

state.on('change', (change) => {
  if(change.property[0] ==='atti_c'){
    //Attitude new values to FC
  }
});


var connection = new SerialPort('/dev/ttyACM0', {
  baudRate: 115200
});

//Handle parent messages
process.on('message', msg => {
  switch (msg.type) {
    case 'atti':

      break;
    default:
      console.log('unknown message received');
      break;
  }
  console.log('Message from parent', msg);
});


class QEmitter extends EventEmitter {}

const QE = new QEmitter();

mavlinkParser = new MAVLink();

QE.on('armed', () => {
  state.set('status.armed', true);
  process.send({type: 'info', payload: 'Motors armed'})
});



connection.on('data', function (data) {
  mavlinkParser.parseBuffer(data);
});

// Attach an event handler for any valid MAVLink message
mavlinkParser.on('message', function (message) {
  console.log('Got a message of any type!');
  console.log(message);
});


mavlinkParser.on('HEARTBEAT', function (message) {
  console.log('Got a heartbeat message!');
  console.log(message); // message is a HEARTBEAT message
});


mavlinkParser.on('RC_CHANNELS_RAW', function (message) {
  const ch8 = message.chan8_raw;
  console.log('CH8', ch8); // message is a HEARTBEAT message
  if (ch8 > 1600) {
    QE.emit('start')
  } else if (ch8 < 1400) {
    QE.emit('stop')
  }
});
//
mavlinkParser.on('COMMAND_ACK', function (message) {
  console.log('COMMAND', message.command, 'RESULT', message.result); // message is a HEARTBEAT message
  switch (message.command) {
    case 400: //ARM DISARM
      switch (message.result) {
        case 0: //Command success
          QE.emit('armed');
          break;
      }
      break;
    case 22: //TAKEOFF
      switch (message.result) {
        case 0: //Command success
          //TODO
          break;
      }
      break;
  }
});

QE.on('start', () => {
  process.send({type: 'info', payload: 'Arming motors'});
  armMotors(1); //Do arm
});

QE.on('stop', () => {
  process.send({type: 'info', payload: 'Disarming motors'});
  armMotors(0); //Do disarm
});


setTimeout(function () {
  let request = new mavlink.messages.request_data_stream(1, 1, mavlink.MAV_DATA_STREAM_ALL, 1, 1);
  let p = new Buffer(request.pack(mavlinkParser));
  connection.write(p);
}, 10000);

const armMotors = (action) => {
  let request = new mavlink.messages.command_long(1, 1, mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 1);
  let p = new Buffer(request.pack(mavlinkParser));
  connection.write(p);
};

const takeOff = () => {
  let request = new mavlink.messages.command_long(1, 1, mavlink.MAV_CMD_NAV_TAKEOFF, 1, 1); //TODO
  let p = new Buffer(request.pack(mavlinkParser));
  connection.write(p);
}

