process.send({type: 'info', payload: 'Starting communicator'});
const SerialPort = require('serialport');
const observe = require('observe');
const EventEmitter = require('events');

const mavlink = require('../apm/mavlink.js');

function logger(message){
  process.send({type: 'info', payload: message})
}

const modes = {
  'WAITING': {next: 'INIT', action: setNext},
  'INIT': {next: 'ARMING', action: init},
  'ARMING': {next: 'TAKE_OFF', action: arming},
  'TAKE_OFF': {next: 'SET_GUIDED', action: doTakeoff},
  'SET_GUIDED': {next: 'MISSION', action: setGuided},
  'MISSION': {next: 'LAND', action: mission},
  'LAND': {next: null, action: doLand}
};

const params = {
  status: {
    armed: false,
    mission_started: false,
    last_heartbeat: new Date().getTime(),
    battery_voltage: 0,
    current_mode: 'WAITING'
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
    altitude: 0,
  },
  motors: {
    motor1: 1000,
    motor2: 1000,
    motor3: 1000,
    motor4: 1000,
  }
};

const state = observe(params);

state.on('change', (change) => {
  if (change.property[0] === 'atti_c') {
    //Attitude new values to FC
  } else if (change.property[0] === 'status') {
    if (change.property[1] === 'current_mode') {
      //Mode changed
      const mode = state.subject.status.current_mode;
      logger(`Mode changed to ${mode}`)
      modes[mode].action(setNext);
    }
    //Send new status report
  } else if (change.property[0] === 'atti_r') {
    process.send({type: 'atti_r', payload: state.subject.atti_r})
    if(state.subject.atti_r.altitude > 1.3){
      QE.emit('takeoff_done');
    }
  } else if (change.property[0] === 'motors') {
    process.send({
      type: 'motors', payload: {
        motor1: state.subject.motors.motor1,
        motor2: state.subject.motors.motor2,
        motor3: state.subject.motors.motor3,
        motor4: state.subject.motors.motor4
      }
    })
  }
});

/*STATE ACTIONS */

function setNext(err) {
  if (err) {
    console.error(err)
  }
  const next = modes[state.subject.status.current_mode].next;
  if (!next) {
    process.exit(0)
  }
  state.set('status.current_mode', next);
}


function init(next) {
  console.log('INIT');
  requestDataStreams();
  QE.once('start', () => {
    next();
  });

};

function arming(next) {
  logger('Start arming motrs');
  console.log('ARMING');
  armMotors();
  QE.once('armed', () => {
    state.set('status.armed', true);
    logger('Motors armed');
    next()
  });
};

function doTakeoff(next) {
  logger('Starting takeoff')
  setTimeout(() => {
    takeOff();
  }, 2000);

  QE.once('takeoff_done', () => {
    logger('Takeoff done');
    next();
  });

}

function setGuided(next) {
  console.log('GUIDED NOGPS MODE');
  next();
}

function mission(next) {
  console.log('MISSION');
  setTimeout(() => {
    next();
  }, 20000)

}

function doLand(next) {
  logger('Landing quad');
  land();
  next();
}

/* END OF STATE ACTIONS */


const connection = new SerialPort('/dev/ttyUSB5', {
  baudRate: 57600
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


class QEmitter extends EventEmitter {
}

const QE = new QEmitter();

mavlinkParser = new MAVLink();


/**
 * Parse incoming data
 */
connection.on('data', function (data) {
  mavlinkParser.parseBuffer(data);
});

// Attach an event handler for any valid MAVLink message
mavlinkParser.on('message', (message) => {
  //console.log('Got a message of any type!');
  //console.log('===', message);
});


mavlinkParser.on('HEARTBEAT', (message) => {
  // console.log('Got a heartbeat message!');
  //console.log(message); // message is a HEARTBEAT message
  const custom_mode = message.custom_mode; //0=STABILIZE, 2=ALT HOLD, 5=LOITER, 20=GUIDED_NOGPS
  const current_state = state.subject.status.current_mode;
  if (current_state === 'WAITING') {
    setNext();
  }
});

mavlinkParser.on('STATUSTEXT', (message) => {
  process.send({type: 'info', payload: message.text.replace(/\0/g, '')});
});

mavlinkParser.on('LOG_DATA', (message) => {
  logger(message)
});
mavlinkParser.on('SET_MODE', (message) => {
  console.log('SETMODE', message)
});

mavlinkParser.on('ATTITUDE', (message) => {
  state.set('atti_r.roll', message.roll);
  state.set('atti_r.pitch', message.pitch);
  state.set('atti_r.yaw', message.yaw);
});

mavlinkParser.on('RANGEFINDER', (message) => {
  state.set('atti_r.altitude', message.distance);
});

mavlinkParser.on('SYS_STATUS', (message) => {
  state.set('status.battery_voltage', message.voltage_battery)
});

mavlinkParser.on('SERVO_OUTPUT_RAW', (message) => {
  state.set('motors.motor1', message.servo1_raw);
  state.set('motors.motor2', message.servo2_raw);
  state.set('motors.motor3', message.servo3_raw);
  state.set('motors.motor4', message.servo4_raw);
});


mavlinkParser.on('RC_CHANNELS_RAW', (message) => {
  const ch5 = message.chan5_raw;
  if (ch5 > 1800) {
    QE.emit('start')
  } else if (ch5 < 1800) {
    //QE.emit('stop')
  }
});
//
mavlinkParser.on('COMMAND_ACK', (message) => {
  logger(`COMMAND, ${message.command}, RESULT ${message.result}`)
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
          break;
      }
      break;
  }
});

function sendRequest(request) {
  let p = new Buffer(request.pack(mavlinkParser));
  connection.write(p);
}

function requestDataStreams() {
  let request = new mavlink.messages.request_data_stream(1, 1, mavlink.MAV_DATA_STREAM_ALL, 1, 1);
  let p = new Buffer(request.pack(mavlinkParser));
  connection.write(p);
}

function armMotors(action) {
  sendRequest(new mavlink.messages.command_long(1, 1, mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 1));

};

function takeOff() {
  console.log('Set mode takeoff')
  const altitude = 1.5;
  sendRequest(new mavlink.messages.command_long(1, 1, mavlink.MAV_CMD_NAV_TAKEOFF,
    0, // confirmation
    0, // param1
    0, // param2
    0, // param3
    0, // param4
    0, // param5
    0, // param6
    altitude));
}

function land() {
  console.log('Set mode land');
  const altitude = 0;
  sendRequest(new mavlink.messages.command_long(1, 1, mavlink.MAV_CMD_NAV_LAND,
    0, // confirmation
    0, // param1
    0, // param2
    0, // param3
    0, // param4
    0, // param5
    0, // param6
    altitude));
}

const setMode = () => {
  sendRequest(new mavlink.messages.command_long(1, 1, mavlink.MAV_CMD_DO_SET_MODE,
    0, // confirmation
    0, // param1
    0, // param2
    0, // param3
    0, // param4
    0, // param5
    0, // param6
    altitude));
}
