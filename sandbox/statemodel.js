const observe = require('observe');
const EventEmitter = require('events');

class QEmitter extends EventEmitter {}

const QE = new QEmitter();


const modes = {
  'WAITING': {next: 'INIT', action: setNext},
  'INIT': {next: 'ARMING', action: init},
  'ARMING': {next: 'TAKE_OFF', action: arming},
  'TAKE_OFF': {next: 'MISSION', action: takeOff},
  'MISSION': {next: 'LAND', action: mission},
  'LAND': {next: null, action: land}
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
  }
};


const state = observe(params);


state.on('change', (change) => {
  if (change.property[0] === 'atti_c') {
    //Attitude new values to FC
  }else if (change.property[0] === 'status'){
    if(change.property[1] ==='current_mode'){
      //Mode changed
      const mode = state.subject.status.current_mode;
       modes[mode].action(setNext);
    }
    //Send new status report
    console.log('STATUS CHANGE')
  }
});

/*STATE ACTIONS */

function setNext(err){
  if(err){
    console.error(err)
  }
  const next = modes[state.subject.status.current_mode].next;
  if(!next){
    process.exit(0)
  }
  state.set('status.current_mode', next);
}


function init (next) {
  console.log('INIT');

  next();
};

function arming (next) {
  console.log('ARMING');
  next()
};

function takeOff(next){
  console.log('TAKEOFF');
  next();
}

function mission(next){
  console.log('MISSION');
  next();
}

function land(next){
  console.log('LAND');
  next();
}

/* END OF STATE ACTIONS */

state.set('status.current_mode', 'INIT');