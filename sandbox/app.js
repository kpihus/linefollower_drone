/*
RC_CHANNELS_OVERRIDE - set raw ch value
MANUAL_CONTROL - controlling the vehicle using standard joystick axes
*/
var mavlink = require('mavlink');
var SerialPort = require('serialport');
var serialport = new SerialPort('/dev/ttyACM0', {
  baudRate: 115200
});

/* VARIABLES */
let status = 'Connecting...'
let startMission = false;
let batteryVoltage = 0;
let batteryCurrent = 0;
let bateryEmpty_voltage = 14
/* VARIABLES */


let myMAV = new mavlink(1,1,"v1.0",["common"]);
console.log('startup')

myMAV.on("ready", function() {
  //parse incoming serial data
  serialport.on('data', function(data) {
    myMAV.parse(data);
  });



  //listen for messages
  myMAV.on("RC_CHANNELS_RAW", function(message, fields) {
    const ch8  = fields.chan8_raw;
    if(ch8 > 1700){
      startMission = true;
    }else if (ch8<1200){
      startMission = false;
    }
  });
  
  myMAV.on('SYS_STATUS', (message, fields) => {
    batteryVoltage = fields.voltage_battery;
    batteryCurrent = fields.current_battery;
  });

  myMAV.on('STATUSTEXT', (message, fields) => {
    status = fields.text;
    console.log(fields)
  });
  myMAV.on('ATTITUDE', (message, fields) => {
    status = fields.text;
    console.log(fields)
  });


  myMAV.createMessage('MAV_CMD_COMPONENT_ARM_DISARM',
    1,
    function(message) {
      serialport.write(message.buffer);
    });
});


