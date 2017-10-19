const say = require('say');

class Speak {
  constructor(){
    this.messages = [];
  }

  setMessage(message){
    this.messages.push(message);
  }
  getMessage(){
    return this.messages.shift();
  }

  speak(message){
    return new Promise((resolve, reject)=>{
      say.speak(message, (err) => {
        if(err){
          return reject(err);
        }
        return resolve();
      })
    })

  }

}


module.exports = Speak;