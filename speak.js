const say = require('say');


class Speak {
  constructor() {
    this.messages = [];
    this.talking = false;
  }

  setMessage(message) {
    this.messages.push(message);
    if (!this.talking) {
      this.say();
    }
  }

  getMessage() {
    return this.messages.shift();
  }

  say() {
    this.talking = true;
    say.speak(this.getMessage(), null, 1.3, (err) => {
      if (err) {
        //Cant speak ? Tough luck...
      }
      if (this.messages.length > 0) {
        this.say();
      }else{
        this.talking= false;
      }
    });
  };

}


module.exports = Speak;