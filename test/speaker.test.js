const speaker = require('../speak');

describe('Speaker', ()=>{

  it('Shold speak', (done)=>{
    const speak = new speaker();
    speak.setMessage('First message');
    speak.setMessage('second message');
    speak.setMessage('third message');
    setTimeout(()=>{
      done();
    },10000)
  })
});