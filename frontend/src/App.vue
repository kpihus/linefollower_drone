<template>
    <div id="app">
        <div class="container-fluid">
            <div class="row">

                <div class="col-md-2">
          <span class="badge" v-bind:class="{'badge-success': alive, 'badge-danger': !alive}">
                            <i class="fa fa-plug" aria-hidden="true"></i>
          </span>
                </div>

                <div class="col-md-2">
          <span class="badge"
                v-bind:class="{'badge-success': status.battery_voltage > 14000, 'badge-danger': status.battery_voltage < 14000}">
                            <i class="fa fa-battery-half" aria-hidden="true">{{status.battery_voltage / 1000}} V</i>
          </span>
                </div>

                <div class="col-md-2">
          <span class="badge"
                v-bind:class="{'badge-success': status.mission_started, 'badge-danger': !status.mission_started}">
                            <i class="fa fa-rocket" aria-hidden="true">{{status.current_mode}}</i>
          </span>
                </div>


                <div class="col-md-2">
                              <span class="badge"
                                    v-bind:class="{'badge-success': !status.armed, 'badge-danger': status.armed}">
                            <i class="fa fa-bolt" aria-hidden="true"></i>
          </span>
                </div>
                <div class="col-md-2"></div>
                <div class="col-md-2"></div>

            </div>
            <div class="row">
                <div class="col-md-4">
                    <span>ROLL: {{atti.roll}}</span>
                </div>
                <div class="col-md-4">
                    <span>PITCH: {{atti.pitch}}</span>
                </div>
                <div class="col-md-4">
                    <span>YAW: {{atti.yaw}}</span>
                </div>
            </div>
            <div class="row">
                <div class="col-md-3">
                    <span>MOTOR1: {{motors.motor1}}</span>
                </div>
                <div class="col-md-3">
                    <span>MOTOR2: {{motors.motor2}}</span>
                </div>
                <div class="col-md-3">
                    <span>MOTOR3: {{motors.motor3}}</span>
                </div>
                <div class="col-md-3">
                    <span>MOTOR4: {{motors.motor4}}</span>
                </div>
            </div>
        </div>
    </div>
</template>

<script>
  export default {
    name: 'app',
    data() {
      return {
        msg: 'Welcome to Your Vue.js App'
      }
    },
    computed: {
      alive() {
        return this.$store.getters.connected;
      },
      status() {
        return this.$store.getters.status;
      },
      atti(){
        return this.$store.getters.atti;
      },
      motors(){
        return this.$store.getters.motors;
      }
    }
  }
</script>

<style>
    .badge {
        font-size: 24px;
        margin-right: 2px;
    }
</style>
