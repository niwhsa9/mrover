<template>
  <div class='wrap'>
    <h2>SA Arm Controls</h2>
    <div class='controls-flex'>
      <h4>Arm Mode</h4>
      <div class='form-check'>
        <input
          v-model='mode'
          class='form-check-input'
          type='radio'
          id='disabled'
          value='disabled'
        />
        <label class='form-check-label' for='disabled'>Disabled</label>
      </div>
      <div class='form-check'>
        <input v-model='mode' class='form-check-input' type='radio' id='thr' value='throttle' />
        <label class='form-check-label' for='thr'>Throttle</label>
      </div>
    </div>
    <div class="controls-flex">
      <button class="btn btn-primary" @click="zero('sa_z', 0)">Zero Ground</button>
      <p>Corer Position: {{ corer_position }} inches</p>
      <button class="btn btn-primary" @click="zero('sa_z', corer_position+sensor_height)">Zero Sensor</button>
      <p>Plunger Position: {{ plunger_position-plunger_height }} inches</p>
      
      <!--      <MotorAdjust-->
      <!--        v-if="mode == 'position'"-->
      <!--        :options="[-->
      <!--          { esw_name: 'sa_x', display_name: 'X' },-->
      <!--          { esw_name: 'sa_y', display_name: 'Y' },-->
      <!--          { esw_name: 'sa_z', display_name: 'Z' },-->
      <!--          { esw_name: 'sampler', display_name: 'Sampler' },-->
      <!--          { esw_name: 'sensor_actuator', display_name: 'Sensor Actuator' }-->
      <!--        ]"-->
      <!--      />-->
    </div>
  </div>
</template>

<script lang='ts'>
import { defineComponent } from 'vue'
import { mapActions, mapState } from 'vuex'
import MotorAdjust from './MotorAdjust.vue'

const UPDATE_HZ = 40

export default defineComponent({
  components: {
    MotorAdjust
  },
  data() {
    return {
      mode: 'disabled',
      corer_position: 0,
      plunger_position: 0,
      sensor_height: 5.36,
      plunger_height: 5.5
    }
  },

  created: function() {
    this.interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads()
      const gamepad = gamepads.find(gamepad => gamepad && gamepad.id.includes('Microsoft'))
      if (!gamepad) return

      this.sendMessage({
        type: 'sa_controller',
        axes: gamepad.axes,
        buttons: gamepad.buttons.map(button => button.value)
      })

      this.sendMessage({
        type: 'sa_mode',
        mode: this.mode
      })
    }, 1000 / UPDATE_HZ)
  },

  beforeUnmount() {
    window.clearInterval(this.interval)
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    zero: function(name: string, value: number) {
      this.sendMessage({type: "arm_adjust", name: name, value: value})
    }
  }
})
</script>

<style scoped>
.wrap {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-items: center;
  width: 100%;
  height: auto;
}

.wrap h2 h4 {
  margin: 0;
  font-size: 1.5em;
  font-weight: bold;
  text-align: center;
  width: 100%;
  padding: 5px 0;
}

.controls-flex {
  flex-wrap: wrap;
  display: flex;
  align-items: center;
  width: 100%;
  column-gap: 20px;
  padding-left: 10px;
  margin-bottom: 5px;
  margin-top: 5px;
}

</style>
