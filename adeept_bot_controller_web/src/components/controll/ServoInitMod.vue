<template>
  <div class="wrapper">
    <v-text-field
      label="PWM"
      type="number"
      class="pwm-inputer"
      maxlength="2"
      v-model="PWMNum"
      placeholder="Num Requier"
      outlined
      hide-details
      dense
      width='.2rem'
    >
    </v-text-field>
    <div class="button-wrapper">
      <ControlButton
        v-for="(item, index) in buttons"
        :key="index"
        class="buttons"
        :style="item.style"
        :attr="item.attr"
      />
    </div>
  </div>
</template>

<script>
import ControlButton from '@/components/controll/child/ControlButton.vue'

export default ({
  name: 'ServoInitMod',
  components: {
    ControlButton
  },
  data: () => ({
    PWMNum: 0,
    buttons: [
      {
        style: { width: '15%' },
        attr: {
          isIcon: false,
          content: '<',
          sendContent: 'SiLeft 0'
        }
      }, {
        style: { width: '40%' },
        attr: {
          isIcon: false,
          content: 'setPWM',
          sendContent: 'PWMMS 0'
        }
      }, {
        style: { width: '15%' },
        attr: {
          isIcon: false,
          content: '>',
          sendContent: 'SiRight 0'
        }
      }, {
        style: { width: '14%' },
        attr: {
          isIcon: true,
          content: 'mdi-cog-counterclockwise',
          sendContent: 'PWMD'
        }
      }
    ],
    cols: 3
  }),
  watch: {
    PWMNum () {
      console.log(this.PWMNum)
      if (this.PWMNum < 0 || this.PWMNum > 15) {
        console.log('change')
        setTimeout(() => {
          this.PWMNum = 0
        }, 1)
      }

      for (const i in this.buttons) {
        let content = this.buttons[i].attr.sendContent
        if (content === 'PWMINIT') { return }
        content = content.split(' ')[0] + ' ' + this.PWMNum
        this.buttons[i].attr.sendContent = content
      }
    }
  }
})
</script>

<style lang="stylus" scoped>
  .wrapper
    display flex
    justify-content center
    align-items center
    padding-top .8rem
    .pwm-inputer
      width 30%
    .button-wrapper
      width 70%
      .buttons
        width 70%
        margin -.5rem 0 0 4%
</style>
