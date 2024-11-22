<template>
  <div class="button-child">
    <ControlButton
      v-for="(item, index) in buttonsDetail"
      :key="index"
      :attr="item"
      :style="{width:buttonWidth}"
    />
  </div>
</template>

<script>
import ControlButton from '@/components/controll/child/ControlButton'

export default ({
  name: 'ButtonsChild',
  components: {
    ControlButton
  },
  props: {
    buttons: Array,
    cols: Number
  },
  data: () => ({
    buttonsDetail: null,
    buttonsState: []
  }),
  computed: {
    buttonWidth () {
      if (this.cols === 3) {
        return '30%'
      } else if (this.cols === 4) {
        return '23%'
      } else if (this.cols === 1) {
        return '100%'
      }
      return '30%'
    }
  },
  mounted () {
    const temp = []
    for (const i in this.buttons) {
      const args = this.buttons[i]
      const button = {
        isIcon: false,
        content: '', // 显示内容
        sendContent: undefined, // 发送的 websocket 内容
        sendKey: undefined, // 对应按下（和弹起）的按键
        upSendContent: undefined, // 按钮弹起时发送的 websocket 内容
        reversSendContent: undefined // 按钮是否是开关型的，以及关闭时发送的 ws 内容
      }

      if (args === '') {
        temp.push(button)
        continue
      }

      let jtemp = 0
      for (const j in button) {
        button[j] = args[jtemp]
        jtemp++
      }

      temp.push(button)
    }
    this.buttonsDetail = temp
  }
})
</script>

<style lang="stylus" scoped>
  @import '~@css/mixins.styl'
  @import '~@css/varibles.styl'
  .button-child
    display flex
    flex-wrap wrap
    justify-content space-between
    width 100%
    .buttons
      margin-top .5rem
</style>
