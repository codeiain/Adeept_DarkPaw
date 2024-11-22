<template>
  <v-btn
    :class="buttonClass"
    :style="buttonStyle"
    small
    retain-focus-on-click
    @mousedown="handleMouseDown"
    @mouseup="handleMouseUp"
    @touchstart="handleMouseDown"
    @touchend="handleMouseUp"
  >
    <v-icon v-if="attr.isIcon">{{attr.content}}</v-icon>
    <span class="texts" v-else>{{attr.content}}</span>
  </v-btn>
</template>

<script>
import { mapActions, mapMutations } from 'vuex'

export default ({
  name: 'OneClickButton',
  props: {
    attr: Object
  },
  data: () => ({
    buttonClass: ['buttons', 'clickable']
  }),
  computed: {
    buttonStyle () {
      return {
        opacity: this.attr.content === '' ? 0 : ''
      }
    }
  },
  methods: {
    handleMouseDown (sendContent, reversSendContent) {
      this.changeWsContent(this.attr.sendContent)
    },
    handleMouseUp () {
      this.changeWsContent(this.attr.upSendContent)
    },
    ...mapActions(['changeWsContent']),
    ...mapMutations(['setKeyEvent'])
  },
  mounted () {
    if (this.attr === '') {
      this.attr = {
        isIcon: false,
        content: ''
      }
    }

    this.setKeyEvent([this.attr.sendKey, () => { this.changeWsContent(this.attr.sendContent) }, 'down'])
    this.setKeyEvent([this.attr.sendKey, () => { this.changeWsContent(this.attr.upSendContent) }, 'up'])
  }
})
</script>

<style lang="stylus" scoped>
  @import '~@css/mixins.styl'
  @import '~@css/varibles.styl'
  .buttons
    margin-top .5rem
    min-width 0 !important
  .clickable
    pass
  .action
    // background-color #aaa !important
    // box-shadow 0 0 0
    // transform translate(.05rem,.05rem)
</style>
