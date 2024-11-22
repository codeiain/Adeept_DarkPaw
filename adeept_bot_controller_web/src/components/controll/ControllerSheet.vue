<template>
  <v-sheet class="mod-sheet">
    <p class="mod-title">{{modName}}</p>
    <div class="mod-wrapper">
      <slot></slot>
    </div>
  </v-sheet>
</template>

<script>
export default ({
  name: 'ControllerSheet',
  props: {
    modName: {
      type: String,
      default: 'No Mod Name'
    }
  }
})
</script>

<style lang="stylus" scoped>
  @import '~@css/mixins.styl'
  @import '~@css/varibles.styl'

  .mod-wrapper >>> .v-input__slider
    margin-bottom -.5rem
    .v-label
      border-radius .3rem
      background-color #f0f0f0
      padding 0 .3rem
      margin 0
    .v-slider
      margin 0 .8rem
      .v-slider__track-fill // 滑槽
        background-color $default_bg_color !important
      .v-slider__track-background // 滑动条
        background-color $default_reversbg_color !important
      .v-slider__thumb // 拖拽点
        background-color #fafafa !important
        &:before // 拖拽点光晕
          background-color #fafafa !important
      .v-slider__thumb-label // 拖拽点标签
        color #aaa !important
        background-color #EDEDED !important

  .mod-sheet
    // overflow hidden
    border-radius .4rem .4rem
    box-shadow 0rem 0rem .7rem .25rem rgba(255,255,255,0.7)
    .mod-title
      border-radius .4rem .4rem 0 0
      padding .2rem 1rem 0 .8rem
      font-weight 500
      margin 0
      background-color $default_comp_titlecolor
      ellpsis()
    .mod-wrapper
      border-radius 0 0 .4rem .4rem
      width 100%
      padding 0.1px 1rem .6rem 1rem // 加个0.1px上内边距防止外边距融合
      background-color $default_comp_bgcolor
</style>
