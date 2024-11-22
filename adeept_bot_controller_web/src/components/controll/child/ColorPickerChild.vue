<template>
  <div class="color-picker-child">
    <v-color-picker
      style="background-color: #EEEEEE;"
      hide-mode-switch
      hide-canvas
      show-swatches
      :swatches="swatches"
      class="ma-2"
      v-model="selectedColor"
    >
    </v-color-picker>
  </div>
</template>

<script>
export default ({
  name: '',
  props: {
    witchColor: String,
    defaultColor: {
      type: String,
      default: '#000000'
    }
  },
  data: () => ({
    swatches: [
      ['#FFFFFF', '#FF0000', '#AA0000'],
      ['#CCCCCC', '#FFFF00', '#AAAA00'],
      ['#999999', '#00FF00', '#00AA00'],
      ['#666666', '#00FFFF', '#00AAAA'],
      ['#333333', '#0000FF', '#0000AA'],
      ['#000000', '#FF00FF', '#AA00AA']
    ],
    selectedColor: '#000000',
    timmer: null // 限制发出颜色改变事件的频率
  }),
  watch: {
    selectedColor: {
      handler (newName, oldName) {
        if (this.timmer) {
          clearTimeout(this.timmer)
        }
        this.timmer = setTimeout(() => (
          this.$emit('colorChange', this.selectedColor)
        ), 12)
      }
    }
  },
  mounted () {
    this.selectedColor = this.defaultColor
  }
})
</script>

<style lang="stylus" scoped>
  @import '~@css/mixins.styl'
  @import '~@css/varibles.styl'
  .color-picker-child >>> .v-color-picker__swatch
    width 16%
    padding 0 1%
  .color-picker-child >>> .v-color-picker__color
    width 100%
</style>
