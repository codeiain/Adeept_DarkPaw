<template>
  <div class="FC-bottons">
    <ControlButton class="button" :attr="buttonAttr"/>
    <v-menu :close-on-content-click="false" absolute>
      <template v-slot:activator="{ on }">
        <v-btn
          class="button colorButton"
          :style="{borderColor : FCColor}"
          small
          v-on="on"
        >
          color
        </v-btn>
      </template>
      <!-- 如果想填充背景色，可以使用 v-list 替代 div。 -->
      <div class="colorSelecter">
        <ColorPickerChild @colorChange="handleColorChange"/>
      </div>
    </v-menu>
  </div>
</template>

<script>
import { mapState, mapActions, mapMutations } from 'vuex'
import ColorPickerChild from '@/components/controll/child/ColorPickerChild.vue'
import ControlButton from '@/components/controll/child/ControlButton.vue'

export default ({
  name: 'FindColorMod',
  components: {
    ColorPickerChild,
    ControlButton
  },
  data: () => ({
    buttonAttr: {
      isIcon: false,
      content: 'start',
      sendContent: 'findColor',
      reversSendContent: 'stopCV'
    }
  }),
  computed: {
    ...mapState(['FCColor'])
  },
  methods: {
    handleColorChange (color) {
      this.changeFCColor(color)
    },
    ...mapMutations(['changeFCColor']),
    ...mapActions(['changeWsContent'])
  },
  watch: {
    FCColor () {
      const HSV = this.RGBToHSV255(this.hexToRgba(this.FCColor))
      this.changeWsContent({
        title: 'findColorSet',
        data: HSV
      })
    }
  }
})
</script>

<style lang="stylus" scoped>
  @import '~@css/mixins.styl'
  @import '~@css/varibles.styl'
  .FC-bottons
    display flex
    flex-wrap wrap
    justify-content space-between
    width 100%
    .button
      margin-top .5rem
      width 100%
  .colorButton
    position relative
    overflow hidden
    &:after // 按钮底下的颜色标识
      UnderColorIndicator()
</style>
