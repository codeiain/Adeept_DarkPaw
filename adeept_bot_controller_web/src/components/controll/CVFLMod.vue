<template>
  <div class="CVFL-wrapper">
    <div class="sliders">
      <v-slider
        v-for="valueKey in valuesKeys"
        :key="valueKey"
        class="mx-4 mt-1"
        v-model="values[valueKey][0]"
        thumb-label
        :label="valueKey"
        hide-details
        :min="values[valueKey][1]"
        :max="values[valueKey][2]"
        @change="handleValueChange(valueKey)"
      ></v-slider>
    </div>
    <div class="CVFL-bottons">
      <ControlButton class="button" :attr="buttonAttr"/>
      <v-btn
        class="colorButton"
        :style="{borderColor : CVFLColor}"
        small
        @click="handleColorChange"
      >
        color
      </v-btn>
    </div>
  </div>
</template>

<script>
import { mapState, mapActions, mapMutations } from 'vuex'
import ControlButton from '@/components/controll/child/ControlButton.vue'

export default ({
  name: 'CVFLMod',
  components: {
    ControlButton
  },
  data: () => ({
    buttonAttr: {
      isIcon: false,
      content: 'start',
      sendContent: 'CVFL',
      reversSendContent: 'stopCV'
    },
    values: {
      L1: [380, 0, 480],
      L2: [440, 0, 480],
      SP: [20, 0, 200]
      // EXP: [0, -25, 25]
    }
  }),
  computed: {
    valuesKeys () {
      const temp = []
      for (const i in this.values) {
        temp.push(i)
      }
      return temp
    },
    rgbKeys () {
      const temp = []
      for (const i in this.rgb) {
        temp.push(i)
      }
      return temp
    },
    ...mapState(['CVFLColor'])
  },
  methods: {
    handleColorChange () {
      if (this.CVFLColor === '#000000') {
        this.changeCVFLColor('#FFFFFF')
        this.changeWsContent('CVFLColorSet 255')
      } else {
        this.changeCVFLColor('#000000')
        this.changeWsContent('CVFLColorSet 0')
      }
    },
    handleValueChange (key) {
      if (key === 'L1') {
        this.changeWsContent('CVFLL1 ' + this.values.L1[0])
      } else if (key === 'L2') {
        this.changeWsContent('CVFLL2 ' + this.values.L2[0])
      } else if (key === 'SP') {
        this.changeWsContent('CVFLSP ' + this.values.SP[0])
      } else if (key === 'EXP') {
        this.changeWsContent('CVFLEXP ' + this.values.EXP[0])
      }
    },
    ...mapActions(['changeWsContent']),
    ...mapMutations(['changeCVFLColor'])
  }
})
</script>

<style lang="stylus" scoped>
  @import '~@css/mixins.styl'
  @import '~@css/varibles.styl'
  .CVFL-wrapper
    display flex
    .sliders
      width 75%
    .sliders >>> label
      margin 0
    .sliders >>> .v-input
      margin-left 0 !important
      margin-right 0 !important
    .CVFL-bottons
      display flex
      flex-direction column
      justify-content space-around
      width 25%
      .button
        width 100%
    .colorButton
      position relative
      overflow hidden
      &:after // 按钮底下的颜色标识
        UnderColorIndicator()
  .colorSelecter >>> .v-color-picker__swatch
    width 16%
    padding 0 1%
  .colorSelecter >>> .v-color-picker__color
    width 100%
</style>
