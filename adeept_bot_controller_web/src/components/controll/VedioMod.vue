<template>
  <div class="vedio-wrapper">
    <canvas ref="canvas" class="canvas" width="640" height="480"></canvas>
    <div class="draw-area">
    </div>
  </div>
</template>

<script>
import { mapMutations } from 'vuex'

export default ({
  name: 'VedioMod',
  data: () => ({
    timmer: null,
    rand: 0
  }),
  mounted () {
    this.setVedioTimmer()
    this.changeSetVedioTimmer(() => { this.setVedioTimmer() })
  },
  destroyed () {
    clearInterval(this.timmer)
  },
  methods: {
    setVedioTimmer () {
      if (this.timmer) {
        clearInterval(this.timmer)
        this.rand = Math.random()
      }

      this.timmer = setInterval(() => {
        const canvas = this.$refs.canvas
        const context = canvas.getContext('2d')
        const image = new Image()
        image.crossOrigin = 'Anonymous'
        image.src = 'http://' + location.hostname + ':5000/video_feed?rand=' + this.rand
        // image.src = 'http://192.168.3.44:5000/video_feed?rand=' + this.rand

        image.onload = () => {
          // console.log(image.src)
          // 将图片按像素写入画布
          // console.log(image.height)
          context.drawImage(image, 0, 0, 640, 480)
          // 读取图片某位置像素信息(x,y为需要读取的像素位置)
          // var imageData = context.getImageData(50, 50, 1, 1).data
          // console.log(this.rgbToHsv([imageData[0], imageData[1], imageData[2]]))
        }
      }, 1000 / 24)
    },
    ...mapMutations(['changeSetVedioTimmer'])
  }
})
</script>

<style lang="stylus" scoped>
  @import '~@css/mixins.styl'
  @import '~@css/varibles.styl'
  .vedio-wrapper
    position relative
    overflow hidden
    border-radius .4rem
    // max-width 640px
    height 0
    width 100%
    padding-bottom 75%
    margin-top .5rem
    .draw-area
      position absolute
      top 0
      padding-bottom 75%
      width 100%
      // background-color rgba(0,0,255,0.5)
    .vedio
      top .5rem
      margin-bottom -8px // 消除神秘的 8px 像素高度。
      width 100%
    .canvas
      width 100%
      // height 360px
      // width 480px
</style>
