<template>
  <div
    :class="maskClass"
    :style="maskStyle"
    v-ripple="ripple"
  >
    <div
      class="mask-font"
      :style="maskFontStyle"
    >{{maskContent + contentSuffix}}</div>
  </div>
</template>

<script>
export default ({
  name: 'CommonMask',
  props: {
    status: Number, // static,loding,showing,disable ，用数字表示
    content: String
  },
  data: () => ({
    ripple: true,
    maskContent: '',
    contentSuffix: '',
    maskClass: ['mask', 'mask-hover'],
    maskStyle: {
      'padding-bottom': '0px',
      'background-color': '',
      transition: ''
    },
    maskFontStyle: {
      'margin-top': '0px'
    },
    loadingInterval: null,
    intervalCounter: 0
  }),
  handleMaskClick () {
    if (!this.scaning) {

      // this.maskFontStyle['margin-top'] = '-200px'
    }
  },
  methods: {
    isLoading () {
      if (this.loadingInterval) {
        this.intervalCounter = 0
        this.contentSuffix = ''
        this.loadingInterval = clearInterval(this.loadingInterval)
      } else {
        this.loadingInterval = setInterval(() => { // 使用箭头函数让 this 指向组件本身
          this.intervalCounter++
          if (this.intervalCounter > 3) {
            this.intervalCounter = 0
            this.contentSuffix = ''
          } else {
            this.contentSuffix = this.contentSuffix + '.'
          }
        }, 666)
      }
    }
  },
  watch: {
    status () {
      if (this.status === 1) {
        this.ripple = false
        this.isLoading()
        this.$set(this.maskClass, 1, '')
        this.maskStyle['background-color'] = 'rgba(22,22,22,0.3)'
        this.maskFontStyle['margin-top'] = '0'
      } else if (this.status === 2) {
        this.isLoading()
        this.maskStyle['background-color'] = 'rgba(22,22,22,0.0)'
        this.maskStyle.transition = 'background-color 0.5s ease-in'
        this.maskFontStyle['margin-top'] = '-400px'
      }
    }
  },
  mounted () {
    this.maskContent = this.content
  }
})
</script>

<style lang="stylus" scoped>
  .mask
    display flex
    justify-content center
    flex-direction column
    cursor pointer
    background-color rgba(22,22,22,0.6)
    width 100%
    height 100%
    .mask-font
      transition margin-top 0.5s ease-in
      text-align center
      font-size 35px
      font-weight 600
   .mask-hover
    &:hover
      background-color rgba(22,22,22,0.1)
</style>
