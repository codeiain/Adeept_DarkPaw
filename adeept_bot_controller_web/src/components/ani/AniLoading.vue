<template>
  <span class="light" :style="lightStyle"></span>
</template>

<script>
export default {
  name: 'AniLoading',
  data: () => ({
    lightStyle: {
      height: '52px',
      'margin-left': '-10%',
      'transition-duration': '1.3s'
    },
    timer: null
  }),
  mounted () {
    this.lightStyle.height = this.$el.parentNode.offsetHeight + 50 + 'px'
    if (!this.$el.parentNode.style.position) { this.$el.parentNode.style.position = 'relative' } // 保证父元素设置了 position

    this.timer = setInterval(() => {
      if (this.lightStyle['margin-left'] === '-10%') {
        this.lightStyle['margin-left'] = '105%'
        this.lightStyle['transition-duration'] = '1.3s'
      } else {
        this.lightStyle['margin-left'] = '-10%'
        this.lightStyle['transition-duration'] = '0s'
      }
    }, 1300)
  }
}
</script>

<style lang="stylus" scoped>
  .light
    position absolute
    transition: margin-left linear
    width 0
    box-shadow: 0rem 0rem .9rem .1rem rgba(199, 199, 199, 1)
</style>
