<template>
  <div class="status-wrapper">
    <v-chip
      class="ma-2 chips"
      v-for="(item,index) in chips"
      :key="index"
      :color="chipColor[index]"
      ref="chips"
      text-color="white"
    >
      <b class="chip-title">{{ item[0] + ' ' + item[1] }}</b>
      {{ item[2] + item[3] }}
    </v-chip>
  </div>
</template>

<script>
import { mapState, mapActions } from 'vuex'
export default ({
  name: 'ArmControlMod',
  props: {
    title: String
  },
  data: () => ({
    chips: [
      ['CPU', 'Temp', 50, '°C', 55, 70],
      ['CPU', 'Usage', 75, '%', 70, 85],
      ['RAM', 'Usage', 90, '%', 70, 85]
    ],
    infoInterval: null
  }),
  computed: {
    chipColor () {
      const colors = []
      for (const i in this.chips) {
        if (this.chips[i][2] < this.chips[i][4]) { // 多半是绿的，所以先判断绿不绿省一点计算时间
          colors.push('green')
        } else if (this.chips[i][2] < this.chips[i][5]) {
          colors.push('orange')
        } else {
          colors.push('red')
        }
      }
      return colors
    },
    ...mapState(['wsResponse'])
  },
  watch: {
    wsResponse () {
      if (this.wsResponse.title === 'get_info') {
        const data = this.wsResponse.data
        console.log(data)
        for (let i = 0; i < this.chips.length; i++) {
          this.$set(this.chips[i], 2, data[i])
        }
      }
    }
  },
  methods: {
    ...mapActions(['changeWsContent'])
  },
  mounted () {
    this.infoInterval = setInterval(() => {
      this.changeWsContent('get_info')
    }, 5000)
  },
  destroyed () {
    clearInterval(this.infoInterval)
  }
})
</script>

<style lang="stylus" scoped>
  @import '~@css/mixins.styl'
  @import '~@css/varibles.styl'
  .status-wrapper
    display flex
    margin 0 -.3rem
    justify-content center
    flex-wrap wrap
    .chips
      box-shadow #000000 -0rem 0rem .1rem 1px
      width 100%
      margin 0 !important
      margin-top .4rem !important
      padding 0 4%
      display flex
      justify-content center
      .chip-title
        padding-right .3rem
</style>
