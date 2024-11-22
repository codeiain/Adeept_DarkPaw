<template>
  <div class="radar-wrapper" ref="wrapper">
      <div class="container" ref="container"></div>
      <div class="maskWrapper" @click="handleMaskClick" ref="mask">
        <CommonMask :status="maskStatus" content="SCAN"></CommonMask>
      </div>
  </div>
</template>

<script>
/* eslint-disable no-dupe-keys */
import echarts from 'echarts'
import { mapState, mapActions } from 'vuex'
import CommonMask from '@/components/common/CommonMask.vue'

export default ({
  name: 'Radar',
  components: {
    CommonMask
  },
  data: () => ({
    RadarChart: null,
    dom: null,
    scaning: false,
    maskStatus: 0
  }),
  computed: {
    ...mapState(['wsResponse'])
  },
  methods: {
    handleWindowResize () {
      this.dom.style.height = this.dom.offsetWidth + 'px'
      this.mask.style.height = this.$refs.wrapper.offsetHeight + 'px'
      this.RadarChart.resize()
    },
    handleMaskClick () {
      if (!this.scaning) {
        this.maskStatus = 1
        this.scaning = true
        this.changeWsContent('scan')
      }
    },
    drawChart (data) {
      var option = null
      var series = []
      if (!data) {
        series = [{
          coordinateSystem: 'polar',
          name: 'line',
          type: 'line',
          data: [[0, 0], [0, 0]]
        }]
      }

      function dis (data) {
        return {
          coordinateSystem: 'polar',
          name: 'line',
          type: 'line',
          lineStyle: {
            color: '#22ee22'
          },
          itemStyle: {
            color: 'black'
          },
          data: data
        }
      }

      for (const i in data) {
        const r = data[i][0]
        const theta = data[i][1]
        series.push(dis([[0, theta], [r, theta]]))
      }

      // series.push({
      //   type: 'line',
      //   coordinateSystem: 'polar',
      //   areaStyle: {
      //     normal: {
      //       color: '#00bcd4',
      //       opacity: 1
      //     }
      //   },
      //   data: [[200, 0], [200, 270], [200, 180]],
      //   symbolSize: 0,
      //   lineStyle: {
      //     normal: {
      //       width: 0
      //     }
      //   }
      // })

      option = {
        // title: {
        //   text: 'Radar'
        // },
        polar: {
        },
        tooltip: {
          trigger: 'axis',
          axisPointer: {
            type: 'cross'
          }
        },
        angleAxis: {
          type: 'value',
          // show: false,
          min: 15,
          max: 375,
          startAngle: 165,
          clockwise: true
        },
        radiusAxis: {
          // inverse: 'true'
          // max: 3
        },
        series: series
      }

      if (option && typeof option === 'object') {
        this.RadarChart.clear()
        this.RadarChart.setOption(option, true)
      }
    },
    ...mapActions(['changeWsContent'])
  },
  watch: {
    wsResponse () {
      if (this.wsResponse.title === 'scanResult') {
        this.maskStatus = 2
        this.scaning = false
        console.log(this.wsResponse)
        const data = this.wsResponse.data
        this.drawChart(data)
      }
    }
  },
  mounted () {
    this.mask = this.$refs.mask
    this.dom = this.$refs.container
    this.RadarChart = echarts.init(this.dom)
    this.drawChart()

    window.addEventListener('resize', this.handleWindowResize)
    this.handleWindowResize()
  },
  distory () {
    window.removeEventListener('resize', this.handleWindowResize)
  }
})
</script>

<style lang="stylus" scoped>
  .radar-wrapper
    position relative
    width: 100%
    overflow hidden
    margin-top .5rem
    padding-bottom 52%
    height 0
    border-radius .4rem
    .container
      background-color #fff
      width 100%
    .maskWrapper
      position absolute
      bottom 0
      width 100%
</style>
