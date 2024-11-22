<template>
  <div>
    <ButtonsChild :buttons="buttons" :cols="cols"/>
    <v-slider
      class="mx-4 mt-1"
      v-model="speed"
      thumb-label
      label="speed"
      hide-details
    ></v-slider>
  </div>
</template>

<script>
import ButtonsChild from '@/components/controll/child/ButtonsChild.vue'
import { mapActions } from 'vuex'

export default ({
  name: 'MoveControlMod',
  components: {
    ButtonsChild
  },
  data: () => ({
    buttons: [
      '',
      [true, 'mdi-arrow-up-thick', 'forward', 87, 'DS'],
      '',
      [true, 'mdi-arrow-left-thick', 'left', 65, 'TS'],
      [true, 'mdi-arrow-down-thick', 'backward', 83, 'DS'],
      [true, 'mdi-arrow-right-thick', 'right', 68, 'TS']
    ],
    cols: 3,
    speed: 100,
    timmer: null
  }),
  methods: {
    ...mapActions(['changeWsContent'])
  },
  watch: {
    speed () {
      if (this.timmer) {
        clearTimeout(this.timmer)
      }
      this.timmer = setTimeout(() => {
        this.changeWsContent('wsB ' + String(this.speed))
      }, 300)
    }
  }
})
</script>

<style lang="stylus" scoped>
</style>
