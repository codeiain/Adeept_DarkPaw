<template>
  <ButtonsChild :buttons="buttons" :cols="cols"/>
</template>

<script>
import ButtonsChild from '@/components/controll/child/ButtonsChild.vue'

export default ({
  name: 'ActionsMod',
  components: {
    ButtonsChild
  },
  data: () => ({
    buttons: [
      [false, 'Motion Get', 'motionGet', '', '', 'stopCV'],
      [false, 'Auto Matic', 'automatic', '', '', 'automaticOff'],
      [false, 'POLICE LIGHT', 'police', '', '', 'policeOff'],
      [false, 'Track Line', 'trackLine', '', '', 'trackLineOff']
    ],
    cols: 1
  })
})
</script>

<style lang="stylus" scoped>
</style>
