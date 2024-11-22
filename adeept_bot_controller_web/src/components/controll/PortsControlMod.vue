<template>
  <div class="ports-wrapper">
    <ButtonsChild :buttons="buttons" :cols="cols"/>
  </div>
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
      [false, 'P1', 'Switch_1_on', '', '', 'Switch_1_off'],
      [false, 'P2', 'Switch_2_on', '', '', 'Switch_2_off'],
      [false, 'P3', 'Switch_3_on', '', '', 'Switch_3_off']
    ],
    cols: 3
  })
})
</script>

<style lang="stylus" scoped>
  .ports-wrapper >>> .buttons
    padding-left 0
    padding-right 0
    min-width 28px
</style>
