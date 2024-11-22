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
      [false, 'ARM', 'AR'],
      [false, 'HOME', 'home'],
      [false, 'PT', 'PT']
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
