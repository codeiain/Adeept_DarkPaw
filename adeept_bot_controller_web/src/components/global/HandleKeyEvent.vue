<template>
  <div></div>
</template>
<script>
import { mapState } from 'vuex'

export default {
  name: 'HandleKeyEvent',
  computed: {
    ...mapState(['keyEvents'])
  },
  created () {
    document.onkeydown = (e) => {
      if (e.repeat) {
        return
      }
      for (const i in this.keyEvents.down) {
        if (e.keyCode === Number(i)) {
          const func = this.keyEvents.down[i]
          func()
        }
      }
    }
    document.onkeyup = (e) => {
      for (const i in this.keyEvents.up) {
        if (e.keyCode === Number(i)) {
          const func = this.keyEvents.up[i]
          func()
        }
      }
    }
  }
}
</script>
