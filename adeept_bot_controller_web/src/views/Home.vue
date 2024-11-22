<template>
  <div class="home">
    <ControllArea/>
  </div>
</template>

<script>
// @ is an alias to /src
import ControllArea from '@/components/ControllArea.vue'
// import ControllArea from '@/components/ControllAreaRaspTank.vue'

export default {
  name: 'Home',
  components: {
    ControllArea
  }
}
</script>
