<template>
  <v-app-bar
    app
    dark
    class="appBar"
  >
    <div class="d-flex align-center">
      <v-img
        alt="Logo"
        class="shrink mr-2"
        contain
        src="/api/img/icons/logo.png"
        transition="scale-transition"
        width="55"
      />

      <span
        alt="Name"
        class="shrink d-none d-sm-block"
        min-width="100"
        width="100"
        style="font-size: 2rem;margin-left: .8rem;"
      >Adeept Bot Contorller</span>
    </div>

    <v-spacer></v-spacer>

    <!-- <ModelSelecter/> -->
  </v-app-bar>
</template>

<script>
// import ModelSelecter from '@/components/appbar/ModelSelecter.vue'

export default {
  name: 'AppBar',
  components: {
    // ModelSelecter
  }
}

</script>

<style lang="stylus" scoped>
  @keyframes shadowLight
  {
    from {box-shadow: 0rem -.2rem .7rem .1rem rgba(100, 100, 100, 1)}
    to {box-shadow: 0rem .2rem .7rem .1rem rgba(177, 177, 177, 1)}
  }
  .appBar
    animation: shadowLight 4s
    animation-iteration-count: infinite
    animation-direction: alternate

  .appBar
    // position relative
    z-index 100
    box-shadow 0rem -.2rem .7rem .1rem rgba(177, 177, 177, 1)
</style>
