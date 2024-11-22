<template>
  <v-app>
    <HandleWebsocket/>
    <HandleKeyEvent/>

    <AppBar/>

    <v-content>
      <Home/>
    </v-content>
  </v-app>
</template>

<script>
import AppBar from '@/views/AppBar.vue'
import Home from '@/views/Home.vue'
import HandleWebsocket from '@/components/global/HandleWebsocket.vue'
import HandleKeyEvent from '@/components/global/HandleKeyEvent.vue'

export default {
  name: 'App',

  components: {
    AppBar,
    Home,
    // ModelSelecter,
    HandleWebsocket,
    HandleKeyEvent
  },

  data: () => ({
    //
  })
}

var isMobile = navigator.userAgent.toLowerCase().match(/(ipod|iphone|android|coolpad|mmp|smartphone|midp|wap|xoom|symbian|j2me|blackberry|wince)/i) != null // 进行userAgent匹配
if (isMobile) {
  console.log('mobaView')
}
</script>

<style lang="stylus">
  @import '~@css/varibles.styl'
  @import '~@css/mixins.styl'

  ::-webkit-scrollbar{display: none;}
  html
    // font-size: 50px !important // 1 rem = html 标签的 font-size 属性所设置的 px 值。vuetify 似乎改变了这个设置，所以加个 !important。
    touch-action: manipulation // 解决移动端 300ms 延迟问题。再见 fastclick.css ！
    user-select: none
  body
    // font-size: 16px !important // 将 body 的字体大小设置回默认的 16 px。这样不会留下改变字体大小产生的后遗症。
  html
    touch-action: manipulation // 解决移动端 300ms 延迟问题。再见 fastclick.css ！

  .col
    flex-basis: auto
</style>
