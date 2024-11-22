import Vue from 'vue'
import Vuetify from 'vuetify/lib'
import 'typeface-roboto/index.css' // 引入本地的Roboto字体资源
import '@mdi/font/css/materialdesignicons.css' // 引入本地的Material Design Icons资源

Vue.use(Vuetify)

export default new Vuetify({
  icons: {
    iconfont: 'mdi' // 设置使用本地的icon资源
  },
  breakpoint: {
    thresholds: {
      xs: 760,
      sm: 960,
      md: 1280,
      lg: 1920
    },
    scrollBarWidth: 24
  }
})
