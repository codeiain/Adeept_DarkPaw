import Vue from 'vue'
import vuetify from './plugins/vuetify'
import App from './App.vue'
import './registerServiceWorker'
import router from './router'
import store from './store'

import '@css/normalize.min.css'

Vue.config.productionTip = false

// 绑定在 Vue 组件原型上的工具函数。用于将 RGB 数值转换为（opencv 可读的) HSV 数值
Vue.prototype.RGBToHSV255 = function (arr) {
  var h = 0; var s = 0; var v = 0
  var r = arr[0]; var g = arr[1]; var b = arr[2]
  arr.sort(function (a, b) {
    return a - b
  })
  var max = arr[2]
  var min = arr[0]
  v = max / 255
  if (max === 0) {
    s = 0
  } else {
    s = 1 - (min / max)
  }
  if (max === min) {
    h = 0// 事实上，max===min的时候，h无论为多少都无所谓
  } else if (max === r && g >= b) {
    h = 60 * ((g - b) / (max - min)) + 0
  } else if (max === r && g < b) {
    h = 60 * ((g - b) / (max - min)) + 360
  } else if (max === g) {
    h = 60 * ((b - r) / (max - min)) + 120
  } else if (max === b) {
    h = 60 * ((r - g) / (max - min)) + 240
  }
  h = parseInt(Math.floor(h / 2))
  s = parseInt(s * 255)
  v = parseInt(v * 255)
  return [h, s, v]
}

// 绑定在 Vue 组件原型上的工具函数。用于将 HEX 数值（#000000这样的）转换为 RGB 数值
Vue.prototype.hexToRgba = function (hex, opacity) {
  if (opacity) {
    return [parseInt('0x' + hex.slice(1, 3)), parseInt('0x' + hex.slice(3, 5)), parseInt('0x' + hex.slice(5, 7)), opacity]
  }
  return [parseInt('0x' + hex.slice(1, 3)), parseInt('0x' + hex.slice(3, 5)), parseInt('0x' + hex.slice(5, 7))]
}

new Vue({
  router,
  store,
  vuetify,
  render: h => h(App)
}).$mount('#app')
