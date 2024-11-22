import Vue from 'vue'
import Vuex from 'vuex'

Vue.use(Vuex)

export default new Vuex.Store({
  state: {
    setVedioTimmer: null, // 保存一个函数，用于刷新视频流。暂时不考虑一个页面多开多个视频流的可能性
    CVFLColor: '#FFFFFF',
    FCColor: '#FFFF00',
    nowAction: 'none', // 每次只能同时进行一种功能
    // 使用 websocket 的规则：当前一个传输得到了返回值后，才可以进行下一个传输。
    wsContent: '', // 当次 ws 传输值
    wsResponse: '', // 当次 ws 返回值
    keyEvents: { // 储存按键与其对应的事件
      down: {}, // 65:func1,87:func2 etc...
      up: {}
    }
  },
  actions: {
    changeWsContent (ctx, content) {
      ctx.commit('changeWsContent', content)
    },
    changeWsResponse (ctx, content) {
      if (content) {
        ctx.commit('changeWsResponse', content)
      }
    }
  },
  mutations: {
    changeSetVedioTimmer (state, func) {
      state.setVedioTimmer = func
    },
    changeCVFLColor (state, CVFLColor) {
      state.CVFLColor = CVFLColor
    },
    changeFCColor (state, FCColor) {
      console.log(FCColor)
      state.FCColor = FCColor
    },
    changeWsContent (state, content) {
      state.wsContent = content
    },
    changeWsResponse (state, content) {
      state.wsResponse = content
    },
    setKeyEvent (state, args) {
      if (args && args[0] && args[1] && args[2]) { // 0：按键码 1：对应函数 2：'up' 或 'down'
        state.keyEvents[args[2]][args[0]] = args[1]
      }
    }
  },
  modules: {
  }
})
