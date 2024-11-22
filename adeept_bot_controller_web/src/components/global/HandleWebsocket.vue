<template>
  <div>
    <v-snackbar
      class="reconnect-tip"
      color="grey lighten-4 black--text"
      v-model="reconnectTip"
      :timeout="0"
    >
      Connect Failed
      <v-btn
        color="pink"
        text
      >
        Reconnecting
      </v-btn>
      <AniLoading/>

      <v-overlay
        absolute
        :opacity="0"
      >
      </v-overlay>
    </v-snackbar>
  </div>
</template>
<script>
import { mapState, mapActions } from 'vuex'
import AniLoading from '@/components/ani/AniLoading.vue'

export default {
  name: 'HandleWebsocket',
  data () {
    return {
      websock: null,
      reconnectTip: false
    }
  },
  components: {
    AniLoading
  },
  computed: {
    ...mapState(['wsContent', 'setVedioTimmer'])
  },
  created () {
    this.initWebSocket()
  },
  destroyed () {
    this.websock.close() // 离开路由之后断开websocket连接
  },
  methods: {
    initWebSocket () { // 初始化weosocket
      // console.log('now location -- ' + location.hostname)
      const wsuri = 'ws://' + location.hostname + ':8888'
      // const wsuri = 'ws://' + '192.168.3.44' + ':8888'
      this.websock = new WebSocket(wsuri)
      this.websock.onmessage = this.websocketonmessage
      this.websock.onopen = this.websocketonopen
      this.websock.onerror = this.websocketonerror
      this.websock.onclose = this.websocketclose
    },
    websocketonopen () { // 连接建立之后执行send方法发送数据
      // const actions = { admin: '123456' }
      // this.websocketsend(JSON.stringify(actions))
      this.websock.send('admin:123456')
    },
    websocketonerror () { // 连接建立失败重连
      // this.initWebSocket()
    },
    websocketonmessage (e) { // 数据接收
      if (this.reconnectTip) { // 如果 reconnectTip 为真，认为这是一次掉线重连
        this.reconnectTip = false
        this.setVedioTimmer()
      }

      // console.log('============ responseData ============')
      let redata = null
      try {
        redata = JSON.parse(e.data)
      } catch {
        redata = e.data
        // console.log('response Data not JSON')
      }
      // console.log(redata)
      this.changeWsResponse(redata)
    },
    websocketsend (Data) { // 数据发送
      if (Data) {
        // console.log('============ sendingData ============')
        try {
          Data = JSON.stringify(Data)
        } catch (e) {
          // console.log('can`t send with JSON')
        }
        // console.log(Data)
        this.websock.send(Data)
        this.changeWsContent('')
      }
    },
    websocketclose (e) { // 连接断开时重连
      // console.log('断开连接', e)
      this.reconnectTip = true
      this.initWebSocket()
    },
    ...mapActions(['changeWsResponse', 'changeWsContent'])
  },
  watch: {
    wsContent () {
      if (this.wsContent) {
        try {
          this.websocketsend(this.wsContent)
        } catch (InvalidStateError) {
          console.log('连接已关闭或正在连接中，无法发送数据')
          this.changeWsContent('')
        }
      }
    }
  }
}
</script>
