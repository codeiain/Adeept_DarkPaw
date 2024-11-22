module.exports = {
  pwa: {
    iconPaths: {
      favicon32: 'favicon.ico',
      favicon16: 'favicon.ico',
      appleTouchIcon: 'favicon.ico',
      maskIcon: 'favicon.ico',
      msTileImage: 'favicon.ico'
    }
  },
  configureWebpack: { // configureWebpack 如果它对应一个对象，那么表示将这个对象与默认配置对象合并。
    resolve: {
      alias: { // 在这里添加项目路径的别名。@ 表示到 src 目录下的路径。
        '@css': '@/assets/css',
        '@img': '@/assets/img',
        '@common': '@/components/common',
        '@pImg': '../../public/statics/mock/img'
      }
    }
  },
  devServer: { // 开发环境服务器设置
    host: '0.0.0.0', // ip
    port: 8087, // 设置端口号
    // https: false, // https:{type:Boolean}
    // open: false, // 配置自动启动浏览器
    proxy: { // 设置代理。可以在开发环境下重定向 api 地址到本地静态文件
      '/api': {
        target: 'http://localhost:8087/', // 重定向目标服务器。这里还是填本地的开发服务器。
        pathRewrite: {
          '^/api': '/'
        }
      }
    }
  },
  transpileDependencies: [
    'vuetify'
  ]
}
