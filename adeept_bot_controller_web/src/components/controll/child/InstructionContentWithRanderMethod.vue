<!-- 如果是直接使用 template ，可以直接用标签对的形式使用 vuetify 组件。 -->
<!-- <template>
  <v-expansion-panel-content>
    123
  </v-expansion-panel-content>
</template>

<script>
export default ({
  name: 'InstructionContent'
})
</script> -->

<script>
// 与直接使用 template 时不同，使用渲染函数必须先导入才能使用 vuetify 的组件。
import VExpansionPanelContent from 'vuetify/lib/components/VExpansionPanel/VExpansionPanelContent'
import VExpansionPanelHeader from 'vuetify/lib/components/VExpansionPanel/VExpansionPanelHeader'
import VExpansionPanel from 'vuetify/lib/components/VExpansionPanel/VExpansionPanel'

export default ({
  name: 'InstructionContent',
  render: function (createElement) {
    return createElement(
      VExpansionPanel,
      [
        createElement(
          VExpansionPanelHeader,
          this.$slots.default // default是默认插槽
        ),
        createElement(
          VExpansionPanelContent,
          [
            createElement(
              'h1',
              'CONTENT'
            )
          ]
        )
      ]
    )
  },
  props: {
    contentObj: {
      type: Object,
      required: true
    },
    type: Number
  }
})
</script>

<style>
</style>
