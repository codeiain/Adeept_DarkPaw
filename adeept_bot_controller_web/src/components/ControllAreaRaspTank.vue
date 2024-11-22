<template>
<!--  <div class="controll-area">
    <v-sheet class="sheets"></v-sheet>
  </div> -->
  <!-- fluid 用于撑开宽高 -->
  <div class="area-bg">
    <div class="area-wrapper">
      <v-container class="controll-area">
        <v-row>
          <v-col md="6" sm="7" xs="12" order="6">
            <v-row class="innerRow">
              <v-col cols="12">
                <ControllerSheet modName="Vedio">
                  <VedioMod/>
                </ControllerSheet>
              </v-col>
              <v-col cols="12" v-show="this.$vuetify.breakpoint.name === 'xs' ? false : true">
                <ControllerSheet modName="Instruction">
                  <InstructionMod/>
                </ControllerSheet>
              </v-col>
            </v-row>
          </v-col>
          <v-col md="4" sm="5" xs="12" order="9">
            <v-row class="innerRow">
              <v-col cols="12">
                <ControllerSheet modName="Move Control">
                  <MoveControlMod/>
                </ControllerSheet>
              </v-col>
              <v-col cols="12">
                <ControllerSheet modName="Arm Control">
                  <ArmControlMod/>
                </ControllerSheet>
              </v-col>
              <v-col cols="12">
                <ControllerSheet modName="CVFL Control">
                  <CVFLMod/>
                </ControllerSheet>
              </v-col>
            </v-row>
          </v-col>
          <v-col md="2" sm="12" xs="12" :order="12">
            <v-row class="innerRow">
              <v-col md="12" sm="3" xs="12">
                <ControllerSheet modName="Hard Ware">
                  <StatusMod/>
                </ControllerSheet>
              </v-col >
              <v-col md="12" sm="4" xs="12">
                <ControllerSheet modName="Actions">
                  <ActionsMod/>
                </ControllerSheet>
              </v-col>
              <v-col md="12" sm="5" xs="12">
                <v-row class="innerRow">
                  <v-col md="12" sm="12" xs="6">
                    <ControllerSheet modName="FC Control">
                      <FindColorMod/>
                    </ControllerSheet>
                  </v-col>
                </v-row>
              </v-col>
            </v-row>
          </v-col>
        </v-row>
      </v-container>
    </div>
  </div>
</template>

<script>
import ControllerSheet from '@/components/controll/ControllerSheet.vue'
import VedioMod from '@/components/controll/VedioMod.vue'
import StatusMod from '@/components/controll/StatusMod.vue'
import ArmControlMod from '@/components/controll/ArmControlMod.vue'
import ActionsMod from '@/components/controll/ActionsModRaspTank.vue'
import MoveControlMod from '@/components/controll/MoveControlMod.vue'
import CVFLMod from '@/components/controll/CVFLMod.vue'
import FindColorMod from '@/components/controll/FindColorMod.vue'
import InstructionMod from '@/components/controll/InstructionMod.vue'

export default ({
  name: 'ControllArea',
  components: {
    ControllerSheet,
    VedioMod,
    StatusMod,
    ArmControlMod,
    ActionsMod,
    MoveControlMod,
    CVFLMod,
    FindColorMod,
    InstructionMod
  }
})
</script>

<style lang="stylus" scoped>
  @import '~@css/mixins.styl'
  @import '~@css/varibles.styl'

  .area-bg
    display flex // 子元素垂直填充
    position absolute
    min-height 100%
    width 100%
    background url('/api/img/bg.jpg') fixed center top
  .area-wrapper
    margin 0 auto
    min-height 100%
    min-width 320px
    width 85rem
    max-width 85rem
    // background-color #FFFF99
    background-color $default_bg_color
  .innerRow
    margin -12px
  .controll-area
    min-height 100%
    max-width 80rem
    // min-width 320px
    .sheets
      background-color #b0bec5
</style>
