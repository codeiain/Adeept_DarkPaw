<template>
  <v-expansion-panels
    class="expansion"
    accordion
    mandatory
    v-model="witchOpen"
  >
    <v-expansion-panel>
      <v-expansion-panel-header v-slot="{ open }">
        <!-- no-gutters 取消 row 的间距 -->
        <v-row no-gutters>
          <v-col cols="4">Base Control</v-col>
          <v-col
            cols="4"
            class="text--secondary"
            v-for="i in instructContent['Base Control'][0]"
            :key="i"
          >
            <v-fade-transition leave-absolute>
              <span
                v-if="open"
              >
                {{ i }}
              </span>
            </v-fade-transition>
          </v-col>
        </v-row>
      </v-expansion-panel-header>
      <v-expansion-panel-content>
        <v-row no-gutters>
          <v-spacer></v-spacer>
          <v-col cols="4" v-for="(item,index) in instructContent['Base Control'][1]" :key="index">
            <div v-for="(item,key) in item" :key="key">
              {{ key }} - {{ item }}<br>
            </div>
          </v-col>
        </v-row>
      </v-expansion-panel-content>
    </v-expansion-panel>

    <v-expansion-panel>
      <v-expansion-panel-header>About Us</v-expansion-panel-header>
      <v-expansion-panel-content style="text-indent: 2rem;">
        {{instructContent['About Us']}}
      </v-expansion-panel-content>
    </v-expansion-panel>
  </v-expansion-panels>

</template>

<script>
export default ({
  name: 'InstructionMod',
  data: () => ({
    cols: 1,
    witchOpen: 0,
    instructContent: {
      'Base Control': [
        ['Move Control', 'Arm Control'],
        [{
          W: 'move forward',
          A: 'turn left',
          S: 'move backward',
          D: 'turn right'
        }, {
          I: 'arm up',
          J: 'arm left turn',
          K: 'arm down',
          L: 'arm right turn',
          U: 'grab',
          O: 'loose',
          P: 'hand up',
          ';': 'hand down'
        }]
      ],
      'About Us': 'Adeept is a technical service team of open source software and hardware. Dedicated to applying the Internet and the latest industrial technology in open source area, we strive to provide best hardware support and software service for general makers and electronic enthusiasts around the world. We aim to create infinite possibilities with sharing. No matter what field you are in, we can lead you into the electronic world and bring your ideas into reality.',
      'Contact Us': 'Have a technical question for Tech Support? support@adeept.com'
    }
  })
})
</script>

<style lang="stylus" scoped>
  .expansion
    user-select text
    padding-top .5rem
    .v-expansion-panel-header
      font-weight 650
</style>
