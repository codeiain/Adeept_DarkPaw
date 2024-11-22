<template>
  <v-expansion-panel v-if="type==0">
    <v-expansion-panel-header v-slot="{ open }">
      <!-- no-gutters 取消 row 的间距 -->
      <v-row no-gutters>
        <v-col cols="4">
          <slot></slot>
        </v-col>
        <v-col
          cols="4"
          class="text--secondary"
          v-for="i in InsContent[0]"
          :key="i"
        >
          <v-fade-transition leave-absolute>
            <span v-if="open">
              {{ i }}
            </span>
          </v-fade-transition>
        </v-col>
      </v-row>
    </v-expansion-panel-header>
    <v-expansion-panel-content>
      <v-row no-gutters>
        <v-spacer></v-spacer>
        <v-col cols="4">
          <div v-for="(item,key) in InsContent[1][0]" :key="key">
            {{ key }} - {{ item }}<br>
          </div>
        </v-col>
        <v-col cols="4">
          <div v-for="(item,key) in InsContent[1][1]" :key="key">
            {{ key }} - {{ item }}<br>
          </div>
        </v-col>
      </v-row>
    </v-expansion-panel-content>
  </v-expansion-panel>
  <v-expansion-panel v-if="type==1">
    <v-expansion-panel-header v-slot="{ open }">
      <!-- no-gutters 取消 row 的间距 -->
      <v-row no-gutters>
        <v-col cols="4">
          <slot></slot>
        </v-col>
        <v-col
          cols="4"
          class="text--secondary"
          v-for="i in InsContent[0]"
          :key="i"
        >
          <v-fade-transition leave-absolute>
            <span v-if="open">
              {{ i }}
            </span>
          </v-fade-transition>
        </v-col>
      </v-row>
    </v-expansion-panel-header>
    <v-expansion-panel-content>
      <v-row no-gutters>
        <v-spacer></v-spacer>
        <v-col cols="4">
          <div v-for="(item,key) in InsContent[1][0]" :key="key">
            {{ key }} - {{ item }}<br>
          </div>
        </v-col>
        <v-col cols="4">
          <div v-for="(item,key) in InsContent[1][1]" :key="key">
            {{ key }} - {{ item }}<br>
          </div>
        </v-col>
      </v-row>
    </v-expansion-panel-content>
  </v-expansion-panel>
</template>

<script>
export default ({
  name: 'InstructionMod',
  props: {
    InsContent: {
      required: true
    },
    type: Number
  }
})
</script>

<style>
</style>
