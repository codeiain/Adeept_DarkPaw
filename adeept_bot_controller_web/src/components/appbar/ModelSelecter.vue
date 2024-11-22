<template>
  <v-menu offset-y>
    <template v-slot:activator="{ on }">
      <v-btn
        dark
        text
        v-on="on"
      >
        Dropdown
      </v-btn>
    </template>
    <v-list>
      <v-list-item
        v-for="(item, index) in items"
        :key="index"
      >
        <v-list-item-title>{{ item }}</v-list-item-title>
      </v-list-item>
    </v-list>
  </v-menu>
</template>

<script>

export default ({
  name: 'ModelSelecter',
  data: () => ({
    items: ['Foo', 'Bar', 'Fizz', 'Buzz']
  })
})
</script>

<style lang="stylus" scoped>
  .model-selecter >>> .v-input__slot
    border 0
  .model-selecter
    .v-input__control
      .v-input__slot
        border 2px !important
</style>
