module.exports = {
  env: {
    node: true,
  },
  extends: ["eslint:recommended", "plugin:vue/recommended", "prettier"],
  rules: {
    // override/add rules settings here, such as:
    // 'vue/no-unused-vars': 'error'
    "vue/prop-name-casing": "off",
  },
};
