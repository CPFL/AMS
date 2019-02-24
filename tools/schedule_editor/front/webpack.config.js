let path = require('path');
let webpackMerge = require('webpack-merge');
let commonConfig = require('./webpack.config.common.js');

module.exports = webpackMerge(commonConfig, {
    devtool: 'inline-source-map',
    entry: {
    app: './src/index'
  },
  output: {
    filename: 'bundle.[chunkhash:8].js'
  },
  devServer: {
    compress: true,
    open: true,
    port: 3000
  }
});
