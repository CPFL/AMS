let path = require('path');
let webpackMerge = require('webpack-merge');
let commonConfig = require('./webpack.config.common.js');

module.exports = webpackMerge(commonConfig, {
  devtool: 'inline-source-map',
  entry: {
    app: './src/index'
  },
  output: {
    path: path.join(process.cwd(), '/static/'),
    filename: '[name].js'
  },
  devServer: {
    contentBase: './static/schedule_editor/',
    port: 3000,
  }
});
