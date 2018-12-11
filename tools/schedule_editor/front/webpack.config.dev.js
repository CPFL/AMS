let path = require('path');
let webpackMerge = require('webpack-merge');
let commonConfig = require('./webpack.config.common.js');

module.exports = webpackMerge(commonConfig, {
  devtool: 'inline-source-map',
  entry: {
    app: './src/index'
  },
  output: {
    path: path.join(process.cwd(), '/static/schedule_editor'),
    publicPath: '/static/schedule_editor',
    filename: '[name].js'
  },
});
