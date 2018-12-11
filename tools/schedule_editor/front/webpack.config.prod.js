let path = require('path');
let webpack = require('webpack');
let webpackMerge = require('webpack-merge');
let commonConfig = require('./webpack.config.common.js');

module.exports = webpackMerge(commonConfig,{
    entry: {
        app: './src/index'
    },
    output: {
        path: path.join(__dirname, '/static/schedule_editor'),
        publicPath: '/static/schedule_editor',
        filename: 'bundle.[chunkhash:8].js',
    },
  plugins: [
    new webpack.DefinePlugin({
      'process.env.NODE_ENV': '"production"'
    })
  ],

});
