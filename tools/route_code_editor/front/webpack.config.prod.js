let path = require('path');
let webpack = require('webpack');
let webpackMerge = require('webpack-merge');
let commonConfig = require('./webpack.config.common.js');

const pkg = require('./package.json');

module.exports = webpackMerge(commonConfig,{
    entry: {
        vendor: Object.keys(pkg.dependencies).concat('./src/vendor'),
        app: './src/index'
    },
    output: {
        path: path.join(__dirname, 'static/route_code_editor'),
        publicPath: '/static/route_code_editor',
        filename: 'bundle.[chunkhash:8].js',
    },
  plugins: [
    new webpack.DefinePlugin({
      'process.env.NODE_ENV': '"production"'
    }),
    new webpack.optimize.UglifyJsPlugin()
  ],

});
