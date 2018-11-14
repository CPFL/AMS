let path = require('path');
let webpack = require('webpack');
let CopyWebpackPlugin = require('copy-webpack-plugin');
let HtmlWebpackPlugin = require('html-webpack-plugin');
let CleanWebpackPlugin = require('clean-webpack-plugin');

let baseHref = process.env.WP_BASE_HREF ? process.env.WP_BASE_HREF : '/route_code_editor';

module.exports = {
  resolve: {
    alias: {
      'three/OrbitControls': path.join(__dirname, 'node_modules/three/examples/js/controls/OrbitControls.js'),
      'three/utils/Detector': path.join(__dirname, 'node_modules/three/examples/js/Detector.js'),
      'three/utils/Stats': path.join(__dirname, 'node_modules/three/examples/js/libs/stats.min.js'),
      'three/ColladaLoader': path.join(__dirname, 'node_modules/three/examples/js/loaders/ColladaLoader.js'),
      'three/TDSLoader': path.join(__dirname, 'node_modules/three/examples/js/loaders/TDSLoader.js'),
      'three/PCDLoader': path.join(__dirname, 'node_modules/three/examples/js/loaders/PCDLoader.js'),
    }
  },

  node: {
    fs: 'empty',
    tls: 'empty'
  },

  plugins: [
    new CleanWebpackPlugin(["static"], {verbose: false}),
    new HtmlWebpackPlugin({
      template: 'public/index.html'
    }),
    new CopyWebpackPlugin([{from: 'public/'}]),
    new webpack.optimize.CommonsChunkPlugin({
      names: ['vendor', 'manifest'],
      minChunks: Infinity,
      filename: '[name].[chunkhash:8].js'
    }),
    new webpack.LoaderOptionsPlugin({
      minimize: true,
      debug: false
    }),
    new webpack.DefinePlugin({
      WP_BASE_HREF: JSON.stringify(baseHref)
    }),
    new webpack.ProvidePlugin({
      'THREE': 'three',
    })
  ],
  module: {
    loaders: [
      {
        test: /\.js$/,
        loaders: 'babel-loader',
        include: path.join(__dirname, 'src'),
      },
      {
        test: /\.css/,
        loaders: ["style-loader", "css-loader"]
      },
      {
        test: /\.es6$/,
        exclude: /node_modules/,
        loader: 'babel',
        query: {
          presets: ['es2015']
        }
      }


    ]
  }
};
