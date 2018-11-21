let path = require('path');
let webpackMerge = require('webpack-merge');
let commonConfig = require('./webpack.config.common.js');

const pkg = require('./package.json');

module.exports = webpackMerge(commonConfig,{
    devtool: 'inline-source-map',
    entry: {
        vendor: Object.keys(pkg.dependencies).concat('./src/vendor'),
        app: './src/index'
    },
    output: {
        path: path.join(process.cwd(), '/static/route_code_editor'),
        publicPath: '/static/route_code_editor',
        filename: '[name].js'
    },

});
