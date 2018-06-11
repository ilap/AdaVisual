const path = require('path');
const webpack = require('webpack')

module.exports = {
  entry: './src/app',
  devtool: 'cheap-source-map',
  output: {
    filename: 'adavisual.js',
    path: path.resolve(__dirname, 'dist'),
    devtoolModuleFilenameTemplate: '[absolute-resource-path]',
    publicPath: 'http://localhost:3000',
    globalObject: 'this'
  },
  mode: 'development',
  plugins: [
     new webpack.SourceMapDevToolPlugin({
        filename: '[name].js.map'
     }),
  ]
};
