const path = require('path');
const webpack = require('webpack')

module.exports = {
  entry: './src/app.js',
  devtool: 'cheap-source-map',
  output: {
    filename: 'adavisual.js',
    path: path.resolve(__dirname, 'dist'),
    devtoolModuleFilenameTemplate: '/Users/ilap/Projects/Tradebots/visualisation/src/app.js'
  },
  plugins: [
     new webpack.SourceMapDevToolPlugin({
        filename: '[name].js.map'
     }),
  ],
};
