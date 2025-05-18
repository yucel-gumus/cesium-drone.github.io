const path = require('path');
const CopyWebpackPlugin = require('copy-webpack-plugin');

module.exports = {
  entry: './src/main.js',
  output: {
    filename: 'bundle.js',
    path: path.resolve(__dirname, 'dist'),
    clean: true
  },
  mode: 'development',
  module: {
    rules: [
      {
        test: /\.css$/,
        use: ['style-loader', 'css-loader']
      }
    ]
  },
  plugins: [
    new CopyWebpackPlugin({
      patterns: [
        { from: 'public', to: '' },
        { from: 'src/points.json' },
        { from: 'src/index.html' },
        { from: 'src/style.css' },
        { from: 'CesiumDrone.glb' },
        { from: 'src/ui-controller.js' },
      ]
    })
  ],
  devServer: {
    static: [
      {
        directory: path.join(__dirname, 'dist'),
        publicPath: '/'
      },
      {
        directory: path.join(__dirname, 'src'),
        publicPath: '/'
      }
    ],
    compress: true,
    port: 9000,
    hot: true
  }
};
