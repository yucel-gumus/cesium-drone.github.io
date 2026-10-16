const path = require("path");
const CopyWebpackPlugin = require("copy-webpack-plugin");
const webpack = require("webpack");
const cleanDroneModel = require("./scripts/clean-drone-model.cjs");
require("dotenv").config({ quiet: true });

module.exports = {
  entry: "./src/main.js",
  output: {
    filename: "bundle.js",
    path: path.resolve(__dirname, "dist"),
    clean: true,
  },
  mode: "production",
  // Cesium is already minified; the app bundle stays small and readable in source maps.
  devtool: "source-map",
  module: {
    rules: [
      {
        test: /\.css$/,
        use: ["style-loader", "css-loader"],
      },
    ],
  },
  plugins: [
    new webpack.DefinePlugin({
      "process.env.CESIUM_ION_TOKEN": JSON.stringify(
        process.env.CESIUM_ION_TOKEN || "",
      ),
    }),
    new CopyWebpackPlugin({
      patterns: [
        { from: "public", to: "" },
        { from: "src/index.html" },
        { from: "CesiumDrone.glb", to: "drone.glb", transform: cleanDroneModel },
        {
          from: "node_modules/cesium/Build/Cesium",
          to: "cesium",
          globOptions: { ignore: ["**/index.js", "**/index.cjs"] },
          info: { minimized: true },
        },
      ],
    }),
  ],
  performance: { assetFilter: (filename) => filename === "bundle.js" },
  devServer: {
    static: [
      {
        directory: path.join(__dirname, "dist"),
        publicPath: "/",
      },
    ],
    compress: true,
    port: 9000,
    hot: false,
    client: { overlay: true },
  },
};
