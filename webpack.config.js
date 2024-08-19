const path = require('path');

module.exports = {
  entry: './dronefocus.js', // Giriş noktası (entry point)
  output: {
    filename: './createRota.js', // Çıktı dosyası
    path: path.resolve(__dirname, 'dist') // Çıktı dizini
  },
  mode: 'production', // Prodüksiyon modu (optimizasyonlar yapar)
};
