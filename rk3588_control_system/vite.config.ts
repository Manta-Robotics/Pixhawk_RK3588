import { defineConfig } from 'vite';
import { cpSync, mkdirSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
const root=fileURLToPath(new URL('.',import.meta.url));
const apiProxy = {
  '/api': 'http://127.0.0.1:3000',
  '/health': 'http://127.0.0.1:3000',
  '/socket.io': { target: 'http://127.0.0.1:3000', ws: true }
};

export default defineConfig({
  root: 'web',
  publicDir: false,
  plugins:[{
    name:'original-console-assets',
    closeBundle(){
      cpSync(`${root}frontend/assets`,`${root}dist/web/assets`,{recursive:true});
      mkdirSync(`${root}dist/web/js`,{recursive:true});
      cpSync(`${root}frontend/js/mediamtx-webrtc-reader.js`,`${root}dist/web/js/mediamtx-webrtc-reader.js`);
    }
  }],
  build: { outDir: '../dist/web', emptyOutDir: true, assetsDir: 'bundles' },
  // Preview serves the copied images/video/scripts from dist/web. Do not
  // inherit the development asset proxies, which require a running backend.
  preview: { proxy: apiProxy },
  server: {
    host: '127.0.0.1',
    proxy: {
      ...apiProxy,
      '/assets': 'http://127.0.0.1:3000',
      '/js': 'http://127.0.0.1:3000'
    }
  }
});
