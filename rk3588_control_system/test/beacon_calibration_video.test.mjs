import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
const html=fs.readFileSync(new URL('../frontend/mobile-preview-kimi-k26.html',import.meta.url),'utf8');
const source=html.slice(html.indexOf('function currentGimbalImageSource(){'),html.indexOf('\nfunction stopBeaconCalibrationPreview()'));
function select(transport,video,image){
  const ctx={state:{transportMode:'live'},gimbalVideoTransport:transport,$:s=>s==='#liveGimbalWebrtc'?video:image};
  vm.createContext(ctx);vm.runInContext(source,ctx);return ctx.currentGimbalImageSource();
}
test('calibration uses the visible MJPEG feed after WebRTC fallback, even if an old stream object remains',()=>{
  const video={hidden:true,srcObject:{},readyState:4},image={hidden:false,naturalWidth:1280};
  assert.equal(select('MJPEG',video,image),image);
});
test('calibration uses an active decoded WebRTC source and never displays a hidden frozen source',()=>{
  const video={hidden:false,srcObject:{},readyState:4},image={hidden:true,naturalWidth:1280};
  assert.equal(select('WEBRTC',video,image),video);
  video.hidden=true;assert.equal(select('WEBRTC',video,image),null);
});
test('a loading WebRTC element does not count as a usable calibration image',()=>{
  assert.equal(select('WEBRTC',{hidden:false,srcObject:{},readyState:0},{hidden:true,naturalWidth:0}),null);
});
test('overlay geometry remains usable while both video sources are unavailable',()=>{
  const ctx={currentGimbalImageSource:()=>null,$:()=>({clientWidth:960,clientHeight:540})};
  vm.createContext(ctx);const start=html.indexOf('function gimbalVideoMapping('),end=html.indexOf('\nfunction drawGimbalOverlay()',start);
  vm.runInContext(html.slice(start,end),ctx);const map=ctx.gimbalVideoMapping();
  assert.equal(map.sourceWidth,1920);assert.equal(map.sourceHeight,1080);assert.equal(map.scale,.5);
});
