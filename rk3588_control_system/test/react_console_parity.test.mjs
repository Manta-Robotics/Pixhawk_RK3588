import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {createRequire} from 'node:module';
import {fileURLToPath} from 'node:url';
import ts from 'typescript';
import {parseFragment} from 'parse5';
import {renderToStaticMarkup} from 'react-dom/server';
import {createElement} from 'react';

const root=new URL('../',import.meta.url);
const read=path=>fs.readFileSync(new URL(path,root),'utf8');
const original=read('frontend/mobile-preview-kimi-k26.html');
function canonical(node){
  if(node.nodeName==='#comment')return null;
  if(node.nodeName==='#text')return node.value;
  // The responsive drive wrapper groups the existing manual/follow controls
  // without changing their content, attributes or ordering.
  if(node.tagName==='div'&&node.attrs.some(a=>a.name==='class'&&a.value==='drive-control-area')
    &&node.parentNode?.attrs?.some(a=>a.name==='class'&&a.value==='drive-body')){
    return (node.childNodes??[]).map(canonical).flat().filter(n=>n!==null);
  }
  if(node.tagName==='script'||node.tagName==='link'&&node.attrs.some(a=>a.name==='rel'&&a.value==='preload'))return null;
  const attributes={};
  for(const {name,value} of node.attrs??[]){
    if(name.startsWith('on')||name==='src'&&value==='')continue; // Handlers are bound by TS; empty media URLs are omitted by React.
    if(name==='style'){
      attributes.style=Object.fromEntries(value.split(';').filter(s=>s.includes(':')).map(s=>{const index=s.indexOf(':');return [s.slice(0,index).trim(),s.slice(index+1).trim()];}).sort(([a],[b])=>a.localeCompare(b)));
    }else attributes[name]=value;
  }
  const children=[];
  for(const child of (node.childNodes??[]).map(canonical).flat().filter(n=>n!==null)){
    // Removing comments coalesces adjacent text nodes without changing text.
    if(typeof child==='string'&&typeof children.at(-1)==='string')children[children.length-1]+=child;
    else children.push(child);
  }
  return {tag:node.tagName??node.nodeName,attributes:Object.fromEntries(Object.entries(attributes).sort(([a],[b])=>a.localeCompare(b))),children};
}
test('React preserves the original controls, text, icons and inline styles inside the responsive wrapper',()=>{
  const code=ts.transpileModule(read('web/src/ConsoleLayout.tsx'),{compilerOptions:{jsx:ts.JsxEmit.ReactJSX,module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2022}}).outputText;
  const exports={};new Function('exports','require',code)(exports,createRequire(fileURLToPath(root)));
  const actual=renderToStaticMarkup(createElement(exports.ConsoleLayout));
  const body=original.split(/<body[^>]*>/)[1].split('<script')[0];
  assert.deepEqual(canonical(parseFragment(actual)),canonical(parseFragment(body)));
});
test('React uses the original CSS verbatim apart from line endings, including landscape rules',()=>{
  assert.equal(read('web/src/console.css').replace(/\r\n/g,'\n'),original.match(/<style>([\s\S]*?)<\/style>/)[1].replace(/\r\n/g,'\n'));
});
test('React keeps original body attributes, viewport and assets',()=>{
  const index=read('web/index.html');
  assert.match(index,/<body data-theme="dark" data-language="zh">/);
  assert.match(index,/initial-scale=1, viewport-fit=cover, user-scalable=no/);
  assert.match(read('web/src/main.tsx'),/createRoot\(document.body\)/);
  assert.match(read('web/src/console-controller.ts'),/closest\("#imuCalibrate"\)\)openImuCalibrationWizard/);
});
