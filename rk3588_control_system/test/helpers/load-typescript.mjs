import fs from 'node:fs';
import vm from 'node:vm';
import {resolve, dirname} from 'node:path';
import ts from 'typescript';

// Execute migrated browser modules in an isolated realm, with injected I/O.
export function loadTypeScript(path, globals = {}) {
  const context = vm.createContext({console, ...globals});
  const cache = new Map();
  function load(filename) {
    filename = resolve(filename);
    if (cache.has(filename)) return cache.get(filename);
    const exports = {};
    cache.set(filename, exports);
    const source = ts.transpileModule(fs.readFileSync(filename, 'utf8'), {
      compilerOptions: {module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022}
    }).outputText;
    const run = vm.runInContext('(function(exports, require){' + source + '\n})', context, {filename});
    run(exports, specifier => {
      if (!specifier.startsWith('.')) throw new Error('Unexpected test dependency: ' + specifier);
      return load(resolve(dirname(filename), specifier + '.ts'));
    });
    return exports;
  }
  return load(path);
}
