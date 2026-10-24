type Callback = (...args: any[]) => void;
type Listener = {target: EventTarget; type: string; listener: EventListenerOrEventListenerObject; options?: boolean | AddEventListenerOptions};

/** Scope timers and listeners to a React mount, including effect remounts
 * that reuse DOM nodes. No browser global is patched.
 */
export function createConsoleScope() {
  const timeouts=new Set<number>(),intervals=new Set<number>(),frames=new Set<number>();
  const listeners: Listener[]=[];
  const writes=new Map<PropertyKey,PropertyDescriptor|undefined>();
  let disposed=false;
  const listen=(target:EventTarget,type:string,listener:EventListenerOrEventListenerObject,options?:boolean|AddEventListenerOptions)=>{
    if(disposed)return;
    target.addEventListener(type,listener,options);
    listeners.push({target,type,listener,options});
  };
  const setTimeout=(callback:Callback,delay=0,...args:any[])=>{
    const id=globalThis.window.setTimeout(()=>{timeouts.delete(id);if(!disposed)callback(...args);},delay);
    timeouts.add(id);return id;
  };
  const clearTimeout=(id?:number)=>{globalThis.window.clearTimeout(id);if(id!==undefined)timeouts.delete(id);};
  const setInterval=(callback:Callback,delay=0,...args:any[])=>{
    const id=globalThis.window.setInterval(()=>{if(!disposed)callback(...args);},delay);
    intervals.add(id);return id;
  };
  const clearInterval=(id?:number)=>{globalThis.window.clearInterval(id);if(id!==undefined)intervals.delete(id);};
  const requestAnimationFrame=(callback:FrameRequestCallback)=>{
    const id=globalThis.window.requestAnimationFrame(at=>{frames.delete(id);if(!disposed)callback(at);});frames.add(id);return id;
  };
  const cancelAnimationFrame=(id:number)=>{globalThis.window.cancelAnimationFrame(id);frames.delete(id);};
  const timers={setTimeout,clearTimeout,setInterval,clearInterval,requestAnimationFrame,cancelAnimationFrame};
  function facade<T extends Window|Document>(target:T):T {
    return new Proxy(target,{
      get(object,key){
        if(key==='addEventListener')return (type:string,listener:EventListenerOrEventListenerObject,options?:boolean|AddEventListenerOptions)=>{
          listen(object,type,listener,options);
        };
        if(key in timers)return timers[key as keyof typeof timers];
        const value=Reflect.get(object,key,object);
        // Constructors and exported app helpers must retain their prototypes.
        if(typeof value==='function'&&['removeEventListener','querySelector','querySelectorAll','getElementById','createElement','createElementNS','createTreeWalker','dispatchEvent','getComputedStyle'].includes(String(key)))return value.bind(object);
        return value;
      },
      set(object,key,value){
        if((object as unknown)===globalThis.window&&!writes.has(key))writes.set(key,Object.getOwnPropertyDescriptor(object,key));
        return Reflect.set(object,key,value,object);
      }
    });
  }
  return {
    ...timers,
    listen,
    window:facade(globalThis.window),document:facade(globalThis.document),
    dispose(){
      disposed=true;
      for(const id of timeouts)globalThis.window.clearTimeout(id);
      for(const id of intervals)globalThis.window.clearInterval(id);
      for(const id of frames)globalThis.window.cancelAnimationFrame(id);
      for(const {target,type,listener,options} of listeners)target.removeEventListener(type,listener,options);
      for(const [key,descriptor] of writes){if(descriptor)Object.defineProperty(globalThis.window,key,descriptor);else Reflect.deleteProperty(globalThis.window,key);}
      timeouts.clear();intervals.clear();frames.clear();listeners.length=0;writes.clear();
    }
  };
}
