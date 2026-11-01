// Rolling diagnostic step RMS. Each edge is inserted/removed once; control
// coordinates never depend on these statistics.
export class UwbStepMetrics {
  constructor(maxAgeMs=5000,capacity=240){
    this.maxAgeMs=maxAgeMs;this.capacity=capacity;this.items=new Array(capacity);this.clear();
  }
  clear(){this.head=0;this.size=0;this.rawSum=0;this.outputSum=0;this.pairs=0;}
  removeOldest(){
    if(!this.size)return;
    this.items[this.head]=undefined;this.head=(this.head+1)%this.capacity;this.size--;
    if(this.size){
      const next=this.items[this.head];
      if(next.edge){this.rawSum-=next.edge.raw;this.outputSum-=next.edge.output;this.pairs--;next.edge=null;}
    }
    if(!this.pairs){this.rawSum=0;this.outputSum=0;}
  }
  append(sample){
    while(this.size&&sample.mono-this.items[this.head].mono>this.maxAgeMs)this.removeOldest();
    if(this.size===this.capacity)this.removeOldest();
    const previous=this.size?this.items[(this.head+this.size-1)%this.capacity]:null;
    const item={...sample,edge:null};
    if(previous&&[previous.x,previous.y,sample.x,sample.y].every(Number.isFinite)){
      item.edge={raw:(sample.rawX-previous.rawX)**2+(sample.rawY-previous.rawY)**2,
        output:(sample.x-previous.x)**2+(sample.y-previous.y)**2};
      this.rawSum+=item.edge.raw;this.outputSum+=item.edge.output;this.pairs++;
    }
    this.items[(this.head+this.size)%this.capacity]=item;this.size++;
  }
  snapshot(){return {pairs:this.pairs,rawStepRmsCm:this.pairs?Math.sqrt(Math.max(0,this.rawSum)/this.pairs):null,
    smoothedStepRmsCm:this.pairs?Math.sqrt(Math.max(0,this.outputSum)/this.pairs):null};}
}
