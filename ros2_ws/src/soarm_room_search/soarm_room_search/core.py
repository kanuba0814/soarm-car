"""Dependency-free domain core for the room desk search workflow."""
import hashlib, json, math, re
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

class State(str, Enum):
    EMPTY='EMPTY'; INTENT_PROPOSED='INTENT_PROPOSED'; INTENT_CONFIRMED='INTENT_CONFIRMED'; NAVIGATING_TO_SCAN='NAVIGATING_TO_SCAN'; AT_SCAN_POSE='AT_SCAN_POSE'; SCANNING='SCANNING'; SCAN_READY='SCAN_READY'; ANALYZING='ANALYZING'; DESK_READY='DESK_READY'; DOCK_PREVIEW='DOCK_PREVIEW'; DOCK_CONFIRMED='DOCK_CONFIRMED'; NAVIGATING_TO_DOCK='NAVIGATING_TO_DOCK'; ARRIVED='ARRIVED'; FAILED='FAILED'; CANCELED='CANCELED'; BLOCKED='BLOCKED'

class WorkflowError(Exception):
    def __init__(self, code, message): self.code, self.message=code,message; super().__init__(message)

def sha256(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def load_pgm(path):
    raw=Path(path).read_bytes(); parts=raw.split(b'\n'); header=[]; index=0
    while len(header)<3:
        line=parts[index].strip(); index+=1
        if line and not line.startswith(b'#'): header.append(line)
    if header[0] != b'P5': raise ValueError('only binary P5 PGM is supported')
    w,h=map(int,header[1].split()); maximum=int(header[2]);
    if maximum != 255: raise ValueError('PGM max value must be 255')
    data=b'\n'.join(parts[index:])
    if len(data) != w*h: raise ValueError('invalid PGM dimensions')
    return w,h,data
@dataclass
class MapBundle:
    yaml_path: str; image_path: str; map_id: str; resolution: float; origin: tuple; width: int; height: int; pixels: bytes; yaml_sha256: str; image_sha256: str
    @classmethod
    def load(cls, yaml_path, map_id):
        text=Path(yaml_path).read_text(); values={}
        for line in text.splitlines():
            if ':' in line:
                key,value=line.split(':',1); values[key.strip()]=value.strip().strip('"\'')
        image=Path(yaml_path).parent / values['image']; w,h,pixels=load_pgm(image)
        origin=tuple(float(x.strip()) for x in values['origin'].strip('[]').split(','))
        return cls(str(yaml_path),str(image),map_id,float(values['resolution']),origin,w,h,pixels,sha256(yaml_path),sha256(image))
    def world_to_cell(self,x,y): return int((x-self.origin[0])/self.resolution), int((y-self.origin[1])/self.resolution)
    def free(self,x,y):
        cx,cy=self.world_to_cell(x,y)
        if not (0<=cx<self.width and 0<=cy<self.height): return False
        return self.pixels[(self.height-1-cy)*self.width+cx] >= 250
    def footprint_free(self,x,y,radius=0.30,margin=0.05):
        step=self.resolution/2; r=radius+margin
        for ix in range(math.ceil(-r/step),math.floor(r/step)+1):
            for iy in range(math.ceil(-r/step),math.floor(r/step)+1):
                if ix*ix+iy*iy <= (r/step)**2 and not self.free(x+ix*step,y+iy*step): return False
        return True
@dataclass
class Room:
    id: str; aliases: list; polygon: list; scan_pose: tuple
    def contains(self,p):
        x,y=p[:2]; inside=False
        for a,b in zip(self.polygon,self.polygon[1:]+self.polygon[:1]):
            if (a[1]>y)!=(b[1]>y) and x < (b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]: inside=not inside
        return inside
    def valid(self,bundle):
        if len(self.polygon)<3 or len(set(self.aliases)) != len(self.aliases): return False
        edges=list(zip(self.polygon,self.polygon[1:]+self.polygon[:1]))
        def cross(a,b,c): return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
        for i,(a,b) in enumerate(edges):
            for j,(c,d) in enumerate(edges):
                if abs(i-j) in (0,1,len(edges)-1): continue
                if cross(a,b,c)*cross(a,b,d)<0 and cross(c,d,a)*cross(c,d,b)<0:return False
        return self.contains(self.scan_pose) and bundle.footprint_free(*self.scan_pose[:2])

def parse_intent(text, rooms):
    m=re.fullmatch(r'\s*去\s*(.+?)\s*找书桌\s*',text)
    if not m: raise WorkflowError('INTENT_REJECTED','only “去 <房间> 找书桌” is permitted')
    name=m.group(1); found=[r for r in rooms if name in r.aliases]
    if len(found)!=1: raise WorkflowError('INTENT_REJECTED','unknown or ambiguous room alias')
    return {'room_id':found[0].id,'text':text}

def unwrap_complete(yaws, sectors=8):
    if len(yaws)<2:return False
    total=0.; seen=set(); last=yaws[0]
    for value in yaws:
        d=(value-last+math.pi)%(2*math.pi)-math.pi; total+=d; last=value; seen.add(int((value%(2*math.pi))/(2*math.pi/sectors)))
    return abs(total)>=2*math.pi-0.25 and len(seen)==sectors

def desk_hypotheses(observations):
    """Structured RGB-D seam; observations are {x,y,z,width,quality}."""
    return [o for o in observations if o.get('quality',0)>=.6 and .5<=o.get('height',o.get('z',0))<=1.2 and o.get('width',0)>=.5]

@dataclass
class Workflow:
    state: State=State.EMPTY; bundle: MapBundle=None; rooms: list=field(default_factory=list); proposal: dict=None; scan_revision: int=0; desks:list=field(default_factory=list); candidates:list=field(default_factory=list); reason:str=''; selected:dict=None
    def _need(self,*states):
        if self.state not in states: raise WorkflowError('INVALID_STATE','operation is not permitted in '+self.state.value)
    def load(self,bundle,rooms):
        if len({a for r in rooms for a in r.aliases}) != sum(len(r.aliases) for r in rooms) or not all(r.valid(bundle) for r in rooms): raise WorkflowError('INVALID_ROOM','rooms invalid or aliases conflict')
        self.bundle,self.rooms,self.state=bundle,rooms,State.EMPTY
    def propose(self,text): self._need(State.EMPTY,State.CANCELED,State.FAILED,State.BLOCKED); self.proposal=parse_intent(text,self.rooms); self.state=State.INTENT_PROPOSED; return self.proposal
    def confirm_intent(self): self._need(State.INTENT_PROPOSED); self.state=State.INTENT_CONFIRMED
    def scan_goal(self): self._need(State.INTENT_CONFIRMED); self.state=State.NAVIGATING_TO_SCAN
    def at_scan(self): self._need(State.NAVIGATING_TO_SCAN); self.state=State.AT_SCAN_POSE
    def start_scan(self): self._need(State.AT_SCAN_POSE); self.state=State.SCANNING
    def finish_scan(self,yaws):
        self._need(State.SCANNING)
        if not unwrap_complete(yaws): raise WorkflowError('INCOMPLETE_SCAN','yaw or sector coverage incomplete')
        self.scan_revision+=1; self.state=State.SCAN_READY
    def analyze(self,observations): self._need(State.SCAN_READY); self.state=State.ANALYZING; self.desks=desk_hypotheses(observations); self.state=State.DESK_READY; return self.desks
    def preview(self,revision=None,path_feasible=lambda x:True):
        self._need(State.DESK_READY)
        if revision is not None and revision!=self.scan_revision: raise WorkflowError('STALE_REVISION','scan revision stale')
        room=next(r for r in self.rooms if r.id==self.proposal['room_id']); out=[]
        for d in self.desks:
            x,y=d['dock']; distance=math.hypot(x-d['x'],y-d['y'])
            if room.contains((x,y)) and d.get('quality',0)>=.6 and self.bundle.footprint_free(x,y) and .45<=distance<=1.5 and d.get('line_of_sight',False) and path_feasible((x,y)):
                out.append(dict(d,score=round(d['quality']*10-distance,6)))
        if not out: raise WorkflowError('NO_SAFE_DOCK','no candidate passed safety filters')
        self.candidates=sorted(out,key=lambda d:(-d['score'],d['x'],d['y'])); self.state=State.DOCK_PREVIEW; return self.candidates
    def confirm_dock(self,index=0): self._need(State.DOCK_PREVIEW); self.selected=self.candidates[index]; self.state=State.DOCK_CONFIRMED
    def dock_goal(self): self._need(State.DOCK_CONFIRMED); self.state=State.NAVIGATING_TO_DOCK
    def arrived(self): self._need(State.NAVIGATING_TO_DOCK); self.state=State.ARRIVED
    def cancel(self): self.state=State.CANCELED; self.reason='canceled'
