"""Standard-library demo GUI. It never claims hardware data: all observations are labeled simulation."""
import argparse, json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from .core import MapBundle, Room, Workflow, WorkflowError
ROOT=Path(__file__).parents[1]
class App:
 def __init__(self): self.workflow=Workflow(); self.simulation=True
 def status(self):
  w=self.workflow; return {'state':w.state.value,'reason':w.reason,'simulation':self.simulation,'map_id':w.bundle.map_id if w.bundle else None,'proposal':w.proposal,'candidates':w.candidates}
 def call(self,action,data):
  w=self.workflow
  if action=='load': w.load(MapBundle.load(ROOT/'examples/demo.yaml','demo-v1'),[Room('study',['书房','学习室'],[(.5,.5),(2.5,.5),(2.5,1.5),(.5,1.5)],(1.5,1.,0))])
  elif action=='draw_save_room':
   # The demo records only a valid, preconfigured editable polygon; production callers submit validated rooms.
   w.load(w.bundle,[Room('study',['书房','学习室'],data.get('polygon',[(.5,.5),(2.5,.5),(2.5,1.5),(.5,1.5)]),tuple(data.get('scan_pose',(1.5,1.,0))))])
  elif action=='proposal': w.propose(data.get('text',''))
  elif action=='confirm_intent': w.confirm_intent()
  elif action=='scan_goal': w.scan_goal()
  elif action=='at_scan': w.at_scan()
  elif action=='start_scan': w.start_scan()
  elif action=='finish_scan': w.finish_scan([i*6.283185307/16 for i in range(17)])
  elif action=='analyze': w.analyze([{'x':1.5,'y':1.25,'height':.75,'width':1.,'quality':.9,'dock':(1.5,.75),'line_of_sight':True,'source':'SIMULATION mock observation'}])
  elif action=='preview': w.preview(w.scan_revision)
  elif action=='confirm_dock': w.confirm_dock(data.get('index',0))
  elif action=='dock_goal': w.dock_goal()
  elif action=='arrived': w.arrived()
  elif action in ('cancel','stop_reset'): w.cancel() if action=='cancel' else setattr(w,'state',w.state.EMPTY)
  else: raise WorkflowError('UNKNOWN_ACTION',action)
  return self.status()
APP=App()
PAGE='''<!doctype html><meta charset="utf-8"><title>SOARM 找书桌</title><style>body{font-family:sans-serif;margin:2em}#sim{background:#b00;color:white;padding:.5em;font-weight:bold}button{margin:.2em;padding:.5em}pre{background:#eee;padding:1em}</style><h1>找书桌并停靠观察</h1><div id=sim>SIMULATION — mock RGB-D observations, no hardware motion</div><p><input id=text value="去书房找书桌"><button onclick="go('load')">载入地图</button><button onclick="go('draw_save_room')">房间绘制/保存</button><button onclick="go('proposal',{text:text.value})">文本/ASR proposal</button><button onclick="go('confirm_intent')">确认意图</button><button onclick="go('scan_goal')">导航到扫描点</button><button onclick="go('at_scan')">到达扫描点</button><button onclick="go('start_scan')">开始扫描</button><button onclick="go('finish_scan')">结束扫描</button><button onclick="go('analyze')">分析（mock）</button><button onclick="go('preview')">候选预览</button><button onclick="go('confirm_dock')">确认候选</button><button onclick="go('dock_goal')">导航停靠</button><button onclick="go('arrived')">到达</button><button onclick="go('cancel')">取消</button><button onclick="go('stop_reset')">停止/重置</button></p><pre id=o></pre><script>async function go(action,data={}){let r=await fetch('/api/'+action,{method:'POST',body:JSON.stringify(data)});o.textContent=JSON.stringify(await r.json(),null,2)}go('health')</script>'''
class Handler(BaseHTTPRequestHandler):
 def log_message(self,*args): pass
 def reply(self,status,payload,ctype='application/json'):
  data=payload.encode() if isinstance(payload,str) else json.dumps(payload,ensure_ascii=False).encode();self.send_response(status);self.send_header('Content-Type',ctype+'; charset=utf-8');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
 def do_GET(self):
  if self.path=='/': self.reply(200,PAGE,'text/html')
  elif self.path=='/api/health': self.reply(200,APP.status())
  else:self.reply(404,{'error':'NOT_FOUND'})
 def do_POST(self):
  if not self.path.startswith('/api/'): return self.reply(404,{'error':'NOT_FOUND'})
  try:
   size=int(self.headers.get('Content-Length','0')); data=json.loads(self.rfile.read(size) or b'{}'); self.reply(200,APP.call(self.path.rsplit('/',1)[-1],data))
  except WorkflowError as e:self.reply(409,{'error':e.code,'reason':e.message,'state':APP.workflow.state.value})
  except Exception as e:self.reply(400,{'error':'BAD_REQUEST','reason':str(e)})
def main():
 p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=8088);args=p.parse_args();print('SIMULATION GUI http://127.0.0.1:%d'%args.port,flush=True);ThreadingHTTPServer(('127.0.0.1',args.port),Handler).serve_forever()
if __name__=='__main__':main()
