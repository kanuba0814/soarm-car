"""Locked, bounded standard-library HTTP console for the single controller."""
import argparse, json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from .controller import SearchController, SimulationPathVerifier
from .core import CommandASR, MapBundle, Room, RoomStore, Workflow, WorkflowError
ROOT = Path(__file__).parents[1]
class ImmediateHandle:
    def cancel_goal_async(self): return None
class SimulationNavigation:
    def send(self, pose, accepted, completed): accepted(ImmediateHandle()); completed(True)
class App:
    def __init__(self, room_path, asr_argv=None, controller=None, enqueue=None, status_provider=None):
        self.room_path = room_path
        self.asr = CommandASR(asr_argv)
        self.controller = controller or SearchController(
            Workflow(), SimulationNavigation(), SimulationPathVerifier(), True)
        self.enqueue = enqueue
        self.status_provider = status_provider or self.controller.status
    def dispatch(self, action, data):
        if self.enqueue is not None and action in (
            'draw_save_room', 'scan_goal', 'start_scan', 'finish_scan',
            'analyze', 'preview', 'dock_goal', 'cancel',
        ):
            self.enqueue(action, data)
            return dict(self.controller.status(), pending=True, accepted=True)
        return self.call(action, data)
    def call(self, action, data):
        controller, workflow = self.controller, self.controller.workflow
        with controller.lock:
            if action == 'load':
                if not controller.simulation:
                    raise WorkflowError(
                        'HARDWARE_CONFIG_LOCKED',
                        'hardware map bundle is loaded from verified ROS parameters')
                workflow.load(
                    MapBundle.load(ROOT / 'examples/demo.yaml', 'demo-v1'),
                    [Room(
                        'study', ['书房', '学习室'],
                        [(.5, .5), (2.5, .5), (2.5, 1.5), (.5, 1.5)],
                        (1.5, 1., 0),
                    )],
                )
            elif action == 'draw_save_room':
                if workflow.bundle is None:
                    raise WorkflowError('NOT_READY', 'load a map bundle before defining rooms')
                room = Room(
                    str(data.get('room_id', '')).strip(),
                    [str(alias).strip() for alias in data.get('aliases', [])],
                    [tuple(point) for point in data['polygon']],
                    tuple(data['scan_pose']),
                )
                rooms = [existing for existing in workflow.rooms if existing.id != room.id]
                rooms.append(room)
                workflow.load(workflow.bundle, rooms)
                RoomStore.save(self.room_path, workflow.bundle, rooms)
            elif action == 'proposal': workflow.propose(data.get('text', ''))
            elif action == 'asr_proposal': workflow.proposal = self.asr.propose(workflow.rooms); workflow.state = workflow.state.INTENT_PROPOSED
            elif action == 'confirm_intent': workflow.confirm_intent()
            elif action == 'scan_goal': controller.navigate_scan(next(room for room in workflow.rooms if room.id == workflow.proposal['room_id']).scan_pose)
            elif action == 'start_scan': workflow.start_scan()
            elif action == 'finish_scan': workflow.finish_scan([(1.5, 1., i, i * 6.283185307 / 16) for i in range(17)])
            elif action == 'analyze': workflow.analyze([{'x':1.5,'y':1.25,'height':.75,'width':1.,'quality':.9,'dock':(1.5,.75),'line_of_sight':True,'source':'SIMULATION mock observation'}])
            elif action == 'preview': controller.preview()
            elif action == 'confirm_dock': workflow.confirm_dock(data.get('index', 0))
            elif action == 'dock_goal': controller.navigate_dock(workflow.selected['dock'] + (0.,))
            elif action in ('cancel', 'stop_reset'): controller.cancel()
            else: raise WorkflowError('UNKNOWN_ACTION', action)
            return controller.status()
PAGE = '''<!doctype html><meta charset=utf-8><meta name=viewport content="width=device-width"><title>SOARM 任务控制台</title><style>:root{--ink:#d9e6ed;--panel:#172a36;--line:#365061;--accent:#e35d33;--action:#26708d}*{box-sizing:border-box}body{background:#0c151b;color:var(--ink);font:15px ui-monospace,monospace;margin:0}.top{padding:18px 24px;background:#182a38;border-bottom:2px solid var(--accent)}.badge{background:var(--accent);padding:5px 8px;font-weight:bold}.grid{display:grid;grid-template-columns:1.15fr .85fr;gap:16px;padding:16px}.card{background:var(--panel);border:1px solid var(--line);padding:16px;box-shadow:4px 4px 0 #071015}h2{font-size:14px;letter-spacing:.08em;text-transform:uppercase}button,input{margin:4px;padding:9px;background:#10222d;color:white;border:1px solid var(--line)}button{background:var(--action);cursor:pointer}button:hover{filter:brightness(1.18)}button.stop{background:#9c3428}canvas{background:#263943;width:100%;height:220px;border:1px solid #547184}pre{white-space:pre-wrap;max-height:300px;overflow:auto}.fields{display:grid;grid-template-columns:1fr 1fr}.fields input{width:calc(100% - 8px)}@media(max-width:760px){.grid{grid-template-columns:1fr}.fields{grid-template-columns:1fr}}</style><header class=top><b>SOARM / ROOM SEARCH CONTROL</b> <span class=badge>SIMULATION · MOCK PATH/RGB-D</span></header><main class=grid><section class=card><h2>01 · 阶段控制</h2><input id=t value="去书房找书桌" aria-label=中文指令><p><button onclick="go('load')">载入演示地图</button><button onclick="go('proposal',{text:t.value})">生成意图</button><button onclick="go('confirm_intent')">确认意图</button><button onclick="go('scan_goal')">导航扫描点</button><button onclick="go('start_scan')">开始 360° 扫描</button><button onclick="go('finish_scan')">校验扫描</button><button onclick="go('analyze')">分析书桌</button><button onclick="go('preview')">规划停靠候选</button><button onclick="go('confirm_dock')">确认候选</button><button onclick="go('dock_goal')">导航停靠</button><button class=stop onclick="go('cancel')">停止 / 取消</button></p></section><section class=card><h2>02 · 语义房间编辑器</h2><div class=fields><input id=rid value=study placeholder="room_id"><input id=aliases value="书房,学习室" placeholder="别名，逗号分隔"><input id=sx value=1.5 placeholder="扫描点 X"><input id=sy value=1.0 placeholder="扫描点 Y"><input id=syaw value=0 placeholder="扫描朝向 rad"></div><canvas id=map width=500 height=220 aria-label=点击绘制房间></canvas><button onclick="points=[];draw()">清空多边形</button><button onclick="saveRoom()">验证并保存房间</button><p>点击画布逐点定义边界；每个 room_id 独立保存，别名全局唯一。</p></section><section class=card><h2>03 · 状态 / 候选</h2><pre id=o></pre></section><section class=card><h2>04 · 事件审计</h2><pre id=log></pre></section></main><script>let points=[];map.onclick=e=>{let r=map.getBoundingClientRect();points.push([.5+e.offsetX/r.width*2,.5+e.offsetY/r.height]);draw()};function draw(){let c=map.getContext('2d');c.clearRect(0,0,map.width,map.height);c.strokeStyle='#e35d33';c.lineWidth=3;c.beginPath();points.forEach((p,i)=>{let x=(p[0]-.5)/2*map.width,y=(p[1]-.5)*map.height;i?c.lineTo(x,y):c.moveTo(x,y)});if(points.length>2)c.closePath();c.stroke();log.textContent='polygon '+JSON.stringify(points)}function saveRoom(){go('draw_save_room',{room_id:rid.value,aliases:aliases.value.split(',').map(x=>x.trim()).filter(Boolean),polygon:points,scan_pose:[+sx.value,+sy.value,+syaw.value]})}async function go(a,d={}){let r=await fetch('/api/'+a,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)});let x=await r.json();o.textContent=JSON.stringify(x,null,2);log.textContent+=`\n${new Date().toLocaleTimeString()} ${a}: ${r.status}`};go('health')</script>'''
def server(app):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args): pass
        def reply(self, status, payload, content_type='application/json'):
            raw = payload.encode() if isinstance(payload, str) else json.dumps(payload, ensure_ascii=False).encode(); self.send_response(status); self.send_header('Content-Type', content_type + '; charset=utf-8'); self.send_header('Content-Length', str(len(raw))); self.end_headers(); self.wfile.write(raw)
        def do_GET(self):
            if self.path == '/':
                mode = 'SIMULATION · MOCK PATH/RGB-D' if app.controller.simulation else 'HARDWARE · FAIL-CLOSED'
                self.reply(200, PAGE.replace('SIMULATION · MOCK PATH/RGB-D', mode), 'text/html')
            elif self.path == '/api/health':
                self.reply(200, app.status_provider())
            else:
                self.reply(404, {'error': 'NOT_FOUND'})
        def do_POST(self):
            if not self.path.startswith('/api/'): return self.reply(404, {'error':'NOT_FOUND'})
            size = int(self.headers.get('Content-Length', '0'))
            if size > 16384: return self.reply(413, {'error':'REQUEST_TOO_LARGE'})
            try: self.reply(200, app.dispatch(self.path.rsplit('/', 1)[-1], json.loads(self.rfile.read(size) or b'{}')))
            except WorkflowError as error: self.reply(409, {'error':error.code, 'reason':error.message, 'state':app.controller.workflow.state.value})
            except Exception as error: self.reply(400, {'error':'BAD_REQUEST', 'reason':str(error)})
    return Handler
def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--port', type=int, default=8088); parser.add_argument('--rooms', default='/tmp/soarm-room-search-rooms.json'); args = parser.parse_args()
    app = App(args.rooms); print('SIMULATION GUI http://127.0.0.1:%d' % args.port, flush=True); ThreadingHTTPServer(('127.0.0.1', args.port), server(app)).serve_forever()
if __name__ == '__main__': main()
