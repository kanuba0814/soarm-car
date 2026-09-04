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
    def __init__(self, room_path, asr_argv=None, controller=None, enqueue=None):
        self.room_path = room_path
        self.asr = CommandASR(asr_argv)
        self.controller = controller or SearchController(Workflow(), SimulationNavigation(), SimulationPathVerifier(), True)
        self.enqueue = enqueue
    def dispatch(self, action, data):
        if self.enqueue is not None and action in ('scan_goal', 'start_scan', 'finish_scan', 'analyze', 'preview', 'dock_goal', 'cancel'):
            self.enqueue(action, data)
            return dict(self.controller.status(), pending=True, accepted=True)
        return self.call(action, data)
    def call(self, action, data):
        controller, workflow = self.controller, self.controller.workflow
        with controller.lock:
            if action == 'load': workflow.load(MapBundle.load(ROOT / 'examples/demo.yaml', 'demo-v1'), [Room('study', ['书房', '学习室'], [(.5,.5),(2.5,.5),(2.5,1.5),(.5,1.5)], (1.5,1.,0))])
            elif action == 'draw_save_room':
                room = Room('study', ['书房', '学习室'], [tuple(point) for point in data['polygon']], tuple(data.get('scan_pose', (1.5,1.,0))))
                workflow.load(workflow.bundle, [room]); RoomStore.save(self.room_path, workflow.bundle, [room])
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
PAGE = '''<!doctype html><meta charset=utf-8><title>SOARM 任务控制台</title><style>body{background:#101820;color:#d9e6ed;font:15px system-ui;margin:0}.top{padding:18px;background:#182a38}.badge{background:#e35d33;padding:5px;font-weight:bold}.grid{display:grid;grid-template-columns:1fr 1fr;gap:16px;padding:16px}.card{border:1px solid #365061;padding:14px}button{margin:4px;padding:8px;background:#21485d;color:white;border:0}canvas{background:#263943;width:100%;height:180px}pre{white-space:pre-wrap}@media(max-width:700px){.grid{grid-template-columns:1fr}}</style><header class=top><b>SOARM / 找书桌停靠观察</b> <span class=badge>SIMULATION · MOCK PATH/RGB-D</span></header><main class=grid><section class=card><h2>阶段轨与操作</h2><input id=t value="去书房找书桌" aria-label=中文指令><p><button onclick="go('load')">载入地图</button><button onclick="go('proposal',{text:t.value})">生成意图</button><button onclick="go('confirm_intent')">确认意图</button><button onclick="go('scan_goal')">导航扫描点</button><button onclick="go('start_scan')">开始扫描</button><button onclick="go('finish_scan')">完成扫描</button><button onclick="go('analyze')">分析 mock</button><button onclick="go('preview')">候选预览</button><button onclick="go('confirm_dock')">确认候选</button><button onclick="go('dock_goal')">导航停靠</button><button onclick="go('cancel')">停止/取消</button></p></section><section class=card><h2>房间画布</h2><canvas id=map width=500 height=180 aria-label=点击绘制房间></canvas><button onclick="go('draw_save_room',{polygon:points})">保存房间</button><p>点击画布添加 polygon 点；scan pose 使用示例安全点。</p></section><section class=card><h2>状态/候选/淘汰原因</h2><pre id=o></pre></section><section class=card><h2>事件日志</h2><pre id=log></pre></section></main><script>let points=[];map.onclick=e=>{let r=map.getBoundingClientRect();points.push([.5+e.offsetX/r.width*2,.5+e.offsetY/r.height]);log.textContent='polygon '+JSON.stringify(points)};async function go(a,d={}){let r=await fetch('/api/'+a,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)});let x=await r.json();o.textContent=JSON.stringify(x,null,2);log.textContent+=`\n${a}: ${r.status}`};go('health')</script>'''
def server(app):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args): pass
        def reply(self, status, payload, content_type='application/json'):
            raw = payload.encode() if isinstance(payload, str) else json.dumps(payload, ensure_ascii=False).encode(); self.send_response(status); self.send_header('Content-Type', content_type + '; charset=utf-8'); self.send_header('Content-Length', str(len(raw))); self.end_headers(); self.wfile.write(raw)
        def do_GET(self): self.reply(200, PAGE, 'text/html') if self.path == '/' else self.reply(200, app.controller.status()) if self.path == '/api/health' else self.reply(404, {'error':'NOT_FOUND'})
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
