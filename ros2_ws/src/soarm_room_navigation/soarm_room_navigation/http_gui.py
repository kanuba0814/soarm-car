"""Loopback-only Chinese room-navigation console and strict HTTP boundary."""
import hashlib
import json
import threading
from dataclasses import replace
import secrets
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler

from .controller import NONTERMINAL
from .core import NavigationError, Plan, Room, RoomStore, parse_plan
from .planner import FixturePlanner


PAGE = r'''<!doctype html>
<html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>SOARM 房间导航</title>
<style>
:root{--bg:#101719;--panel:#172326;--line:#415156;--ink:#e4ece9;--muted:#9ba9a7;--amber:#e5a646;--stop:#d95040;--ok:#57bd87;--error:#f27d70}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:14px ui-monospace,Consolas,monospace}.bar{height:56px;display:flex;align-items:center;justify-content:space-between;padding:0 20px;border-bottom:1px solid var(--line);background:#121f22}.brand{font-weight:bold;letter-spacing:.08em}.tag{border:1px solid var(--line);padding:5px 7px;color:var(--muted)}.stop{background:var(--stop)!important;border-color:#f27d70!important}.layout{display:grid;grid-template-columns:minmax(480px,1.5fr) minmax(310px,.8fr);gap:12px;padding:12px;min-height:calc(100vh - 56px)}.panel{border:1px solid var(--line);background:var(--panel);padding:12px}.map-wrap{position:relative;min-height:440px;display:grid;place-items:center;background:#0a0e0f}canvas{max-width:100%;max-height:65vh;image-rendering:pixelated;border:1px solid #69777a}h2{font-size:12px;letter-spacing:.12em;margin:0 0 10px;color:var(--amber)}label{display:block;color:var(--muted);font-size:12px;margin:8px 0 3px}input,textarea,button{font:inherit}input,textarea{width:100%;background:#0f181a;border:1px solid var(--line);color:var(--ink);padding:8px}input:disabled{opacity:.55}textarea{height:56px;resize:vertical}button{cursor:pointer;color:var(--ink);background:#24383b;border:1px solid #668187;padding:8px 10px;margin:8px 6px 0 0}button:disabled{opacity:.45;cursor:not-allowed}.row{display:grid;grid-template-columns:1fr 1fr 1fr;gap:7px}.status{white-space:pre-wrap;overflow:auto;max-height:220px;background:#0b1112;border-left:3px solid var(--amber);padding:9px}.timeline{display:grid;gap:6px}.step{padding:7px;border-left:3px solid #526366;background:#10191b}.step.active{border-color:var(--amber)}.step.done{border-color:var(--ok)}.maphint{position:absolute;left:16px;bottom:16px;background:#101719dd;padding:6px;color:var(--muted)}.notice{min-height:38px;margin:0 0 10px;padding:9px;border-left:3px solid var(--ok);background:#102019}.notice.error{border-color:var(--error);background:#281816}.room-list{max-height:170px;overflow:auto;border:1px solid var(--line);background:#0f181a}.room-item{width:100%;margin:0;padding:8px;text-align:left;border:0;border-bottom:1px solid #334246;background:transparent}.room-item.selected{background:#294044;border-left:3px solid var(--amber)}.room-meta{color:var(--muted);font-size:12px}.editor-mode{color:var(--amber);font-size:12px}.choice{display:inline-flex;gap:4px;align-items:center;margin:8px 12px 0 0;color:var(--muted)}.choice input{width:auto}.hidden{display:none}@media(max-width:780px){.layout{grid-template-columns:1fr}.map-wrap{min-height:300px}.bar{padding:0 10px}.tag{display:none}}
</style>
<header class="bar"><span class="brand">SOARM / 房间导航控制台</span><span id="mode" class="tag"></span><button id="arm" onclick="call('arm')">解除停止锁定</button><button class="stop" onclick="call('stop')">停止导航</button></header>
<main class="layout"><section class="panel"><h2>占据地图 / 房间语义</h2><div class="map-wrap"><canvas id="map"></canvas><span class="maphint" id="coords">x -- / y --</span></div><div id="notice" class="notice">准备新增房间。</div><p class="editor-mode" id="editorMode">新增房间</p><button onclick="newRoom()">新增房间</button><button id="editButton" onclick="editSelected()">编辑已选房间</button><label>房间 ID</label><input id="rid" placeholder="例如 study"><label>别名（逗号分隔）</label><input id="aliases" placeholder="例如 书房,学习室"><label class="choice"><input type="radio" name="goalMode" value="auto" checked onchange="syncEditor()">自动安全中心</label><label class="choice"><input type="radio" name="goalMode" value="manual" onchange="syncEditor()">手动目标点</label><label class="choice"><input id="requireYaw" type="checkbox" onchange="syncEditor()">固定到达朝向</label><div id="goalFields" class="row hidden"><div><label>目标 X</label><input id="gx" type="number" step="0.01"></div><div><label>目标 Y</label><input id="gy" type="number" step="0.01"></div></div><div id="yawField"><label>目标朝向 rad</label><input id="gyaw" type="number" step="0.01" value="0"></div><div id="clickFields"><label class="choice"><input type="radio" name="clickMode" value="polygon" checked>边界点</label><label class="choice"><input type="radio" name="clickMode" value="goal">到达点</label></div><button onclick="clearPoly()">清除边界</button><button id="saveButton" onclick="saveRoom()">创建房间</button></section>
<aside class="panel"><h2 id="roomCount">已保存房间（0）</h2><div id="roomList" class="room-list"></div><h2 style="margin-top:18px">中文任务</h2><textarea id="instruction" placeholder="例如：去书房"></textarea><button onclick="propose(false)">生成计划</button><button id="fixture" onclick="propose(true)">演示计划</button><button id="confirm" onclick="confirmPlan()">确认执行</button><h2 style="margin-top:18px">执行时间线</h2><div id="timeline" class="timeline"></div><h2 style="margin-top:18px">状态</h2><pre id="status" class="status"></pre></aside></main>
<script>
let meta,pixels,points=[],status={},csrf='',selectedRoomId=null,editor={mode:'create',selected:null,revision:null};
const C=document.getElementById('map'),ctx=C.getContext('2d'),statusEl=el('status'),mode=el('mode'),timeline=el('timeline'),fixture=el('fixture'),confirm=el('confirm'),coords=el('coords');
function el(id){return document.getElementById(id)}
function world(x,y){let a=meta.origin[2],dx=x-meta.origin[0],dy=y-meta.origin[1];return[(Math.cos(a)*dx+Math.sin(a)*dy)/meta.resolution,(-Math.sin(a)*dx+Math.cos(a)*dy)/meta.resolution]}
function cell(x,y){let a=meta.origin[2],lx=x*meta.resolution,ly=y*meta.resolution;return[meta.origin[0]+Math.cos(a)*lx-Math.sin(a)*ly,meta.origin[1]+Math.sin(a)*lx+Math.cos(a)*ly]}
function px(p){let q=world(...p);return[q[0],meta.height-q[1]]}
function path(poly,color,close=true){if(!poly.length)return;ctx.strokeStyle=color;ctx.beginPath();poly.forEach((p,i)=>{let q=px(p);i?ctx.lineTo(...q):ctx.moveTo(...q)});if(close)ctx.closePath();ctx.stroke()}
function draw(){if(!meta)return;C.width=meta.width;C.height=meta.height;let image=ctx.createImageData(meta.width,meta.height);for(let i=0;i<pixels.length;i++){let p=pixels[i],j=i*4;image.data[j]=p;image.data[j+1]=p;image.data[j+2]=p;image.data[j+3]=255}ctx.putImageData(image,0,0);ctx.lineWidth=1;for(const room of status.rooms||[]){path(room.polygon,'#57bd87');let q=px(room.goal_pose);ctx.fillStyle='#57bd87';ctx.fillRect(q[0]-2,q[1]-2,5,5);ctx.fillText(room.aliases.join('/')+' ['+room.id+']',q[0]+5,q[1]-5)}path(points,'#e5a646',points.length>2)}
function pointerPoint(e){let r=C.getBoundingClientRect(),width=r.width-2*C.clientLeft,height=r.height-2*C.clientTop;return cell(e.offsetX/width*meta.width,(1-e.offsetY/height)*meta.height)}
C.onclick=e=>{if(!meta)return;let p=pointerPoint(e);if(goalMode()==='manual'&&document.querySelector('input[name=clickMode]:checked').value==='goal'){el('gx').value=p[0].toFixed(2);el('gy').value=p[1].toFixed(2)}else{points.push(p)}draw()};
C.onmousemove=e=>{if(!meta)return;let p=pointerPoint(e);coords.textContent='x '+p[0].toFixed(2)+' / y '+p[1].toFixed(2)};
function clearPoly(){points=[];draw()}
function banner(text,error=false){let node=el('notice');node.textContent=text;node.className='notice'+(error?' error':'')}
function defaultId(){let ids=new Set((status.rooms||[]).map(r=>r.id));for(let i=1;;i++){let candidate='room-'+i;if(!ids.has(candidate))return candidate}}
function goalMode(){return document.querySelector('input[name=goalMode]:checked').value}function newRoom(){selectedRoomId=null;editor={mode:'create',selected:null,revision:null};points=[];el('rid').disabled=false;el('rid').value=defaultId();el('aliases').value='';el('gx').value='';el('gy').value='';el('gyaw').value='0';document.querySelector('input[name=goalMode][value=auto]').checked=true;el('requireYaw').checked=false;syncEditor();banner('新增草稿尚未保存。');draw()}
function editSelected(){let room=(status.rooms||[]).find(r=>r.id===selectedRoomId);if(!room){banner('请先从已保存房间中选择一项。',true);return}editor={mode:'update',selected:room.id,revision:room.revision};points=room.polygon.map(p=>p.slice());el('rid').value=room.id;el('rid').disabled=true;el('aliases').value=room.aliases.join(',');el('gx').value=room.goal_pose[0];el('gy').value=room.goal_pose[1];el('gyaw').value=room.goal_pose[2];document.querySelector('input[name=goalMode][value='+room.goal_mode+']').checked=true;el('requireYaw').checked=room.require_yaw;syncEditor();banner('正在编辑 '+room.id+'，ID 已锁定。');draw()}
function selectRoom(id){selectedRoomId=id;renderRooms()}
function syncEditor(){let updating=editor.mode==='update',manual=goalMode()==='manual',fixed=el('requireYaw').checked;el('editorMode').textContent=updating?'编辑已选房间（'+editor.selected+'）':'新增房间';el('saveButton').textContent=updating?'保存对 '+editor.selected+' 的修改':'创建房间';el('goalFields').classList.toggle('hidden',!manual);el('yawField').classList.toggle('hidden',!fixed);el('clickFields').classList.toggle('hidden',!manual);el('gx').disabled=!manual;el('gy').disabled=!manual;el('gyaw').disabled=!fixed}
function renderRooms(){let rooms=status.rooms||[];el('roomCount').textContent='已保存房间（'+rooms.length+'）';let list=el('roomList');list.replaceChildren();for(let room of rooms){let button=document.createElement('button');button.className='room-item'+(room.id===selectedRoomId?' selected':'');button.onclick=()=>selectRoom(room.id);let title=document.createElement('strong'),detail=document.createElement('div');title.textContent=room.aliases.join(' / ');detail.className='room-meta';detail.textContent=room.id+' · 版本 '+room.revision;button.appendChild(title);button.appendChild(detail);list.appendChild(button)}el('editButton').disabled=!rooms.some(r=>r.id===selectedRoomId)}
async function request(action,data={}){let response=await fetch('/api/'+action,{method:'POST',headers:{'Content-Type':'application/json','X-CSRF-Token':csrf},body:JSON.stringify(data)});let payload=await response.json();if(payload.csrf_token)csrf=payload.csrf_token;return{response,payload}}
async function call(action,data={}){try{let result=await request(action,data);status=result.payload;render();if(!result.response.ok)banner(result.payload.error+': '+(result.payload.reason||'请求被拒绝'),true);return result.payload}catch(error){banner('请求失败：'+error.message,true)}}
function render(){csrf=status.csrf_token||csrf;statusEl.textContent=JSON.stringify(status,null,2);mode.textContent=(status.planner||'')+' | '+(status.runtime||'');fixture.style.display=status.fixture_allowed?'inline-block':'none';confirm.disabled=status.state!=='PROPOSAL'||!status.proposal_id;el('arm').disabled=!!status.task_unresolved||status.state==='RUNNING_NAV'||status.state==='RUNNING_WAIT'||status.state==='SETTLING'||status.state==='CANCELING';timeline.replaceChildren();for(let [i,a] of ((status.plan||{}).actions||[]).entries()){let d=document.createElement('div');d.className='step '+(i<status.step?'done':i===status.step?'active':'');d.textContent=(i+1)+'. '+({navigate:'前往房间',wait:'等待',return_to_start:'返回本次出发位置'}[a.kind]||a.kind)+(a.room_id?' '+a.room_id:'')+(a.fallback_room_id?'（导航失败时改去 '+a.fallback_room_id+'）':'')+(a.seconds?' '+a.seconds+' 秒':'');timeline.appendChild(d)}renderRooms();syncEditor();draw()}
async function load(){meta=await (await fetch('/api/map')).json();pixels=Uint8Array.from(atob(meta.pixels_b64),x=>x.charCodeAt(0));status=await (await fetch('/api/health')).json();csrf=status.csrf_token;newRoom();render()}
async function saveRoom(){try{let manual=goalMode()==='manual',fixed=el('requireYaw').checked,x=el('gx').value.trim(),y=el('gy').value.trim(),yaw=el('gyaw').value.trim();if((manual&&(!x||!y||![x,y].every(value=>Number.isFinite(Number(value)))))||(fixed&&(!yaw||!Number.isFinite(Number(yaw))))){banner('请填写有限的手动目标坐标'+(fixed?'和朝向':'')+'。',true);return}let data={mode:editor.mode,room_id:el('rid').value,aliases:el('aliases').value.split(',').map(x=>x.trim()).filter(Boolean),polygon:points,goal_mode:manual?'manual':'auto',require_yaw:fixed};if(manual)data.goal_pose=[Number(x),Number(y),fixed?Number(yaw):0];else if(fixed)data.goal_yaw=Number(yaw);if(editor.mode==='update'){data.expected_room_revision=editor.revision}let result=await request('save_room',data);status=result.payload;render();if(!result.response.ok){banner(result.payload.error+': '+(result.payload.reason||'保存失败')+'；草稿已保留。',true);return}if(editor.mode==='create'){let savedId=data.room_id;newRoom();render();banner('已创建房间 '+savedId+'。可以继续新增下一间。')}else{editor.revision=(status.rooms||[]).find(r=>r.id===editor.selected).revision;banner('房间已更新。');render()}}catch(error){banner('保存失败：'+error.message+'；草稿已保留。',true)}}
function propose(fixture){call('propose',{text:el('instruction').value,fixture})}function confirmPlan(){call('confirm',{proposal_id:status.proposal_id})}
setInterval(()=>fetch('/api/health').then(r=>r.json()).then(x=>{status=x;render()}).catch(()=>{}),700);load()
</script></html>'''


class App:
    def __init__(self, controller, bundle, rooms_path, planner, fixture_allowed=False, runtime='ROS', arm=None, readiness=None):
        self.controller, self.bundle, self.rooms_path, self.planner, self.arm = controller, bundle, rooms_path, planner, arm
        self.fixture_allowed, self.runtime = fixture_allowed, runtime
        self.readiness, self.arm_status = readiness, 'NOT_REQUESTED'
        self.planning_lock, self.planning_future = threading.Lock(), None
        self.csrf_token = secrets.token_urlsafe(32)
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='room-nav-planner')

    def close(self):
        self.executor.shutdown(wait=False, cancel_futures=True)

    def status(self):
        result = self.controller.status()
        with self.controller.lock:
            rooms = [{'id': room.id, 'aliases': room.aliases, 'polygon': room.polygon,
                      'goal_pose': room.goal_pose, 'goal_mode': room.goal_mode,
                      'require_yaw': room.require_yaw, 'revision': room.revision} for room in self.controller.rooms]
        result.update({'planner': self.planner.status, 'fixture_allowed': self.fixture_allowed, 'runtime': self.runtime,
                       'csrf_token': self.csrf_token, 'task_unresolved': self.controller.state.name == 'CANCELING',
                       'ready': bool(self.readiness and self.readiness()), 'arm_status': self.arm_status, 'rooms': rooms})
        return result

    def _save_room(self, data):
        mode = data.get('mode', 'create')
        room_id = str(data.get('room_id', '')).strip()
        if mode not in ('create', 'update'):
            raise NavigationError('BAD_REQUEST', 'room save mode must be create or update')
        if mode == 'update' and (not isinstance(data.get('expected_room_revision'), int) or isinstance(data.get('expected_room_revision'), bool)):
            raise NavigationError('STALE_ROOM', 'update requires an expected room revision')
        with self.controller.lock:
            if self.controller.state in NONTERMINAL:
                raise NavigationError('GOAL_ACTIVE', 'room edits are blocked while a task is active')
            existing = next((item for item in self.controller.rooms if item.id == room_id), None)
            if mode == 'create' and existing is not None:
                raise NavigationError('ROOM_EXISTS', 'create requires a new room ID')
            if mode == 'update' and existing is None:
                raise NavigationError('ROOM_NOT_FOUND', 'update requires an existing room ID')
            if mode == 'update' and data['expected_room_revision'] != existing.revision:
                raise NavigationError('STALE_ROOM', 'room was changed; reload it before updating')
            revision = 1 if existing is None else existing.revision + 1
            goal_mode = data.get('goal_mode')
            if goal_mode is None:
                goal_mode = 'manual' if 'goal_pose' in data else 'auto'
            if goal_mode not in ('auto', 'manual'):
                raise NavigationError('BAD_REQUEST', 'goal mode must be auto or manual')
            default_yaw = False if goal_mode == 'auto' else True
            require_yaw = data.get('require_yaw', default_yaw)
            if not isinstance(require_yaw, bool):
                raise NavigationError('BAD_REQUEST', 'require_yaw must be boolean')
            polygon = tuple(tuple(point) for point in data.get('polygon', []))
            if goal_mode == 'auto':
                goal_pose = self.bundle.safe_polygon_goal(polygon, require_yaw, data.get('goal_yaw', 0.0))
            else:
                goal_pose = tuple(data.get('goal_pose', []))
            room = Room(room_id, tuple(str(x).strip() for x in data.get('aliases', [])), polygon,
                        goal_pose, revision, goal_mode, require_yaw)
            rooms = list(self.controller.rooms) if existing is None else [room if item.id == room_id else item for item in self.controller.rooms]
            if existing is None:
                rooms.append(room)
            aliases = [alias for item in rooms for alias in item.aliases]
            if len(aliases) != len(set(aliases)) or not room.valid(self.bundle):
                raise NavigationError('INVALID_ROOM', 'room aliases, polygon, map extent, goal containment, or required footprint clearance are invalid')
            next_revision = self.controller.room_revision + 1
            RoomStore.save(self.rooms_path, self.bundle, rooms, next_revision)
            self.controller.replace_rooms(rooms)

    def _plan_async(self, generation, revision, rooms, planner, text):
        try:
            raw = planner.plan(text, rooms)
            parsed = parse_plan(raw, rooms, self.bundle, '', revision)
            if isinstance(parsed, Plan):
                request_identity = f'{parsed.proposal_id}:{generation}:{self.csrf_token}'
                parsed = replace(parsed, proposal_id=hashlib.sha256(request_identity.encode()).hexdigest())
            self.controller.complete_planning(generation, parsed)
        except NavigationError as error:
            self.controller.planning_failed(generation, error)
        except Exception:
            self.controller.planning_failed(generation, NavigationError('PLANNER_FAILURE', 'planner worker failed'))

    def dispatch(self, action, data):
        if action == 'save_room':
            self._save_room(data)
        elif action == 'propose':
            text = data.get('text', '')
            if not isinstance(text, str) or not text.strip() or len(text) > 500:
                raise NavigationError('INVALID_INPUT', 'instruction must be 1-500 characters')
            planner = FixturePlanner() if data.get('fixture') and self.fixture_allowed else self.planner
            with self.planning_lock:
                if self.planning_future is not None and not self.planning_future.done():
                    raise NavigationError('PLANNER_BUSY', 'one model request is already running; stop remains available')
                generation, revision, rooms = self.controller.begin_planning()
                self.planning_future = self.executor.submit(self._plan_async, generation, revision, rooms, planner, text)
        elif action == 'confirm':
            self.controller.confirm(data.get('proposal_id'))
        elif action == 'stop':
            self.controller.stop()
        elif action == 'arm':
            if self.arm is None or not self.arm():
                raise NavigationError('ARM_REFUSED', 'explicit arm requires a terminal task, fresh healthy map-matching localization, and a drained gate')
        else:
            raise NavigationError('UNKNOWN_ACTION', action)
        return self.status()


def server(app, port):
    origin = 'http://127.0.0.1:' + str(port)
    host = '127.0.0.1:' + str(port)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, status, payload, content_type='application/json'):
            raw = payload.encode() if isinstance(payload, str) else json.dumps(payload, ensure_ascii=False).encode()
            self.send_response(status); self.send_header('Content-Type', content_type + '; charset=utf-8')
            self.send_header('Content-Length', str(len(raw))); self.send_header('Cache-Control', 'no-store')
            self.end_headers(); self.wfile.write(raw)

        def do_GET(self):
            if self.path == '/': return self.reply(200, PAGE, 'text/html')
            if self.path == '/api/health': return self.reply(200, app.status())
            if self.path == '/api/map': return self.reply(200, app.bundle.map_payload())
            return self.reply(404, {'error': 'NOT_FOUND'})

        def do_POST(self):
            if not self.path.startswith('/api/'):
                return self.reply(404, {'error': 'NOT_FOUND'})
            if self.headers.get('Host') != host or self.headers.get('Origin') != origin:
                return self.reply(403, {'error': 'SAME_ORIGIN_REQUIRED'})
            if not secrets.compare_digest(self.headers.get('X-CSRF-Token', ''), app.csrf_token):
                return self.reply(403, {'error': 'CSRF_REJECTED'})
            if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                return self.reply(415, {'error': 'JSON_REQUIRED'})
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 16384:
                    raise NavigationError('REQUEST_TOO_LARGE', 'request must be 1-16384 bytes')
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise NavigationError('BAD_REQUEST', 'JSON object required')
                self.reply(200, app.dispatch(self.path.rsplit('/', 1)[-1], data))
            except NavigationError as error:
                self.reply(409, {'error': error.code, 'reason': error.message, **app.status()})
            except (ValueError, json.JSONDecodeError):
                self.reply(400, {'error': 'BAD_REQUEST'})
    return Handler
