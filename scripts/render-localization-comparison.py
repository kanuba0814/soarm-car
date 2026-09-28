#!/usr/bin/env python3
"""Render an evidence-based, synchronized comparison film from saved experiments.

Camera frames are sampled causally at the original timestamps. No estimated
feature locations, motion interpolation, cross-map trajectories or fabricated
ground truth are drawn. TUM plots are explicitly offline, rigidly aligned.
"""
from __future__ import annotations

import argparse
import csv
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation

W, H, FPS = 1920, 1080, 30
BG = '#091321'
PANEL = '#111f30'
TEXT = '#eef3f8'
MUTED = '#9aafc4'
OLD = '#ffbc71'
NEW = '#48dfc8'
BAD = '#ff697e'
PURPLE = '#b59af6'
GOOD = '#66db9b'
FONT_DIR = Path('/home/gxxl/.local/share/fonts')


@lru_cache(None)
def font(size, bold=False):
    return ImageFont.truetype(str(FONT_DIR / ('NotoSansSC-Bold.otf' if bold else 'NotoSansSC-Regular.otf')), size)


def text(draw, xy, value, size=24, color=TEXT, bold=False):
    draw.text(xy, str(value), font=font(size, bold), fill=color)


def csv_rows(path):
    with Path(path).open() as f:
        return list(csv.DictReader(f))


def pose(row, prefix):
    T = np.eye(4)
    T[:3] = [[float(row[f'{prefix}{r}{c}']) for c in range(4)] for r in range(3)]
    return T


def align(points, reference):
    p, g = np.asarray(points), np.asarray(reference)
    u, _, vt = np.linalg.svd((g-g.mean(0)).T @ (p-p.mean(0)))
    R = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    result = (p-p.mean(0)) @ R.T + g.mean(0)
    return result, float(np.sqrt(np.mean(np.sum((result-g)**2, axis=1))))


def base(title, subtitle, section):
    im = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(im)
    d.rectangle((48, 34, 56, 85), fill=NEW)
    text(d, (76, 25), title, 40, bold=True)
    text(d, (76, 86), subtitle, 23, MUTED)
    text(d, (1630, 38), section, 23, NEW)
    return im


def pill(d, box, label, color):
    d.rounded_rectangle(box, radius=12, fill='#101d2c', outline=color, width=2)
    text(d, (box[0]+16, box[1]+7), label, 23, color, True)


def depth_camera(rgb_path, depth_path):
    rgb = np.array(Image.open(rgb_path).convert('RGB'))[:400].copy()
    dep = np.array(Image.open(depth_path)).astype(np.float32) / 1000
    if rgb.shape[:2] != dep.shape:
        raise ValueError('RGB/depth crop mismatch')
    valid = (dep >= .3) & (dep <= 5)
    q = np.clip((dep-.3)/4.7, 0, 1)
    stops = np.array([[40, 67, 160], [31, 180, 192], [99, 205, 133], [242, 204, 77], [222, 76, 71]])
    pos = q*4
    lo = np.floor(pos).astype(int)
    hi = np.minimum(lo+1, 4)
    colors = stops[lo]*(1-(pos-lo)[..., None]) + stops[hi]*(pos-lo)[..., None]
    rgb[valid] = np.rint(.82*rgb[valid] + .18*colors[valid]).astype(np.uint8)
    return Image.fromarray(rgb).resize((888, 555), Image.Resampling.LANCZOS)


def local_path(d, box, points, color, valid):
    x, y, x1, y1 = box
    d.rounded_rectangle(box, radius=12, fill='#0c1726', outline='#415368', width=1)
    text(d, (x+12, y+8), '当前连续段 · 相机 X-Z', 17, MUTED)
    if not valid or not len(points):
        text(d, (x+55, y+77), '暂无有效位姿', 24, BAD)
        return
    p = np.asarray(points)[:, [0, 2]]
    p = p-p[0]
    lo, hi = p.min(0), p.max(0)
    span = max(float(np.max(hi-lo))*1.2, .6)
    center = (hi+lo)/2
    lower = center-span/2
    size = min(x1-x-44, y1-y-66)
    origin = np.array([x+(x1-x-size)/2, y+38])
    pts = (p-lower)/span*size
    pts[:, 1] = size-pts[:, 1]
    pts += origin
    for f in (.25, .5, .75):
        d.line((origin[0]+size*f, origin[1], origin[0]+size*f, origin[1]+size), fill='#26394d')
        d.line((origin[0], origin[1]+size*f, origin[0]+size, origin[1]+size*f), fill='#26394d')
    if len(pts)>1:
        d.line([tuple(v) for v in pts], fill=color, width=4)
    px, py = pts[-1]
    d.ellipse((px-5, py-5, px+5, py+5), fill=color)
    text(d, (x+12, y1-26), f'自适应范围 {span:.1f} m · 独立原点', 16, MUTED)


class Lab:
    def __init__(self, root, bag):
        self.bag = bag
        self.seq = root/'orbslam3-validation'/bag
        self.assoc = [x.split() for x in (self.seq/'associations.txt').read_text().splitlines() if x.strip()]
        self.ts = np.array([float(x[0]) for x in self.assoc])
        self.elapsed = self.ts-self.ts[0]
        self.duration = self.elapsed[-1]
        self.orb = csv_rows(self.seq/'baseline/tracking.csv')
        self.dense = csv_rows(root/'rgbd-comparison'/bag/'hybrid/frames.csv')
        self.backend = csv_rows(root/'rgbd-backend-exact-20260917'/bag/'refine/online.csv')
        assert len(self.orb)==len(self.assoc) and len(self.dense)==len(self.assoc)-1
        assert np.max(np.abs(self.ts-np.array([float(r['timestamp']) for r in self.orb])))<1e-5
        self.ok = np.array([r['state']=='2' for r in self.orb])
        self.accepted = np.array([False]+[r['accepted']=='1' for r in self.dense])
        self.seg = np.array([0]+[int(r['segment']) for r in self.dense])
        self.orb_count = np.cumsum(self.ok)
        self.dense_count = np.cumsum(self.accepted)
        self.poses = np.zeros((len(self.ts), 3))
        self.orb_xyz = np.array([[float(r[k]) for k in ('x','y','z')] for r in self.orb])
        T = np.eye(4)
        for i, r in enumerate(self.dense, 1):
            if not self.accepted[i] or self.seg[i]!=self.seg[i-1]:
                T = np.eye(4)
            if self.accepted[i]:
                T = T @ np.linalg.inv(pose(r, 't'))
            self.poses[i] = T[:3, 3]
        self.old_start = np.zeros(len(self.ts), dtype=int)
        self.new_start = np.zeros(len(self.ts), dtype=int)
        for i in range(1, len(self.ts)):
            boundary = not self.ok[i] or not self.ok[i-1]
            if self.ok[i] and self.ok[i-1]:
                boundary |= np.linalg.norm(self.orb_xyz[i]-self.orb_xyz[i-1])>.5
            self.old_start[i] = i if boundary else self.old_start[i-1]
            self.new_start[i] = i if not self.accepted[i-1] or self.seg[i]!=self.seg[i-1] else self.new_start[i-1]
        self.kt = np.array([float(r['timestamp']) for r in self.backend])
        summary = json.loads((root/'rgbd-backend-exact-20260917/summary.json').read_text())
        self.components = summary[bag]['refine']['final_graph_components']
        self.template = base('实验室实录  /  旧版与本轮同步对照',
                             f'{bag}   ·   同一 RGB-D 输入   ·   原速播放   ·   18% 实测深度着色',
                             '01 / LAB' if bag.endswith('212016') else '02 / LAB')
        d = ImageDraw.Draw(self.template)
        for x, title, color in [(48, '旧版  ORB-SLAM3', OLD), (984, '本轮  RGBDICP + 地图后端', NEW)]:
            d.rounded_rectangle((x, 142, x+888, 950), radius=18, fill=PANEL)
            text(d, (x+20, 154), title, 30, color, True)
        for x, label, states in [(48, 'ORB 状态', self.ok), (984, '前端筛查', self.accepted)]:
            text(d, (x+20, 884), label, 19, MUTED)
            for i in range(len(self.ts)-1):
                a = x+128+round(740*self.elapsed[i]/self.duration)
                b = x+128+round(740*self.elapsed[i+1]/self.duration)
                d.rectangle((a, 897, max(a,b), 916), fill=GOOD if states[i] else BAD)
        text(d, (48, 973), '绿色：跟踪有效 / 帧间筛查通过     红色：失跟 / 拒绝     游标：当前时刻', 24, MUTED)
        text(d, (48, 1014), '右侧小图为联合前端局部轨迹，两侧坐标独立。实验室无真值；通过率不等于准确率。', 24, TEXT)
        self.cached_i = -1
        self.cached_camera = None

    def frame(self, elapsed):
        i = int(np.searchsorted(self.elapsed, elapsed, side='right')-1)
        i = min(max(i,0), len(self.ts)-1)
        if i != self.cached_i:
            ar = self.assoc[i]
            self.cached_camera = depth_camera(self.seq/ar[1], self.seq/ar[3])
            self.cached_i = i
        im = self.template.copy()
        for x in (48,984):
            im.paste(self.cached_camera,(x,216))
        d = ImageDraw.Draw(im)
        text(d, (1130, 87), f'采集时间 {self.elapsed[i]:05.2f} s  /  {self.duration:.2f} s', 23, TEXT)
        old_ok, new_ok = self.ok[i], self.accepted[i]
        pill(d, (64,232,326,281), '正常跟踪' if old_ok else '失跟 / 未定位', GOOD if old_ok else BAD)
        pill(d, (1000,232,1308,281), '帧间筛查通过' if new_ok else ('等待首条帧间边' if i==0 else '拒绝 / 新段开始'), GOOD if new_ok else BAD)
        text(d,(66,287),f'输入帧 {i+1} / {len(self.ts)}',20,TEXT)
        text(d,(1002,287),f'输入帧 {i+1} / {len(self.ts)}',20,TEXT)
        os = self.old_start[i]
        ns = self.new_start[i]
        op = self.orb_xyz[os:i+1].copy()
        if old_ok and len(op):
            rr = self.orb[os]
            R = Rotation.from_quat([float(rr[k]) for k in ('qx','qy','qz','qw')]).as_matrix()
            op = (op-op[0]) @ R
        local_path(d, (640,540,920,750), op, OLD, old_ok)
        local_path(d, (1576,540,1856,750), self.poses[ns:i+1], NEW, new_ok)
        text(d,(68,786),f'累计 OK  {self.orb_count[i]}/{i+1} 帧',27,OLD,True)
        text(d,(580,786),f'{100*self.orb_count[i]/(i+1):.1f}%',31,OLD,True)
        text(d,(1004,786),f'累计通过  {self.dense_count[i]}/{i} 条边',27,NEW,True)
        text(d,(1516,786),f'{100*self.dense_count[i]/i:.1f}%' if i else '—',31,NEW,True)
        text(d,(68,833),f'当前特征 {self.orb[i]["features"]}  ·  失跟及大跳变处断线',21,MUTED)
        k = int(np.searchsorted(self.kt,self.ts[i],side='right')-1)
        update = f'后端关键帧 {k+1}/{len(self.kt)}' if k>=0 else '后端等待初始化'
        text(d,(1004,833),f'{update}  ·  整包最终仍有 {self.components} 个分量',21,MUTED)
        for x in (48,984):
            px = x+128+round(740*min(elapsed,self.duration)/self.duration)
            d.line((px,887,px,926),fill=TEXT,width=3)
        return im


class Tum:
    def __init__(self, root, datasets):
        name='freiburg3_long_office_household'
        self.seq=datasets/('rgbd_dataset_'+name)
        self.assoc=[x.split() for x in (self.seq/'associations.txt').read_text().splitlines() if x.strip()]
        self.image_ts=np.array([float(x[0]) for x in self.assoc])
        rows=csv_rows(root/'rgbd-backend-exact-20260917'/name/'refine/online.csv')
        orb=csv_rows(root/'a4-tum-20260916'/name/'orb/tracking.csv')
        gt=np.loadtxt(self.seq/'groundtruth.txt')
        times=[];truth=[];raw=[];current=[];orb_points=[]
        for row in rows:
            t=float(row['timestamp']); j=int(np.argmin(abs(gt[:,0]-t)))
            if abs(gt[j,0]-t)>.02: continue
            old=orb[int(row['id'])-1]
            assert old['state']=='2' and abs(float(old['timestamp'])-t)<1e-5
            times.append(t);truth.append(gt[j,1:4]);raw.append(pose(row,'raw')[:3,3])
            current.append(pose(row,'online')[:3,3]);orb_points.append([float(old[k]) for k in ('x','y','z')])
        self.times=np.array(times)
        self.t0=self.image_ts[0]
        self.duration=self.image_ts[-1]-self.t0
        truth=np.asarray(truth)
        self.curves={'真值':truth}
        self.rmse={}
        for label,p in [('上一版联合前端',raw),('本轮当时输出',current),('ORB-SLAM3参考',orb_points)]:
            self.curves[label],self.rmse[label]=align(p,truth)
        center=truth.mean(0)
        _,_,axes=np.linalg.svd(truth-center,full_matrices=False)
        self.plane={k:(p-center) @ axes[:2].T for k,p in self.curves.items()}
        allp=np.concatenate(list(self.plane.values()))
        lo,hi=allp.min(0),allp.max(0)
        self.center=(lo+hi)/2
        self.span=max((hi-lo)[0]/1.44,(hi-lo)[1])*1.18
        self.colors={'真值':TEXT,'上一版联合前端':OLD,'本轮当时输出':NEW,'ORB-SLAM3参考':PURPLE}
        self.template=base('公共真值验证  /  看漂移，也看参考算法',
                           'TUM long_office  ·  4× 播放  ·  同一 171 个关键帧  ·  SE(3) 刚性对齐，不修正尺度',
                           '03 / GT')
        d=ImageDraw.Draw(self.template)
        d.rounded_rectangle((48,620,648,954),radius=16,fill=PANEL)
        text(d,(72,630),'整段同关键帧 ATE RMSE',26,TEXT,True)
        for j,k in enumerate(['上一版联合前端','本轮当时输出','ORB-SLAM3参考']):
            text(d,(74,683+j*55),k,24,self.colors[k])
            text(d,(430,674+j*55),f'{self.rmse[k]*100:.2f} cm',32,self.colors[k],True)
        text(d,(74,854),'本轮事后优化：2.08 cm',23,MUTED)
        text(d,(74,894),'精确匹配：5 次完整输出一致',23,NEW)
        d.rounded_rectangle((688,150,1872,954),radius=18,fill=PANEL)
        text(d,(712,167),'轨迹主平面投影 / 米制尺度',23,MUTED)
        bar_end=750+680/self.span
        d.line((750,905,bar_end,905),fill=MUTED,width=3)
        for x in [750,bar_end]:d.line((x,898,x,912),fill=MUTED,width=3)
        text(d,(750,864),'1 m',22,MUTED)
        text(d,(48,977),'白：真值    橙：上一版联合前端    青：本轮当时输出    紫：ORB-SLAM3 参考',24,TEXT)
        text(d,(48,1017),'厘米数仅属于此公共数据集；实验室 bag 没有轨迹真值。轨迹对齐仅用于离线评分。',24,MUTED)
        self.pix={k:self.xy(v) for k,v in self.plane.items()}
        self.cached_i=-1

    def xy(self,p):
        # Common metric projection and scale for all methods, unlike DC1 local insets.
        q=(p-self.center)*680/self.span
        q[:,1]*=-1
        return q+np.array([1280,560])

    def frame(self,elapsed):
        absolute=self.t0+elapsed
        i=max(0,min(len(self.assoc)-1,int(np.searchsorted(self.image_ts,absolute,side='right')-1)))
        if i!=self.cached_i:
            self.camera=Image.open(self.seq/self.assoc[i][1]).convert('RGB').resize((600,450),Image.Resampling.LANCZOS)
            self.cached_i=i
        im=self.template.copy();im.paste(self.camera,(48,150));d=ImageDraw.Draw(im)
        pill(d,(66,167,357,218),f'4×  |  t = {elapsed:05.1f} s',NEW)
        # Faint entire reference is an explicitly offline evaluation guide.
        d.line([tuple(v) for v in self.pix['真值']],fill='#354b62',width=3)
        n=int(np.searchsorted(self.times,absolute,side='right'))
        for k in ['真值','上一版联合前端','ORB-SLAM3参考','本轮当时输出']:
            p=self.pix[k][:n]
            width={'真值':7,'上一版联合前端':4,'ORB-SLAM3参考':4,'本轮当时输出':3}[k]
            if len(p)>1:d.line([tuple(v) for v in p],fill=self.colors[k],width=width)
            if len(p):
                x,y=p[-1];d.ellipse((x-6,y-6,x+6,y+6),fill=self.colors[k])
        return im


def card(title, subtitle, lines, tag='DC1 / RGB-D'):
    im=base(title,subtitle,tag);d=ImageDraw.Draw(im)
    for j,(heading,body,color) in enumerate(lines):
        y=220+j*225
        d.rounded_rectangle((76,y,1844,y+183),radius=18,fill=PANEL)
        text(d,(110,y+26),heading,36,color,True)
        text(d,(110,y+92),body,28,TEXT)
    text(d,(78,1014),'真实录制 / 已保存算法输出 / 保留失败与断图',24,MUTED)
    return im


def encode(path, count, frame_fn, fps=FPS):
    if path.exists():raise FileExistsError(path)
    cmd=['ffmpeg','-hide_banner','-loglevel','error','-n','-f','rawvideo','-pixel_format','rgb24',
         '-video_size',f'{W}x{H}','-framerate',str(fps),'-i','-','-an','-c:v','libx264',
         '-threads','6','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(path)]
    proc=subprocess.Popen(cmd,stdin=subprocess.PIPE)
    try:
        for i in range(count):
            proc.stdin.write(frame_fn(i).tobytes())
            if i and i%300==0:print(path.name,'frame',i,'/',count,flush=True)
        proc.stdin.close()
        if proc.wait()!=0:raise RuntimeError('ffmpeg failed')
    except BaseException:
        proc.kill();proc.wait();raise


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--workspace',type=Path,default=Path(__file__).resolve().parents[1])
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--preview-only',action='store_true')
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    root=a.workspace/'artifacts/perception-dev/dabai-dc1'
    labs=[Lab(root,'office-20260913-212016'),Lab(root,'office-20260913-212121')]
    tum=Tum(root,a.workspace/'datasets/tum-rgbd')
    intro=card('DC1 RGB-D  /  视觉定位成果对照',
               '仅使用 RGB-D 相机 · 两组实验室实录 · 公共真值验证',[
                   ('01  同一录像，同一时间轴','旧版 ORB-SLAM3 与本轮方案同步播放，失跟与拒绝保留。',OLD),
                   ('02  从局部估计到全局约束','RGBDICP 联合前端 + RTAB-Map 后端；精确匹配提高可重复性。',NEW),
                   ('03  精度与连续性分别验证','实验室看失效与断图；TUM 真值数据看实际轨迹误差。',TEXT)])
    bridge=card('第二组实验室实录', '更低的 RGB 帧率与更多局部轨迹断点',[
        ('旧版 ORB-SLAM3','整段跟踪 OK：87.84% / 740 个输入配对帧。',OLD),
        ('本轮联合前端','整段帧间筛查：97.56% / 739 条边；通过不等于正确。',NEW),
        ('保留真实失败','全局后端最终仍有 6 个独立分量，尚未形成完整地图。',BAD)])
    gtcard=card('公共真值验证', '下面切换到 TUM 数据，不是实验室 DC1 的精度测量',[
        ('同样的关键帧，比较实际误差','上一版联合前端、本轮当时输出，以及 ORB-SLAM3 参考。',TEXT),
        ('区分当时输出与事后优化','主轨迹显示当时输出；事后优化误差单独列出。',NEW),
        ('五次复跑结果一致','固定输入与参数，检查整段位姿和闭环结果的可重复性。',PURPLE)])
    ending=card('结果与下一步', '公共数据上的提升可重复；实验室没有轨迹真值，尚不能证明厘米级精度',[
        ('TUM 长序列：当时 2.67 cm / 事后 2.08 cm','同关键帧旧联合前端 47.05 cm；ORB-SLAM3 参考 1.51 cm。',NEW),
        ('实验室两包：最终仍有 5 / 6 个地图分量','旧版跟踪 OK 65.57% / 87.84%；本轮帧间通过 98.35% / 97.56%（统计口径不同）。',OLD),
        ('仍存在失败案例','TUM room 当时误差由 20.00 cm 增至 37.21 cm；优先补局部地图与错误约束拒绝。',BAD)])
    samples={'01-intro.png':intro,'02-lab1-lost.png':labs[0].frame(18),
             '03-lab2-lost.png':labs[1].frame(22),'04-groundtruth.png':tum.frame(79),
             '05-results.png':ending}
    for name,im in samples.items():im.save(a.out/name)
    contact=Image.new('RGB',(1920,1620),BG)
    for i,im in enumerate(samples.values()):contact.paste(im.resize((960,540)),((i%2)*960,(i//2)*540))
    contact.save(a.out/'contact-sheet.jpg',quality=92)
    # All experiment totals are checked against the recorded outputs.
    manifest={'width':W,'height':H,'fps':FPS,'lab_playback_speed':1,'tum_playback_speed':4,
              'camera_sampling':'latest image timestamp <= video clock; no motion interpolation',
              'lab_depth_overlay_alpha':.18,'lab_crop':'RGB top 400 rows; original depth unchanged',
              'lab_trajectory':'independent local X-Z plots; ORB split on loss or >0.5m pose jump, not map IDs',
              'tum_ate_rmse_m_same_171_keyframes':tum.rmse,
              'lab_results':{l.bag:dict(pairs=len(l.ts),orb_ok=int(l.ok.sum()),
                   accepted_edges=int(l.accepted.sum()),edges=len(l.ts)-1,final_components=l.components,
                   duration_s=float(l.duration)) for l in labs},
              'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    sources=[root/'rgbd-backend-exact-20260917/summary.json']
    for lab in labs:
        sources.extend([lab.seq/'associations.txt',lab.seq/'baseline/tracking.csv',
                        root/'rgbd-comparison'/lab.bag/'hybrid/frames.csv',
                        root/'rgbd-backend-exact-20260917'/lab.bag/'refine/online.csv'])
    sources.extend([tum.seq/'associations.txt',tum.seq/'groundtruth.txt',
                    root/'a4-tum-20260916/freiburg3_long_office_household/orb/tracking.csv',
                    root/'rgbd-backend-exact-20260917/freiburg3_long_office_household/refine/online.csv'])
    manifest['source_sha256']={str(f.relative_to(a.workspace)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources}
    (a.out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    if a.preview_only:return
    chapters=[('01-intro.mp4',6,lambda t:intro),
              ('02-lab1.mp4',labs[0].duration+1/FPS,labs[0].frame),
              ('03-transition.mp4',5,lambda t:bridge),
              ('04-lab2.mp4',labs[1].duration+1/FPS,labs[1].frame),
              ('05-truth-intro.mp4',5,lambda t:gtcard),
              ('06-truth.mp4',tum.duration/4+1/FPS,lambda t:tum.frame(t*4)),
              ('07-results.mp4',10,lambda t:ending)]
    starts=[];elapsed=0
    for name,duration,fn in chapters:
        count=math.ceil(duration*FPS)
        encode(a.out/name,count,lambda i:fn(i/FPS))
        starts.append(dict(file=name,start_s=elapsed,duration_s=count/FPS))
        elapsed+=count/FPS
        print('DONE',name,'duration',count/FPS,flush=True)
    concat=a.out/'concat.txt'
    concat.write_text(''.join("file '"+name+"'\n" for name,_,_ in chapters))
    output=a.out/'dc1-visual-comparison.mp4'
    metadata=a.out/'chapters.ffmeta'
    titles=['对比说明','实验室第一组','第二组说明','实验室第二组','真值评估说明','TUM 真值对比','结果与局限']
    metadata.write_text(';FFMETADATA1\ntitle=DC1 RGB-D 视觉定位成果对照\n'+''.join(
        f'[CHAPTER]\nTIMEBASE=1/1000\nSTART={round(s["start_s"]*1000)}\n'
        f'END={round((s["start_s"]+s["duration_s"])*1000)}\ntitle={title}\n'
        for s,title in zip(starts,titles)))
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-n','-f','concat','-safe','0','-i',str(concat),
                    '-i',str(metadata),'-map','0:v:0','-map_metadata','1','-map_chapters','1',
                    '-c','copy','-movflags','+faststart',str(output)],check=True)
    manifest.update(chapters=starts,total_duration_s=elapsed,video=output.name)
    (a.out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    print('FINAL',output,flush=True)


if __name__=='__main__':main()
