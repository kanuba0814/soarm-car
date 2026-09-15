"""Extract lossless images and unique nearest timestamp pairs; no depth stretching."""
import argparse, json
from pathlib import Path
import cv2, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, CameraInfo
p=argparse.ArgumentParser(); p.add_argument('bag'); p.add_argument('out'); a=p.parse_args()
out=Path(a.out); out.mkdir(parents=True,exist_ok=True)
for kind in ('rgb','depth'): (out/kind).mkdir(exist_ok=True)
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=a.bag,storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
frames={'rgb':[],'depth':[]}; infos={}; quality=[]
while r.has_next():
    topic,data,recv=r.read_next()
    if topic.endswith('camera_info'):
        m=deserialize_message(data,CameraInfo); infos[topic]={'k':list(m.k),'d':list(m.d),'width':m.width,'height':m.height}; continue
    if topic not in ('/camera/color/image_raw','/camera/depth/image_raw'): continue
    m=deserialize_message(data,Image); kind='rgb' if '/color/' in topic else 'depth'
    dtype=np.uint8 if kind=='rgb' else np.dtype('>u2' if m.is_bigendian else '<u2')
    im=np.frombuffer(m.data,dtype=dtype).reshape(m.height,m.step//np.dtype(dtype).itemsize)
    im=im[:,:m.width*(3 if kind=='rgb' else 1)]
    if kind=='rgb': im=cv2.cvtColor(im.reshape(m.height,m.width,3),cv2.COLOR_RGB2BGR)
    else: im=im.astype(np.uint16)
    ts=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    name=f'{kind}/{len(frames[kind]):06d}.png'; cv2.imwrite(str(out/name),im,[cv2.IMWRITE_PNG_COMPRESSION,1]); frames[kind].append((ts,name))
    if kind=='rgb': quality.append({'t':ts,'laplacian_var':float(cv2.Laplacian(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),cv2.CV_64F).var()),'brightness':float(im.mean())})
depth=np.array([f[0] for f in frames['depth']]); used=set(); pairs=[]; rejected=[]
for t,name in frames['rgb']:
    j=int(np.argmin(abs(depth-t)))
    if abs(depth[j]-t)>0.03 or j in used: rejected.append(t); continue
    used.add(j); pairs.append((t,name,depth[j],frames['depth'][j][1]))
with (out/'associations.txt').open('w') as f:
    for t,n,d,dn in pairs: f.write(f'{t:.9f} {n} {d:.9f} {dn}\n')
report={'counts':{k:len(v) for k,v in frames.items()},'pairs':len(pairs),'rejected_rgb':len(rejected),'pair_dt_ms_p50_p95_max':np.percentile([abs(t-d)*1000 for t,_,d,_ in pairs],[50,95,100]).tolist(),'camera_info':infos,'quality':quality,'rejected_timestamps':rejected}
(out/'input.json').write_text(json.dumps(report,indent=2))
# Representative images with depth discontinuities overlaid on the assumed common top 400 rows.
tiles=[]
for i in np.linspace(0,len(pairs)-1,8,dtype=int):
    t,n,_,dn=pairs[i]; rgb=cv2.imread(str(out/n)); d=cv2.imread(str(out/dn),-1)
    vis=rgb[:400].copy(); edge=cv2.Canny(np.uint8(np.clip(d/20,0,255)),10,25); vis[edge>0]=(0,0,255)
    cv2.putText(vis,f'{i}: {t-pairs[0][0]:.1f}s',(10,25),0,.7,(0,255,0),2)
    tiles.append(cv2.resize(vis,(480,300)))
cv2.imwrite(str(out/'samples.jpg'),np.vstack([np.hstack(tiles[:4]),np.hstack(tiles[4:])]))
print(json.dumps({k:v for k,v in report.items() if k not in ('quality','rejected_timestamps','camera_info')},indent=2))
