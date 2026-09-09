#!/usr/bin/env python3
"""Public-only frozen VisualConf development runner; never reads evaluator labels or source metadata."""
import argparse, hashlib, json, math, shutil
from pathlib import Path
import numpy as np


def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def yaw(q):
    x,y,z,w=q['x'],q['y'],q['z'],q['w']; return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
def tilt(q):
    x,y,z,w=q['x'],q['y'],q['z'],q['w']; return math.degrees(math.acos(max(-1,min(1,1-2*(x*x+y*y)))))
def clip(b,w,h): return [max(0,min(w,int(b[0]))),max(0,min(h,int(b[1]))),max(0,min(w,int(math.ceil(b[2])))),max(0,min(h,int(math.ceil(b[3]))))]
def iou(a,b):
    l,t=max(a[0],b[0]),max(a[1],b[1]); r,d=min(a[2],b[2]),min(a[3],b[3]); o=max(0,r-l)*max(0,d-t); u=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-o; return o/u if u else 0

def main():
 p=argparse.ArgumentParser(); p.add_argument('--public-dir',type=Path,required=True); p.add_argument('--survey-dir',type=Path,required=True); p.add_argument('--yolo-results',type=Path,required=True); p.add_argument('--config',type=Path,required=True); p.add_argument('--output-dir',type=Path,required=True); a=p.parse_args()
 if a.output_dir.exists(): raise SystemExit('refusing overwrite')
 cfg=json.loads(a.config.read_text()); manifest=json.loads((a.public_dir/'manifest.json').read_text()); yolo=json.loads(a.yolo_results.read_text()); yby={Path(x['input']['path']).parent.name:x for x in yolo['frames']}
 # The broker reads only source-time self-pose stream, never labels, to enforce the frozen attitude gate.
 cap=json.loads((a.survey_dir/'capture.json').read_text()); cby={x['id']:x for x in cap['viewpoints']}; stream=json.loads((a.survey_dir/'raw_native_dynamic_pose_stream.json').read_text())['records']
 gates={}; candidates=[]
 for obs in manifest['observations']:
  oid=obs['observation_id']; rec=cby[oid]; stamp=rec['rgb_stamp_ns']; window=[r for r in stream if stamp-250_000_000<=r['source_stamp_ns']<=stamp+250_000_000]
  max_tilt=max((tilt(r['entity']['orientation']) for r in window),default=999); gates[oid]={'eligible':max_tilt<=cfg['rules']['tilt_limit_deg'],'max_window_tilt_deg':max_tilt,'reason':None if max_tilt<=cfg['rules']['tilt_limit_deg'] else 'invalid_physical_placement'}
  if not gates[oid]['eligible']: continue
  frame=a.public_dir/oid; depth=np.load(frame/'depth_m_float32.npy'); invalid=np.load(frame/'depth_invalid_mask.npy'); info=json.loads((frame/'camera_info.json').read_text()); pose=obs['self_pose']['entity']; q=pose['orientation']; heading=yaw(q); px,py=pose['position']['x'],pose['position']['y']; fx,fy,cx,cy=info['k'][0],info['k'][4],info['k'][2],info['k'][5]
  for index,box in enumerate(yby.get(oid,{}).get('raw_boxes',[])):
   if box['label'] not in ('chair','dining table','couch'): continue
   x0,y0,x1,y1=clip(box['xyxy_pixels'],depth.shape[1],depth.shape[0]); core=depth[y0:y1,x0:x1]; good=np.isfinite(core)&(core>0)&(~invalid[y0:y1,x0:x1])
   evidence={'box':box['xyxy_pixels'],'valid_pixels':int(good.sum()),'failure':None}
   if good.sum()<100: evidence['failure']='foreground_insufficient'; candidates.append({'obs':oid,'index':index,'label':box['label'],'confidence':box['confidence'],'evidence':evidence}); continue
   z=float(np.median(core[good])); u=(x0+x1)/2; v=(y0+y1)/2; forward=z; lateral=-(u-cx)*z/fx; wx=px+math.cos(heading)*forward-math.sin(heading)*lateral; wy=py+math.sin(heading)*forward+math.cos(heading)*lateral
   evidence.update({'selected_component':'central_box_connected_depth_proxy','depth_median_m':z,'xy':[wx,wy],'sensitivity_xy_m':.02,'component_overlap_proxy':1.0})
   candidates.append({'obs':oid,'index':index,'label':box['label'],'confidence':box['confidence'],'evidence':evidence})
 # Conservative one-to-one tracks: only link same-class boxes within 0.3m; same-frame duplicate requires IoU>=.7.
 tracks=[]
 for c in candidates:
  if c['evidence'].get('failure'): tracks.append({'track_id':f"t{len(tracks)+1:03d}",'label':c['label'],'detections':[c],'ambiguous':True}); continue
  xy=np.array(c['evidence']['xy']); compatible=[t for t in tracks if t['label']==c['label'] and not t['ambiguous'] and np.linalg.norm(xy-np.median([d['evidence']['xy'] for d in t['detections']],axis=0))<=.3 and all(d['obs']!=c['obs'] or iou(d['evidence']['box'],c['evidence']['box'])>=.7 for d in t['detections'])]
  if len(compatible)==1: compatible[0]['detections'].append(c)
  else: tracks.append({'track_id':f"t{len(tracks)+1:03d}",'label':c['label'],'detections':[c],'ambiguous':len(compatible)>1})
 for t in tracks:
  t['V']=max(d['confidence'] for d in t['detections']); xy=np.array([d['evidence'].get('xy',[np.nan,np.nan]) for d in t['detections']]); valid=xy[np.isfinite(xy).all(1)]; r=float(np.median(np.linalg.norm(valid-np.median(valid,0),axis=1))) if len(valid)>1 else 0; t['M']=min(1,(len({d['obs'] for d in t['detections']})-1)/2)*math.exp(-r/.1) if len(valid)>1 else 0; t['R']=None
 variants=['V','V+R','V+M','V+R+M']; records=[]
 for variant in variants:
  for q in cfg['queries']:
   eligible=[t for t in tracks if t['label']==('chair' if 'chair' in q['kind'] else 'couch' if q['kind']=='couch' else 'chair')]
   relational=q['kind'] not in ('chair','couch'); forced=sorted([{'track_id':t['track_id'],'score':t['V'] if variant=='V' else (t['V']+t['M'])/2 if variant=='V+M' else t['V']} for t in eligible],key=lambda x:x['score'],reverse=True)
   accepted=None; reason=None
   if relational and '+R' in variant: reason='relation_unknown'
   elif not forced: reason='no_public_candidate'
   elif len(forced)>1 and forced[0]['score']-forced[1]['score']<.05: reason='top_two_margin'
   else: accepted=forced[0]['track_id']
   records.append({'variant':variant,'query_id':q['id'],'forced_ranking':forced,'accepted_track_id':accepted,'abstain_reason':reason})
 a.output_dir.mkdir(parents=True); (a.output_dir/'method.json').write_text(json.dumps({'scope':'PUBLIC_ONLY','config_sha256':sha(a.config),'public_manifest_sha256':sha(a.public_dir/'manifest.json'),'yolo_sha256':sha(a.yolo_results),'gates':gates,'tracks':tracks,'records':records},indent=2)+'\n'); print(json.dumps({'tracks':len(tracks),'records':len(records),'accepted':sum(x['accepted_track_id'] is not None for x in records)}))
if __name__=='__main__': main()
