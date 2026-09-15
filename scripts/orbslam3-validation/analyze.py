"""Summarize all states, never infer success solely from exported trajectories."""
import json, sys, re
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(sys.argv[1]); results={}
for run in sorted(root.glob('office-*/*')):
    if not (run/'CameraTrajectory.txt').exists(): continue
    data=np.genfromtxt(run/'tracking.csv',delimiter=',',names=True)
    ts=data['timestamp']; t=ts-ts[0]; ok=data['state']==2
    segments=[]
    for i in range(len(t)):
        if not ok[i] and (i==0 or ok[i-1]): start=i
        if not ok[i] and (i==len(t)-1 or ok[i+1]):
            segments.append({'start_s':float(t[start]),'end_s':float(t[i]),'span_s':float(t[i]-t[start]),'frames':i-start+1,'states':list(map(int,np.unique(data['state'][start:i+1])))})
    log=(run/'run.log').read_text(); inp=json.loads((run.parent/'input.json').read_text())
    xyz=np.column_stack([data[x] for x in ('x','y','z')]); dt=np.diff(ts); step=np.linalg.norm(np.diff(xyz,axis=0),axis=1); valid=ok[1:]&ok[:-1]
    s={'processed':len(t),'input_rgb':inp['counts']['rgb'],'ok_frames':int(ok.sum()),'ok_percent':float(ok.mean()*100),'ok_over_all_rgb_percent':float(ok.sum()/inp['counts']['rgb']*100),'state_counts':{str(int(k)):int((data['state']==k).sum()) for k in np.unique(data['state'])},'duration_s':float(t[-1]),'non_ok_segments':segments,'track_ms_p50_p95_p99_max':np.percentile(data['track_ms'],[50,95,99,100]).tolist(),'track_ms_mean':float(data['track_ms'].mean()),'track_exceeds_next_interval_percent':float((data['track_ms'][:-1]>dt*1000).mean()*100),'new_map_events':log.count('Creation of new map with id:'),'active_map_reset_requests':log.count('Active map reset received'),'relocalization_successes':log.count('Relocalized!!'),'loop_events':log.count('*Loop detected'),'merge_events':log.count('*Merge detected'),'completed':len(t)==inp['pairs'],'max_adjacent_ok_translation_m':float(step[valid].max()) if valid.any() else None,'input_gap_ms_p95_max':np.percentile(dt*1000,[95,100]).tolist()}
    quality=inp['quality']; qt=np.array([q['t'] for q in quality]); blur=np.array([q['laplacian_var'] for q in quality]); qidx=np.array([np.argmin(abs(qt-x)) for x in ts])
    s['laplacian_median_ok_nonok']=[float(np.median(blur[qidx][mask])) if mask.any() else None for mask in (ok,~ok)]
    (run/'summary.json').write_text(json.dumps(s,indent=2)); results[run.parent.name+"/"+run.name]=s
    fig,axs=plt.subplots(4,1,figsize=(12,10),sharex=True)
    axs[0].plot(t,data['state'],lw=.8); axs[0].set_yticks([0,1,2,3,4],['NO_IMG','INIT','OK','RECENT_LOST','LOST']); axs[0].set_title(run.parent.name+' | ORB-SLAM3 RGB-D (experimental calibration)')
    axs[1].plot(t,data['map_points'],lw=.7); axs[1].set_ylabel('Tracked map points')
    axs[2].plot(t,data['track_ms'],lw=.7); axs[2].axhline(33.33,color='r',ls='--',label='30 Hz budget'); axs[2].set_ylabel('TrackRGBD ms'); axs[2].legend()
    axs[3].plot(qt-ts[0],blur,lw=.7); axs[3].set_ylabel('Laplacian variance'); axs[3].set_xlabel('Seconds from first paired frame')
    for ax in axs:
        for seg in segments: ax.axvspan(seg['start_s'],seg['end_s'],color='red',alpha=.15)
        ax.grid(alpha=.2)
    fig.tight_layout(); fig.savefig(run/'diagnostics.png',dpi=130); plt.close(fig)
(root/'summary.json').write_text(json.dumps(results,indent=2)); print(json.dumps(results,indent=2))
