import json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(sys.argv[1]);inputs=Path(sys.argv[2]);summary={};sensitivity={}
def longest(mask,dt):
 best=cur=0.
 for yes,t in zip(mask,dt):
  cur=cur+t if yes else 0.;best=max(best,cur)
 return float(best)
def mats(d):
 ts=np.tile(np.eye(4),(len(d),1,1))
 for r in range(3):
  for c in range(4):ts[:,r,c]=d[f't{r}{c}']
 return ts
def disagreement(a,b):
 diff=a@np.linalg.inv(b);dist=np.linalg.norm(diff[:,:3,3],axis=1);ang=np.degrees(np.arccos(np.clip((np.trace(diff[:,:3,:3],axis1=1,axis2=2)-1)/2,-1,1)))
 return dist,ang
for bag in sorted(root.glob('office-*')):
 runs={};dense={};rtabs={};orb=np.genfromtxt(inputs/bag.name/'baseline/tracking.csv',delimiter=',',names=True);t0=orb['timestamp'][0]
 for run in sorted(bag.iterdir()):
  if not (run/'complete.json').exists():continue
  d=np.genfromtxt(run/'frames.csv',delimiter=',',names=True)
  if run.name.startswith('rtabmap'):
   ok=d['ok']>0;dt=np.r_[0,np.diff(d['timestamp'])];rtabs[run.name]=d
   epoch=d['map_epoch'] if 'map_epoch' in d.dtype.names else np.zeros(len(d));step=np.linalg.norm(np.diff(np.column_stack([d[k] for k in ['x','y','z']]),axis=0),axis=1);same=ok[1:]&ok[:-1]&(epoch[1:]==epoch[:-1]);validstep=step[same]
   runs[run.name]={'frames':len(d),'reported_ok_pct':float(ok.mean()*100),'longest_ok_s':longest(ok,dt),'odom_ms_p50_p95':np.percentile(d['odom_ms'],[50,95]).tolist(),'backend_ms_p50_p95':np.percentile(d['backend_ms'][d['backend_processed']>0],[50,95]).tolist(),'backend_nodes':int(d['backend_processed'].sum()),'loop_events':int((d['loop_id']>0).sum()),'proximity_events':int((d['proximity_id']>0).sum()),'explicit_resets':int(epoch.max()),'max_adjacent_ok_step_m':float(validstep.max()) if len(validstep) else None}
  else:
   ok=d['accepted']>0;idx=d['i'].astype(int);stride=int(json.loads((run/'config.json').read_text())['stride']);orb_bad=(orb['state'][idx]!=2)|(orb['state'][idx-stride]!=2);dense[run.name]=d
   runs[run.name]={'edges':len(d),'returned_pct':float(d['success'].mean()*100),'screened_pct':float(ok.mean()*100),'longest_screened_s':longest(ok,d['dt']),'forward_ms_p50_p95':np.percentile(d['forward_ms'],[50,95]).tolist(),'forward_reverse_ms_p95':float(np.percentile(d['forward_ms']+d['reverse_ms'],95)),'orb_non_ok_edges':int(orb_bad.sum()),'screen_pass_given_orb_non_ok_pct':float(ok[orb_bad].mean()*100) if orb_bad.any() else None,'fb_median_m':float(np.median(d['fb_m'][ok])) if ok.any() else None,'depth_agreement_median':float(np.median(d['depth_agreement'][ok])) if ok.any() else None}
 if all(k in dense for k in ['rgbd','icp','hybrid']):
  r,g,h=[dense[k] for k in ['rgbd','icp','hybrid']];assert np.array_equal(r['i'],g['i']) and np.array_equal(r['i'],h['i'])
  dist,ang=disagreement(mats(r),mats(g));consensus=(r['accepted']>0)&(g['accepted']>0)&(dist<.03)&(ang<2);fallback=(h['accepted']==0)&consensus;selected=(h['accepted']>0)|fallback;union=(r['accepted']>0)|(g['accepted']>0)|(h['accepted']>0)
  summary_fusion={'policy':'hybrid first; on rejection require RGBD and ICP both screened and mutual difference <3cm/<2deg, choose RGBD, never average','selected_pct':float(selected.mean()*100),'hybrid_pct':float(h['accepted'].mean()*100),'extra_edges':int(fallback.sum()),'longest_selected_s':longest(selected,h['dt']),'candidate_union_pct_not_validated':float(union.mean()*100),'both_rgbd_icp_screened':int(((r['accepted']>0)&(g['accepted']>0)).sum()),'rgbd_icp_consensus_edges':int(consensus.sum()),'three_methods_forward_reverse_p95_ms':float(np.percentile(sum(dense[k]['forward_ms']+dense[k]['reverse_ms'] for k in ['rgbd','icp','hybrid']),95))}
  out=np.column_stack([h['i'],h['timestamp'],selected,np.where(h['accepted']>0,1,np.where(fallback,2,0)),dist,ang]);np.savetxt(bag/'selector.csv',out,delimiter=',',header='i,timestamp,accepted,source_0_none_1_hybrid_2_rgbd_consensus,rgbd_icp_diff_m,rgbd_icp_diff_deg',comments='')
  runs['selector']=summary_fusion
 if 'hybrid-stride2' in dense and 'hybrid' in dense:
  short=dense['hybrid'];long=dense['hybrid-stride2'];idx=long['i'].astype(int);sT=mats(short);composed=sT[idx-1]@sT[idx-2];dist,ang=disagreement(mats(long),composed)
  shortok=(short['accepted'][idx-1]>0)&(short['accepted'][idx-2]>0);longok=long['accepted']>0;both=shortok&longok;agree=(dist<.03)&(ang<2)
  runs['multi_rate']={'intervals':len(long),'both_short_edges_screened_pct':float(shortok.mean()*100),'long_edge_screened_pct':float(longok.mean()*100),'long_only_candidates_not_validated':int((longok&~shortok).sum()),'short_only_intervals':int((shortok&~longok).sum()),'both_pass_count':int(both.sum()),'agreement_given_both_pass_pct':float(agree[both].mean()*100) if both.any() else None,'union_candidate_pct_not_validated':float((shortok|longok).mean()*100)}
 if 'hybrid' in dense:
  d=dense['hybrid'];strict=(d['accepted']>0)&(d['fb_m']<.01)&(d['fb_deg']<1)&(d['depth_agreement']>.7)
  sensitivity[bag.name]={'strict_gate_pct':float(strict.mean()*100),'strict_gate':'base gate plus FB <1cm/<1deg and depth agreement >70%'}
 summary[bag.name]=runs
 fig,axs=plt.subplots(2,1,figsize=(12,7));axs[0].set_title(bag.name+' | dense frontends: provisional screen (not accuracy)')
 names=list(dense)
 for j,name in enumerate(names):
  d=dense[name];good=d['accepted']>0;axs[0].scatter(d['timestamp']-t0,np.full(len(d),j),c=np.where(good,'#2a9d8f','#e76f51'),s=5,marker='|')
 axs[0].set_yticks(range(len(names)),names);axs[0].set_xlabel('Seconds');axs[0].grid(alpha=.2)
 for j,(name,d) in enumerate(rtabs.items()):axs[1].scatter(d['timestamp']-t0,np.full(len(d),j),c=np.where(d['ok']>0,'#2a9d8f','#e76f51'),s=5,marker='|')
 axs[1].set_yticks(range(len(rtabs)),list(rtabs));axs[1].set_title('RTAB-Map: reported tracking state (different metric)');axs[1].set_xlabel('Seconds');axs[1].grid(alpha=.2);fig.tight_layout();fig.savefig(bag/'comparison.png',dpi=130);plt.close(fig)
(root/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))

(root/'threshold-sensitivity.json').write_text(json.dumps(sensitivity,indent=2))
