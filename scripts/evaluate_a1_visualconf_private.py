#!/usr/bin/env python3
"""Private A1 development evaluator; consumes GT only after public method outputs are frozen."""
import argparse,json
from pathlib import Path

def cls(name):
 if name.startswith(('ChairA_01','ChairD_01')): return 'chair'
 if name.startswith('KitchenTable_01'): return 'dining table'
 if name.startswith('SofaC_01'): return 'couch'
 if name.startswith('CoffeeTable_01'): return 'coffee table'
 if name.startswith('Tablet_01'): return 'tablet'
 return None
def main():
 p=argparse.ArgumentParser();p.add_argument('--private-dir',type=Path,required=True);p.add_argument('--method',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True);a=p.parse_args()
 if a.output_dir.exists():raise SystemExit('refusing overwrite')
 gt=json.loads((a.private_dir/'instances.json').read_text()); m=json.loads(a.method.read_text()); byobs={x['observation_id']:x for x in gt['observations']}
 # Definite observed GT sets, with taxonomy correction preserved explicitly.
 visible={k:[] for k in ('chair','dining table','couch')}
 for obs in gt['observations']:
  for x in obs['visible_instances']:
   c=cls(x['source_name'])
   if c in visible: visible[c].append((obs['observation_id'],x['label_id']))
 gt_def={'Q1':bool(visible['chair']),'Q2':bool(visible['couch']),'Q3':False,'Q4':False,'Q5':False,'Q6':False}
 rows=[]
 for r in m['records']:
  q=r['query_id']; accepted=r['accepted_track_id']; correct=None if not gt_def[q] else False
  # No identity recovery may use majority vote: tracks remain unknown absent private one-to-one matching.
  if accepted and q in ('Q1','Q2'): correct=False
  rows.append({'variant':r['variant'],'query_id':q,'gt_status':'definite' if gt_def[q] else 'ambiguous_or_insufficient_geometry','accepted_track_id':accepted,'correct':correct,'abstain_reason':r['abstain_reason']})
 definite=[x for x in rows if x['gt_status']=='definite']; accepted=[x for x in definite if x['accepted_track_id']]
 report={'scope':'PRIVATE_EVALUATOR_ONLY','taxonomy_version':'a1-taxonomy-v1-explicit-tablet-coffee-correction','taxonomy':{'KitchenTable_01':'dining table','CoffeeTable_01':'coffee table','Tablet_01':'tablet'},'rows':rows,'metrics':{'records':len(rows),'definite_gt_denominator':len(definite),'coverage':len(accepted)/len(definite) if definite else 0,'wrong_accepted':sum(x['correct'] is False for x in accepted),'candidate_recall':'NOT_COMPUTED: no GT identity is supplied to public tracks','top1':'NOT_COMPUTED: private one-to-one constituent box assignment required for identity credit','mrr':'NOT_COMPUTED: private one-to-one constituent box assignment required for identity credit'},'warning':'No majority vote or GT repair is applied to public tracks. Relation GT is ambiguous/insufficient under observed-surface evidence and preserved as such.'}
 a.output_dir.mkdir(parents=True);(a.output_dir/'evaluation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report['metrics']))
if __name__=='__main__':main()
