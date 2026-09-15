"""Conservative continuity gate for these offline experiments, not a navigation certification."""
import json,sys
from pathlib import Path
results=json.loads(Path(sys.argv[1]).read_text()); failed=False
if not results: raise SystemExit('No completed runs')
for name,s in results.items():
    reasons=[]
    if not s['completed']: reasons.append('incomplete frame processing')
    if s['ok_percent']<95: reasons.append('OK coverage <95%')
    if s['new_map_events']>1: reasons.append('map restarted')
    if s['max_adjacent_ok_translation_m']>0.5: reasons.append('raw pose discontinuity >0.5 m/frame (may include map corrections)')
    failed|=bool(reasons)
    print(name+': '+('FAIL: '+'; '.join(reasons) if reasons else 'PASS continuity screen only; calibration/accuracy unverified'))
sys.exit(1 if failed else 0)
