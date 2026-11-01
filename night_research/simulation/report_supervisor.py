"""Background publication checkpoints while the experiment process does real work."""
from pathlib import Path
from datetime import datetime,timezone
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]
deadline=datetime(2026,10,31,22,tzinfo=timezone.utc).timestamp()
next_update=time.time()+15*60
while time.time()<deadline:
    if (ROOT/'STOP').exists():break
    if time.time()>=next_update:
        result=subprocess.run(['python',str(ROOT/'simulation'/'publish_progress.py'),'--export'],cwd=ROOT,capture_output=True,text=True,timeout=150)
        print(datetime.now(timezone.utc).isoformat(),'progress export',result.returncode,result.stdout,result.stderr[-1000:],flush=True)
        next_update=time.time()+15*60
    time.sleep(min(30.,max(deadline-time.time(),.1)))
if time.time()>=deadline:
    # The experiment checks the deadline every 100 integration steps. Only committed
    # case JSON files are included, so an unclosed final trace cannot become a result.
    result=subprocess.run(['python',str(ROOT/'simulation'/'publish_progress.py'),'--final','--export'],cwd=ROOT,capture_output=True,text=True,timeout=180)
    print(datetime.now(timezone.utc).isoformat(),'final export',result.returncode,result.stdout,result.stderr,flush=True)
