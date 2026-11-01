"""One material-control process per container, released automatically on exit."""
import fcntl
import os

def acquire(path='/tmp/pai_material_controller.lock'):
 lease=open(path,'a+')
 try:fcntl.flock(lease,fcntl.LOCK_EX | fcntl.LOCK_NB)
 except BlockingIOError:
  lease.close()
  raise SystemExit('Another material controller is running; stop it before starting this one.')
 lease.seek(0);lease.truncate();lease.write(str(os.getpid()));lease.flush()
 return lease
