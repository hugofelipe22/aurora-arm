import subprocess, sys, os
r = os.path.dirname(os.path.abspath(__file__))
for cmd in (['src/codegen.py'], ['tests/test_aurora.py'], ['src/memorial.py'], ['src/traceability.py']):
    subprocess.run([sys.executable] + cmd, cwd=r, check=True)
