import sys
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import inspect
from isaacsim.robot.policy.examples.controllers import PolicyController
sig = str(inspect.signature(PolicyController.__init__))
with open('sig.txt', 'w') as f:
    f.write(sig)
app.close()
sys.exit(0)
