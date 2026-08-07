import os
import omni.usd

usd_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pupper_v3", "pupper_v3.usd")
omni.usd.get_context().open_stage(usd_path)
print(f"[pupper] opened stage: {usd_path}")
