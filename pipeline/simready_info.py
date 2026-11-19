"""Download SimReady packages and print what a task file needs: the root USD, its bounding box (m, at scale 1,
in the file's own axes), the authored mass, the number of rigid bodies and joints, and the licence.

    python pipeline/simready_info.py dishware_mug_a01 tape_clear_a01

Run it in Arena's venv (source ~/IsaacLab-Arena/yam.env). Search names in pipeline/simready_catalog.txt.
"""

import json
import sys
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "lib"))
import simready  # noqa: E402  (its import hook is harmless here)
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402

for package in sys.argv[1:]:
    url = f"{simready.HF}/api/datasets/{simready.REPO}/tree/{simready.REVISION}/{package}/simready_usd"
    listing = json.loads(urllib.request.urlopen(urllib.request.Request(url, headers={"User-Agent": "x"}), timeout=60).read())
    roots = [e["path"].rsplit("/", 1)[1] for e in listing if e["type"] == "file" and e["path"].endswith((".usd", ".usda", ".usdc"))]
    if not roots:
        print(f"{package}: no root USD"); continue
    path = simready.fetch(package, roots[0])
    stage = Usd.Stage.Open(str(path))
    up = UsdGeom.GetStageUpAxis(stage)
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedRange()
    lo, hi = [v * mpu for v in box.GetMin()], [v * mpu for v in box.GetMax()]
    prims = list(stage.Traverse())
    bodies = [p for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    joints = [p for p in prims if p.IsA(UsdPhysics.Joint)]
    masses = [UsdPhysics.MassAPI(p).GetMassAttr().Get() for p in bodies if p.HasAPI(UsdPhysics.MassAPI)]
    lic = path.parents[1] / ".metadata" / "LICENSE.txt"
    print(f"{package}: usd={roots[0]} up={up} size_m=({hi[0]-lo[0]:.3f}, {hi[1]-lo[1]:.3f}, {hi[2]-lo[2]:.3f}) "
          f"min=({lo[0]:.3f}, {lo[1]:.3f}, {lo[2]:.3f}) mass={masses} bodies={len(bodies)} joints={len(joints)} "
          f"licence={'yes' if lic.is_file() else 'missing'}")
