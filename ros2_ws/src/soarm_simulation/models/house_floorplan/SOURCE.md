# Source model

The generated Gazebo structure is based on the `Ground floor` level in:

- Repository: <https://github.com/xtrinch/sweethome3d-house-plans.git>
- Commit: `f0af4c6258ab5fcdabac1654e0796f9b18b44e94`
- Input: `house.sh3d` (`Home.xml` inside the archive)
- Input Git blob: `c7b3f14e910f60fcfe2ea539d223504f900b7834`
- Input SHA-256: `1cc879ec4dbe435264a30973ec85ee5a0a221b314ebdf3c98680bf20cb6685e2`

The upstream repository does not declare a license, so its 34 MB archive,
images, furniture meshes, and textures are not redistributed here. The checked-in
SDF and occupancy map are generated metric structure data. The generator first
verifies the exact upstream archive SHA-256, then reads the model directly and
verifies the expected 21 walls, 10 door-like openings, and 2.4384 m wall height.

All source objects whose names contain `door` are converted to open door frames.
Door leaves, swing arcs, roof, and ceiling are omitted. Three selected table
items are generated separately as `house_furniture`; other loose furniture is
omitted for a deterministic localization/navigation environment. Windows remain
collision-solid because a ground robot cannot traverse them and lidar commonly
observes their wall plane.

Regenerate from a pinned checkout:

```bash
git clone https://github.com/xtrinch/sweethome3d-house-plans.git /tmp/sweethome3d-house-plans
git -C /tmp/sweethome3d-house-plans checkout f0af4c6258ab5fcdabac1654e0796f9b18b44e94
python3 tools/generate_house_floorplan.py /tmp/sweethome3d-house-plans/house.sh3d
```
