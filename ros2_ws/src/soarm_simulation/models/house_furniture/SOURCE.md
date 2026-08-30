# Source furniture

This model contains three lightweight table approximations selected from the
pinned `house.sh3d` documented in `../house_floorplan/SOURCE.md`:

| Gazebo obstacle | Sweet Home 3D item | Size (m) | Purpose |
| --- | --- | --- | --- |
| `coffee_table` | Coffee table | 1.17 x 0.49 x 0.44 | Low lidar/collision obstacle in the living room |
| `small_table` | Marble table | 1.20 x 1.20 x 0.70 | Central route-planning obstacle |
| `desk` | Desk | 1.60 x 0.80 x 0.72 | Obstacle in the right-side room |

Positions, yaw, and dimensions come directly from the source model. Geometry is
reduced to one tabletop and four legs per item, all attached to one canonical
static link for reliable DART collision handling; upstream furniture meshes and
textures are not redistributed because the source repository declares no
license. The occupancy map conservatively marks each full tabletop footprint as
occupied, matching how a mobile planner should treat overhanging furniture.
