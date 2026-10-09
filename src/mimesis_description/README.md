# mimesis_description

Robot description of our myCobot 280 Jetson Nano: URDF (xacro), meshes, and the ros2_control
interface. Everything that needs to know what the arm looks like or how its joints connect
(RViz, robot_state_publisher, ros2_control, MoveIt, Gazebo) reads it from here.

## Run it

```bash
make build
make shell
ros2 launch mimesis_description display.launch.py            # sliders move the model
ros2 launch mimesis_description display.launch.py jsp:=none  # follow the real arm's /joint_states
```

Run the launch from a terminal on the VM's desktop, not over SSH, so RViz can open its window.

## Files

| File | What it is |
| --- | --- |
| `urdf/mimesis.urdf.xacro` | Top level: `world` + arm + ros2_control. Add the gripper and camera here. |
| `urdf/mycobot_280_jn.xacro` | The arm as a macro (`prefix`, `parent`, origin block). |
| `urdf/mimesis.ros2_control.xacro` | Joint interfaces. `use_mock_hardware:=true` (default) uses fake hardware, `false` our future C++ driver plugin. |
| `urdf/mycobot_280_jn_inertials.yaml` | Mass, centre of mass, inertia per link. **Generated**, don't edit. |
| `meshes/mycobot_280_jn/visual/*.stl` | Detailed meshes, metres, in each link's frame. |
| `meshes/mycobot_280_jn/collision/*.stl` | Convex hulls of the visual meshes, for fast collision checks. |
| `tools/convert_meshes.py` | Regenerates the meshes and the inertials from Elephant's originals. |
| `test/check_urdf.sh` | `make test` expands the xacro and runs `check_urdf` on it. |

Frames: `world` → `base_link` (bottom of the Jetson base) → `link1` … `link6` → `flange`.
Joints are `joint1` … `joint6`, numbered like the real arm and like pymycobot.

## Where it comes from, and what was fixed

The joint origins and meshes come from Elephant Robotics' `mycobot_280_jn.urdf`
([mycobot_ros2](https://github.com/elephantrobotics/mycobot_ros2), branch `humble`, BSD-2-Clause,
see `meshes/mycobot_280_jn/LICENSE`). That file was not usable as it was:

- `velocity="0"` on every joint, so MoveIt and the controllers could not move anything;
- no collision or inertial data, so no collision checking and no Gazebo;
- two meshes in millimetres and five in metres, and `joint5.dae` references a material that
  doesn't exist, which makes some loaders drop the whole part;
- no `base_link`, links named like joints, packaged together with 25 other robots.

Checked when this package was made:

- The kinematics are identical to Elephant's file (forward kinematics compared on 200 random
  poses: largest difference 1e-5 m, from using exact pi/2).
- Joint limits match what pymycobot enforces for `MyCobot280`.
- All links connect correctly at zero and at a bent pose.

## Not verified yet (do these on the real arm)

1. **Joint directions.** For each joint: +10° on the arm (pymycobot `send_angle`) and +10° on the
   slider must turn the same way. If one doesn't, the driver flips that joint's sign; don't
   change the URDF.
2. **Forward kinematics against the arm.** At 4-5 poses, compare pymycobot `get_coords()`
   (millimetres) with `ros2 run tf2_ros tf2_echo base_link flange`. The firmware's base frame may
   sit at a different height from our `base_link`; a constant offset in z is expected, anything
   else is not.
3. **Masses.** Estimates (0.6 kg base, 0.6 kg arm). Weigh the arm, edit `BASE_MASS` and
   `ARM_MASS` in `tools/convert_meshes.py`, and re-run it.
4. **Velocity and effort limits** (`max_velocity`, `max_effort` in `mycobot_280_jn.xacro`) are
   placeholders. Measure them before trusting MoveIt's timing.
5. **Flange.** `flange` is at `link6`'s origin. Move it to the real mounting face once measured.

## Regenerating the meshes

Only needed if the source meshes or the masses change. Runs outside the dev container:

```bash
git clone --depth 1 -b humble https://github.com/elephantrobotics/mycobot_ros2.git /tmp/mycobot_ros2
python3 -m venv /tmp/venv && /tmp/venv/bin/pip install trimesh pycollada scipy numpy
/tmp/venv/bin/python src/mimesis_description/tools/convert_meshes.py \
  /tmp/mycobot_ros2/mycobot_description/urdf/mycobot_280_jn
```
