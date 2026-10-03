# CHAMP patches for the Go2 simulation

Notes on the local modifications to the CHAMP tree under `src/go2_simulation/champ/`.

CHAMP here is **not** a git submodule — it is tracked directly by the parent `go2_simulation`
repository (149 files; only `champ/include/champ` is a submodule), so these patches travel with
the repo rather than being lost on a re-clone. (The root [`README.md`](../../README.md) still
describes CHAMP as "cloned separately", which no longer matches how this workspace is laid out.)

The Go2 simulation itself lives in the sibling packages `go2_config`, `go2_description` and
`livox_laser_simulation_RO2`. See [`../../README.md`](../../README.md) for setup, and its
"Gazebo performance (real-time factor)" section for the Gazebo tuning.

## 1. `champ_gazebo/launch/gazebo.launch.py` — gate the foot-contact odometry

**Patch:** a new `publish_foot_contacts` launch argument (default `true`, declared at lines
40–44) that gates the `contact_sensor` node via `condition=IfCondition(publish_foot_contacts)`
at line 131.

**Why:** foot-contact odometry is unreliable under Gazebo, and the file's own TODO (lines
123–125) records that running `contact_sensor` cuts the real-time factor roughly in half. In
simulation `go2_config` passes `publish_foot_contacts:=false` and gets odometry from Gazebo
ground truth instead, via its `ground_truth_odom` node.

Note this argument is read by `IfCondition`, which does accept the lowercase `"true"` its default
uses — unlike the `headless` argument in the same file (see §3).

## 2. `champ_description` / `champ_bringup` — wrap the URDF string

**Patch:** wrap the `xacro` `Command(...)` results in `ParameterValue(..., value_type=str)`:

- `champ_description/launch/description.launch.py:30`
- `champ_bringup/launch/bringup.launch.py:169` and `:184`

**Why:** without it, the URDF string is YAML-parsed on its way into the `robot_description` /
`urdf` parameter, so URDF content that happens to look like YAML (or a bare scalar) breaks.
Declaring the type explicitly keeps it a string.

## 3. Gotcha: `headless` must be a Python literal, not `true`/`false`

Not itself a patch, but a constraint on how `champ_gazebo/launch/gazebo.launch.py` may be called,
and it is easy to trip over.

`gzclient` is started under:

```python
condition=IfCondition(PythonExpression([" not ", headless])),   # line 92
```

`PythonExpression` **evaluates the substituted string as Python source**, so `headless` must
substitute to `True` or `False`. The launch-file-style lowercase `true` / `false` yields the
expression ` not false`, which raises `NameError: name 'false' is not defined`. This is exactly
why the file's own default is the capitalised `"False"` (line 37) — passing anything else from a
caller silently breaks it.

The failure mode is nastier than the traceback suggests: launch aborts *after* `gzserver` has
already been started, so you are left with an **orphaned `gzserver`** (reparented to
`systemd --user`) that keeps running with no robot spawned in it, while `/clock` and the other
gzserver-published topics still look healthy. If a launch dies with a traceback but the sim
"seems to be up", check for orphans before re-launching.

`go2_config` works around this by emitting the literal `True`/`False` while accepting any casing
on the command line.

## 4. Gotcha: a Gazebo sensor with `<visualize>false</visualize>` and no `<always_on>` never updates

Same class of trap as §3, in the opposite direction, and it is easy to introduce while tuning
performance.

Gazebo Classic's `RaySensor` only updates if `<always_on>` is `true` **or** something is
subscribed to the sensor's internal scan topic — and that topic exists **only when
`<visualize>` is `true`**, because `RaySensor` advertises it as part of enabling visualization.
So a sensor with `<visualize>false</visualize>` and no `<always_on>` has nothing keeping it
awake: it never pumps its update event, sensor plugins never get their `OnNewLaserScans()`
callback, and their publishers exist with `Publisher count: 1` while publishing zero messages.

The diagnostic tell is that the fix is invisible from the ROS side: the topics are listed, the
plugins are loaded, and nothing errors — but `ros2 topic hz` reports no messages. It also makes
the simulation look *much* faster, because the most expensive sensor is not actually running,
which is how it can survive a performance-tuning session unnoticed.

This bit the Go2's Livox Mid-360 in `livox_laser_simulation_RO2/urdf/mid360.xacro`, which now
sets `<always_on>true</always_on>` alongside `<visualize>false</visualize>` — the sensor keeps
updating while `gzclient` still does not draw its 36 000 rays. The Go2's IMU was never affected
because `go2_description/xacro/gazebo.xacro` already sets `<always_on>true</always_on>` on it.
See the root README's performance section.

## 5. Note: `champ_gazebo/worlds/*.world` are still untuned

CHAMP's bundled worlds still use `max_step_size 0.001` / `real_time_update_rate 1000`
(1000 Hz physics), which costs a large share of the real-time factor. They are left as upstream
on purpose: `go2_config` overrides the world through its `world:=` argument and never loads them.
If you launch `champ_gazebo` directly, that override does not apply and you get the slow settings.

The tuning actually used by the Go2 sim lives in `go2_config`: `worlds/*.world` (2 ms / 500 Hz,
shadows off) and a default world of `worlds/playground.world` — `outdoor.world`'s mesh collision
geometry is much more expensive for Livox ray casting. See the root README.
