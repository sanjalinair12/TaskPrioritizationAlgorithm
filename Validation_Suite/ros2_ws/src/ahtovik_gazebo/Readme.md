
# AhToVik vs CBBA Gazebo Validation Setup

This validation setup compares the proposed AhToVik allocator against a simplified CBBA-style consensus baseline under the same ROS 2/Gazebo telemetry stream, perturbation noise, and injected robot failure.

## What the setup measures

- Stability rate (%)
- Churn rate (%)
- Total reallocations
- Completed task count
- Allocation runtime (ms)
- Failure recovery time (s)
- CBBA consensus rounds
- CBBA estimated message count

## Experimental design

Both algorithms receive identical robot telemetry from `/ahtovik/robot_telemetry` and identical failure events from `/ahtovik/failure_events`.

AhToVik ranks robots using the L2 ideal-reference compromise index. CBBA builds robot bundles over task slots and resolves conflicts through repeated consensus-style winner updates. The CBBA implementation is intentionally a validation baseline: it models bundle construction, conflict resolution, consensus rounds, and message cost, but it does not implement a full network communication stack.

## Build

```bash
cd ~/ros2_ws
cp -r /path/to/ahtovik_gazebo src/
colcon build --packages-select ahtovik_gazebo
source install/setup.bash
```

## Quick comparison

```bash
python3 src/ahtovik_gazebo/scripts/run_ahtovik_cbba_gazebo_validation.py --quick --clean
python3 src/ahtovik_gazebo/scripts/analyze_validation_results.py --input-dir ~/ahtovik_cbba_validation_results
```

## Full comparison

```bash
python3 src/ahtovik_gazebo/scripts/run_ahtovik_cbba_gazebo_validation.py --runs 10 --duration 40 --clean
python3 src/ahtovik_gazebo/scripts/analyze_validation_results.py --input-dir ~/ahtovik_cbba_validation_results
```

## Run one algorithm manually

AhToVik:

```bash
ros2 launch ahtovik_gazebo validation.launch.py algorithm:=ahtovik noise_level:=0.05 failure_enabled:=true
```

CBBA:

```bash
ros2 launch ahtovik_gazebo validation.launch.py algorithm:=cbba noise_level:=0.05 failure_enabled:=true cbba_max_rounds:=20 cbba_bundle_size:=3
```

## Output

CSV files are written to:

```bash
~/ahtovik_cbba_validation_results
```

Analysis outputs are written to:

```bash
~/ahtovik_cbba_validation_results/analysis
```



