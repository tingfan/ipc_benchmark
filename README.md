# IPC Benchmark: cydr+zenoh vs LCM

Benchmarks comparing serialization and IPC latency for ROS 2 message types using different backends.

## What's tested

**Serialization only** (`bench_serdes.py`):
- `ros2_pyterfaces` cyclone backend (CDR via Cyclone DDS Python)
- `ros2_pyterfaces` cydr backend (CDR via cydr/msgspec)
- LCM (hand-written struct-based encode/decode)
- Message types: `Image` (1280×960×3), `CompressedImage` (JPEG), `JointState` (16 joints)

**End-to-end IPC** (`bench_ipc.py`):
- cydr serialize → zenoh transport → cydr deserialize
- LCM serialize → LCM UDP multicast → LCM deserialize
- Measures single-process pub/sub latency with embedded timestamps
- Generates box plots comparing the two stacks

## Setup

Requires [pixi](https://pixi.sh) (manages Python 3.12 + all dependencies):

```bash
pixi install
```

For LCM large-message benchmarks, increase the kernel UDP buffer:

```bash
sudo sysctl -w net.core.rmem_max=20971520 net.core.rmem_default=20971520
```

## Run

```bash
# Serialization-only benchmark
pixi run python bench_serdes.py

# IPC (serialize + transport + deserialize) benchmark with box plots
pixi run python bench_ipc.py
```

## Sample results

### Serialization (µs per call)

| Message | Cyclone | cydr | LCM |
|---|---|---|---|
| Image serialize | 126,000 | 560 | 505 |
| Image deserialize | 55,000 | 220 | 204 |
| JointState serialize | 40 | 4.3 | 7.8 |
| JointState deserialize | 39 | 6.4 | 7.9 |

### IPC median latency (µs)

| Message | cydr+zenoh | LCM |
|---|---|---|
| Image (3.7 MB) | 578 | 2,781 |
| JointState (644 B) | 10.7 | 21.3 |

## Dependencies

Managed by pixi. Key packages:
- [ros2-pyterfaces](https://github.com/2lian/ros2-pyterfaces) — ROS 2 message serialization (cyclone + cydr backends)
- [zenoh](https://zenoh.io/) — zero-overhead pub/sub transport
- [lcm](https://github.com/lcm-proj/lcm) — Lightweight Communications and Marshalling
