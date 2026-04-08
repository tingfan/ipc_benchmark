# IPC Benchmark: cydr+zenoh vs LCM

Benchmarks comparing serialization and IPC latency for ROS 2 message types using different backends.

## What's tested

**Serialization only** (`bench_serdes.py`):
- `ros2_pyterfaces` cyclone backend (CDR via Cyclone DDS Python)
- `ros2_pyterfaces` cydr backend (CDR via cydr/msgspec)
- Protobuf via [betterproto](https://github.com/danielgtaylor/python-betterproto) (pure Python protobuf)
- Google [protobuf](https://protobuf.dev/) with C/upb backend
- LCM (hand-written struct-based encode/decode)
- Message types: `Image` (1280×960×3), `CompressedImage` (JPEG), `JointState` (16 joints)

**End-to-end IPC** (`bench_ipc.py`):
- cydr serialize → zenoh transport → cydr deserialize
- betterproto serialize → zenoh transport → betterproto deserialize
- Google protobuf(C) serialize → zenoh transport → protobuf(C) deserialize
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

| Message | Cyclone | cydr | betterproto | protobuf(C) | LCM |
|---|---|---|---|---|---|
| Image serialize | 131,000 | 554 | 526 | 513 | 501 |
| Image deserialize | 56,000 | 213 | 439 | 200 | 206 |
| JointState serialize | 42 | 4.3 | 65 | 0.6 | 7.7 |
| JointState deserialize | 39 | 6.3 | 82 | 1.1 | 7.7 |

### Payload sizes (bytes)

| Message | CDR | Protobuf | LCM |
|---|---|---|---|
| raw Image | 3,686,448 | 3,686,420 | 3,686,433 |
| CompressedImage (JPEG) | 1,102,687 | n/a | n/a |
| JointState (16 joints) | 644 | 543 | 594 |

### IPC median latency (µs)

| Message | cydr+zenoh | betterproto+zenoh | protobuf(C)+zenoh | LCM |
|---|---|---|---|---|
| Image (3.7 MB) | 602 | 2,101 | 584 | 2,852 |
| JointState (644 B) | 10.7 | 10.6 | 8.0 | 19.4 |

### IPC latency box plots

![Image IPC latency](bench_ipc_image.png)

![JointState IPC latency](bench_ipc_joint.png)

## Dependencies

Managed by pixi. Key packages:
- [ros2-pyterfaces](https://github.com/2lian/ros2-pyterfaces) — ROS 2 message serialization (cyclone + cydr backends)
- [zenoh](https://zenoh.io/) — zero-overhead pub/sub transport
- [lcm](https://github.com/lcm-proj/lcm) — Lightweight Communications and Marshalling
- [betterproto](https://github.com/danielgtaylor/python-betterproto) — Pure Python Protobuf 3 code generation
- [protobuf](https://protobuf.dev/) — Google Protocol Buffers with C/upb backend
