# IPC Benchmark: CDR vs Protobuf vs LCM

Benchmarks comparing serialization performance and IPC latency for robotics message types across five serialization backends and two transport layers.

## What's tested

**Serialization only** (`bench_serdes.py`):
- `ros2_pyterfaces` cyclone backend (CDR via Cyclone DDS Python)
- `ros2_pyterfaces` cydr backend (CDR via cydr/msgspec)
- [betterproto](https://github.com/danielgtaylor/python-betterproto) (pure Python protobuf)
- Google [protobuf](https://protobuf.dev/) with C/upb backend
- [LCM](https://github.com/lcm-proj/lcm) (struct-based encode/decode)
- Message types: `Image` (1280×960×3 raw), `CompressedImage` (JPEG), `JointState` (16 joints)
- Roundtrip correctness verified for all backends

**End-to-end IPC** (`bench_ipc.py`):
- cydr + zenoh
- betterproto + zenoh
- Google protobuf(C) + zenoh
- LCM (UDP multicast)
- Measures single-process pub/sub latency with embedded timestamps
- Generates box plots comparing all four stacks

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

**Raw Image (1280×960×3, 3.7 MB):**

| | Cyclone | cydr | betterproto | protobuf(C) | LCM |
|---|---|---|---|---|---|
| serialize | 131,000 | 554 | 526 | 513 | 501 |
| deserialize | 56,000 | 213 | 439 | 200 | 206 |

**CompressedImage (JPEG, ~1.1 MB):**

| | Cyclone | cydr | betterproto | protobuf(C) | LCM |
|---|---|---|---|---|---|
| serialize | 20,780 | 69 | 126 | 115 | ~500 |
| deserialize | 8,540 | 53 | 137 | 48 | ~200 |

**JointState (16 joints):**

| | Cyclone | cydr | betterproto | protobuf(C) | LCM |
|---|---|---|---|---|---|
| serialize | 42 | 4.3 | 65 | 0.6 | 7.7 |
| deserialize | 39 | 6.3 | 82 | 1.1 | 7.7 |

### Payload sizes (bytes)

| Message | CDR | Protobuf | LCM |
|---|---|---|---|
| raw Image | 3,686,448 | 3,686,420 | 3,686,433 |
| CompressedImage (JPEG) | 1,101,772 | 1,101,746 | 1,101,757 |
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
- [betterproto](https://github.com/danielgtaylor/python-betterproto) — pure Python Protobuf 3
- [protobuf](https://protobuf.dev/) — Google Protocol Buffers with C/upb backend
