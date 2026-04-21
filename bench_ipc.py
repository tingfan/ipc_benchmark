#!/usr/bin/env python3
"""Benchmark IPC latency: cydr+zenoh vs LCM for Image and JointState.

Runs publisher and subscriber in separate threads (same process) to measure
end-to-end serialize → transport → deserialize latency on localhost.

Usage:
    pixi run --manifest-path calib3d/pixi.toml python calib3d/bench_ipc.py
"""

import struct
import sys
import threading
import time

import numpy as np

# ── config ───────────────────────────────────────────────────────────────────

H, W, C = 960, 1280, 3
N_JOINTS = 16
N_WARMUP = 20
N_MSGS = 200


# ── helpers ──────────────────────────────────────────────────────────────────

def make_image_data():
    return np.random.randint(0, 256, (H * W * C,), dtype=np.uint8)


def make_joint_data():
    names = [f"joint_{i}" for i in range(N_JOINTS)]
    pos = np.random.randn(N_JOINTS).tolist()
    vel = np.random.randn(N_JOINTS).tolist()
    eff = np.random.randn(N_JOINTS).tolist()
    return names, pos, vel, eff


def stamp_bytes():
    """8-byte big-endian timestamp in nanoseconds."""
    return struct.pack(">Q", time.monotonic_ns())


def unstamp_bytes(data: bytes) -> int:
    return struct.unpack(">Q", data[:8])[0]


def report(label, latencies_ns):
    arr = np.array(latencies_ns) / 1e3  # → µs
    print(f"  {label}")
    print(f"    mean   = {arr.mean():>10.1f} µs")
    print(f"    median = {np.median(arr):>10.1f} µs")
    print(f"    p95    = {np.percentile(arr, 95):>10.1f} µs")
    print(f"    p99    = {np.percentile(arr, 99):>10.1f} µs")
    print(f"    min    = {arr.min():>10.1f} µs")
    print(f"    max    = {arr.max():>10.1f} µs")


# ── cydr + zenoh ─────────────────────────────────────────────────────────────

def bench_zenoh_cydr_image(img_flat):
    import zenoh
    from ros2_pyterfaces.cydr.sensor_msgs.msg import Image

    print("\n=== cydr + zenoh: raw Image ===")

    msg = Image(
        height=np.uint32(H), width=np.uint32(W), encoding=b"bgr8",
        is_bigendian=np.uint8(0), step=np.uint32(W * C),
        data=img_flat,
    )

    latencies = []
    received = threading.Event()
    count = [0]

    def on_sample(sample):
        t_recv = time.monotonic_ns()
        payload = bytes(sample.payload)
        t_send = unstamp_bytes(payload)
        _ = Image.deserialize(payload[8:])
        count[0] += 1
        if count[0] > N_WARMUP:
            latencies.append(t_recv - t_send)
        if count[0] >= N_WARMUP + N_MSGS:
            received.set()

    cfg = zenoh.Config()
    session = zenoh.open(cfg)
    sub = session.declare_subscriber("bench/image", on_sample)

    time.sleep(0.3)  # let subscriber settle

    blob = msg.serialize()
    for _ in range(N_WARMUP + N_MSGS):
        payload = stamp_bytes() + blob
        session.put("bench/image", payload)
        time.sleep(0.001)  # ~1kHz

    received.wait(timeout=10)
    sub.undeclare()
    time.sleep(0.5)
    try:
        session.close()
    except Exception:
        pass  # zenoh close can timeout

    if latencies:
        report(f"end-to-end ({len(latencies)} msgs, {len(blob):,} bytes)", latencies)
    else:
        print("  ⚠ no messages received")
    return latencies


def bench_zenoh_cydr_joint(names, pos, vel, eff):
    import zenoh
    from ros2_pyterfaces.cydr.sensor_msgs.msg import JointState

    print("\n=== cydr + zenoh: JointState ===")

    names_b = np.array([n.encode() for n in names], dtype=np.bytes_)
    msg = JointState(
        name=names_b,
        position=np.array(pos, dtype=np.float64),
        velocity=np.array(vel, dtype=np.float64),
        effort=np.array(eff, dtype=np.float64),
    )

    latencies = []
    received = threading.Event()
    count = [0]

    def on_sample(sample):
        t_recv = time.monotonic_ns()
        payload = bytes(sample.payload)
        t_send = unstamp_bytes(payload)
        _ = JointState.deserialize(payload[8:])
        count[0] += 1
        if count[0] > N_WARMUP:
            latencies.append(t_recv - t_send)
        if count[0] >= N_WARMUP + N_MSGS:
            received.set()

    cfg = zenoh.Config()
    session = zenoh.open(cfg)
    sub = session.declare_subscriber("bench/joint", on_sample)

    time.sleep(0.3)

    blob = msg.serialize()
    for _ in range(N_WARMUP + N_MSGS):
        payload = stamp_bytes() + blob
        session.put("bench/joint", payload)
        time.sleep(0.0002)

    received.wait(timeout=10)
    sub.undeclare()
    time.sleep(0.5)
    try:
        session.close()
    except Exception:
        pass  # zenoh close can timeout

    if latencies:
        report(f"end-to-end ({len(latencies)} msgs, {len(blob):,} bytes)", latencies)
    else:
        print("  ⚠ no messages received")
    return latencies


# ── betterproto + zenoh ──────────────────────────────────────────────────────

def _get_proto_types():
    import betterproto
    from dataclasses import dataclass

    @dataclass
    class ProtoImage(betterproto.Message):
        height: int = betterproto.uint32_field(1)
        width: int = betterproto.uint32_field(2)
        encoding: str = betterproto.string_field(3)
        is_bigendian: int = betterproto.uint32_field(4)
        step: int = betterproto.uint32_field(5)
        data: bytes = betterproto.bytes_field(6)

    @dataclass
    class ProtoJointState(betterproto.Message):
        name: list[str] = betterproto.string_field(1)
        position: list[float] = betterproto.double_field(2)
        velocity: list[float] = betterproto.double_field(3)
        effort: list[float] = betterproto.double_field(4)

    return ProtoImage, ProtoJointState


def bench_zenoh_proto_image(img_flat):
    import zenoh
    ProtoImage, _ = _get_proto_types()

    print("\n=== protobuf + zenoh: raw Image ===")

    msg = ProtoImage(
        height=H, width=W, encoding="bgr8",
        is_bigendian=0, step=W * C,
        data=bytes(img_flat),
    )

    latencies = []
    received = threading.Event()
    count = [0]

    def on_sample(sample):
        t_recv = time.monotonic_ns()
        payload = bytes(sample.payload)
        t_send = unstamp_bytes(payload)
        _ = ProtoImage().parse(payload[8:])
        count[0] += 1
        if count[0] > N_WARMUP:
            latencies.append(t_recv - t_send)
        if count[0] >= N_WARMUP + N_MSGS:
            received.set()

    cfg = zenoh.Config()
    session = zenoh.open(cfg)
    sub = session.declare_subscriber("bench/proto_image", on_sample)

    time.sleep(0.3)

    blob = bytes(msg)
    for _ in range(N_WARMUP + N_MSGS):
        payload = stamp_bytes() + blob
        session.put("bench/proto_image", payload)
        time.sleep(0.001)

    received.wait(timeout=10)
    sub.undeclare()
    time.sleep(0.5)
    try:
        session.close()
    except Exception:
        pass  # zenoh close can timeout

    if latencies:
        report(f"end-to-end ({len(latencies)} msgs, {len(blob):,} bytes)", latencies)
    else:
        print("  ⚠ no messages received")
    return latencies


def bench_zenoh_proto_joint(names, pos, vel, eff):
    import zenoh
    _, ProtoJointState = _get_proto_types()

    print("\n=== protobuf + zenoh: JointState ===")

    msg = ProtoJointState(name=names, position=pos, velocity=vel, effort=eff)

    latencies = []
    received = threading.Event()
    count = [0]

    def on_sample(sample):
        t_recv = time.monotonic_ns()
        payload = bytes(sample.payload)
        t_send = unstamp_bytes(payload)
        _ = ProtoJointState().parse(payload[8:])
        count[0] += 1
        if count[0] > N_WARMUP:
            latencies.append(t_recv - t_send)
        if count[0] >= N_WARMUP + N_MSGS:
            received.set()

    cfg = zenoh.Config()
    session = zenoh.open(cfg)
    sub = session.declare_subscriber("bench/proto_joint", on_sample)

    time.sleep(0.3)

    blob = bytes(msg)
    for _ in range(N_WARMUP + N_MSGS):
        payload = stamp_bytes() + blob
        session.put("bench/proto_joint", payload)
        time.sleep(0.0002)

    received.wait(timeout=10)
    sub.undeclare()
    time.sleep(0.5)
    try:
        session.close()
    except Exception:
        pass  # zenoh close can timeout

    if latencies:
        report(f"end-to-end ({len(latencies)} msgs, {len(blob):,} bytes)", latencies)
    else:
        print("  ⚠ no messages received")
    return latencies


# ── Google protobuf + zenoh ──────────────────────────────────────────────────

def bench_zenoh_gpb_image(img_flat):
    import zenoh
    import sys
    pass  # generated packages are in same directory
    from bench_msgs_pb2 import Image

    print("\n=== protobuf(C) + zenoh: raw Image ===")

    msg = Image(height=H, width=W, encoding="bgr8", is_bigendian=0, step=W * C, data=bytes(img_flat))

    latencies = []
    received = threading.Event()
    count = [0]

    def on_sample(sample):
        t_recv = time.monotonic_ns()
        payload = bytes(sample.payload)
        t_send = unstamp_bytes(payload)
        m = Image()
        m.ParseFromString(payload[8:])
        count[0] += 1
        if count[0] > N_WARMUP:
            latencies.append(t_recv - t_send)
        if count[0] >= N_WARMUP + N_MSGS:
            received.set()

    cfg = zenoh.Config()
    session = zenoh.open(cfg)
    sub = session.declare_subscriber("bench/gpb_image", on_sample)
    time.sleep(0.3)

    blob = msg.SerializeToString()
    for _ in range(N_WARMUP + N_MSGS):
        payload = stamp_bytes() + blob
        session.put("bench/gpb_image", payload)
        time.sleep(0.001)

    received.wait(timeout=10)
    sub.undeclare()
    time.sleep(0.5)
    try:
        session.close()
    except Exception:
        pass  # zenoh close can timeout

    if latencies:
        report(f"end-to-end ({len(latencies)} msgs, {len(blob):,} bytes)", latencies)
    else:
        print("  ⚠ no messages received")
    return latencies


def bench_zenoh_gpb_joint(names, pos, vel, eff):
    import zenoh
    import sys
    pass  # generated packages are in same directory
    from bench_msgs_pb2 import JointState

    print("\n=== protobuf(C) + zenoh: JointState ===")

    msg = JointState(name=names, position=pos, velocity=vel, effort=eff)

    latencies = []
    received = threading.Event()
    count = [0]

    def on_sample(sample):
        t_recv = time.monotonic_ns()
        payload = bytes(sample.payload)
        t_send = unstamp_bytes(payload)
        m = JointState()
        m.ParseFromString(payload[8:])
        count[0] += 1
        if count[0] > N_WARMUP:
            latencies.append(t_recv - t_send)
        if count[0] >= N_WARMUP + N_MSGS:
            received.set()

    cfg = zenoh.Config()
    session = zenoh.open(cfg)
    sub = session.declare_subscriber("bench/gpb_joint", on_sample)
    time.sleep(0.3)

    blob = msg.SerializeToString()
    for _ in range(N_WARMUP + N_MSGS):
        payload = stamp_bytes() + blob
        session.put("bench/gpb_joint", payload)
        time.sleep(0.0002)

    received.wait(timeout=10)
    sub.undeclare()
    time.sleep(0.5)
    try:
        session.close()
    except Exception:
        pass  # zenoh close can timeout

    if latencies:
        report(f"end-to-end ({len(latencies)} msgs, {len(blob):,} bytes)", latencies)
    else:
        print("  ⚠ no messages received")
    return latencies


# ── LCM ──────────────────────────────────────────────────────────────────────

def bench_lcm_image(img_flat):
    import lcm as lcmlib
    import select

    print("\n=== LCM: raw Image ===")

    # LCM encode: stamp(8) + height(4) + width(4) + enc_len(4) + enc + step(4) + n(4) + data
    enc = b"bgr8"
    header = struct.pack(">II", H, W)
    header += struct.pack(">I", len(enc) + 1) + enc + b"\0"
    header += struct.pack(">II", W * C, H * W * C)
    img_bytes = bytes(img_flat)

    latencies = []
    received = threading.Event()
    count = [0]

    def on_msg(channel, data):
        t_recv = time.monotonic_ns()
        t_send = unstamp_bytes(data)
        # "deserialize" — skip stamp, parse header, slice data
        _ = data[8:]  # in real use you'd struct.unpack the header
        count[0] += 1
        if count[0] > N_WARMUP:
            latencies.append(t_recv - t_send)
        if count[0] >= N_WARMUP + N_MSGS:
            received.set()

    lc = lcmlib.LCM("udpm://239.255.76.67:7667?ttl=0")
    lc.subscribe("BENCH_IMAGE", on_msg)

    def recv_loop():
        while not received.is_set():
            rfds, _, _ = select.select([lc.fileno()], [], [], 0.1)
            if rfds:
                lc.handle()

    t = threading.Thread(target=recv_loop, daemon=True)
    t.start()

    time.sleep(0.3)

    for _ in range(N_WARMUP + N_MSGS):
        payload = stamp_bytes() + header + img_bytes
        lc.publish("BENCH_IMAGE", payload)
        time.sleep(0.005)  # ~200Hz — LCM needs more time for large fragmented UDP

    received.wait(timeout=10)

    if latencies:
        report(f"end-to-end ({len(latencies)} msgs, {len(header) + len(img_bytes):,} bytes)", latencies)
    else:
        print("  ⚠ no messages received")
    return latencies


def bench_lcm_joint(names, pos, vel, eff):
    import lcm as lcmlib
    import select
    import sys
    pass  # generated packages are in same directory
    from bench import JointState as LcmJointState

    print("\n=== LCM: JointState ===")

    msg = LcmJointState()
    msg.num_joints = len(names)
    msg.name = names
    msg.position = pos
    msg.velocity = vel
    msg.effort = eff
    body = msg.encode()

    latencies = []
    received = threading.Event()
    count = [0]

    def on_msg(channel, data):
        t_recv = time.monotonic_ns()
        t_send = unstamp_bytes(data)
        _ = LcmJointState.decode(data[8:])
        count[0] += 1
        if count[0] > N_WARMUP:
            latencies.append(t_recv - t_send)
        if count[0] >= N_WARMUP + N_MSGS:
            received.set()

    lc = lcmlib.LCM("udpm://239.255.76.67:7667?ttl=0")
    lc.subscribe("BENCH_JOINT", on_msg)

    def recv_loop():
        while not received.is_set():
            rfds, _, _ = select.select([lc.fileno()], [], [], 0.1)
            if rfds:
                lc.handle()

    t = threading.Thread(target=recv_loop, daemon=True)
    t.start()

    time.sleep(0.3)

    for _ in range(N_WARMUP + N_MSGS):
        payload = stamp_bytes() + body
        lc.publish("BENCH_JOINT", payload)
        time.sleep(0.0002)

    received.wait(timeout=10)

    if latencies:
        report(f"end-to-end ({len(latencies)} msgs, {len(body):,} bytes)", latencies)
    else:
        print("  ⚠ no messages received")
    return latencies


# ── msgpack + zenoh ───────────────────────────────────────────────────────────

def bench_zenoh_msgpack_image(img_flat):
    import zenoh
    import msgpack

    print("\n=== msgpack + zenoh: raw Image ===")

    img_bytes = bytes(img_flat)
    msg = {
        b"height": H, b"width": W, b"encoding": b"bgr8",
        b"is_bigendian": 0, b"step": W * C, b"data": img_bytes,
    }

    latencies = []
    received = threading.Event()
    count = [0]

    def on_sample(sample):
        t_recv = time.monotonic_ns()
        payload = bytes(sample.payload)
        t_send = unstamp_bytes(payload)
        _ = msgpack.unpackb(payload[8:], raw=False)
        count[0] += 1
        if count[0] > N_WARMUP:
            latencies.append(t_recv - t_send)
        if count[0] >= N_WARMUP + N_MSGS:
            received.set()

    cfg = zenoh.Config()
    session = zenoh.open(cfg)
    sub = session.declare_subscriber("bench/mp_image", on_sample)
    time.sleep(0.3)

    blob = msgpack.packb(msg, use_bin_type=True)
    for _ in range(N_WARMUP + N_MSGS):
        payload = stamp_bytes() + blob
        session.put("bench/mp_image", payload)
        time.sleep(0.001)

    received.wait(timeout=10)
    sub.undeclare()
    time.sleep(0.5)
    try:
        session.close()
    except Exception:
        pass

    if latencies:
        report(f"end-to-end ({len(latencies)} msgs, {len(blob):,} bytes)", latencies)
    else:
        print("  ⚠ no messages received")
    return latencies


def bench_zenoh_msgpack_joint(names, pos, vel, eff):
    import zenoh
    import msgpack

    print("\n=== msgpack + zenoh: JointState ===")

    msg = {b"name": names, b"position": pos, b"velocity": vel, b"effort": eff}

    latencies = []
    received = threading.Event()
    count = [0]

    def on_sample(sample):
        t_recv = time.monotonic_ns()
        payload = bytes(sample.payload)
        t_send = unstamp_bytes(payload)
        _ = msgpack.unpackb(payload[8:], raw=False)
        count[0] += 1
        if count[0] > N_WARMUP:
            latencies.append(t_recv - t_send)
        if count[0] >= N_WARMUP + N_MSGS:
            received.set()

    cfg = zenoh.Config()
    session = zenoh.open(cfg)
    sub = session.declare_subscriber("bench/mp_joint", on_sample)
    time.sleep(0.3)

    blob = msgpack.packb(msg, use_bin_type=True)
    for _ in range(N_WARMUP + N_MSGS):
        payload = stamp_bytes() + blob
        session.put("bench/mp_joint", payload)
        time.sleep(0.0002)

    received.wait(timeout=10)
    sub.undeclare()
    time.sleep(0.5)
    try:
        session.close()
    except Exception:
        pass

    if latencies:
        report(f"end-to-end ({len(latencies)} msgs, {len(blob):,} bytes)", latencies)
    else:
        print("  ⚠ no messages received")
    return latencies


# ── eCAL + protobuf(C) ────────────────────────────────────────────────────────

def _ecal_init():
    import ecal.nanobind_core as ecal_core
    ecal_core.initialize("bench_ipc")
    return ecal_core


def _ecal_fini(ecal_core):
    ecal_core.finalize()


def bench_ecal_gpb_image(img_flat):
    ecal_core = _ecal_init()
    from ecal.msg.proto.core import Publisher as ProtoPub, Subscriber as ProtoSub
    from bench_msgs_pb2 import Image

    print("\n=== protobuf(C) + eCAL: raw Image ===")

    msg = Image(height=H, width=W, encoding="bgr8", is_bigendian=0, step=W * C, data=bytes(img_flat))

    latencies = []
    received = threading.Event()
    count = [0]

    def on_msg(publisher_id, data):
        t_recv = time.monotonic_ns()
        t_send = data.send_timestamp
        count[0] += 1
        if count[0] > N_WARMUP:
            latencies.append(t_recv - t_send)
        if count[0] >= N_WARMUP + N_MSGS:
            received.set()

    sub = ProtoSub(Image, "bench/ecal_image")
    sub.set_receive_callback(on_msg)
    pub = ProtoPub(Image, "bench/ecal_image")

    time.sleep(2.0)  # eCAL SHM discovery

    blob_size = len(msg.SerializeToString())
    for _ in range(N_WARMUP + N_MSGS):
        pub.send(msg, time.monotonic_ns())
        time.sleep(0.001)

    received.wait(timeout=30)
    sub.remove_receive_callback()
    _ecal_fini(ecal_core)

    if latencies:
        report(f"end-to-end ({len(latencies)} msgs, {blob_size:,} bytes)", latencies)
    else:
        print("  ⚠ no messages received")
    return latencies


def bench_ecal_gpb_joint(names, pos, vel, eff):
    ecal_core = _ecal_init()
    from ecal.msg.proto.core import Publisher as ProtoPub, Subscriber as ProtoSub
    from bench_msgs_pb2 import JointState

    print("\n=== protobuf(C) + eCAL: JointState ===")

    msg = JointState(name=names, position=pos, velocity=vel, effort=eff)

    latencies = []
    received = threading.Event()
    count = [0]

    def on_msg(publisher_id, data):
        t_recv = time.monotonic_ns()
        t_send = data.send_timestamp
        count[0] += 1
        if count[0] > N_WARMUP:
            latencies.append(t_recv - t_send)
        if count[0] >= N_WARMUP + N_MSGS:
            received.set()

    sub = ProtoSub(JointState, "bench/ecal_joint")
    sub.set_receive_callback(on_msg)
    pub = ProtoPub(JointState, "bench/ecal_joint")

    time.sleep(2.0)

    blob_size = len(msg.SerializeToString())
    for _ in range(N_WARMUP + N_MSGS):
        pub.send(msg, time.monotonic_ns())
        time.sleep(0.0002)

    received.wait(timeout=10)
    sub.remove_receive_callback()
    _ecal_fini(ecal_core)

    if latencies:
        report(f"end-to-end ({len(latencies)} msgs, {blob_size:,} bytes)", latencies)
    else:
        print("  ⚠ no messages received")
    return latencies


# ── main ─────────────────────────────────────────────────────────────────────

def plot_results(results: dict, title: str, filename: str):
    import matplotlib.pyplot as plt

    labels = []
    data = []
    for label, lat_ns in results.items():
        if lat_ns:
            labels.append(label)
            data.append(np.array(lat_ns) / 1e3)  # → µs

    fig, ax = plt.subplots(figsize=(8, 5))
    bp = ax.boxplot(data, tick_labels=labels, patch_artist=True, showfliers=True,
                    flierprops=dict(marker='.', markersize=3, alpha=0.5))

    colors = ['#4C72B0', '#DD8452', '#55A868', '#937860', '#C44E52', '#8172B3']
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)

    ax.set_ylabel('Latency (µs)')
    ax.set_title(title)
    ax.grid(axis='y', alpha=0.3)

    # add median annotation
    for i, d in enumerate(data):
        med = np.median(d)
        ax.annotate(f'{med:.0f} µs', xy=(i + 1, med),
                    xytext=(10, 5), textcoords='offset points',
                    fontsize=8, color='black')

    fig.tight_layout()
    fig.savefig(filename, dpi=150)
    print(f"  → saved {filename}")
    plt.close(fig)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--suffix", default="",
                        help="Suffix for output filenames, e.g. 'default_sysctl' or 'tuned_sysctl'")
    args = parser.parse_args()

    # Detect current UDP buffer settings
    try:
        with open("/proc/sys/net/core/rmem_max") as f:
            rmem_max = int(f.read().strip())
        with open("/proc/sys/net/core/rmem_default") as f:
            rmem_default = int(f.read().strip())
    except Exception:
        rmem_max = rmem_default = -1

    suffix = args.suffix
    if not suffix:
        if rmem_max >= 20_000_000:
            suffix = "tuned_sysctl"
        else:
            suffix = "default_sysctl"

    sysctl_label = f"rmem_max={rmem_max // 1024}K, rmem_default={rmem_default // 1024}K"

    print(f"IPC Benchmark: serialize + transport + deserialize")
    print(f"Image: {W}x{H}x{C} = {W*H*C:,} bytes | JointState: {N_JOINTS} joints")
    print(f"Warmup: {N_WARMUP} msgs, Measured: {N_MSGS} msgs")
    print(f"UDP buffer: {sysctl_label}")
    print(f"Output suffix: {suffix}")

    img_flat = make_image_data()
    names, pos, vel, eff = make_joint_data()

    # Image benchmarks
    z_img = bench_zenoh_cydr_image(img_flat)
    p_img = bench_zenoh_proto_image(img_flat)
    g_img = bench_zenoh_gpb_image(img_flat)
    m_img = bench_zenoh_msgpack_image(img_flat)
    l_img = bench_lcm_image(img_flat)
    e_img = bench_ecal_gpb_image(img_flat)

    # JointState benchmarks
    z_js = bench_zenoh_cydr_joint(names, pos, vel, eff)
    p_js = bench_zenoh_proto_joint(names, pos, vel, eff)
    g_js = bench_zenoh_gpb_joint(names, pos, vel, eff)
    m_js = bench_zenoh_msgpack_joint(names, pos, vel, eff)
    l_js = bench_lcm_joint(names, pos, vel, eff)
    e_js = bench_ecal_gpb_joint(names, pos, vel, eff)

    # Box plots
    print("\n=== Generating plots ===")
    img_results = {
        "cydr\n+zenoh": z_img, "betterproto\n+zenoh": p_img,
        "protobuf(C)\n+zenoh": g_img, "msgpack\n+zenoh": m_img,
        "LCM": l_img, "protobuf(C)\n+eCAL": e_img,
    }
    js_results = {
        "cydr\n+zenoh": z_js, "betterproto\n+zenoh": p_js,
        "protobuf(C)\n+zenoh": g_js, "msgpack\n+zenoh": m_js,
        "LCM": l_js, "protobuf(C)\n+eCAL": e_js,
    }

    plot_results(
        img_results,
        f"IPC Latency: Raw Image ({W}×{H}×{C})  [{sysctl_label}]",
        f"bench_ipc_image_{suffix}.png",
    )
    plot_results(
        js_results,
        f"IPC Latency: JointState ({N_JOINTS} joints)  [{sysctl_label}]",
        f"bench_ipc_joint_{suffix}.png",
    )

    # Save raw latency data for combined plotting later
    import json
    data_file = f"bench_ipc_data_{suffix}.json"
    save_data = {
        "suffix": suffix,
        "sysctl_label": sysctl_label,
        "image": {k.replace("\n", " "): v for k, v in img_results.items()},
        "joint": {k.replace("\n", " "): v for k, v in js_results.items()},
    }
    with open(data_file, "w") as f:
        json.dump(save_data, f)
    print(f"  → saved {data_file}")

    # Also save the canonical names (latest run)
    import shutil
    shutil.copy(f"bench_ipc_image_{suffix}.png", "bench_ipc_image.png")
    shutil.copy(f"bench_ipc_joint_{suffix}.png", "bench_ipc_joint.png")
    print(f"  → also copied to bench_ipc_image.png, bench_ipc_joint.png")


def plot_combined():
    """Load saved JSON data from both sysctl runs and produce a 2×2 figure."""
    import json
    import matplotlib.pyplot as plt

    suffixes = ["default_sysctl", "tuned_sysctl"]
    row_labels = ["Default sysctl (rmem 208 KB)", "Tuned sysctl (rmem 20 MB)"]
    col_keys = ["image", "joint"]
    col_titles = [
        f"Raw Image ({W}×{H}×{C})",
        f"JointState ({N_JOINTS} joints)",
    ]
    colors = ['#4C72B0', '#DD8452', '#55A868', '#937860', '#C44E52', '#8172B3']

    runs = {}
    for s in suffixes:
        path = f"bench_ipc_data_{s}.json"
        with open(path) as f:
            runs[s] = json.load(f)

    fig, axes = plt.subplots(2, 2, figsize=(16, 10))

    # First pass: collect y-range per column so both rows share the same scale
    col_ymax = [0.0, 0.0]
    for row, sfx in enumerate(suffixes):
        for col, ck in enumerate(col_keys):
            for lat_ns in runs[sfx][ck].values():
                if lat_ns:
                    arr = np.array(lat_ns) / 1e3
                    # Use 95th percentile × 1.15 for y-limit (ignore extreme outliers)
                    col_ymax[col] = max(col_ymax[col], np.percentile(arr, 95) * 1.15)

    # Second pass: plot
    for row, (sfx, row_label) in enumerate(zip(suffixes, row_labels)):
        for col, (ck, ct) in enumerate(zip(col_keys, col_titles)):
            ax = axes[row][col]
            results = runs[sfx][ck]

            labels = []
            data = []
            for label, lat_ns in results.items():
                if lat_ns:
                    labels.append(label.replace(" ", "\n"))
                    data.append(np.array(lat_ns) / 1e3)

            if not data:
                ax.set_visible(False)
                continue

            bp = ax.boxplot(data, tick_labels=labels, patch_artist=True,
                            showfliers=True,
                            flierprops=dict(marker='.', markersize=2, alpha=0.4))
            for patch, color in zip(bp['boxes'], colors[:len(data)]):
                patch.set_facecolor(color)
                patch.set_alpha(0.7)

            for i, d in enumerate(data):
                med = np.median(d)
                if med > 1000:
                    lbl = f"{med/1000:.1f}ms"
                else:
                    lbl = f"{med:.0f}µs"
                ax.annotate(lbl, xy=(i + 1, med),
                            xytext=(8, 4), textcoords='offset points',
                            fontsize=7, color='black')

            ax.set_ylim(0, col_ymax[col])
            ax.set_ylabel('Latency (µs)')
            ax.set_title(f"{ct}\n{row_label}", fontsize=10)
            ax.tick_params(axis='x', rotation=30, labelsize=7)
            ax.grid(axis='y', alpha=0.3)

    fig.suptitle("IPC Latency: Default vs Tuned sysctl UDP Buffer", fontsize=14, y=0.98)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fname = "bench_ipc_combined.png"
    fig.savefig(fname, dpi=150)
    print(f"  → saved {fname}")
    plt.close(fig)


if __name__ == "__main__":
    import sys
    if "--combine" in sys.argv:
        plot_combined()
    else:
        main()
