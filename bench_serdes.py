#!/usr/bin/env python3
"""Benchmark serialize/deserialize of a 1280x960x3 Image using ros2-pyterfaces.

Compares the cyclone and cydr backends for both raw Image and CompressedImage
(JPEG-encoded via OpenCV).

Install:
    pip install 'ros2_pyterfaces[cyclone,cydr]' opencv-python numpy
"""

import time
import numpy as np

# ── helpers ──────────────────────────────────────────────────────────────────

H, W, C = 960, 1280, 3
N_WARMUP = 5
N_ITERS = 200

# Collect results for plotting: {(msg_type, backend): {"ser": [µs,...], "deser": [µs,...]}}
RESULTS = {}


def bench(name: str, fn, n_warmup=N_WARMUP, n_iters=N_ITERS):
    for _ in range(n_warmup):
        fn()
    timings = []
    for _ in range(n_iters):
        t0 = time.perf_counter()
        fn()
        timings.append((time.perf_counter() - t0) * 1e6)
    mean_us = sum(timings) / len(timings)
    print(f"  {name:.<50s} {mean_us:>9.1f} µs  ({n_iters} iters)")
    return timings


def record(msg_type: str, backend: str, ser_timings: list, deser_timings: list):
    """Store per-iteration serdes timings for later plotting."""
    RESULTS[(msg_type, backend)] = {"ser": ser_timings, "deser": deser_timings}


def make_random_image():
    return np.random.randint(0, 256, (H, W, C), dtype=np.uint8)


def jpeg_encode(img: np.ndarray, quality=90) -> bytes:
    import cv2
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    assert ok
    return bytes(buf)


# ── cyclone backend ──────────────────────────────────────────────────────────

def bench_cyclone(img_flat: np.ndarray, jpeg_bytes: bytes):
    from ros2_pyterfaces.cyclone.sensor_msgs.msg import Image, CompressedImage

    print("\n=== Cyclone backend ===")

    # raw Image — construct once, benchmark only ser/deser
    msg = Image(
        height=H, width=W, encoding="bgr8",
        is_bigendian=0, step=W * C,
        data=img_flat.tolist(),
    )
    blob = msg.serialize()
    print(f"  Raw Image blob size: {len(blob):,} bytes")

    def ser_raw():
        msg.serialize()

    def deser_raw():
        Image.deserialize(blob)

    s = bench("serialize   (raw Image)", ser_raw)
    d = bench("deserialize (raw Image)", deser_raw)
    record("Image", "Cyclone", s, d)

    # CompressedImage — construct once
    cmsg = CompressedImage(format="jpeg", data=list(jpeg_bytes))
    cblob = cmsg.serialize()
    print(f"  CompressedImage blob size: {len(cblob):,} bytes")

    def ser_comp():
        cmsg.serialize()

    def deser_comp():
        CompressedImage.deserialize(cblob)

    s = bench("serialize   (CompressedImage)", ser_comp)
    d = bench("deserialize (CompressedImage)", deser_comp)
    record("Compressed", "Cyclone", s, d)

    # roundtrip correctness
    rt = Image.deserialize(blob)
    assert rt.height == H and rt.width == W and rt.encoding == "bgr8"
    assert rt.data == msg.data, "cyclone raw Image roundtrip MISMATCH"
    crt = CompressedImage.deserialize(cblob)
    assert crt.data == cmsg.data, "cyclone CompressedImage roundtrip MISMATCH"
    print("  ✓ roundtrip correctness verified")


# ── cydr backend ─────────────────────────────────────────────────────────────

def bench_cydr(img_flat: np.ndarray, jpeg_bytes: bytes):
    from ros2_pyterfaces.cydr.sensor_msgs.msg import Image, CompressedImage

    print("\n=== cydr backend ===")

    # raw Image — construct once, benchmark only ser/deser
    msg = Image(
        height=np.uint32(H), width=np.uint32(W), encoding=b"bgr8",
        is_bigendian=np.uint8(0), step=np.uint32(W * C),
        data=img_flat,
    )
    blob = msg.serialize()
    print(f"  Raw Image blob size: {len(blob):,} bytes")

    def ser_raw():
        msg.serialize()

    def deser_raw():
        Image.deserialize(blob)

    s = bench("serialize   (raw Image)", ser_raw)
    d = bench("deserialize (raw Image)", deser_raw)
    record("Image", "cydr", s, d)

    # CompressedImage — construct once
    jpeg_arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    cmsg = CompressedImage(format=b"jpeg", data=jpeg_arr)
    cblob = cmsg.serialize()
    print(f"  CompressedImage blob size: {len(cblob):,} bytes")

    def ser_comp():
        cmsg.serialize()

    def deser_comp():
        CompressedImage.deserialize(cblob)

    s = bench("serialize   (CompressedImage)", ser_comp)
    d = bench("deserialize (CompressedImage)", deser_comp)
    record("Compressed", "cydr", s, d)

    # roundtrip correctness
    rt = Image.deserialize(blob)
    assert rt.height == H and rt.width == W and rt.encoding == b"bgr8"
    assert np.array_equal(rt.data, img_flat), "cydr raw Image roundtrip MISMATCH"
    crt = CompressedImage.deserialize(cblob)
    assert np.array_equal(crt.data, jpeg_arr), "cydr CompressedImage roundtrip MISMATCH"
    print("  ✓ roundtrip correctness verified")


# ── Protobuf (betterproto) ───────────────────────────────────────────────────

def _get_proto_types():
    """Define protobuf message types via betterproto (no .proto file needed)."""
    import betterproto
    from dataclasses import dataclass, field as dc_field

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

    @dataclass
    class ProtoCompressedImage(betterproto.Message):
        format: str = betterproto.string_field(1)
        data: bytes = betterproto.bytes_field(2)

    return ProtoImage, ProtoJointState, ProtoCompressedImage


def bench_proto_image(img_flat: np.ndarray, jpeg_bytes: bytes):
    ProtoImage, _, ProtoCompressedImage = _get_proto_types()

    print("\n=== Protobuf/betterproto (raw Image) ===")
    msg = ProtoImage(
        height=H, width=W, encoding="bgr8",
        is_bigendian=0, step=W * C,
        data=bytes(img_flat),
    )
    blob = bytes(msg)
    print(f"  blob size: {len(blob):,} bytes")

    def ser():
        bytes(msg)
    def deser():
        ProtoImage().parse(blob)

    s = bench("serialize   (raw Image)", ser)
    d = bench("deserialize (raw Image)", deser)
    record("Image", "betterproto", s, d)

    rt = ProtoImage().parse(blob)
    assert rt.height == H and rt.width == W and rt.encoding == "bgr8"
    assert rt.data == bytes(img_flat), "protobuf raw Image roundtrip MISMATCH"
    print("  ✓ roundtrip correctness verified")

    # CompressedImage
    cmsg = ProtoCompressedImage(format="jpeg", data=jpeg_bytes)
    cblob = bytes(cmsg)
    print(f"  CompressedImage blob size: {len(cblob):,} bytes")

    def ser_comp():
        bytes(cmsg)
    def deser_comp():
        ProtoCompressedImage().parse(cblob)

    s = bench("serialize   (CompressedImage)", ser_comp)
    d = bench("deserialize (CompressedImage)", deser_comp)
    record("Compressed", "betterproto", s, d)

    crt = ProtoCompressedImage().parse(cblob)
    assert crt.data == jpeg_bytes, "betterproto CompressedImage roundtrip MISMATCH"
    print("  ✓ CompressedImage roundtrip verified")


def bench_proto_jointstate(names, pos, vel, eff):
    _, ProtoJointState, _ = _get_proto_types()

    print("\n=== Protobuf/betterproto (JointState) ===")
    msg = ProtoJointState(name=names, position=pos, velocity=vel, effort=eff)
    blob = bytes(msg)
    print(f"  blob size: {len(blob):,} bytes")

    def ser():
        bytes(msg)
    def deser():
        ProtoJointState().parse(blob)

    s = bench("serialize   (JointState)", ser)
    d = bench("deserialize (JointState)", deser)
    record("JointState", "betterproto", s, d)

    rt = ProtoJointState().parse(blob)
    assert rt.name == names and rt.position == pos
    print("  ✓ roundtrip correctness verified")


# ── Google Protobuf (C/upb backend) ──────────────────────────────────────────

def bench_google_proto_image(img_flat: np.ndarray, jpeg_bytes: bytes):
    import sys
    pass  # generated packages are in same directory
    from bench_msgs_pb2 import Image, CompressedImage

    print("\n=== Google protobuf/upb (raw Image) ===")
    msg = Image(height=H, width=W, encoding="bgr8", is_bigendian=0, step=W * C, data=bytes(img_flat))
    blob = msg.SerializeToString()
    print(f"  blob size: {len(blob):,} bytes")

    def ser():
        msg.SerializeToString()
    def deser():
        m = Image()
        m.ParseFromString(blob)

    s = bench("serialize   (raw Image)", ser)
    d = bench("deserialize (raw Image)", deser)
    record("Image", "protobuf(C)", s, d)

    rt = Image()
    rt.ParseFromString(blob)
    assert rt.height == H and rt.width == W and rt.encoding == "bgr8"
    assert rt.data == bytes(img_flat), "google protobuf raw Image roundtrip MISMATCH"
    print("  ✓ roundtrip correctness verified")

    # CompressedImage
    cmsg = CompressedImage(format="jpeg", data=jpeg_bytes)
    cblob = cmsg.SerializeToString()
    print(f"  CompressedImage blob size: {len(cblob):,} bytes")

    def ser_comp():
        cmsg.SerializeToString()
    def deser_comp():
        m = CompressedImage()
        m.ParseFromString(cblob)

    s = bench("serialize   (CompressedImage)", ser_comp)
    d = bench("deserialize (CompressedImage)", deser_comp)
    record("Compressed", "protobuf(C)", s, d)

    crt = CompressedImage()
    crt.ParseFromString(cblob)
    assert crt.data == jpeg_bytes, "google protobuf CompressedImage roundtrip MISMATCH"
    print("  ✓ CompressedImage roundtrip verified")


def bench_google_proto_jointstate(names, pos, vel, eff):
    import sys
    pass  # generated packages are in same directory
    from bench_msgs_pb2 import JointState

    print("\n=== Google protobuf/upb (JointState) ===")
    msg = JointState(name=names, position=pos, velocity=vel, effort=eff)
    blob = msg.SerializeToString()
    print(f"  blob size: {len(blob):,} bytes")

    def ser():
        msg.SerializeToString()
    def deser():
        m = JointState()
        m.ParseFromString(blob)

    s = bench("serialize   (JointState)", ser)
    d = bench("deserialize (JointState)", deser)
    record("JointState", "protobuf(C)", s, d)

    rt = JointState()
    rt.ParseFromString(blob)
    assert list(rt.name) == names and list(rt.position) == pos
    print("  ✓ roundtrip correctness verified")


# ── LCM ───────────────────────────────────────────────────────────────────────

def bench_lcm_image(img_flat: np.ndarray, jpeg_bytes: bytes):
    import sys
    pass  # generated packages are in same directory
    from bench import Image as LcmImage, CompressedImage as LcmCompressedImage

    print("\n=== LCM (raw Image) ===")
    msg = LcmImage()
    msg.height = H
    msg.width = W
    msg.encoding = "bgr8"
    msg.step = W * C
    msg.data_len = len(img_flat)
    msg.data = bytes(img_flat)
    blob = msg.encode()
    print(f"  blob size: {len(blob):,} bytes")

    def ser():
        msg.encode()
    def deser():
        LcmImage.decode(blob)

    s = bench("serialize   (raw Image)", ser)
    d = bench("deserialize (raw Image)", deser)
    record("Image", "LCM", s, d)

    rt = LcmImage.decode(blob)
    assert rt.height == H and rt.width == W and rt.encoding == "bgr8"
    assert rt.data == bytes(img_flat), "LCM raw Image roundtrip MISMATCH"
    print("  ✓ roundtrip correctness verified")

    # CompressedImage
    cmsg = LcmCompressedImage()
    cmsg.format = "jpeg"
    cmsg.data_len = len(jpeg_bytes)
    cmsg.data = jpeg_bytes
    cblob = cmsg.encode()
    print(f"  CompressedImage blob size: {len(cblob):,} bytes")

    def ser_comp():
        cmsg.encode()
    def deser_comp():
        LcmCompressedImage.decode(cblob)

    s = bench("serialize   (CompressedImage)", ser_comp)
    d = bench("deserialize (CompressedImage)", deser_comp)
    record("Compressed", "LCM", s, d)

    crt = LcmCompressedImage.decode(cblob)
    assert crt.data == jpeg_bytes, "LCM CompressedImage roundtrip MISMATCH"
    print("  ✓ CompressedImage roundtrip verified")


def bench_lcm_jointstate(names, pos, vel, eff):
    import sys
    pass  # generated packages are in same directory
    from bench import JointState as LcmJointState

    print("\n=== LCM (JointState) ===")
    msg = LcmJointState()
    msg.num_joints = len(names)
    msg.name = names
    msg.position = pos
    msg.velocity = vel
    msg.effort = eff
    blob = msg.encode()
    print(f"  blob size: {len(blob):,} bytes")

    def ser():
        msg.encode()
    def deser():
        LcmJointState.decode(blob)

    s = bench("serialize   (JointState)", ser)
    d = bench("deserialize (JointState)", deser)
    record("JointState", "LCM", s, d)

    rt = LcmJointState.decode(blob)
    assert rt.name == names and list(rt.position) == pos
    print("  ✓ roundtrip correctness verified")


# ── msgpack ────────────────────────────────────────────────────────────────────

def bench_msgpack_image(img_flat: np.ndarray, jpeg_bytes: bytes):
    import msgpack

    print("\n=== msgpack (raw Image) ===")

    img_bytes = bytes(img_flat)
    msg = {
        b"height": H, b"width": W, b"encoding": b"bgr8",
        b"is_bigendian": 0, b"step": W * C, b"data": img_bytes,
    }
    blob = msgpack.packb(msg, use_bin_type=True)
    print(f"  blob size: {len(blob):,} bytes")

    def ser():
        msgpack.packb(msg, use_bin_type=True)
    def deser():
        msgpack.unpackb(blob, raw=False)

    s = bench("serialize   (raw Image)", ser)
    d = bench("deserialize (raw Image)", deser)
    record("Image", "msgpack", s, d)

    rt = msgpack.unpackb(blob, raw=False)
    assert rt[b"height"] == H and rt[b"width"] == W
    assert rt[b"data"] == img_bytes, "msgpack raw Image roundtrip MISMATCH"
    print("  ✓ roundtrip correctness verified")

    # CompressedImage
    cmsg = {b"format": b"jpeg", b"data": jpeg_bytes}
    cblob = msgpack.packb(cmsg, use_bin_type=True)
    print(f"  CompressedImage blob size: {len(cblob):,} bytes")

    def ser_comp():
        msgpack.packb(cmsg, use_bin_type=True)
    def deser_comp():
        msgpack.unpackb(cblob, raw=False)

    s = bench("serialize   (CompressedImage)", ser_comp)
    d = bench("deserialize (CompressedImage)", deser_comp)
    record("Compressed", "msgpack", s, d)

    crt = msgpack.unpackb(cblob, raw=False)
    assert crt[b"data"] == jpeg_bytes, "msgpack CompressedImage roundtrip MISMATCH"
    print("  ✓ CompressedImage roundtrip verified")


def bench_msgpack_jointstate(names, pos, vel, eff):
    import msgpack

    print("\n=== msgpack (JointState) ===")

    msg = {b"name": names, b"position": pos, b"velocity": vel, b"effort": eff}
    blob = msgpack.packb(msg, use_bin_type=True)
    print(f"  blob size: {len(blob):,} bytes")

    def ser():
        msgpack.packb(msg, use_bin_type=True)
    def deser():
        msgpack.unpackb(blob, raw=False)

    s = bench("serialize   (JointState)", ser)
    d = bench("deserialize (JointState)", deser)
    record("JointState", "msgpack", s, d)

    rt = msgpack.unpackb(blob, raw=False)
    assert rt[b"name"] == names and rt[b"position"] == pos
    print("  ✓ roundtrip correctness verified")


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    print(f"Image size: {W}x{H}x{C}  ({W*H*C:,} bytes raw)")
    print(f"Warmup: {N_WARMUP}, Iterations: {N_ITERS}")

    img = make_random_image()
    img_flat = img.ravel()
    jpeg_bytes = jpeg_encode(img)
    print(f"JPEG size: {len(jpeg_bytes):,} bytes (quality=90)")

    bench_cyclone(img_flat, jpeg_bytes)
    bench_cydr(img_flat, jpeg_bytes)
    bench_proto_image(img_flat, jpeg_bytes)
    bench_google_proto_image(img_flat, jpeg_bytes)
    bench_lcm_image(img_flat, jpeg_bytes)
    bench_msgpack_image(img_flat, jpeg_bytes)

    # cross-backend: verify cyclone and cydr produce identical blobs
    print("\n=== Cross-backend check ===")
    from ros2_pyterfaces.cyclone.sensor_msgs.msg import Image as CycloneImage
    from ros2_pyterfaces.cydr.sensor_msgs.msg import Image as CydrImage

    cmsg = CycloneImage(
        height=H, width=W, encoding="bgr8",
        is_bigendian=0, step=W * C,
        data=img_flat.tolist(),
    )
    dmsg = CydrImage(
        height=np.uint32(H), width=np.uint32(W), encoding=b"bgr8",
        is_bigendian=np.uint8(0), step=np.uint32(W * C),
        data=img_flat,
    )
    cblob = cmsg.serialize()
    dblob = dmsg.serialize()
    assert cblob == dblob, f"cyclone vs cydr blob MISMATCH (len {len(cblob)} vs {len(dblob)})"
    print("  ✓ cyclone and cydr produce identical serialized bytes")

    # ── JointState (16 joints) ───────────────────────────────────────────────
    bench_jointstate()

    # ── Payload size summary ─────────────────────────────────────────────────
    N_JOINTS = 16
    names = [f"joint_{i}" for i in range(N_JOINTS)]
    pos = np.random.randn(N_JOINTS).tolist()
    vel = np.random.randn(N_JOINTS).tolist()
    eff = np.random.randn(N_JOINTS).tolist()
    print_payload_sizes(img_flat, jpeg_bytes, names, pos, vel, eff)

    # ── Serdes plots ─────────────────────────────────────────────────────────
    plot_serdes()


def bench_jointstate():
    N_JOINTS = 16
    names = [f"joint_{i}" for i in range(N_JOINTS)]
    pos = np.random.randn(N_JOINTS).tolist()
    vel = np.random.randn(N_JOINTS).tolist()
    eff = np.random.randn(N_JOINTS).tolist()

    print(f"\n{'='*60}")
    print(f"JointState: {N_JOINTS} joints")
    print(f"{'='*60}")

    # ── cyclone
    from ros2_pyterfaces.cyclone.sensor_msgs.msg import JointState as CycloneJS

    print("\n=== Cyclone backend (JointState) ===")
    msg = CycloneJS(name=names, position=pos, velocity=vel, effort=eff)
    blob = msg.serialize()
    print(f"  blob size: {len(blob):,} bytes")

    def ser():
        msg.serialize()
    def deser():
        CycloneJS.deserialize(blob)

    s = bench("serialize   (JointState)", ser)
    d = bench("deserialize (JointState)", deser)
    record("JointState", "Cyclone", s, d)

    rt = CycloneJS.deserialize(blob)
    assert rt.name == names and rt.position == pos
    print("  ✓ roundtrip correctness verified")

    # ── cydr
    from ros2_pyterfaces.cydr.sensor_msgs.msg import JointState as CydrJS

    print("\n=== cydr backend (JointState) ===")
    names_b = np.array([n.encode() for n in names], dtype=np.bytes_)
    pos_np = np.array(pos, dtype=np.float64)
    vel_np = np.array(vel, dtype=np.float64)
    eff_np = np.array(eff, dtype=np.float64)
    msg2 = CydrJS(name=names_b, position=pos_np, velocity=vel_np, effort=eff_np)
    blob2 = msg2.serialize()
    print(f"  blob size: {len(blob2):,} bytes")

    def ser2():
        msg2.serialize()
    def deser2():
        CydrJS.deserialize(blob2)

    s = bench("serialize   (JointState)", ser2)
    d = bench("deserialize (JointState)", deser2)
    record("JointState", "cydr", s, d)

    rt2 = CydrJS.deserialize(blob2)
    assert np.array_equal(rt2.position, pos_np)
    print("  ✓ roundtrip correctness verified")

    # cross-backend
    assert blob == blob2, f"cyclone vs cydr JointState blob MISMATCH"
    print("\n  ✓ cyclone and cydr JointState blobs identical")

    # ── LCM JointState
    bench_lcm_jointstate(names, pos, vel, eff)

    # ── Protobuf JointState
    bench_proto_jointstate(names, pos, vel, eff)

    # ── Google Protobuf JointState
    bench_google_proto_jointstate(names, pos, vel, eff)

    # ── msgpack JointState
    bench_msgpack_jointstate(names, pos, vel, eff)


def print_payload_sizes(img_flat, jpeg_bytes, names, pos, vel, eff):
    import sys
    pass  # generated packages are in same directory

    from ros2_pyterfaces.cyclone.sensor_msgs.msg import (
        Image as CycloneImage, CompressedImage as CycloneCompressed,
    )
    from ros2_pyterfaces.cydr.sensor_msgs.msg import (
        Image as CydrImage, CompressedImage as CydrCompressed,
    )
    from ros2_pyterfaces.cyclone.sensor_msgs.msg import JointState as CycloneJS
    from ros2_pyterfaces.cydr.sensor_msgs.msg import JointState as CydrJS
    from bench import Image as LcmImage, JointState as LcmJointState, CompressedImage as LcmCompressedImage
    from bench_msgs_pb2 import Image as PbImage, JointState as PbJS

    ProtoImage, ProtoJointState, ProtoCompressedImage = _get_proto_types()

    # Image sizes
    cyc_img = CycloneImage(height=H, width=W, encoding="bgr8", is_bigendian=0, step=W*C, data=img_flat.tolist())
    cyd_img = CydrImage(height=np.uint32(H), width=np.uint32(W), encoding=b"bgr8", is_bigendian=np.uint8(0), step=np.uint32(W*C), data=img_flat)
    bp_img = ProtoImage(height=H, width=W, encoding="bgr8", is_bigendian=0, step=W*C, data=bytes(img_flat))
    gpb_img = PbImage(height=H, width=W, encoding="bgr8", is_bigendian=0, step=W*C, data=bytes(img_flat))
    lcm_img = LcmImage()
    lcm_img.height = H; lcm_img.width = W; lcm_img.encoding = "bgr8"
    lcm_img.step = W * C; lcm_img.data_len = len(img_flat); lcm_img.data = bytes(img_flat)

    # CompressedImage sizes
    cyc_comp = CycloneCompressed(format="jpeg", data=list(jpeg_bytes))
    jpeg_arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    cyd_comp = CydrCompressed(format=b"jpeg", data=jpeg_arr)
    lcm_comp = LcmCompressedImage()
    lcm_comp.format = "jpeg"; lcm_comp.data_len = len(jpeg_bytes); lcm_comp.data = jpeg_bytes

    # JointState sizes
    names_b = np.array([n.encode() for n in names], dtype=np.bytes_)
    cyc_js = CycloneJS(name=names, position=pos, velocity=vel, effort=eff)
    cyd_js = CydrJS(name=names_b, position=np.array(pos), velocity=np.array(vel), effort=np.array(eff))
    bp_js = ProtoJointState(name=names, position=pos, velocity=vel, effort=eff)
    gpb_js = PbJS(name=names, position=pos, velocity=vel, effort=eff)
    lcm_js = LcmJointState()
    lcm_js.num_joints = len(names); lcm_js.name = names
    lcm_js.position = pos; lcm_js.velocity = vel; lcm_js.effort = eff

    raw = W * H * C

    print(f"\n{'='*60}")
    print(f"Payload sizes (bytes)")
    print(f"{'='*60}")
    print(f"  Raw pixel data: {raw:,}")
    print(f"  JPEG data:      {len(jpeg_bytes):,}")
    print()
    print(f"  {'Message':<25s} {'CDR':>10s} {'Protobuf':>10s} {'Proto(C)':>10s} {'LCM':>10s} {'msgpack':>10s}")
    print(f"  {'-'*25} {'-'*10} {'-'*10} {'-'*10} {'-'*10} {'-'*10}")

    import msgpack
    mp_img = msgpack.packb({b"height": H, b"width": W, b"encoding": b"bgr8", b"is_bigendian": 0, b"step": W*C, b"data": bytes(img_flat)}, use_bin_type=True)
    print(f"  {'raw Image':<25s} {len(cyc_img.serialize()):>10,} {len(bytes(bp_img)):>10,} {len(gpb_img.SerializeToString()):>10,} {len(lcm_img.encode()):>10,} {len(mp_img):>10,}")
    bp_comp = ProtoCompressedImage(format="jpeg", data=jpeg_bytes)
    from bench_msgs_pb2 import CompressedImage as PbComp
    gpb_comp = PbComp(format="jpeg", data=jpeg_bytes)
    mp_comp = msgpack.packb({b"format": b"jpeg", b"data": jpeg_bytes}, use_bin_type=True)
    print(f"  {'CompressedImage (JPEG)':<25s} {len(cyc_comp.serialize()):>10,} {len(bytes(bp_comp)):>10,} {len(gpb_comp.SerializeToString()):>10,} {len(lcm_comp.encode()):>10,} {len(mp_comp):>10,}")
    mp_js = msgpack.packb({b"name": names, b"position": pos, b"velocity": vel, b"effort": eff}, use_bin_type=True)
    print(f"  {'JointState (16 joints)':<25s} {len(cyc_js.serialize()):>10,} {len(bytes(bp_js)):>10,} {len(gpb_js.SerializeToString()):>10,} {len(lcm_js.encode()):>10,} {len(mp_js):>10,}")


def plot_serdes():
    import matplotlib.pyplot as plt

    msg_types = ["Image", "Compressed", "JointState"]
    msg_labels = ["Raw Image (3.7 MB)", "CompressedImage (JPEG ~1.1 MB)", "JointState (16 joints)"]
    backends = ["Cyclone", "cydr", "betterproto", "protobuf(C)", "LCM", "msgpack"]
    colors = ['#4C72B0', '#55A868', '#DD8452', '#C44E52', '#8172B3', '#937860']

    for op, op_key in [("Serialize", "ser"), ("Deserialize", "deser")]:
        fig, axes = plt.subplots(1, 3, figsize=(14, 5))
        fig.suptitle(f"{op} Latency", fontsize=14)

        for ax, mt, ml in zip(axes, msg_types, msg_labels):
            data = []
            labels = []
            for b in backends:
                key = (mt, b)
                if key in RESULTS:
                    data.append(RESULTS[key][op_key])
                    labels.append(b)

            if not data:
                continue

            bp = ax.boxplot(data, tick_labels=labels, patch_artist=True, showfliers=True,
                            flierprops=dict(marker='.', markersize=2, alpha=0.4))
            for patch, color in zip(bp['boxes'], colors[:len(data)]):
                patch.set_facecolor(color)
                patch.set_alpha(0.7)

            # annotate medians
            for i, d in enumerate(data):
                med = np.median(d)
                if med > 1000:
                    label = f"{med/1000:.1f}ms"
                elif med < 1:
                    label = f"{med:.2f}µs"
                else:
                    label = f"{med:.1f}µs"
                ax.annotate(label, xy=(i + 1, med),
                            xytext=(8, 4), textcoords='offset points',
                            fontsize=7, color='black')

            ax.set_title(ml, fontsize=10)
            ax.set_ylabel('µs')
            ax.tick_params(axis='x', rotation=30, labelsize=8)
            ax.grid(axis='y', alpha=0.3)

        fig.tight_layout()
        fname = f"bench_serdes_{op.lower()}.png"
        fig.savefig(fname, dpi=150)
        print(f"  → saved {fname}")
        plt.close(fig)


if __name__ == "__main__":
    main()
