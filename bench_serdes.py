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


def bench(name: str, fn, n_warmup=N_WARMUP, n_iters=N_ITERS):
    for _ in range(n_warmup):
        fn()
    t0 = time.perf_counter()
    for _ in range(n_iters):
        fn()
    elapsed = time.perf_counter() - t0
    mean_us = elapsed / n_iters * 1e6
    print(f"  {name:.<50s} {mean_us:>9.1f} µs  ({n_iters} iters)")
    return mean_us


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

    bench("serialize   (raw Image)", ser_raw)
    bench("deserialize (raw Image)", deser_raw)

    # CompressedImage — construct once
    cmsg = CompressedImage(format="jpeg", data=list(jpeg_bytes))
    cblob = cmsg.serialize()
    print(f"  CompressedImage blob size: {len(cblob):,} bytes")

    def ser_comp():
        cmsg.serialize()

    def deser_comp():
        CompressedImage.deserialize(cblob)

    bench("serialize   (CompressedImage)", ser_comp)
    bench("deserialize (CompressedImage)", deser_comp)

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

    bench("serialize   (raw Image)", ser_raw)
    bench("deserialize (raw Image)", deser_raw)

    # CompressedImage — construct once
    jpeg_arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    cmsg = CompressedImage(format=b"jpeg", data=jpeg_arr)
    cblob = cmsg.serialize()
    print(f"  CompressedImage blob size: {len(cblob):,} bytes")

    def ser_comp():
        cmsg.serialize()

    def deser_comp():
        CompressedImage.deserialize(cblob)

    bench("serialize   (CompressedImage)", ser_comp)
    bench("deserialize (CompressedImage)", deser_comp)

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

    bench("serialize   (raw Image)", ser)
    bench("deserialize (raw Image)", deser)

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

    bench("serialize   (CompressedImage)", ser_comp)
    bench("deserialize (CompressedImage)", deser_comp)

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

    bench("serialize   (JointState)", ser)
    bench("deserialize (JointState)", deser)

    rt = ProtoJointState().parse(blob)
    assert rt.name == names and rt.position == pos
    print("  ✓ roundtrip correctness verified")


# ── Google Protobuf (C/upb backend) ──────────────────────────────────────────

def bench_google_proto_image(img_flat: np.ndarray, jpeg_bytes: bytes):
    import sys
    pass  # bench_msgs_pb2 is in same directory
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

    bench("serialize   (raw Image)", ser)
    bench("deserialize (raw Image)", deser)

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

    bench("serialize   (CompressedImage)", ser_comp)
    bench("deserialize (CompressedImage)", deser_comp)

    crt = CompressedImage()
    crt.ParseFromString(cblob)
    assert crt.data == jpeg_bytes, "google protobuf CompressedImage roundtrip MISMATCH"
    print("  ✓ CompressedImage roundtrip verified")


def bench_google_proto_jointstate(names, pos, vel, eff):
    import sys
    pass  # bench_msgs_pb2 is in same directory
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

    bench("serialize   (JointState)", ser)
    bench("deserialize (JointState)", deser)

    rt = JointState()
    rt.ParseFromString(blob)
    assert list(rt.name) == names and list(rt.position) == pos
    print("  ✓ roundtrip correctness verified")


# ── LCM ───────────────────────────────────────────────────────────────────────

def bench_lcm_image(img_flat: np.ndarray):
    from lcm_types import LcmImage

    print("\n=== LCM (raw Image) ===")
    msg = LcmImage(
        timestamp=0, height=H, width=W,
        encoding="bgr8", step=W * C,
        data=bytes(img_flat),
    )
    blob = msg.encode()
    print(f"  blob size: {len(blob):,} bytes")

    def ser():
        msg.encode()
    def deser():
        LcmImage.decode(blob)

    bench("serialize   (raw Image)", ser)
    bench("deserialize (raw Image)", deser)

    rt = LcmImage.decode(blob)
    assert rt.height == H and rt.width == W and rt.encoding == "bgr8"
    assert rt.data == bytes(img_flat), "LCM raw Image roundtrip MISMATCH"
    print("  ✓ roundtrip correctness verified")


def bench_lcm_jointstate(names, pos, vel, eff):
    from lcm_types import LcmJointState

    print("\n=== LCM (JointState) ===")
    msg = LcmJointState(
        timestamp=0, num_joints=len(names),
        name=names, position=pos, velocity=vel, effort=eff,
    )
    blob = msg.encode()
    print(f"  blob size: {len(blob):,} bytes")

    def ser():
        msg.encode()
    def deser():
        LcmJointState.decode(blob)

    bench("serialize   (JointState)", ser)
    bench("deserialize (JointState)", deser)

    rt = LcmJointState.decode(blob)
    assert rt.name == names and rt.position == pos
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
    bench_lcm_image(img_flat)

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

    bench("serialize   (JointState)", ser)
    bench("deserialize (JointState)", deser)

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

    bench("serialize   (JointState)", ser2)
    bench("deserialize (JointState)", deser2)

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


def print_payload_sizes(img_flat, jpeg_bytes, names, pos, vel, eff):
    import sys
    pass  # bench_msgs_pb2 is in same directory

    from ros2_pyterfaces.cyclone.sensor_msgs.msg import (
        Image as CycloneImage, CompressedImage as CycloneCompressed,
    )
    from ros2_pyterfaces.cydr.sensor_msgs.msg import (
        Image as CydrImage, CompressedImage as CydrCompressed,
    )
    from ros2_pyterfaces.cyclone.sensor_msgs.msg import JointState as CycloneJS
    from ros2_pyterfaces.cydr.sensor_msgs.msg import JointState as CydrJS
    from lcm_types import LcmImage, LcmJointState
    from bench_msgs_pb2 import Image as PbImage, JointState as PbJS

    ProtoImage, ProtoJointState, ProtoCompressedImage = _get_proto_types()

    # Image sizes
    cyc_img = CycloneImage(height=H, width=W, encoding="bgr8", is_bigendian=0, step=W*C, data=img_flat.tolist())
    cyd_img = CydrImage(height=np.uint32(H), width=np.uint32(W), encoding=b"bgr8", is_bigendian=np.uint8(0), step=np.uint32(W*C), data=img_flat)
    bp_img = ProtoImage(height=H, width=W, encoding="bgr8", is_bigendian=0, step=W*C, data=bytes(img_flat))
    gpb_img = PbImage(height=H, width=W, encoding="bgr8", is_bigendian=0, step=W*C, data=bytes(img_flat))
    lcm_img = LcmImage(timestamp=0, height=H, width=W, encoding="bgr8", step=W*C, data=bytes(img_flat))

    # CompressedImage sizes
    cyc_comp = CycloneCompressed(format="jpeg", data=list(jpeg_bytes))
    jpeg_arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    cyd_comp = CydrCompressed(format=b"jpeg", data=jpeg_arr)

    # JointState sizes
    names_b = np.array([n.encode() for n in names], dtype=np.bytes_)
    cyc_js = CycloneJS(name=names, position=pos, velocity=vel, effort=eff)
    cyd_js = CydrJS(name=names_b, position=np.array(pos), velocity=np.array(vel), effort=np.array(eff))
    bp_js = ProtoJointState(name=names, position=pos, velocity=vel, effort=eff)
    gpb_js = PbJS(name=names, position=pos, velocity=vel, effort=eff)
    lcm_js = LcmJointState(timestamp=0, num_joints=len(names), name=names, position=pos, velocity=vel, effort=eff)

    raw = W * H * C

    print(f"\n{'='*60}")
    print(f"Payload sizes (bytes)")
    print(f"{'='*60}")
    print(f"  Raw pixel data: {raw:,}")
    print(f"  JPEG data:      {len(jpeg_bytes):,}")
    print()
    print(f"  {'Message':<25s} {'CDR':>10s} {'Protobuf':>10s} {'Proto(C)':>10s} {'LCM':>10s}")
    print(f"  {'-'*25} {'-'*10} {'-'*10} {'-'*10} {'-'*10}")
    print(f"  {'raw Image':<25s} {len(cyc_img.serialize()):>10,} {len(bytes(bp_img)):>10,} {len(gpb_img.SerializeToString()):>10,} {len(lcm_img.encode()):>10,}")
    bp_comp = ProtoCompressedImage(format="jpeg", data=jpeg_bytes)
    from bench_msgs_pb2 import CompressedImage as PbComp
    gpb_comp = PbComp(format="jpeg", data=jpeg_bytes)
    print(f"  {'CompressedImage (JPEG)':<25s} {len(cyc_comp.serialize()):>10,} {len(bytes(bp_comp)):>10,} {len(gpb_comp.SerializeToString()):>10,} {'n/a':>10s}")
    print(f"  {'JointState (16 joints)':<25s} {len(cyc_js.serialize()):>10,} {len(bytes(bp_js)):>10,} {len(gpb_js.SerializeToString()):>10,} {len(lcm_js.encode()):>10,}")


if __name__ == "__main__":
    main()
