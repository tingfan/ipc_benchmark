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


if __name__ == "__main__":
    main()
