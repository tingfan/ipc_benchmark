#!/usr/bin/env python3
"""Write protobuf messages (JointState, Image, CompressedImage) to MCAP,
verify roundtrip via mcap-protobuf-support reader, then test rerun conversion.

Uses Foxglove well-known protobuf schemas (foxglove.RawImage,
foxglove.CompressedImage) so that rerun's MCAP loader can render images
natively without any rerun SDK calls.

References:
  - foxglove/mcap protobuf writer tests
  - rerun.io/blog/introducing-experimental-support-for-mcap-file-format
  - github.com/foxglove/schemas  (foxglove.RawImage, foxglove.CompressedImage)
"""

import subprocess
import sys
import time
from io import BytesIO
from pathlib import Path

import numpy as np
from google.protobuf.timestamp_pb2 import Timestamp

from bench_msgs_pb2 import JointState
from foxglove_schemas_protobuf.CompressedImage_pb2 import (
    CompressedImage as FoxCompressedImage,
)
from foxglove_schemas_protobuf.RawImage_pb2 import RawImage as FoxRawImage
from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory
from mcap_protobuf.writer import Writer

# ── constants ────────────────────────────────────────────────────────────────

H, W, C = 960, 1280, 3
N_JOINTS = 16
MCAP_FILE = Path("bench_msgs.mcap")
RRD_FILE = Path("bench_msgs.rrd")


# ── helpers ──────────────────────────────────────────────────────────────────

def make_random_image() -> np.ndarray:
    return np.random.randint(0, 256, (H, W, C), dtype=np.uint8)


def jpeg_encode(img: np.ndarray, quality: int = 90) -> bytes:
    import cv2
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    assert ok
    return bytes(buf)


def read_protobuf_messages(stream: BytesIO):
    """Read back all decoded protobuf messages from an MCAP stream."""
    return make_reader(
        stream, decoder_factories=[DecoderFactory()]
    ).iter_decoded_messages()


# ── write MCAP ───────────────────────────────────────────────────────────────

def write_mcap(path: Path) -> dict:
    """Write all three message types to an MCAP file. Returns the source messages.

    Images use Foxglove well-known schemas (foxglove.RawImage / foxglove.CompressedImage)
    so rerun can render them natively.  JointState uses the custom bench.JointState schema
    (decoded via protobuf reflection layer).
    """
    img = make_random_image()
    img_flat = img.ravel()
    jpeg_bytes = jpeg_encode(img)

    names = [f"joint_{i}" for i in range(N_JOINTS)]
    pos = np.random.randn(N_JOINTS).tolist()
    vel = np.random.randn(N_JOINTS).tolist()
    eff = np.random.randn(N_JOINTS).tolist()

    now_s = int(time.time())
    ts = Timestamp(seconds=now_s, nanos=0)
    t_ns = now_s * 1_000_000_000

    # Foxglove RawImage (rerun renders this as an image)
    raw_msg = FoxRawImage(
        timestamp=ts, frame_id="camera",
        width=W, height=H, encoding="bgr8",
        step=W * C, data=bytes(img_flat),
    )
    # Foxglove CompressedImage (rerun renders this as an image)
    comp_msg = FoxCompressedImage(
        timestamp=ts, frame_id="camera",
        format="jpeg", data=jpeg_bytes,
    )
    # Custom JointState (rerun decodes via protobuf reflection)
    joint_msg = JointState(name=names, position=pos, velocity=vel, effort=eff)

    with open(path, "wb") as f:
        with Writer(f) as writer:
            writer.write_message("/camera/image_raw", raw_msg, log_time=t_ns)
            writer.write_message("/camera/image_compressed", comp_msg, log_time=t_ns + 1_000_000)
            for i in range(5):
                js = JointState(
                    name=names,
                    position=[p + 0.01 * i for p in pos],
                    velocity=vel,
                    effort=eff,
                )
                writer.write_message("/joint_states", js, log_time=t_ns + i * 10_000_000)

    size = path.stat().st_size
    print(f"  wrote {path}  ({size:,} bytes)")
    return {
        "raw": raw_msg,
        "compressed": comp_msg,
        "joint": joint_msg,
        "names": names,
        "pos": pos,
        "jpeg_bytes": jpeg_bytes,
        "img_flat_bytes": bytes(img_flat),
    }


# ── read-back & verify ──────────────────────────────────────────────────────

def verify_mcap(path: Path, src: dict):
    """Read MCAP back and verify protobuf roundtrip correctness."""
    counts = {}
    with open(path, "rb") as f:
        buf = BytesIO(f.read())

    for schema, channel, message, decoded in read_protobuf_messages(buf):
        topic = channel.topic
        counts[topic] = counts.get(topic, 0) + 1

        if topic == "/camera/image_raw":
            assert decoded.DESCRIPTOR.full_name == "foxglove.RawImage"
            assert decoded.width == W and decoded.height == H
            assert decoded.encoding == "bgr8"
            assert decoded.data == src["img_flat_bytes"]
        elif topic == "/camera/image_compressed":
            assert decoded.DESCRIPTOR.full_name == "foxglove.CompressedImage"
            assert decoded.format == "jpeg"
            assert decoded.data == src["jpeg_bytes"]
        elif topic == "/joint_states":
            assert decoded.DESCRIPTOR.full_name == "bench.JointState"
            assert list(decoded.name) == src["names"]
            assert len(decoded.position) == N_JOINTS

    assert counts.get("/camera/image_raw") == 1, f"expected 1 RawImage, got {counts.get('/camera/image_raw')}"
    assert counts.get("/camera/image_compressed") == 1
    assert counts.get("/joint_states") == 5, f"expected 5 JointState, got {counts.get('/joint_states')}"
    print(f"  ✓ roundtrip verified: {counts}")


# ── rerun conversion test ────────────────────────────────────────────────────

def test_rerun_convert(mcap_path: Path, rrd_path: Path):
    """Test: rerun mcap convert input.mcap -o output.rrd"""
    rrd_path.unlink(missing_ok=True)

    cmd = [sys.executable, "-m", "rerun", "mcap", "convert", str(mcap_path), "-o", str(rrd_path)]
    print(f"  $ {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)

    print(f"  return code: {result.returncode}")
    if result.stdout.strip():
        print(f"  stdout: {result.stdout.strip()}")
    if result.stderr.strip():
        print(f"  stderr: {result.stderr.strip()}")

    if result.returncode == 0 and rrd_path.exists():
        size = rrd_path.stat().st_size
        print(f"  ✓ rerun converted {mcap_path} → {rrd_path}  ({size:,} bytes)")
        return True
    else:
        print(f"  ✗ rerun conversion failed (code {result.returncode})")
        return False


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    print(f"Image: {W}x{H}x{C}, JointState: {N_JOINTS} joints\n")

    print("1. Writing protobuf messages to MCAP …")
    src = write_mcap(MCAP_FILE)

    print("\n2. Reading back & verifying roundtrip …")
    verify_mcap(MCAP_FILE, src)

    print("\n3. Testing rerun MCAP → RRD conversion …")
    ok = test_rerun_convert(MCAP_FILE, RRD_FILE)

    if ok:
        print(f"\nDone. Open in rerun viewer with:\n  rerun {RRD_FILE}")
    else:
        print(f"\nMCAP file is valid. Open directly in rerun:\n  rerun {MCAP_FILE}")


if __name__ == "__main__":
    main()
