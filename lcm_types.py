"""Hand-written LCM types matching sensor_msgs/Image and JointState for benchmarking."""

import struct


class LcmImage:
    """LCM type for a raw image (mirrors sensor_msgs/Image layout)."""

    __slots__ = ("timestamp", "height", "width", "encoding", "step", "data")

    def __init__(self, timestamp=0, height=0, width=0, encoding="", step=0, data=b""):
        self.timestamp = timestamp
        self.height = height
        self.width = width
        self.encoding = encoding
        self.step = step
        self.data = data

    def encode(self):
        buf = bytearray()
        buf += struct.pack(">q", self.timestamp)
        buf += struct.pack(">I", self.height)
        buf += struct.pack(">I", self.width)
        enc = self.encoding.encode("utf-8") if isinstance(self.encoding, str) else self.encoding
        buf += struct.pack(">I", len(enc) + 1)
        buf += enc + b"\0"
        buf += struct.pack(">I", self.step)
        buf += struct.pack(">I", len(self.data))
        buf += bytes(self.data)
        return bytes(buf)

    @classmethod
    def decode(cls, data):
        obj = cls()
        off = 0
        obj.timestamp, = struct.unpack_from(">q", data, off); off += 8
        obj.height, = struct.unpack_from(">I", data, off); off += 4
        obj.width, = struct.unpack_from(">I", data, off); off += 4
        enc_len, = struct.unpack_from(">I", data, off); off += 4
        obj.encoding = data[off:off + enc_len - 1].decode("utf-8"); off += enc_len
        obj.step, = struct.unpack_from(">I", data, off); off += 4
        n, = struct.unpack_from(">I", data, off); off += 4
        obj.data = data[off:off + n]
        return obj


class LcmJointState:
    """LCM type for joint state (16 joints)."""

    __slots__ = ("timestamp", "num_joints", "name", "position", "velocity", "effort")

    def __init__(self, timestamp=0, num_joints=0, name=None, position=None, velocity=None, effort=None):
        self.timestamp = timestamp
        self.num_joints = num_joints
        self.name = name or []
        self.position = position or []
        self.velocity = velocity or []
        self.effort = effort or []

    def encode(self):
        buf = bytearray()
        buf += struct.pack(">q", self.timestamp)
        buf += struct.pack(">I", self.num_joints)
        for n in self.name:
            s = n.encode("utf-8") if isinstance(n, str) else n
            buf += struct.pack(">I", len(s) + 1)
            buf += s + b"\0"
        buf += struct.pack(f">{self.num_joints}d", *self.position)
        buf += struct.pack(f">{self.num_joints}d", *self.velocity)
        buf += struct.pack(f">{self.num_joints}d", *self.effort)
        return bytes(buf)

    @classmethod
    def decode(cls, data):
        obj = cls()
        off = 0
        obj.timestamp, = struct.unpack_from(">q", data, off); off += 8
        obj.num_joints, = struct.unpack_from(">I", data, off); off += 4
        obj.name = []
        for _ in range(obj.num_joints):
            slen, = struct.unpack_from(">I", data, off); off += 4
            obj.name.append(data[off:off + slen - 1].decode("utf-8")); off += slen
        obj.position = list(struct.unpack_from(f">{obj.num_joints}d", data, off)); off += 8 * obj.num_joints
        obj.velocity = list(struct.unpack_from(f">{obj.num_joints}d", data, off)); off += 8 * obj.num_joints
        obj.effort = list(struct.unpack_from(f">{obj.num_joints}d", data, off))
        return obj
