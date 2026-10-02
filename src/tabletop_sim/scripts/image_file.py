"""Dependency-free RGB PNG writer."""

from pathlib import Path
import struct
import zlib


def save_png(frame, path: Path) -> None:
    def chunk(kind: bytes, data: bytes) -> bytes:
        body = kind + data
        return struct.pack('>I', len(data)) + body + struct.pack('>I', zlib.crc32(body))

    stride = frame.width * 3
    pixels = b''.join(
        b'\0' + frame.rgb[row * stride : (row + 1) * stride] for row in range(frame.height)
    )
    header = struct.pack('>IIBBBBB', frame.width, frame.height, 8, 2, 0, 0, 0)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(
        b'\x89PNG\r\n\x1a\n'
        + chunk(b'IHDR', header)
        + chunk(b'IDAT', zlib.compress(pixels))
        + chunk(b'IEND', b'')
    )
