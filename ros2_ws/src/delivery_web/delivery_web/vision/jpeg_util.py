"""Minimal valid JPEG helpers for mock / placeholder frames."""

from __future__ import annotations

import struct
import zlib
from typing import Optional


def _png_to_jpeg_via_pil(png: bytes) -> Optional[bytes]:
    try:
        from io import BytesIO

        from PIL import Image  # type: ignore

        im = Image.open(BytesIO(png))
        buf = BytesIO()
        im.convert("RGB").save(buf, format="JPEG", quality=70)
        return buf.getvalue()
    except Exception:  # noqa: BLE001
        return None


def _minimal_png(w: int, h: int, rgb_fill: tuple[int, int, int]) -> bytes:
    r, g, b = rgb_fill
    raw = bytearray()
    for _y in range(h):
        raw.append(0)
        for _x in range(w):
            raw.extend((r, g, b))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + tag
            + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(bytes(raw), 6))
        + chunk(b"IEND", b"")
    )


def mock_wrist_jpeg(label: str = "MOCK WRIST") -> bytes:
    """Return a small valid JPEG for mock/offline UI testing."""
    try:
        from io import BytesIO

        from PIL import Image, ImageDraw, ImageFont  # type: ignore

        im = Image.new("RGB", (320, 200), (20, 32, 48))
        draw = ImageDraw.Draw(im)
        draw.rectangle([4, 4, 315, 195], outline=(61, 184, 168), width=3)
        draw.text((24, 80), label, fill=(228, 236, 244))
        draw.text((24, 110), "Mock Mode — No Hardware", fill=(128, 148, 168))
        buf = BytesIO()
        im.save(buf, format="JPEG", quality=75)
        return buf.getvalue()
    except Exception:  # noqa: BLE001
        pass
    png = _minimal_png(320, 200, (30, 80, 90))
    jpg = _png_to_jpeg_via_pil(png)
    if jpg and jpg[:2] == b"\xff\xd8":
        return jpg
    # Smallest valid 1x1 JPEG (fallback)
    return bytes.fromhex(
        "ffd8ffe000104a46494600010100000100010000ffdb004300"
        "080606070605080707070909080a0c140d0c0b0b0c1912130f141d1a"
        "1f1e1d1a1c1c20242e2720222c231c1c2837292c3031343434"
        "1f27393d38323c2e333432ffdb0043010909090c0b0c180d0d"
        "1832211c213232323232323232323232323232323232323232"
        "323232323232323232323232323232323232323232323232"
        "ffc00011080001000103011100021101031101ffc4001f000001"
        "0501010101010100000000000000000102030405060708090a0b"
        "ffc400b5100000020103030204030505030404000000017d0102"
        "030004110521314106071641227108142891a1b1c108233352"
        "f0ffc4001a0100030101010100000000000000000000000102"
        "0304ffc4002f1100010002000404040301040100000000000001"
        "02031104052131061213415122337191a1ffc400140101000000"
        "00000000000000000000000000ffda000c0301000211031100"
        "3f00aa3f0000ffd9"
    )
