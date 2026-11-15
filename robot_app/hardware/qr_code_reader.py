from __future__ import annotations

from typing import Any


def decode_qr_codes(frame: Any, encoding: str = "utf-8") -> list[dict[str, Any]]:
    """Decode all QR codes in a frame and infer the two-tier shelf from image position."""
    from PIL import Image
    from pyzbar.pyzbar import ZBarSymbol, decode

    image = Image.fromarray(frame)
    frame_height = image.height
    output: list[dict[str, Any]] = []
    for code in decode(image, symbols=[ZBarSymbol.QRCODE]):
        value = code.data.decode(encoding, errors="ignore").strip()
        if not value:
            continue
        center_x = int(code.rect.left + code.rect.width / 2)
        center_y = int(code.rect.top + code.rect.height / 2)
        output.append({
            "qr_code": value,
            "level_number": 2 if center_y < frame_height / 2 else 1,
            "center_x": center_x,
            "center_y": center_y,
        })
    return output
