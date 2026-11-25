"""Authenticated loopback preview for the virtual Pinky only; never a camera proxy."""

import hmac
import io
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

from PIL import Image, ImageDraw


def frame_bytes(driver, image_ref):
    with driver.lock:
        registration = driver.registration
        if (
            not registration
            or registration["image_ref"] != image_ref
            or registration["perception_boot_id"] != driver.perception_boot_id
            or time.monotonic() >= registration["expires_mono"]
        ):
            return None
        boxes = [candidate["box"][:] for candidate in registration["candidates"]]
    picture = Image.new("RGB", (640, 480), "#e8edf2")
    draw = ImageDraw.Draw(picture)
    draw.text((18, 15), "VIRTUAL FOLLOW TEST - SYNTHETIC PEOPLE", fill="#172a42")
    for index, box in enumerate(boxes):
        x1, y1, x2, y2 = (
            round(box[i] * (640 if i % 2 == 0 else 480)) for i in range(4)
        )
        center = (x1 + x2) // 2
        draw.ellipse((center - 25, y1 + 10, center + 25, y1 + 60), fill="#718096")
        draw.rounded_rectangle(
            (x1 + 20, y1 + 70, x2 - 20, y2 - 20), radius=16, fill="#718096"
        )
        draw.rectangle((x1, y1, x2, y2), outline="#146c94", width=3)
        draw.text((x1 + 5, y2 - 15), f"Candidate {index + 1}", fill="#172a42")
    output = io.BytesIO()
    picture.save(output, format="JPEG")
    return output.getvalue()


def start_preview(driver, port, token):
    if not token or len(token) < 24:
        raise ValueError(
            "ROPORT_FOLLOW_FRAME_TOKEN must contain at least 24 characters"
        )

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if not hmac.compare_digest(
                self.headers.get("Authorization", "").encode(),
                f"Bearer {token}".encode(),
            ):
                self.send_error(401)
                return
            path = urlsplit(self.path).path
            data = (
                frame_bytes(driver, path[len("/frames/") :])
                if path.startswith("/frames/")
                else None
            )
            if data is None:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *_args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server
