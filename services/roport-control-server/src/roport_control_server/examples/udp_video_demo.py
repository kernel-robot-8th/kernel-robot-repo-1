"""Loopback-only encoded-frame sender / mock AI receiver; no camera or inference."""

import argparse
import json
import socket
import time
from pathlib import Path
from uuid import UUID

from ..communication.udp_video import (
    DATAGRAM_BYTES,
    MAX_FRAME_BYTES,
    FrameInfo,
    FrameReceiver,
    FrameSender,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("role", choices=("send", "receive"))
    parser.add_argument("--session", required=True, type=UUID)
    parser.add_argument("--file", type=Path, help="encoded JPEG or PNG (sender)")
    parser.add_argument("--codec", choices=("jpeg", "png"), default="jpeg")
    parser.add_argument("--camera", default="cam1")
    parser.add_argument("--port", type=int, default=5301)
    parser.add_argument("--source-port", type=int, default=5302)
    parser.add_argument("--frames", type=int, default=10)
    parser.add_argument("--fps", type=float, default=10)
    parser.add_argument("--bitrate", type=int, default=8_000_000)
    args = parser.parse_args()
    if args.frames < 1 or not all(
        1 <= p <= 65535 for p in (args.port, args.source_port)
    ):
        parser.error("positive frame count and valid ports required")
    key = ("omx1", args.camera, "video")
    peer = ("127.0.0.1", args.source_port)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        if args.role == "send":
            if not args.file:
                parser.error("--file is required for sending")
            with args.file.open("rb") as source:
                frame = source.read(MAX_FRAME_BYTES + 1)
            sock.bind(peer)
            sender = FrameSender(
                sock, ("127.0.0.1", args.port), fps=args.fps, bitrate=args.bitrate
            )
            for frame_id in range(args.frames):
                info = FrameInfo(
                    *key,
                    args.session,
                    frame_id,
                    time.time_ns(),
                    f"obs-{frame_id}",
                    args.codec,
                )
                sender.send(frame, info)
        else:
            sock.bind(("127.0.0.1", args.port))
            sock.settimeout(0.1)
            receiver = FrameReceiver({key: (peer, args.session)})
            print("Listening on loopback (mock AI; no inference)", flush=True)
            try:
                while True:
                    try:
                        packet, source = sock.recvfrom(DATAGRAM_BYTES + 1)
                    except TimeoutError:
                        receiver.expire()
                        continue
                    result = receiver.feed(packet, source)
                    if result:
                        info, frame = result
                        print(
                            json.dumps(
                                {
                                    "camera": info.camera,
                                    "frame_id": info.frame_id,
                                    "bytes": len(frame),
                                    "stats": receiver.stats,
                                }
                            ),
                            flush=True,
                        )
            except KeyboardInterrupt:
                print(json.dumps(receiver.stats), flush=True)


if __name__ == "__main__":
    main()
