"""Render packaged GLBs, without product photos or external network resources.

After npm run prepare:assets, run from a graphical session (or xvfb-run):
env -u PYTHONPATH uv run --isolated --no-project --with PyQt6==6.11.0
  --with PyQt6-WebEngine==6.11.0 python -I tools/render_display_thumbnails.py
GPU/Qt differences may change pixels; regenerate only after a model/style change.
"""

import base64
import hashlib
import json
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Thread

from PyQt6.QtCore import QTimer, QUrl
from PyQt6.QtGui import QImage
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWidgets import QApplication

ASSETS = (
    Path(__file__).resolve().parents[1] / "src/roport_control_server/spatial_assets"
)
MODELS = {"pinky-pro.webp": "pinky", "omx-f.webp": "omx"}
HTML = b"""<!doctype html><meta charset="utf-8">
<script type="importmap">{"imports":{"three":"/vendor/three/esm/three.module.js"}}</script>
<script>window.onerror = message => { window.result = {error: String(message)}; };</script>
<script type="module">
import * as THREE from 'three';
import {GLTFLoader} from '/vendor/three/examples/jsm/loaders/GLTFLoader.js';
try {
  const renderer = new THREE.WebGLRenderer({antialias: true, alpha: true});
  renderer.setSize(512, 512);
  renderer.setPixelRatio(1);
  const images = {};
  for (const [file, kind] of [['pinky-pro.webp', 'pinky'], ['omx-f.webp', 'omx']]) {
    const model = (await new GLTFLoader().loadAsync(`/models/${kind}/robot.glb`)).scene;
    const scene = new THREE.Scene();
    scene.add(model, new THREE.HemisphereLight(0xffffff, 0x858e94, 2.3));
    const light = new THREE.DirectionalLight(0xffffff, 3);
    light.position.set(-2, -3, 7);
    scene.add(light);
    const sphere = new THREE.Box3().setFromObject(model).getBoundingSphere(new THREE.Sphere());
    const r = sphere.radius * 1.08;
    const camera = new THREE.OrthographicCamera(-r, r, r, -r, r * 0.01, r * 10);
    camera.up.set(0, 0, 1);
    camera.position.copy(sphere.center).add(new THREE.Vector3(1, -1, 0.7).multiplyScalar(r * 3));
    camera.lookAt(sphere.center);
    renderer.render(scene, camera);
    images[file] = renderer.domElement.toDataURL('image/webp', 0.92);
    model.traverse(node => { if (node.isMesh) { node.geometry.dispose(); node.material.dispose(); } });
  }
  renderer.dispose();
  window.result = {images};
} catch (error) { window.result = {error: String(error)}; }
</script>"""


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ASSETS), **kwargs)

    def do_GET(self):
        if self.path != "/thumbnail.html":
            return super().do_GET()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.end_headers()
        self.wfile.write(HTML)

    def log_message(self, *_):
        pass


def main():
    app = QApplication(["render-display-thumbnails"])
    view = QWebEngineView()
    result = {}
    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    worker = Thread(target=server.serve_forever, daemon=True)
    worker.start()
    poll = QTimer()

    def receive(value):
        if value:
            result.update(value)
            poll.stop()
            app.quit()

    poll.timeout.connect(lambda: view.page().runJavaScript("window.result", receive))
    poll.start(250)
    QTimer.singleShot(30000, app.quit)
    try:
        view.load(QUrl(f"http://127.0.0.1:{server.server_port}/thumbnail.html"))
        view.show()
        app.exec()
    finally:
        poll.stop()
        view.close()
        server.shutdown()
        server.server_close()
        worker.join()
    assert "images" in result, result or "Rendering timed out"
    assert set(result["images"]) == set(MODELS), result.keys()
    images, sources = {}, {}
    for filename, kind in MODELS.items():
        prefix, encoded = result["images"][filename].split(",", 1)
        assert prefix == "data:image/webp;base64", prefix
        data = base64.b64decode(encoded, validate=True)
        image = QImage.fromData(data)
        assert image.width() == image.height() == 512 and image.hasAlphaChannel()
        alphas = {
            image.pixelColor(x, y).alpha()
            for x in range(0, 512, 8)
            for y in range(0, 512, 8)
        }
        assert 0 in alphas and 255 in alphas, "Empty or opaque render"
        images[filename] = data
        model = f"models/{kind}/robot.glb"
        sources[filename] = {
            "model": model,
            "sha256": hashlib.sha256((ASSETS / model).read_bytes()).hexdigest(),
        }
    # Validate both renders before replacing either tracked image.
    for filename, data in images.items():
        (ASSETS / filename).write_bytes(data)
    (ASSETS / "thumbnails.json").write_text(json.dumps(sources, indent=2) + "\n")
    print("Rendered and validated 2 transparent 512 x 512 thumbnails")


if __name__ == "__main__":
    main()
