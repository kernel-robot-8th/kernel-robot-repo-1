import * as THREE from "three";
import { OrbitControls } from "./vendor/three/examples/jsm/controls/OrbitControls.js";
import { GLTFLoader } from "./vendor/three/examples/jsm/loaders/GLTFLoader.js";

import { renderTrial, trialDisconnected } from "./trial.js";

const $ = (id) => document.getElementById(id);
let ids = [], inventory = [];
const title = (id) => inventory.find((r) => r.id === id)?.name ?? id;
const kind = (id) => inventory.find((r) => r.id === id)?.kind;
let selected,
  rows = [],
  views,
  models = {},
  definitions = {};
let demo = false, trial = false,
  playing = true,
  dropped = new Set(),
  mapCenter,
  mapExtent;
const instances = {},
  previews = {},
  buttons = {},
  labels = {};
const vector = new THREE.Vector3();
const mapLabels = [];
const installedPoses = {};

function displayedPose(row) {
  return (demo ? installedPoses[row.robot_id] : null) ?? row.pose ?? installedPoses[row.robot_id] ?? null;
}

function installationPose(mount) {
  const base = models.omx.getObjectByName("link0_visual_0");
  if (!base) throw new Error("OMX base mesh missing");
  const bounds = new THREE.Box3().setFromObject(base).applyMatrix4(
    new THREE.Matrix4().makeRotationZ(mount.yaw - mount.edge_yaw),
  );
  // Align the physical base footprint with the support edge, not the arm's full reach.
  const offset = new THREE.Vector3(-(bounds.min.x + bounds.max.x) / 2, -bounds.max.y, -bounds.min.z);
  offset.applyAxisAngle(new THREE.Vector3(0, 0, 1), mount.edge_yaw);
  return [...new THREE.Vector3(...mount.edge_center).add(offset), Math.atan2(Math.sin(mount.yaw), Math.cos(mount.yaw))];
}

let trialNodes, trialRouteKey;
const trialRouteObjects = [];
function makeTrialRoutes(config) {
  const key = JSON.stringify(config.trial_routes);
  if (key === trialRouteKey) return;
  trialRouteKey = key;
  for (const line of trialRouteObjects.splice(0)) {
    views.map.scene.remove(line);
    line.geometry.dispose();
    line.material.dispose();
  }
  for (const route of config.trial_routes) {
    const points = route.map((id) => new THREE.Vector3(...config.trial_nodes[id], 0.015));
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(points),
      new THREE.LineBasicMaterial({color: 0x257164}),
    );
    views.map.scene.add(line);
    trialRouteObjects.push(line);
  }
  views.map.dirty = true;
}

function text(tag, value, className = "") {
  const node = document.createElement(tag);
  node.textContent = value;
  node.className = className;
  return node;
}
function view(element) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color("#dfe4e7");
  scene.add(new THREE.HemisphereLight(0xffffff, 0x858e94, 2.3));
  const light = new THREE.DirectionalLight(0xffffff, 3.0);
  light.position.set(-2, -3, 7);
  scene.add(light);
  const camera = new THREE.PerspectiveCamera(42, 1, 0.005, 300);
  camera.up.set(0, 0, 1);
  // Retain unchanged previews for canvas readback in browser regression checks.
  // This trades buffer memory for the dirty-only rendering loop below.
  const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  element.prepend(renderer.domElement);
  const controls = new OrbitControls(camera, renderer.domElement);
  const result = { scene, camera, renderer, controls, dirty: true };
  controls.addEventListener("change", () => { result.dirty = true; });
  controls.maxPolarAngle = Math.PI / 2 - 0.01;
  controls.minDistance = 0.1;
  controls.maxDistance = 200;
  new ResizeObserver(() => {
    const w = element.clientWidth,
      h = element.clientHeight;
    if (!w || !h) return;
    renderer.setSize(w, h, false);
    result.dirty = true;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }).observe(element);
  renderer.domElement.addEventListener("webglcontextlost", (e) => {
    e.preventDefault();
    $("message").textContent =
      "3D 그래픽 연결이 끊겼습니다. 화면을 다시 열어 주세요.";
    document.body.dataset.ready = "false";
  });
  return result;
}
function home(top = false) {
  document
    .querySelectorAll("[data-view]")
    .forEach((b) =>
      b.setAttribute(
        "aria-pressed",
        String(b.dataset.view === (top ? "top" : "3d")),
      ),
    );
  const v = views.map;
  v.controls.target.copy(mapCenter);
  v.camera.position
    .copy(mapCenter)
    .add(
      new THREE.Vector3(
        top ? 0 : -0.65,
        top ? -0.001 : -0.85,
        top ? 1.6 : 1.2,
      ).multiplyScalar(mapExtent),
    );
  v.controls.update();
}
function makeMap(map) {
  const group = new THREE.Group();
  group.position.set(map.origin[0], map.origin[1], 0);
  group.rotation.z = map.origin[2];
  const width = map.width * map.resolution,
    height = map.height * map.resolution;
  const texture = new THREE.TextureLoader().load(map.image, () => { views.map.dirty = true; });
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.magFilter = THREE.NearestFilter;
  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(width, height),
    new THREE.MeshStandardMaterial({ map: texture, roughness: 1 }),
  );
  floor.position.set(width / 2, height / 2, -0.003);
  group.add(floor);
  const walls = new THREE.InstancedMesh(
    new THREE.BoxGeometry(1, 1, 1),
    new THREE.MeshStandardMaterial({ color: 0xc3cbd0, roughness: 0.9 }),
    map.walls.length,
  );
  const matrix = new THREE.Matrix4(),
    q = new THREE.Quaternion();
  map.walls.forEach(([x, y, length], i) => {
    matrix.compose(
      new THREE.Vector3(
        (x + length / 2) * map.resolution,
        (map.height - y - 0.5) * map.resolution,
        map.wall_height / 2,
      ),
      q,
      new THREE.Vector3(
        length * map.resolution,
        map.resolution,
        map.wall_height,
      ),
    );
    walls.setMatrixAt(i, matrix);
  });
  group.add(walls);
  views.map.scene.add(group);
  group.updateMatrixWorld(true);
  mapCenter = group.localToWorld(new THREE.Vector3(width / 2, height / 2, 0));
  mapExtent = Math.max(width, height, 0.5);
  home();
  if (map.layout) {
    const points = [], edges = [], nodes = new Map(map.layout.nodes.map(n => [n.id, n]));
    for (const node of nodes.values()) {
      const position = new THREE.Vector3(...node.position, 0.012);
      points.push(...position);
      for (const id of node.links) {
        if (node.id < id) edges.push(...position, ...nodes.get(id).position, 0.012);
      }
      const label = text("span", node.place ? `${node.place} · ${node.id}` : node.nav2_switch ? `${node.id} · Nav2` : node.id, "map-node");
      label.dataset.node = node.id;
      label.title = "배치 원본의 노드 위치 · 주행 승인 아님";
      $("labels").append(label);
      mapLabels.push({position, label});
    }
    const geometry = values => new THREE.BufferGeometry().setAttribute("position", new THREE.Float32BufferAttribute(values, 3));
    views.map.scene.add(new THREE.LineSegments(geometry(edges), new THREE.LineBasicMaterial({color: 0x81939e})));
    views.map.scene.add(new THREE.Points(geometry(points), new THREE.PointsMaterial({color: 0x526774, size: 0.035})));
  }
}
function robot(type) {
  const group = models[type].clone(true);
  group.userData.joints = definitions[type].map((j) => {
    const node = group.getObjectByName(j.node);
    if (!node) throw new Error(`model node missing: ${j.node}`);
    return {
      ...j,
      node,
      zero: node.quaternion.clone(),
      position: node.position.clone(),
      axis: new THREE.Vector3(...j.axis),
    };
  });
  return group;
}
function poseModel(group, row) {
  for (const j of group.userData.joints) {
    let angle = row.joints[j.name];
    if (j.mimic) {
      const driver = row.joints[j.mimic.joint];
      angle =
        driver === undefined
          ? undefined
          : driver * j.mimic.multiplier + j.mimic.offset;
    }
    j.node.quaternion.copy(j.zero);
    j.node.position.copy(j.position);
    if (angle === undefined) continue;
    if (j.type === "prismatic")
      j.node.position.add(
        j.axis.clone().applyQuaternion(j.zero).multiplyScalar(angle),
      );
    else
      j.node.quaternion.multiply(
        new THREE.Quaternion().setFromAxisAngle(j.axis, angle),
      );
  }
}
function select(id) {
  selected = id;
  ids.forEach((key) => {
    buttons[key].setAttribute("aria-pressed", String(key === id));
    labels[key].classList.toggle("selected", key === id);
    previews[key].visible = key === id;
  });
  views.detail.dirty = true;
  const model = previews[id];
  const bounds = new THREE.Box3().setFromObject(model),
    center = bounds.getCenter(new THREE.Vector3());
  const extent = Math.max(
    ...bounds.getSize(new THREE.Vector3()).toArray(),
    0.15,
  );
  views.detail.controls.target.copy(center);
  views.detail.camera.position
    .copy(center)
    .add(new THREE.Vector3(1, -1, 0.7).multiplyScalar(extent * 1.8));
  views.detail.controls.update();
  updateDetails();
}
function updateDetails() {
  const row = rows.find((r) => r.robot_id === selected);
  $("name").textContent = title(selected);
  $("state").textContent = row
    ? `${row.mode}${row.stale ? " · 갱신 지연 / 미연결" : ""}`
    : "상태 미수신";
  const values = [
    [
      "좌표 / 방향",
      row?.pose ? row.pose.map((n) => n.toFixed(2)).join(" / ") : "—",
    ],
    ["속도 (m/s)", row?.velocity?.toFixed(2) ?? "—"],
  ];
  if (installedPoses[selected]) values.push(["설치 배치 (기준값)", "받침 앞 변 · 상호 대향 · 실기 미검증"]);
  if (row?.pose_source) values.push(["위치 근거", row.pose_source]);
  for (const [name, angle] of Object.entries(row?.joints ?? {}))
    values.push([name, `${((angle * 180) / Math.PI).toFixed(1)}°`]);
  if (!Object.keys(row?.joints ?? {}).length)
    values.push(["관절값", "미수신 · 기준 자세"]);
  $("telemetry").replaceChildren(
    ...values.flatMap(([k, v]) => [text("dt", k), text("dd", v)]),
  );
  $("freshness").textContent =
    row?.age == null
      ? "수신 이력 없음"
      : `${row.age.toFixed(1)}초 전${row.stale ? " · 마지막 수신 상태" : ""}`;
  $("freshness").style.color = row?.stale ? "var(--warn)" : "var(--muted)";
  $("focus").disabled = !row || !displayedPose(row);
  $("drop").textContent = dropped.has(selected)
    ? "선택 장비 수신 복원"
    : "선택 장비 수신 중단";
}
function receive(raw) {
  try {
    const data = JSON.parse(raw);
    if (trial) {
      renderTrial(data);
      if (data.trial?.available) makeTrialRoutes({trial_routes: data.trial.routes, trial_nodes: trialNodes});
    }
    rows = data.robots;
    for (const row of rows) {
      const id = row.robot_id,
        model = instances[id];
      if (!model) continue;
      buttons[id].dataset.stale = row.stale;
      buttons[id].querySelector("small").textContent = trial ? row.mode : row.stale
        ? "미수신 / 지연"
        : row.mode;
      const pose = displayedPose(row);
      const poseKey = JSON.stringify([pose, row.joints]);
      if (model.userData.poseKey !== poseKey) {
        views.map.dirty = true;
        if (id === selected) views.detail.dirty = true;
        model.userData.poseKey = poseKey;
      }
      model.visible = !!pose;
      if (pose) {
        model.position.set(...pose.slice(0, 3));
        model.rotation.z = pose[3];
      }
      poseModel(model, row);
      poseModel(previews[id], row);
      labels[id].classList.toggle("stale", row.stale);
      labels[id].textContent = title(id) + (installedPoses[id] && (demo || !row.pose) ? " · 설치 배치" : "");
    }
    $("source").textContent = data.source;
    $("task-source").textContent = data.task_source;
    if (data.tasks)
      $("tasks").replaceChildren(
        ...(data.tasks.length
          ? data.tasks.map((task) => {
              const item = text("div", "", "task");
              const progress = document.createElement("progress");
              progress.max = 100;
              progress.value = task.progress;
              progress.setAttribute("aria-label", `${task.task_id} 진행률`);
              const stage = text("div", task.stage);
              stage.append(progress);
              item.append(
                text("span", task.task_id),
                text("small", task.robot),
                stage,
                text("small", task.status),
              );
              return item;
            })
          : [text("p", "수신된 작업이 없습니다.", "note")]),
      );
    $("errors").textContent =
      data.errors?.join("\n") ||
      "수신된 오류 정보 없음 · 장비 정상 판정과 별개";
    if (demo) {
      playing = data.playing;
      dropped = new Set(data.dropped);
      $("play").textContent = playing ? "일시 정지" : "동작 재생";
      $("play").setAttribute("aria-pressed", String(playing));
      $("clock").textContent = `${data.demo_time.toFixed(1)} s`;
      if (document.activeElement !== $("time"))
        $("time").value = data.demo_time;
    }
    updateDetails();
    $("message").textContent = data.message;
  } catch (error) {
    $("message").textContent = `표시 오류: ${error.message}`;
  }
}
async function start(config) {
  demo = config.demo;
  trial = config.trial === true;
  inventory = config.robots;
  ids = inventory.map((r) => r.id);
  selected = ids[0];
  $("robot-count").textContent = ids.length;
  $("fleet-count").textContent = ids.length;
  $("mode-link").textContent = demo ? "OFFLINE 보기" : "DEMO 보기";
  $("mode-link").href = demo ? "?mode=offline" : "?mode=demo";
  $("mode").textContent = demo
    ? "DEMO · 모의 상태"
    : "OFFLINE · 표시 입력 미연결";
  $("demo-controls").hidden = !demo;
  $("trial-panel").hidden = !trial;
  $("pinky-trial-details").hidden = !trial;
  $("pinky-detail").hidden = trial;
  $("offline-actions").hidden = trial;
  if (trial) {
    $("mode").textContent = "TRIAL · MQTT 시나리오";
    $("inventory-label").textContent = "고정 자리";
    $("scope-note").textContent = "시험 제어 · 출발/더미 승인 수동";
  }
  views = { map: view($("map")), detail: view($("model")) };
  makeMap(config.map);
  $("map-name").textContent = config.map.id + (config.map.layout ? ` · 실측 ${config.map.layout.version}` : config.example_map ? " · 예시 지도" : "");
  $("map-detail").textContent = config.map.layout
    ? `map · 벽 ${(config.map.wall_height * 100).toFixed(0)}cm 기준 · ${config.map.layout.graph_version}`
    : "map · m / rad · 벽 높이는 표시용";
  if (config.map.layout) document.querySelector(".map-note").textContent = "실측 지도 · 점유셀 돌출 표시 · 설치 배치는 기준값";
  if (demo && !config.example_map) document.querySelector(".map-note").textContent += " · 움직임은 예시 경로";
  if (trial) {
    trialNodes = config.trial_nodes;
    makeTrialRoutes(config);
  }
  const loader = new GLTFLoader();
  await Promise.all(
    ["pinky", "omx"].map(async (type) => {
      const [gltf, joints] = await Promise.all([
        loader.loadAsync(`./models/${type}/robot.glb`),
        fetch(`./models/${type}/joints.json`).then((r) => r.json()),
      ]);
      models[type] = gltf.scene;
      definitions[type] = joints;
    }),
  );
  for (const mount of config.map.layout?.mounts ?? []) {
    if (ids.includes(mount.robot_id)) installedPoses[mount.robot_id] = installationPose(mount);
  }
  for (const id of ids) {
    instances[id] = robot(kind(id));
    instances[id].visible = false;
    views.map.scene.add(instances[id]);
    previews[id] = robot(kind(id));
    views.detail.scene.add(previews[id]);
    const button = text("button", "", "robot");
    button.setAttribute("aria-label", title(id) + " 선택");
    const img = document.createElement("img");
    img.src = kind(id) === "pinky" ? "./pinky-pro.webp" : "./omx-f.webp";
    img.alt = "";
    const copy = document.createElement("span");
    copy.append(text("b", title(id)), text("small", "미수신"));
    button.append(img, copy);
    button.onclick = () => select(id);
    buttons[id] = button;
    $("robots").append(button);
    labels[id] = text("span", title(id), "label");
    labels[id].dataset.robot = id;
    $("labels").append(labels[id]);
  }
  select(selected);
  document.querySelectorAll("[data-view]").forEach(
    (b) =>
      (b.onclick = () => {
        document
          .querySelectorAll("[data-view]")
          .forEach((x) => x.setAttribute("aria-pressed", String(x === b)));
        home(b.dataset.view === "top");
      }),
  );
  const paths = {
    left: "M3 10a9 9 0 1 1 2 9 M3 4v6h6",
    home: "M3 9V3h6 M15 3h6v6 M21 15v6h-6 M9 21H3v-6",
    right: "M21 10A9 9 0 1 0 19 19 M21 4v6h-6",
  };
  for (const [id, path] of Object.entries(paths))
    $(id).innerHTML =
      `<svg viewBox="0 0 24 24" aria-hidden="true"><path d="${path}"/></svg>`;
  $("home").onclick = () => home();
  ["left", "right"].forEach(
    (id) =>
      ($(id).onclick = () => {
        views.map.camera.position
          .sub(views.map.controls.target)
          .applyAxisAngle(
            new THREE.Vector3(0, 0, 1),
            id === "left" ? 0.3 : -0.3,
          )
          .add(views.map.controls.target);
        views.map.controls.update();
      }),
  );
  $("focus").onclick = () => {
    const row = rows.find((r) => r.robot_id === selected);
    const pose = row && displayedPose(row);
    if (!pose) return;
    views.map.controls.target.set(...pose.slice(0, 3));
    views.map.camera.position
      .copy(views.map.controls.target)
      .add(new THREE.Vector3(-0.8, -1, 1));
    views.map.controls.update();
  };
  $("play").onclick = () => sendControl({action: "play", value: !playing});
  $("time").oninput = () => sendControl({action: "seek", value: Number($("time").value)});
  $("drop").onclick = () => sendControl({action: "drop", robot_id: selected, value: !dropped.has(selected)});

  document.body.dataset.ready = "true";
  let last = 0,
    begin = performance.now(),
    frames = 0;
  function draw(now) {
    requestAnimationFrame(draw);
    if (document.hidden || now - last < 32) return;
    last = now;
    let rendered = false;
    for (const v of Object.values(views)) {
      v.controls.update();
      if (v.dirty) {
        v.renderer.render(v.scene, v.camera);
        v.dirty = false;
        rendered = true;
      }
    }
    for (const id of ids) {
      const model = instances[id],
        label = labels[id];
      label.hidden = !model.visible;
      if (!model.visible) continue;
      vector
        .copy(model.position)
        .add(new THREE.Vector3(0, 0, kind(id) === "pinky" ? 0.2 : 0.4))
        .project(views.map.camera);
      label.hidden = vector.z > 1 || vector.z < -1;
      label.style.left = `${(vector.x * 0.5 + 0.5) * $("map").clientWidth}px`;
      label.style.top = `${(-vector.y * 0.5 + 0.5) * $("map").clientHeight}px`;
    }
    for (const {position, label} of mapLabels) {
      vector.copy(position).project(views.map.camera);
      label.hidden = vector.z < -1 || vector.z > 1 || Math.abs(vector.x) > 1 || Math.abs(vector.y) > 1;
      label.style.left = `${(vector.x * 0.5 + 0.5) * $("map").clientWidth}px`;
      label.style.top = `${(-vector.y * 0.5 + 0.5) * $("map").clientHeight}px`;
    }
    if (rendered) frames++;
    if (now - begin >= 1000) {
      $("fps").textContent =
        `${((frames * 1000) / (now - begin)).toFixed(0)} render fps`;
      $("fps").dataset.value = (frames * 1000) / (now - begin);
      begin = now;
      frames = 0;
    }
  }
  requestAnimationFrame(draw);
}
let socket, retry, lastReceived = 0, revision = -1, session;
const requestedMode = new URLSearchParams(location.search).get("mode");
let mode = ["offline", "demo", "trial"].includes(requestedMode) ? requestedMode : "auto";
function sendControl(data) {
  if (demo && socket?.readyState === WebSocket.OPEN) socket.send(JSON.stringify(data));
}
function disconnected() {
  if (trial) trialDisconnected();
  $("source").textContent = "서버 연결 끊김 · 재연결 중";
  $("task-source").textContent = "연결 끊김 · 마지막 작업 상태";
  for (const row of rows) {
    row.stale = true;
    buttons[row.robot_id].dataset.stale = "true";
    buttons[row.robot_id].querySelector("small").textContent = "미수신 / 지연";
    labels[row.robot_id].classList.add("stale");
  }
  document.querySelectorAll("#demo-controls button, #demo-controls input").forEach((b) => b.disabled = true);
  updateDetails();
}
function connect() {
  clearTimeout(retry);
  const url = new URL("/api/state", location.href);
  url.protocol = location.protocol === "https:" ? "wss:" : "ws:";
  url.searchParams.set("mode", mode);
  socket = new WebSocket(url);
  session = undefined;
  revision = -1;
  lastReceived = performance.now();
  socket.onmessage = (event) => {
    try {
      const data = JSON.parse(event.data);
      if (!Number.isSafeInteger(data.revision) || typeof data.session_id !== "string" || !Array.isArray(data.robots)) throw new Error("invalid snapshot");
      if (session !== undefined && session !== data.session_id) throw new Error("unexpected session");
      if (data.revision <= revision) return;
      session = data.session_id;
      revision = data.revision;
      lastReceived = performance.now();
      receive(event.data);
      document.querySelectorAll("#demo-controls button, #demo-controls input").forEach((b) => b.disabled = false);
    } catch {
      socket.close();
    }
  };
  socket.onclose = () => { disconnected(); retry = setTimeout(connect, 1500); };
  socket.onerror = () => socket.close();
}
for (const button of document.querySelectorAll("[data-tab]")) {
  button.onclick = () => {
    if (trial) $(button.dataset.tab === "pinky" ? "trial-pinky-panel" : "trial-overview").append($("trial-panel"));
    for (const tab of document.querySelectorAll("[data-tab]")) {
      const active = tab === button;
      tab.setAttribute("aria-pressed", String(active));
      $(tab.dataset.tab).hidden = !active;
    }
  };
}
async function boot() {
  const response = await fetch(`/api/bootstrap?mode=${mode}`, {cache: "no-store"});
  if (!response.ok) throw new Error(`bootstrap: ${response.status}`);
  const bootstrap = await response.json();
  mode = bootstrap.mode;
  await start(bootstrap);
  connect();
  setInterval(() => {
    if (performance.now() - lastReceived > 2500) {
      disconnected();
      if (socket?.readyState === WebSocket.OPEN) socket.close();
    }
  }, 1000);
}
boot().catch((error) => {
  $("message").textContent = `화면 초기화 실패: ${error.message} · 새로고침해 주세요.`;
  document.body.dataset.ready = "false";
});
