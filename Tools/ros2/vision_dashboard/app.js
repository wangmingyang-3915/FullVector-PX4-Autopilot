"use strict";

const el = (id) => document.getElementById(id);
let csrfToken = null;
let latest = null;
let statusBusy = false;

const number = (value, digits = 2) =>
  typeof value === "number" && Number.isFinite(value) ? value.toFixed(digits) : "—";
const age = (seconds) =>
  typeof seconds === "number" ? `${number(seconds * 1000, 0)} ms` : "—";
const fresh = (stream, limit) => stream && typeof stream.age_s === "number" && stream.age_s < limit;
const text = (id, value) => { el(id).textContent = value; };

function stateChip(id, state, label) {
  const node = el(id);
  node.className = `status-chip ${state}`;
  node.textContent = label;
}

function stage(id, state, label) {
  const node = el(id);
  node.className = `stage ${state}`;
  node.querySelector("strong").textContent = label;
}

function row(parent, values, className = "") {
  const tr = document.createElement("tr");
  if (className) tr.className = className;
  for (const value of values) {
    const td = document.createElement("td");
    td.textContent = value;
    tr.appendChild(td);
  }
  parent.appendChild(tr);
}

function renderPipeline(s) {
  const streams = s.streams;
  const data = s.data;
  const service = s.system.ActiveState === "active";
  stage("stageService", service ? (s.system.enabled === "enabled" ? "good" : "warn") : "bad",
    service ? (s.system.enabled === "enabled" ? "运行中 · 自启动" : "运行中 · 未启用自启动") : "服务未运行");

  const imageOK = fresh(streams.image, 1.5);
  stage("stageCamera", imageOK ? "good" : "bad", imageOK ? `${number(streams.image.rate_hz, 1)} Hz` : "图像中断");

  const detectionOK = fresh(streams.detections, 0.5);
  const count = data.detections?.count ?? 0;
  stage("stageTag", detectionOK ? (count > 0 ? "good" : "warn") : "bad",
    detectionOK ? (count > 0 ? `检测到 ${count} 个` : "等待靶标") : "检测话题中断");

  const edges = data.tf?.edges ?? [];
  const tfReady = edges.length === 4 && edges.every((edge) => edge.fresh);
  const tfStatic = edges.length === 4 && edges[0].available && edges[2].available;
  stage("stageTf", tfReady ? "good" : (tfStatic ? "warn" : "bad"),
    tfReady ? "完整且新鲜" : (tfStatic ? "等待动态 TF" : "TF 不完整"));

  const poseOK = fresh(streams.pose, 0.5);
  const valid = data.pose?.position_valid && data.pose?.orientation_valid;
  stage("stagePose", poseOK ? (valid ? "good" : "warn") : "bad",
    poseOK ? (valid ? "位姿有效" : "位姿无效") : "位姿中断");

  const px4OK = s.system.agent_running && fresh(streams.px4, 1.5);
  stage("stagePx4", px4OK ? "good" : (s.system.agent_running ? "warn" : "bad"),
    px4OK ? "双向连接正常" : (s.system.agent_running ? "等待飞控" : "Agent 未运行"));
}

function renderDetections(s) {
  const stream = s.streams.detections;
  const detections = s.data.detections;
  text("detectionRate", `${number(stream.rate_hz, 1)} Hz`);
  text("detectionCount", detections?.count ?? "—");
  const list = el("detectionList");
  list.replaceChildren();
  if (!detections || !fresh(stream, 0.5)) {
    const p = document.createElement("p");
    p.className = "muted";
    p.textContent = "尚未收到新鲜检测数据。";
    list.appendChild(p);
  } else if (detections.count === 0) {
    const p = document.createElement("p");
    p.className = "muted";
    p.textContent = "检测节点正常出流，当前画面未看到标签。";
    list.appendChild(p);
  } else {
    for (const tag of detections.items) {
      const item = document.createElement("div");
      item.className = "tag-row";
      const label = document.createElement("strong");
      label.textContent = `ID ${tag.id}`;
      const detail = document.createElement("span");
      detail.textContent = `${tag.family} · margin ${number(tag.decision_margin, 1)} · hamming ${tag.hamming}`;
      item.append(label, detail);
      list.appendChild(item);
    }
  }
}

function renderPose(s) {
  const pose = s.data.pose;
  const stream = s.streams.pose;
  const isFresh = fresh(stream, 0.5);
  const valid = isFresh && pose?.position_valid && pose?.orientation_valid;
  stateChip("poseValidity", valid ? "good" : (isFresh ? "warn" : "bad"),
    valid ? "有效" : (isFresh ? "无效 / 无靶标" : "位姿中断"));
  text("poseTarget", pose?.target_id ?? "—");
  text("poseRate", `${number(stream.rate_hz, 1)} Hz`);
  const sampleAge = pose?.timestamp_sample_us ? (s.server_time_us - pose.timestamp_sample_us) / 1000 : null;
  text("poseAge", sampleAge === null ? "无有效样本" : `样本 ${number(sampleAge, 0)} ms`);
  const position = pose?.position ?? [];
  text("poseX", number(position[0], 3));
  text("poseY", number(position[1], 3));
  text("poseZ", number(position[2], 3));
  text("poseQuaternion", pose?.q_wxyz?.map((value) => number(value, 4)).join("  ") ?? "—");
}

function renderTf(s) {
  const tf = s.data.tf;
  text("tfTarget", `目标 ID ${tf?.target_id ?? "—"}`);
  const tbody = el("tfRows");
  tbody.replaceChildren();
  if (!tf?.edges?.length) {
    row(tbody, ["等待 TF 数据…", "", "", ""]);
    return;
  }
  for (const edge of tf.edges) {
    const status = edge.fresh ? "正常" : (edge.available ? "过期" : "缺失");
    const stamp = edge.age_ms === null ? "静态" : (edge.age_ms === undefined ? "—" : `${number(edge.age_ms, 0)} ms`);
    const translation = edge.translation?.map((value) => number(value, 3)).join(" / ") ?? "—";
    row(tbody, [`${edge.label}  (${edge.parent} → ${edge.child})`, status, stamp, translation], edge.fresh ? "good" : "warn");
  }
}

function renderPx4(s) {
  const px4 = s.data.px4;
  const stream = s.streams.px4;
  const px4Fresh = fresh(stream, 1.5) && px4?.timestamp_us && Math.abs(s.server_time_us - px4.timestamp_us) < 2_000_000;
  stateChip("px4Freshness", px4Fresh ? "good" : "warn", px4Fresh ? "回传正常" : "回传等待中");
  text("agentState", s.system.agent_running ? "运行中 · UDP 8888" : "未运行");
  text("px4Age", px4Fresh ? age(stream.age_s) : "未收到新鲜数据");
  text("px4Arming", px4Fresh ? px4.arming_name : "—");
  text("px4Mode", px4Fresh ? px4.mode_name : "—");
  text("px4Flags", px4Fresh ? `${px4.failsafe ? "失效保护开启" : "无失效保护"} / ${px4.gcs_connection_lost ? "地面站失联" : "地面站已连接"}` : "—");
  const hb = s.data.heartbeat;
  text("heartbeatState", fresh(s.streams.heartbeat, 0.5) ? `${number(s.streams.heartbeat.rate_hz, 1)} Hz · position=${hb?.position_ready}` : "未收到新鲜心跳");
  return px4Fresh;
}

function renderRestart(s, px4Fresh) {
  const button = el("restartButton");
  const job = s.restart ?? { state: "idle" };
  const permission = s.system.restart_permission === true;
  const arming = px4Fresh ? s.data.px4.arming_state : null;
  const running = job.state === "running";
  button.disabled = !permission || running || (arming !== null && arming !== 1);
  button.textContent = running ? "正在重启…" : (arming === null ? "离线重启视觉链路" : "重启视觉链路");
  if (!permission) text("restartHint", "请先双击 NoMachine 桌面的“启用视觉重启按钮（一次性）”并完成图形授权。页面不会请求或保存 sudo 密码。");
  else if (arming !== null && arming !== 1) text("restartHint", "飞控当前已解锁或状态未知：为避免中断飞行，禁止重启。");
  else if (arming === null) text("restartHint", "PX4 状态不可用。点击后需要先确认飞机已落地且未解锁，脚本仍会拒绝已知的解锁状态。");
  else text("restartHint", "飞控未解锁。点击按钮后在页面内执行重启并检查相机、位姿和 PX4 回传。");

  const states = {
    idle: ["neutral", "待命"], running: ["warn", "执行中"],
    success: ["good", "重启成功"], failed: ["bad", "重启失败"],
  };
  const [chip, label] = states[job.state] ?? states.idle;
  stateChip("restartState", chip, label);
  text("restartOutput", job.output || "尚未从此页面执行重启。");
}

function render(s) {
  latest = s;
  el("liveState").className = "live-tag good";
  text("liveState", "实时连接");
  text("updatedAt", `更新 ${new Date(s.server_time_us / 1000).toLocaleTimeString("zh-CN", { hour12: false })}`);
  text("serviceUptime", s.system.ExecMainStartTimestamp || "—");
  text("temperature", s.system.temperature_c === null || s.system.temperature_c === undefined ? "—" : `${number(s.system.temperature_c, 1)} °C`);
  renderPipeline(s);

  const image = s.data.image;
  text("imageRate", `${number(s.streams.image.rate_hz, 1)} Hz`);
  text("imageSize", image ? `${image.width} × ${image.height} · ${image.encoding}` : "—");
  text("imageAge", age(s.streams.image.age_s));
  const info = s.data.camera_info;
  text("cameraFocal", info ? `${number(info.fx, 2)} / ${number(info.fy, 2)}` : "—");
  if (!fresh(s.streams.image, 1.5)) text("previewPlaceholder", "图像已过期或相机未出流");
  renderDetections(s);
  renderPose(s);
  renderTf(s);
  const px4Fresh = renderPx4(s);
  renderRestart(s, px4Fresh);
  if (el("rawDetails").open) {
    text("rawData", JSON.stringify({ streams: s.streams, data: s.data, system: s.system }, null, 2));
  }
}

async function pollStatus() {
  if (statusBusy) return;
  statusBusy = true;
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    render(await response.json());
  } catch (error) {
    el("liveState").className = "live-tag bad";
    text("liveState", "监控连接中断");
    text("updatedAt", String(error));
    el("restartButton").disabled = true;
  } finally {
    statusBusy = false;
  }
}

async function pollFrame() {
  if (document.hidden) return;
  try {
    const response = await fetch(`/api/frame.jpg?t=${Date.now()}`, { cache: "no-store" });
    if (!response.ok) return;
    const bitmap = await createImageBitmap(await response.blob());
    const canvas = el("previewCanvas");
    canvas.width = bitmap.width;
    canvas.height = bitmap.height;
    const ctx = canvas.getContext("2d");
    ctx.drawImage(bitmap, 0, 0);
    bitmap.close();
    text("previewPlaceholder", fresh(latest?.streams?.image, 1.5) ? "" : "图像已过期或相机未出流");
    const image = latest?.data?.image;
    const detections = latest?.data?.detections;
    if (!image || !detections || !fresh(latest.streams.detections, 0.35)) return;
    const sx = canvas.width / image.width;
    const sy = canvas.height / image.height;
    ctx.lineWidth = 2;
    ctx.font = "bold 15px system-ui";
    for (const tag of detections.items) {
      if (tag.corners.length !== 4) continue;
      ctx.strokeStyle = "#61dfca";
      ctx.fillStyle = "#61dfca";
      ctx.beginPath();
      tag.corners.forEach(([x, y], index) => {
        if (index === 0) ctx.moveTo(x * sx, y * sy);
        else ctx.lineTo(x * sx, y * sy);
      });
      ctx.closePath();
      ctx.stroke();
      ctx.fillText(`ID ${tag.id}`, tag.corners[0][0] * sx, Math.max(18, tag.corners[0][1] * sy - 7));
    }
  } catch (_) {
    // The status cards report camera failures; keep the last usable frame here.
  }
}

async function pollLogs() {
  try {
    const response = await fetch("/api/logs", { cache: "no-store" });
    if (response.ok) text("serviceLogs", (await response.json()).lines || "暂无日志。");
  } catch (_) {
    text("serviceLogs", "无法读取日志；等待监控服务恢复。 ");
  }
}

async function restart() {
  if (!latest || !csrfToken) return;
  const px4 = latest.data.px4;
  const stream = latest.streams.px4;
  const px4Fresh = fresh(stream, 1.5) && px4?.timestamp_us && Math.abs(latest.server_time_us - px4.timestamp_us) < 2_000_000;
  const force = !px4Fresh;
  if (force && !window.confirm("PX4 状态不可用。请先确认飞机已落地且未解锁，然后再继续重启视觉链路。")) return;
  el("restartButton").disabled = true;
  try {
    const response = await fetch("/api/restart", {
      method: "POST",
      headers: { "Content-Type": "application/json", "X-CSRF-Token": csrfToken },
      body: JSON.stringify({ force }),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
    text("restartOutput", result.message);
    await pollStatus();
  } catch (error) {
    text("restartOutput", `重启请求失败：${error}`);
    await pollStatus();
  }
}

async function init() {
  el("restartButton").addEventListener("click", restart);
  try {
    const response = await fetch("/api/bootstrap", { cache: "no-store" });
    csrfToken = (await response.json()).csrf_token;
  } catch (_) {
    text("restartHint", "无法连接监控服务，重启按钮暂不可用。");
  }
  await pollStatus();
  pollFrame();
  pollLogs();
  setInterval(pollStatus, 2000);
  setInterval(pollFrame, 1500);
  setInterval(pollLogs, 10000);
  el("rawDetails").addEventListener("toggle", () => {
    if (el("rawDetails").open && latest) {
      text("rawData", JSON.stringify({ streams: latest.streams, data: latest.data, system: latest.system }, null, 2));
    }
  });
}

init();
