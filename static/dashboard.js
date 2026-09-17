// Dashboard front-end: polls GET /telemetry and renders a simple top-down
// (x, y) canvas plot. Plain vanilla JS, no build step, no frameworks.
// Reminder: everything shown here is simulated telemetry, not a real drone.

const POLL_INTERVAL_MS = 1000;
const PLOT_RANGE_M = 60; // meters shown from center to edge of the canvas

const el = {
  state: document.getElementById("state"),
  battery: document.getElementById("battery"),
  altitude: document.getElementById("altitude"),
  position: document.getElementById("position"),
  velocity: document.getElementById("velocity"),
  gps: document.getElementById("gps"),
  waypointProgress: document.getElementById("waypoint-progress"),
  missionComplete: document.getElementById("mission-complete"),
  plot: document.getElementById("plot"),
  waypointsInput: document.getElementById("waypoints-input"),
  startBtn: document.getElementById("start-mission"),
  abortBtn: document.getElementById("abort-mission"),
  log: document.getElementById("log"),
};

const ctx = el.plot.getContext("2d");

function log(msg) {
  const line = `[${new Date().toLocaleTimeString()}] ${msg}\n`;
  el.log.textContent = line + el.log.textContent;
}

function worldToCanvas(x, y) {
  const w = el.plot.width;
  const h = el.plot.height;
  const scale = (w / 2) / PLOT_RANGE_M;
  // x -> east (right), y -> north (up, so flip for canvas)
  return {
    cx: w / 2 + x * scale,
    cy: h / 2 - y * scale,
  };
}

function drawPlot(telemetry) {
  const w = el.plot.width;
  const h = el.plot.height;
  ctx.clearRect(0, 0, w, h);

  // grid
  ctx.strokeStyle = "#232c40";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(w / 2, 0);
  ctx.lineTo(w / 2, h);
  ctx.moveTo(0, h / 2);
  ctx.lineTo(w, h / 2);
  ctx.stroke();

  // home marker
  const home = worldToCanvas(0, 0);
  ctx.fillStyle = "#4fd1c5";
  ctx.beginPath();
  ctx.arc(home.cx, home.cy, 6, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = "#8b95a8";
  ctx.fillText("home", home.cx + 8, home.cy - 8);

  // waypoints + path
  const waypoints = telemetry.mission.waypoints || [];
  ctx.strokeStyle = "#4a5570";
  ctx.beginPath();
  ctx.moveTo(home.cx, home.cy);
  waypoints.forEach((wp) => {
    const p = worldToCanvas(wp[0], wp[1]);
    ctx.lineTo(p.cx, p.cy);
  });
  ctx.stroke();

  waypoints.forEach((wp, i) => {
    const p = worldToCanvas(wp[0], wp[1]);
    const visited = i < telemetry.mission.current_waypoint_index;
    ctx.fillStyle = visited ? "#3a4a3a" : "#e5e9f0";
    ctx.beginPath();
    ctx.arc(p.cx, p.cy, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = "#8b95a8";
    ctx.fillText(`wp${i}`, p.cx + 6, p.cy + 4);
  });

  // drone position
  const dronePos = worldToCanvas(telemetry.position.x, telemetry.position.y);
  ctx.fillStyle = "#ef6b6b";
  ctx.beginPath();
  ctx.arc(dronePos.cx, dronePos.cy, 7, 0, Math.PI * 2);
  ctx.fill();
}

function renderTelemetry(t) {
  el.state.textContent = t.state;
  el.battery.textContent = `${t.battery_percent.toFixed(1)}%`;
  el.altitude.textContent = t.altitude_m.toFixed(1);
  el.position.textContent = `${t.position.x.toFixed(1)}, ${t.position.y.toFixed(1)}`;
  el.velocity.textContent = `${t.velocity.vx.toFixed(1)}, ${t.velocity.vy.toFixed(1)}, ${t.velocity.vz.toFixed(1)}`;
  el.gps.textContent = `${t.gps.lat.toFixed(6)}, ${t.gps.lon.toFixed(6)}`;
  el.waypointProgress.textContent = `${t.mission.current_waypoint_index} / ${t.mission.waypoints.length}`;
  el.missionComplete.textContent = t.mission.complete ? "yes" : "no";
  drawPlot(t);
}

async function pollTelemetry() {
  try {
    const res = await fetch("/telemetry");
    const data = await res.json();
    renderTelemetry(data);
  } catch (err) {
    log(`telemetry poll failed: ${err}`);
  }
}

function parseWaypoints(text) {
  return text
    .split("\n")
    .map((line) => line.trim())
    .filter((line) => line.length > 0)
    .map((line) => {
      const parts = line.split(",").map((v) => parseFloat(v.trim()));
      const wp = { x: parts[0], y: parts[1] };
      if (parts.length > 2 && !Number.isNaN(parts[2])) wp.z = parts[2];
      return wp;
    });
}

el.startBtn.addEventListener("click", async () => {
  const waypoints = parseWaypoints(el.waypointsInput.value);
  if (waypoints.length === 0) {
    log("enter at least one waypoint as x,y per line");
    return;
  }
  const res = await fetch("/mission", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ waypoints }),
  });
  const data = await res.json();
  if (res.ok) {
    log(`mission set with ${waypoints.length} waypoint(s)`);
    renderTelemetry(data);
  } else {
    log(`mission rejected: ${data.error}`);
  }
});

el.abortBtn.addEventListener("click", async () => {
  const res = await fetch("/abort", { method: "POST" });
  const data = await res.json();
  log("abort sent: forcing return-home");
  renderTelemetry(data);
});

pollTelemetry();
setInterval(pollTelemetry, POLL_INTERVAL_MS);
