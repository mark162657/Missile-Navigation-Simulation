// Thin client over the backend JSON/WebSocket API.

async function getJSON(url) {
  const r = await fetch(url);
  if (!r.ok) {
    let msg = `${r.status} ${r.statusText}`;
    try { const b = await r.json(); if (b.detail) msg = b.detail; } catch { /* ignore */ }
    throw new Error(msg);
  }
  return r.json();
}

async function postJSON(url, body, signal) {
  const r = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
    signal,
  });
  if (!r.ok) {
    let msg = `${r.status} ${r.statusText}`;
    try { const b = await r.json(); if (b.detail) msg = b.detail; } catch { /* ignore */ }
    throw new Error(msg);
  }
  return r.json();
}

export const api = {
  dems: () => getJSON("/api/dems"),
  demGrid: (name, maxSize = 180) => getJSON(`/api/dems/${encodeURIComponent(name)}/grid?max_size=${maxSize}`),
  demElevation: (name, lat, lon) => getJSON(`/api/dems/${encodeURIComponent(name)}/elevation?lat=${lat}&lon=${lon}`),
  profiles: () => getJSON("/api/profiles"),
  saveProfile: (p) => postJSON("/api/profiles", p),
  missions: () => getJSON("/api/missions"),
  mission: (id) => getJSON(`/api/missions/${encodeURIComponent(id)}`),
  results: () => getJSON("/api/results"),
  plan: (req, signal) => postJSON("/api/plan", req, signal),
};

// Live simulation over WebSocket. Returns a controller with .stop().
export function connectLive(spec, handlers = {}) {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws/live`);
  let closed = false;

  ws.addEventListener("open", () => ws.send(JSON.stringify(spec)));
  ws.addEventListener("message", (ev) => {
    let msg;
    try { msg = JSON.parse(ev.data); } catch { return; }
    handlers.onMessage && handlers.onMessage(msg);
  });
  ws.addEventListener("error", () => handlers.onError && handlers.onError(new Error("WebSocket error")));
  ws.addEventListener("close", () => { if (!closed) { closed = true; handlers.onClose && handlers.onClose(); } });

  return {
    stop() {
      closed = true;
      try { ws.send("stop"); } catch { /* ignore */ }
      try { ws.close(); } catch { /* ignore */ }
    },
    get ready() { return ws.readyState === WebSocket.OPEN; },
  };
}
