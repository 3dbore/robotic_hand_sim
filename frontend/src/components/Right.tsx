/* Right column: camera feeds and the Board link / Comms tabs. */
import { useEffect, useState, type ReactNode } from "react";
import { CircleX, House, Plug, RefreshCw, TriangleAlert } from "lucide-react";
import { api } from "../api";
import { goHome, setPrefs, stopAll, usePrefs } from "../state";
import type { TelemetryState } from "../types";
import { Dot, Tabs, fmt } from "../ui";
import { SimPov } from "./View3D";

export function CameraFeeds({ s }: { s: TelemetryState }) {
  const c = s.camera;
  const prefs = usePrefs();
  return (
    <>
      <div className="feed">
        {c.connected ? (
          <>
            <img src="/api/camera/real/mjpeg" alt="Real camera feed" />
            <span className="pill">Real Camera POV</span>
            <span className="live chip"><Dot tone="ok" />Live</span>
            <span className="meta">{c.res} · {fmt(c.fps)} fps · {c.tag_count} tag{c.tag_count === 1 ? "" : "s"}</span>
          </>
        ) : (
          <>
            <span className="pill">Real Camera POV</span>
            <span className="live chip"><Dot tone="" />Offline</span>
            <div className="off">
              <TriangleAlert size={18} />
              <b>No camera signal</b>
              <i>Set up the connection in Comms</i>
            </div>
          </>
        )}
      </div>
      <div className="feed">
        <SimPov />
        <span className="pill">Simulated Camera POV</span>
        <span className="live chip"><Dot tone="ok" />Live</span>
        <span className="meta">sim 512x320 · HFOV 60°</span>
        <label className="hud meta">
          Targeting HUD
          <input type="range" min={0} max={1} step={0.05} value={prefs.guideOpacity} aria-label="Targeting HUD opacity"
            onChange={(e) => setPrefs({ guideOpacity: +e.target.value })} />
        </label>
      </div>
    </>
  );
}

export function StopTabs<T extends string>(p: { tabs: T[]; value: T; onChange: (t: T) => void }) {
  return (
    <Tabs {...p}>
      <span className="keycap">Esc</span>
      <button className="btn danger" onClick={stopAll}><CircleX size={16} />Stop</button>
    </Tabs>
  );
}

/** Remembered so the app can reconnect after a USB drop. */
export const lastPort = { port: "", baud: 9600 };

function BoardLink({ s }: { s: TelemetryState }) {
  const [ports, setPorts] = useState<string[]>([]);
  const [port, setPort] = useState(s.port || "");
  const refresh = () => api.getPorts().then((p) => { setPorts(p); setPort((cur) => cur || p[0] || ""); });
  useEffect(() => { refresh(); }, []);
  const h = s.link_health;
  const connect = () => { lastPort.port = port; lastPort.baud = s.baud || 9600; api.connectSerial(port, lastPort.baud); };
  const disconnect = () => { lastPort.port = ""; api.disconnectSerial(); };
  const shown = s.connected ? s.port : port;
  const opts = ports.includes(shown) || !shown ? ports : [shown, ...ports];
  return (
    <div className="rpanel">
      <div className="row">
        <select className="select tall grow" value={shown} disabled={s.connected} onChange={(e) => setPort(e.target.value)} aria-label="Serial port">
          {opts.length === 0 && <option value="">No ports found</option>}
          {opts.map((p) => <option key={p} value={p}>{p} · {s.baud || 9600} baud</option>)}
        </select>
        <button className="btn icon" aria-label="Refresh ports" onClick={refresh}><RefreshCw size={16} /></button>
        {s.connected
          ? <button className="btn primary" onClick={disconnect}><Plug size={16} />Disconnect</button>
          : <button className="btn primary" disabled={!port} onClick={connect}><Plug size={16} />Connect</button>}
      </div>
      <div className="row">
        <span className="chip"><Dot tone={s.connected ? "ok" : "off"} />{s.connected ? "Linked" : "Not linked"}</span>
        <div className="end">
          <button className={`btn ${s.motors_on ? "on" : ""}`} onClick={() => api.toggleMotors(!s.motors_on)}>{s.motors_on ? "Motors on" : "Motors off"}</button>
          <button className="btn" onClick={() => goHome(s)}><House size={16} />Home</button>
        </div>
      </div>
      <div className="row">
        <span className="h3">Link health</span>
        <span className="chip"><Dot tone={!s.connected ? "" : h.is_healthy ? "ok" : "warn"} />{!s.connected ? "No link" : h.is_healthy ? "Healthy" : "Check link"}</span>
      </div>
      <div className="stats">
        <Stat label="Frames in">{fmt(h.frames_in_per_s, 0)}/s</Stat>
        <Stat label="Rejected" warn={h.rejected > 0}>{h.rejected}</Stat>
        <Stat label="I2C errors" warn={h.i2c_errors > 0}>{h.i2c_errors}</Stat>
        <Stat label="Board resets" warn={h.board_resets > 0}>{h.board_resets}</Stat>
        <Stat label="Jumps ignored" warn={h.jumps_ignored > 0}>{h.jumps_ignored}</Stat>
        <Stat label="Last report">{h.last_report_s < 0 ? "–" : `${fmt(h.last_report_s)} s ago`}</Stat>
        <Stat label="Last boot">{h.last_boot || "–"}</Stat>
      </div>
    </div>
  );
}

function Stat({ label, warn, children }: { label: string; warn?: boolean; children: ReactNode }) {
  return <div className="stat"><i>{label}</i><b className={warn ? "warn" : ""}>{children}</b></div>;
}

function Comms({ s }: { s: TelemetryState }) {
  const c = s.camera;
  const [host, setHost] = useState(c.host || "");
  const [tags, setTags] = useState(c.detect_tags ?? true);
  const busy = c.connected || c.status_tone === "info";
  return (
    <div className="rpanel">
      <span className="h3">Real camera</span>
      <label>
        <span className="fieldlabel">Camera address</span>
        <span className="field"><input className="left" value={host} placeholder="192.168.1.42" onChange={(e) => setHost(e.target.value)} disabled={busy} /></span>
      </label>
      <div className="row">
        <button className={`btn ${tags ? "on" : ""}`} aria-pressed={tags} disabled={busy} onClick={() => setTags(!tags)}>Tags</button>
        <div className="end">
          {busy
            ? <button className="btn primary" onClick={() => api.disconnectCamera()}>Disconnect</button>
            : <button className="btn primary" disabled={!host.trim()} onClick={() => api.connectCamera({ host: host.trim(), tags })}>Connect</button>}
        </div>
      </div>
      <span className="chip"><Dot tone={c.status_tone === "idle" ? "off" : c.status_tone} />{c.status || "Camera offline"}</span>
      {c.error_msg && <p className="errline">{c.error_msg}</p>}
      <p className="note">Frames stream as JPEG over SSH once the camera script is running. Tag detections are drawn on the feed.</p>
    </div>
  );
}

export function RightPanel({ s }: { s: TelemetryState }) {
  const [tab, setTab] = useState<"Board link" | "Comms">("Board link");
  return (
    <div className="col raised">
      <CameraFeeds s={s} />
      <StopTabs tabs={["Board link", "Comms"]} value={tab} onChange={setTab} />
      <div className="scroll">{tab === "Board link" ? <BoardLink s={s} /> : <Comms s={s} />}</div>
    </div>
  );
}
