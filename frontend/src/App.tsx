import { useEffect, useState } from "react";
import { ArrowLeft, ArrowRight, House } from "lucide-react";
import { api } from "./api";
import { Control } from "./components/Control";
import { lastPort, RightPanel } from "./components/Right";
import { ManualControl, SerialLog, ServoTelemetry } from "./components/Telemetry";
import { Viewport } from "./components/View3D";
import { SequencePage } from "./pages/SequencePage";
import { SettingsPage } from "./pages/SettingsPage";
import { prefsNow, stopAll, useLastError, useTelemetry } from "./state";
import type { TelemetryState } from "./types";
import { Dot, Tabs } from "./ui";

type Page = "Telemetry" | "Sequence" | "Settings";

export function BaseJog() {
  const err = useLastError();
  return (
    <>
      {err && <div className="err">{err}</div>}
      <div className="basejog">
        <button className="jogbtn" onClick={() => api.baseJog("left")}><ArrowLeft size={18} />Left</button>
        <button className="jogbtn home" onClick={() => api.baseJog("home")}><House size={18} />Home</button>
        <button className="jogbtn" onClick={() => api.baseJog("right")}><ArrowRight size={18} />Right</button>
      </div>
    </>
  );
}

function TelemetryPage({ s }: { s: TelemetryState }) {
  const [tab, setTab] = useState<"Manual control" | "Servo telemetry">("Manual control");
  return (
    <div className="body3">
      <div className="col left raised">
        <SerialLog s={s} />
        <Tabs tabs={["Manual control", "Servo telemetry"]} value={tab} onChange={setTab} />
        {tab === "Manual control" ? <ManualControl s={s} /> : <ServoTelemetry s={s} />}
      </div>
      <div className="col center">
        <Viewport><BaseJog /></Viewport>
        <Control s={s} />
      </div>
      <RightPanel s={s} />
    </div>
  );
}

export default function App() {
  const s = useTelemetry();
  const [page, setPage] = useState<Page>("Telemetry");

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => { if (e.key === "Escape") stopAll(); };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, []);

  // reconnect to the last port after a USB drop, like the toolkit
  const connected = s?.connected ?? false;
  useEffect(() => {
    if (connected) return;
    const t = setInterval(() => {
      if (prefsNow().autoReconnect && lastPort.port && !api.currentState?.connected) api.connectSerial(lastPort.port, lastPort.baud);
    }, 3000);
    return () => clearInterval(t);
  }, [connected]);
  useEffect(() => { if (s?.connected && s.port) { lastPort.port = s.port; lastPort.baud = s.baud; } }, [s?.connected, s?.port, s?.baud]);

  return (
    <div className="app">
      <nav className="nav">
        <span className="wordmark">3DBORE</span>
        {(["Telemetry", "Sequence", "Settings"] as Page[]).map((p) => (
          <button key={p} className={`navtab ${p === page ? "on" : ""}`} onClick={() => setPage(p)}>{p}</button>
        ))}
        <div className="chips">
          <span className="chip"><Dot tone={connected ? "ok" : "off"} />{connected ? "Board connected" : "Board disconnected"}</span>
          <span className="chip"><Dot tone={s?.motors_on ? "ok" : "off"} />{s?.motors_on ? "Motors on" : "Motors off"}</span>
        </div>
      </nav>
      {!s ? (
        <div className="body2"><p className="note" style={{ padding: 24 }}>Waiting for the backend at /ws (python -m backend.main)</p></div>
      ) : page === "Telemetry" ? <TelemetryPage s={s} /> : page === "Sequence" ? <SequencePage s={s} /> : <SettingsPage s={s} />}
    </div>
  );
}
