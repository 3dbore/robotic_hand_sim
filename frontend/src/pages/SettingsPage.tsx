/* Joint configuration and app preferences. Limits, home and speed come from hil/config.py and are
   read-only here (they must match the firmware); only trim and invert are saved to hil_settings.json. */
import { useEffect, useState } from "react";
import { RefreshCw, Save } from "lucide-react";
import { api } from "../api";
import { lastPort } from "../components/Right";
import { displayAxes, setPrefs, usePrefs } from "../state";
import type { TelemetryState } from "../types";
import { Check, NumField } from "../ui";

const TRIM_MAX = 15;
const SPIKE_GATE = 20; // firmware constant, shown for reference
type Form = Record<string, { trim: number; inverted: boolean }>;

const fromAxes = (s: TelemetryState): Form => Object.fromEntries(s.axes.map((a) => [a.name, { trim: a.trim, inverted: a.inverted }]));

export function SettingsPage({ s }: { s: TelemetryState }) {
  const prefs = usePrefs();
  const [form, setForm] = useState<Form>(() => fromAxes(s));
  const [ports, setPorts] = useState<string[]>([]);
  const [port, setPort] = useState(s.port || lastPort.port);
  const [msg, setMsg] = useState("");
  useEffect(() => { api.getPorts().then(setPorts); }, []);

  const edit = (name: string, patch: Partial<Form[string]>) => { setForm({ ...form, [name]: { ...form[name], ...patch } }); setMsg(""); };
  const reset = () => { setForm(Object.fromEntries(s.axes.map((a) => [a.name, { trim: 0, inverted: false }]))); setMsg("Defaults loaded, not saved yet"); };
  const save = async () => {
    const ok = await api.saveSettings({
      inverted: Object.fromEntries(Object.entries(form).map(([k, v]) => [k, v.inverted])),
      trim: Object.fromEntries(Object.entries(form).map(([k, v]) => [k, v.trim])),
      ...(port ? { port } : {}),
    });
    if (port) lastPort.port = port;
    setMsg(ok ? "Saved" : "Save failed, the backend did not answer");
  };
  const opts = ports.includes(port) || !port ? ports : [port, ...ports];

  return (
    <div className="body2">
      <div className="col raised">
        <div className="phead">
          <h2>Joint configuration</h2><i className="sub">limits, home and trim per axis</i>
          <div className="end">
            {msg && <i className="sub muted">{msg}</i>}
            <button className="btn" onClick={reset}><RefreshCw size={16} />Reset defaults</button>
            <button className="btn primary" onClick={save}><Save size={16} />Save</button>
          </div>
        </div>
        <div className="scroll">
          <table className="cfg">
            <thead><tr><th>Axis</th><th>Ch</th><th>Min</th><th>Max</th><th>Home</th><th>Speed limit</th><th>Trim</th><th>Invert</th></tr></thead>
            <tbody>
              {displayAxes(s.axes).map((a) => (
                <tr key={a.name}>
                  <td className="name">{a.name}</td>
                  <td><i>{a.f}</i></td>
                  <td><NumField pre="min" suf="°" value={a.servo_lo} step={1} readOnly /></td>
                  <td><NumField pre="max" suf="°" value={a.servo_hi} step={1} readOnly /></td>
                  <td><NumField pre="home" suf="°" value={a.home} step={1} readOnly /></td>
                  <td><NumField pre="speed" suf="°/s" value={a.speed} step={1} readOnly /></td>
                  <td><NumField pre="trim" suf="°" value={form[a.name]?.trim ?? 0} min={-TRIM_MAX} max={TRIM_MAX} onCommit={(v) => edit(a.name, { trim: v })} /></td>
                  <td><input type="checkbox" className="switch" aria-label={`Invert ${a.name}`} checked={form[a.name]?.inverted ?? false}
                    onChange={(e) => edit(a.name, { inverted: e.target.checked })} /></td>
                </tr>
              ))}
            </tbody>
          </table>
          <p className="note" style={{ padding: "16px 16px" }}>
            Limits, home and speed match the firmware and are read-only here. Trim (±{TRIM_MAX}°) and invert change only the value sent on the wire, never the limits.
          </p>
        </div>
      </div>

      <div className="col">
        <div className="sect">
          <span className="h3">Serial link</span>
          <label>
            <span className="fieldlabel">Port</span>
            <select className="select tall" style={{ width: "100%" }} value={port} disabled={s.connected} onChange={(e) => setPort(e.target.value)}>
              {opts.length === 0 && <option value="">No ports found</option>}
              {opts.map((p) => <option key={p} value={p}>{p}</option>)}
            </select>
          </label>
          <label>
            <span className="fieldlabel">Baud rate</span>
            <select className="select tall" style={{ width: "100%" }} value={9600} disabled>
              <option value={9600}>9600</option>
            </select>
          </label>
          <Check label="Reconnect automatically" checked={prefs.autoReconnect} onChange={(v) => setPrefs({ autoReconnect: v })} />
        </div>
        <div className="sect">
          <span className="h3">Simulation</span>
          <Check label="Show workspace envelope" checked={prefs.showWorkspace} onChange={(v) => setPrefs({ showWorkspace: v })} />
          <Check label="Show target marker" checked={prefs.showTarget} onChange={(v) => setPrefs({ showTarget: v })} />
          <Check label="Mirror board pose in 3D view" checked={prefs.mirrorBoard} onChange={(v) => setPrefs({ mirrorBoard: v })} />
        </div>
        <div className="sect">
          <span className="h3">Safety</span>
          <NumField pre="Max speed (all axes)" suf="°/s" value={Math.max(...s.axes.map((a) => a.speed))} step={1} readOnly />
          <Check label="Release motors on Esc" checked={prefs.releaseOnEsc} onChange={(v) => setPrefs({ releaseOnEsc: v })} />
          <Check label={`Ignore jumps above ${SPIKE_GATE}°`} checked disabled />
        </div>
      </div>
    </div>
  );
}
