/* Left column of the Telemetry page: serial log, manual control and servo telemetry. */
import { useEffect, useRef, useState } from "react";
import { Minus, Plus, Trash2 } from "lucide-react";
import { api } from "../api";
import { displayAxes, setPrefs, usePrefs } from "../state";
import type { AxisState, TelemetryState } from "../types";
import { Check, Dot, NumField, Slider, fmt, signed } from "../ui";

const GRIP_OFFSET_MAX = 15;

export function SerialLog({ s }: { s: TelemetryState }) {
  const [stat, setStat] = useState(true);
  const [frames, setFrames] = useState(false);
  const box = useRef<HTMLDivElement>(null);
  const logs = s.logs.filter((l) => (l.level === "stat" ? stat : l.level === "frame" ? frames : true));

  useEffect(() => {
    const b = box.current!;
    if (b.scrollHeight - b.scrollTop - b.clientHeight < 40) b.scrollTop = b.scrollHeight; // follow unless scrolled up
  }, [logs.length]);

  return (
    <>
      <div className="phead">
        <h2>Serial log</h2>
        <div className="end">
          <Check label="Show STAT" checked={stat} onChange={setStat} />
          <Check label="Show frames sent" checked={frames} onChange={setFrames} />
          <button className="btn icon sm" aria-label="Clear log" onClick={() => api.clearLogs()}><Trash2 size={16} /></button>
        </div>
      </div>
      <div className="log" ref={box}>
        {logs.map((l, i) => (
          <div key={i} className={l.level === "jump" ? "jump" : l.level === "error" ? "error" : ""}>
            <span className="t">{l.time}</span>{l.message}
          </div>
        ))}
      </div>
    </>
  );
}

const band = (a: AxisState): [number, number] => [a.home + a.sim_lo, a.home + a.sim_hi];

function JointRow({ a, s, jog }: { a: AxisState & { f: string }; s: TelemetryState; jog: number }) {
  const [drag, setDrag] = useState<number | null>(null);
  const v = drag ?? a.command_servo;
  const grip = a.name === "Gripper";
  return (
    <div className="jrow">
      <div className="top">
        <span className="name">{a.name}</span><i>{a.f}</i>
        <span className="val">{fmt(v)}°</span>
        <i>{grip ? `${fmt(s.gripper_percent, 0)}% open` : `joint ${signed(v - a.home)}°`}</i>
      </div>
      <div className="bot">
        <button className="btn" aria-label={`${a.name} minus ${jog}°`} onClick={() => api.jogAxis(a.name, -jog)}><Minus size={16} /></button>
        <Slider label={a.name} min={a.servo_lo} max={a.servo_hi} value={a.command_servo} band={band(a)} tick={a.home}
          onChange={setDrag} onCommit={(x) => { setDrag(null); api.setAxis(a.name, x); }} />
        <button className="btn" aria-label={`${a.name} plus ${jog}°`} onClick={() => api.jogAxis(a.name, jog)}><Plus size={16} /></button>
      </div>
    </div>
  );
}

export function ManualControl({ s }: { s: TelemetryState }) {
  const prefs = usePrefs();
  return (
    <>
      <div className="phead">
        <h2>Manual control</h2><i className="sub">per servo</i>
        <div className="end">
          <select className="select" value={prefs.jog} onChange={(e) => setPrefs({ jog: +e.target.value })} aria-label="Jog step">
            {[0.5, 1, 2, 5, 10].map((j) => <option key={j} value={j}>Jog {j}°</option>)}
          </select>
        </div>
      </div>
      <div className="scroll grow">
        {displayAxes(s.axes).map((a) => <JointRow key={a.name} a={a} s={s} jog={prefs.jog} />)}
        <div className="gripbar">
          <button className="btn" onClick={() => api.setGripperState("open")}>Open</button>
          <button className="btn" onClick={() => api.setGripperState("close")}>Close</button>
          <div className="grow">
            <NumField small pre="close offset" suf="°" value={s.grip_closed_offset} min={-GRIP_OFFSET_MAX} max={GRIP_OFFSET_MAX}
              onCommit={(v) => api.saveSettings({ grip_closed_offset: v })} />
          </div>
        </div>
        <p className="foot">Moves are ramped to stay under each axis speed limit, so all axes arrive together</p>
      </div>
    </>
  );
}

export function ServoTelemetry({ s }: { s: TelemetryState }) {
  return (
    <>
      <div className="phead">
        <h2>Servo telemetry</h2><i className="sub">sent ▾ board <span className="board">●</span></i>
      </div>
      <div className="scroll grow cards">
        {displayAxes(s.axes).map((a) => {
          const pct = (x: number) => `${((Math.min(a.servo_hi, Math.max(a.servo_lo, x)) - a.servo_lo) / (a.servo_hi - a.servo_lo)) * 100}%`;
          const [b0, b1] = band(a);
          return (
            <div className="card" key={a.name}>
              <div className="top">
                <span className="name">{a.name}</span>
                <span>{fmt(a.command_servo)}°</span>
                <span className="board">{fmt(a.board_servo)}°</span>
                <span>{a.name === "Gripper" ? `${fmt(s.gripper_percent, 0)}% open` : `${signed(a.sim_angle)}°`}</span>
              </div>
              <div className="bot">
                <div className="rangebar">
                  <div className="track" />
                  <div className="band" style={{ left: pct(b0), width: `calc(${pct(b1)} - ${pct(b0)})` }} />
                  <div className="sent" style={{ left: pct(a.command_servo) }} />
                  <div className="brd" style={{ left: pct(a.board_servo) }} />
                </div>
                <span className="state"><Dot tone={a.tone} />{a.status}</span>
              </div>
            </div>
          );
        })}
      </div>
    </>
  );
}
