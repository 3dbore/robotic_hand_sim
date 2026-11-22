/* Saved pose sequences (hil/sequence.py + hil/timeline.py). Every edit saves the whole list. */
import { useEffect, useState, type PointerEvent } from "react";
import { ArrowRight, Camera, Copy, Play, Plus, Scan, SkipForward, Square, Trash2 } from "lucide-react";
import { api } from "../api";
import { CameraFeeds, StopTabs } from "../components/Right";
import { Viewport } from "../components/View3D";
import type { SequenceStep, TelemetryState } from "../types";
import { setPrefs, usePrefs } from "../state";
import { Check, NumField, fmt } from "../ui";

interface Seq { name: string; steps: SequenceStep[] }
const SPEED_MARGIN = 0.8; // hil/config.py: moves are timed to this share of each axis's firmware speed

/** Seconds the planner takes from one pose to the next: the slowest axis sets the pace. */
function moveTime(s: TelemetryState, from: Record<string, number> | null, to: Record<string, number>) {
  return Math.max(0.1, ...s.axes.map((a) => Math.abs((to[a.name] ?? a.home) - (from?.[a.name] ?? a.home)) / (a.speed * SPEED_MARGIN)));
}

export function SequencePage({ s }: { s: TelemetryState }) {
  const [seqs, setSeqs] = useState<Seq[]>([]);
  const [sel, setSel] = useState(0);
  const [stepSel, setStepSel] = useState(0);
  const [renaming, setRenaming] = useState<number | null>(null);
  const [dragFrom, setDragFrom] = useState<number | null>(null);
  const [dropAt, setDropAt] = useState<number | null>(null);
  const [holdDrag, setHoldDrag] = useState<{ i: number; x: number; h0: number; h: number } | null>(null);
  const [tab, setTab] = useState<"Step editor" | "Tags">("Step editor");
  const [msg, setMsg] = useState({ text: "", error: false });
  const prefs = usePrefs();

  useEffect(() => { api.getSequences().then((l) => setSeqs(l.map((q) => ({ name: q.name, steps: q.steps })))); }, []);
  // Playback and go-to messages come from the backend; show the newest of theirs and ours.
  useEffect(() => { if (s.sequence.message) setMsg({ text: s.sequence.message, error: s.sequence.error }); }, [s.sequence.message, s.sequence.error]);

  const save = (next: Seq[]) => { setSeqs(next); api.saveSequences(next); };
  const seq = seqs[sel] as Seq | undefined;
  const step = seq?.steps[stepSel];
  const editSeq = (f: (q: Seq) => Seq) => save(seqs.map((q, i) => (i === sel ? f(q) : q)));
  const editStep = (patch: Partial<SequenceStep>) => editSeq((q) => ({ ...q, steps: q.steps.map((st, i) => (i === stepSel ? { ...st, ...patch } : st)) }));
  const times = seq?.steps.map((st, i) => moveTime(s, i ? seq.steps[i - 1].servo : null, st.servo)) ?? [];
  const total = (q: Seq) => q.steps.reduce((t, st, i) => t + st.hold + moveTime(s, i ? q.steps[i - 1].servo : null, st.servo), 0);
  const running = s.sequence.running && s.sequence.name === seq?.name;
  const stepping = running && !s.sequence.auto;

  const uniq = (base: string) => { let n = base, k = 2; while (seqs.some((q) => q.name === n)) n = `${base} ${k++}`; return n; };
  const addSeq = () => { save([...seqs, { name: uniq("New sequence"), steps: [] }]); setSel(seqs.length); setStepSel(0); setRenaming(seqs.length); };
  const dupSeq = () => seq && (save([...seqs, { name: uniq(`${seq.name} copy`), steps: structuredClone(seq.steps) }]), setSel(seqs.length));
  const delSeq = () => {
    if (!seq || !window.confirm(`Delete "${seq.name}" and its ${seq.steps.length} steps?`)) return;
    save(seqs.filter((_, i) => i !== sel)); setSel(Math.max(0, sel - 1)); setStepSel(0);
  };
  /** Store a pose in a step (new at `at`, or replacing step `at`), with a tag if Tag on save is on. */
  const savePose = async (verb: "Added" | "Updated", at: number) => {
    const pose = await api.capturePose();
    if (!pose || !seq) return;
    const old = verb === "Updated" ? seq.steps[at] : undefined;
    const names = seq.steps.map((st) => st.name);
    let name = old?.name ?? `Pose ${seq.steps.length + 1}`;
    for (let k = 2; !old && names.includes(name); k++) name = `Pose ${seq.steps.length + 1} ${k}`;
    const st: SequenceStep = { ...(old ?? { hold: 0.5 }), name, servo: pose };
    let text = `${verb} “${name}”.`, error = false;
    if (prefs.tagOnSave) {
      delete st.tag; // the old tag belongs to the old pose
      const r = await api.captureTag(pose);
      if (r.tag) { st.tag = r.tag; text = `${verb} “${name}” with tag ${r.tag.tags.map((t: { id: number }) => t.id).join(", ")}.`; }
      else text = `${verb} “${name}” without a tag: ${r.error}.${old?.tag ? " Its old tag was removed." : ""}`;
    }
    const steps = old ? seq.steps.map((x, i) => (i === at ? st : x)) : [...seq.steps.slice(0, at), st, ...seq.steps.slice(at)];
    editSeq((q) => ({ ...q, steps }));
    setStepSel(at);
    setMsg({ text, error });
  };
  const addStep = () => seq && savePose("Added", seq.steps.length ? stepSel + 1 : 0);
  const delStep = () => { editSeq((q) => ({ ...q, steps: q.steps.filter((_, i) => i !== stepSel) })); setStepSel(Math.max(0, stepSel - 1)); };
  const captureTag = async () => {
    if (!step) return;
    const r = await api.captureTag(step.servo);
    if (!r.tag) return setMsg({ text: `No tag captured for “${step.name}”: ${r.error}.`, error: true });
    editStep({ tag: r.tag });
    setMsg({ text: `Tag stored in “${step.name}”.`, error: false });
  };
  const clearTag = () => { editStep({ tag: undefined }); setMsg({ text: `Tag removed from “${step?.name}”.`, error: false }); };
  const drop = (to: number) => {
    if (dragFrom === null || dragFrom === to || !seq) return;
    const steps = [...seq.steps];
    const [m] = steps.splice(dragFrom, 1);
    steps.splice(to, 0, m);
    editSeq((q) => ({ ...q, steps }));
    setStepSel(to);
  };
  const holdMove = (e: PointerEvent) => holdDrag && setHoldDrag({ ...holdDrag, h: Math.max(0, Math.round((holdDrag.h0 + (e.clientX - holdDrag.x) / 40) * 10) / 10) });
  const holdUp = () => {
    if (!holdDrag) return;
    const { i, h } = holdDrag;
    setHoldDrag(null);
    editSeq((q) => ({ ...q, steps: q.steps.map((st, k) => (k === i ? { ...st, hold: h } : st)) }));
  };

  return (
    <div className="body3">
      <div className="col left raised">
        <div className="phead">
          <h2>Sequences</h2><i className="sub">{seqs.length} saved</i>
          <div className="end"><button className="btn" onClick={addSeq}><Plus size={16} />New</button></div>
        </div>
        <div className="seqlist scroll grow">
          {seqs.map((q, i) => (
            <div key={i} role="button" tabIndex={0} className={`seqitem ${i === sel ? "on" : ""}`}
              onClick={() => { setSel(i); setStepSel(0); }} onDoubleClick={() => setRenaming(i)}>
              <div className="grow">
                {renaming === i ? (
                  <input autoFocus defaultValue={q.name} aria-label="Sequence name"
                    onBlur={(e) => { const n = e.target.value.trim(); setRenaming(null); if (n && n !== q.name) save(seqs.map((x, k) => (k === i ? { ...x, name: uniq(n) } : x))); }}
                    onKeyDown={(e) => { if (e.key === "Enter") (e.target as HTMLInputElement).blur(); if (e.key === "Escape") { e.stopPropagation(); setRenaming(null); } }} />
                ) : <b>{q.name}</b>}
                <i>{q.steps.length} steps · {fmt(total(q))} s</i>
              </div>
              <ArrowRight size={18} />
            </div>
          ))}
          {seqs.length === 0 && <p className="note">No sequences yet. New starts an empty one; Add step records the current pose.</p>}
        </div>
        <div className="seqfoot">
          <button className="btn" disabled={!seq} onClick={dupSeq}><Copy size={16} />Duplicate</button>
          <button className="btn" disabled={!seq} onClick={delSeq}><Trash2 size={16} />Delete</button>
        </div>
      </div>

      <div className="col center">
        <Viewport />
        <div className="steps">
          <div className="phead">
            <h2>Steps</h2><i className="sub">{seq ? `${seq.name} · ${seq.steps.length} steps` : "No sequence"}</i>
            <div className="end">
              <button className="btn" disabled={!seq} onClick={addStep}><Plus size={16} />Add step</button>
              <button className="btn" disabled={!seq?.steps.length || (running && !stepping)} onClick={() => seq && api.send("step_sequence", { name: seq.name })}
                title="Run one step at a time"><SkipForward size={16} />Next step</button>
              {running
                ? <button className="btn primary" onClick={() => api.stopSequence()}><Square size={14} />Stop</button>
                : <button className="btn primary" disabled={!seq?.steps.length} onClick={() => seq && api.runSequence(seq.name)}><Play size={14} />Run</button>}
            </div>
          </div>
          {msg.text && <p className={`note ${msg.error ? "bad" : ""}`} role="status">{msg.text}</p>}
          <div className="stepgrid" onPointerMove={holdMove} onPointerUp={holdUp}>
            {seq?.steps.map((st, i) => {
              const hold = holdDrag?.i === i ? holdDrag.h : st.hold;
              return (
                <div key={i} className={`step ${i === stepSel ? "on" : ""} ${running && s.sequence.step_index === i ? "run" : ""} ${dropAt === i ? "drop" : ""}`}
                  draggable={!holdDrag} onDragStart={() => setDragFrom(i)} onDragEnd={() => { setDragFrom(null); setDropAt(null); }}
                  onDragOver={(e) => { e.preventDefault(); setDropAt(i); }} onDragLeave={() => setDropAt(null)} onDrop={() => { drop(i); setDropAt(null); }}
                  onClick={() => setStepSel(i)} onDoubleClick={() => api.send("go_step", { name: st.name, servo: st.servo })}
                  title="Drag to reorder, double-click to go there">
                  <div className="blk">
                    <b>{i + 1} · {st.name}</b>
                    <i>move {fmt(times[i])} s</i>
                    {st.tag && <Scan size={16} aria-label="Has a tag" />}
                  </div>
                  <div className="hold" style={{ width: 64 + Math.min(hold, 5) * 12 }}>
                    <b>{fmt(hold)} s</b><i>hold</i>
                    <span className="grip" role="slider" aria-label="Hold time" aria-valuenow={hold} draggable={false}
                      onPointerDown={(e) => { e.stopPropagation(); e.currentTarget.setPointerCapture(e.pointerId); setHoldDrag({ i, x: e.clientX, h0: st.hold, h: st.hold }); }}
                      onPointerMove={holdMove} onPointerUp={holdUp} />
                  </div>
                </div>
              );
            })}
          </div>
        </div>
      </div>

      <div className="col raised">
        <CameraFeeds s={s} />
        <StopTabs tabs={["Step editor", "Tags"]} value={tab} onChange={setTab} />
        <div className="scroll">
          {!step ? <div className="stepedit"><p className="note">Select a step, or Add step to record the current pose.</p></div> : tab === "Step editor" ? (
            <div className="stepedit">
              <span className="h3">Step {stepSel + 1} · {step.name}</span>
              <label>
                <span className="fieldlabel">Name</span>
                <span className="field"><input className="left" key={`${sel}-${stepSel}`} defaultValue={step.name} aria-label="Step name"
                  onBlur={(e) => e.target.value.trim() && e.target.value !== step.name && editStep({ name: e.target.value.trim() })}
                  onKeyDown={(e) => e.key === "Enter" && (e.target as HTMLInputElement).blur()} /></span>
              </label>
              <div className="two">
                <NumField small pre="move" suf="s" value={times[stepSel]} readOnly />
                <NumField small pre="hold" suf="s" value={step.hold} min={0} max={60} onCommit={(v) => editStep({ hold: Math.round(v * 10) / 10 })} />
              </div>
              <div className="row">
                <button className="btn" onClick={() => savePose("Updated", stepSel)} title="Store the current pose in this step"><Camera size={16} />Capture pose</button>
                <button className="btn" onClick={delStep}><Trash2 size={16} />Delete step</button>
              </div>
            </div>
          ) : (
            <div className="stepedit">
              <span className="h3">Step {stepSel + 1} · {step.name}</span>
              <p>{step.tag?.tags?.length ? `Tag ${step.tag.tags.map((t) => t.id).join(", ")} saved with this step` : "No tag saved with this step"}</p>
              <Check label="Tag on save" checked={prefs.tagOnSave} onChange={(v) => setPrefs({ tagOnSave: v })} />
              <div className="row">
                <button className="btn" disabled={!s.camera.connected} onClick={captureTag} title="The arm must be at this step"><Scan size={16} />Capture tag</button>
                <button className="btn" disabled={!step.tag} onClick={clearTag}><Trash2 size={16} />Clear</button>
              </div>
              {!s.camera.connected && <p className="note">Connect the real camera in Telemetry, Comms to capture a tag</p>}
            </div>
          )}
        </div>
      </div>
    </div>
  );
}
