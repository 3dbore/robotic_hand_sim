/* Small shared controls from the HIL component library (DESIGN.md §5). */
import { useRef, useState, type PointerEvent, type ReactNode } from "react";

export const fmt = (v: number, d = 1) => (Number.isFinite(v) ? v.toFixed(d) : "–");
export const signed = (v: number, d = 1) => (v >= 0 ? "+" : "") + fmt(v, d);
const clamp = (v: number, lo: number, hi: number) => Math.min(hi, Math.max(lo, v));

export function Dot({ tone }: { tone: string }) {
  return <span className={`dot ${tone}`} />;
}

export function Tabs<T extends string>({ tabs, value, onChange, children }: { tabs: T[]; value: T; onChange: (t: T) => void; children?: ReactNode }) {
  return (
    <div className="tabs" role="tablist">
      {tabs.map((t) => (
        <button key={t} role="tab" aria-selected={t === value} className={`tab ${t === value ? "on" : ""}`} onClick={() => onChange(t)}>
          {t}
        </button>
      ))}
      {children && <div className="end">{children}</div>}
    </div>
  );
}

/** Field with italic prefix, bold value and unit, as in the Number Field component. */
export function NumField({ pre, value, suf, onCommit, step = 0.1, min, max, readOnly, small }: {
  pre: string; value: number; suf?: string; onCommit?: (v: number) => void; step?: number; min?: number; max?: number; readOnly?: boolean; small?: boolean;
}) {
  const [draft, setDraft] = useState<string | null>(null);
  const commit = () => {
    if (draft === null) return;
    const v = parseFloat(draft);
    setDraft(null);
    if (Number.isFinite(v) && onCommit) onCommit(clamp(v, min ?? -Infinity, max ?? Infinity));
  };
  const d = step < 1 ? 1 : 0;
  return (
    <label className={`field ${small ? "sm" : ""}`}>
      <span className="pre">{pre}</span>
      <input
        type="number" step={step} min={min} max={max} readOnly={readOnly || !onCommit}
        value={draft ?? fmt(value, d)}
        onChange={(e) => setDraft(e.target.value)}
        onBlur={commit}
        onKeyDown={(e) => e.key === "Enter" && (e.target as HTMLInputElement).blur()}
      />
      {suf && <span className="suf">{suf}</span>}
    </label>
  );
}

export function Check({ label, checked, onChange, disabled }: { label: string; checked: boolean; onChange?: (v: boolean) => void; disabled?: boolean }) {
  return (
    <label className="check">
      <input type="checkbox" checked={checked} disabled={disabled} onChange={(e) => onChange?.(e.target.checked)} />
      {label}
    </label>
  );
}

/** Slider with a green allowed band and a home tick. onChange fires while
    dragging, onCommit once on release (that is when a move is requested). */
export function Slider({ min, max, value, band, tick, vertical, onChange, onCommit, label }: {
  min: number; max: number; value: number; band?: [number, number]; tick?: number; vertical?: boolean;
  onChange?: (v: number) => void; onCommit: (v: number) => void; label: string;
}) {
  const ref = useRef<HTMLDivElement>(null);
  const [drag, setDrag] = useState<number | null>(null);
  const shown = drag ?? value;
  const pct = (v: number) => ((clamp(v, min, max) - min) / (max - min)) * 100;
  const at = (e: PointerEvent) => {
    const r = ref.current!.getBoundingClientRect();
    const f = vertical ? (r.bottom - e.clientY) / r.height : (e.clientX - r.left) / r.width;
    return Math.round((min + clamp(f, 0, 1) * (max - min)) * 2) / 2;
  };
  const pos = (p: number) => (vertical ? { bottom: `${p}%` } : { left: `${p}%` });
  const span = (a: number, b: number) =>
    vertical ? { bottom: `${pct(a)}%`, height: `${pct(b) - pct(a)}%` } : { left: `${pct(a)}%`, width: `${pct(b) - pct(a)}%` };
  return (
    <div
      ref={ref} className={`slider ${vertical ? "v" : ""}`} role="slider" tabIndex={0} aria-label={label}
      aria-valuemin={min} aria-valuemax={max} aria-valuenow={shown}
      onPointerDown={(e) => { e.currentTarget.setPointerCapture(e.pointerId); const v = at(e); setDrag(v); onChange?.(v); }}
      onPointerMove={(e) => { if (drag === null) return; const v = at(e); setDrag(v); onChange?.(v); }}
      onPointerUp={(e) => { if (drag === null) return; setDrag(null); onCommit(at(e)); }}
      onKeyDown={(e) => {
        const d = { ArrowRight: 1, ArrowUp: 1, ArrowLeft: -1, ArrowDown: -1 }[e.key];
        if (d) { e.preventDefault(); onCommit(clamp(value + d, min, max)); }
      }}
    >
      <div className="track" />
      {band && <div className="band" style={span(band[0], band[1])} />}
      {tick !== undefined && <div className="tick" style={pos(pct(tick))} />}
      <div className="thumb" style={pos(pct(shown))} />
    </div>
  );
}

/** Rotary knob, ±range, drag up/down or around to turn. */
export function Knob({ value, range, onCommit, children }: { value: number; range: number; onCommit: (v: number) => void; children: ReactNode }) {
  const [drag, setDrag] = useState<{ y: number; v0: number; v: number } | null>(null);
  const shown = drag?.v ?? value;
  return (
    <div
      className="knob" role="slider" tabIndex={0} aria-valuenow={shown} aria-valuemin={-range} aria-valuemax={range}
      onPointerDown={(e) => { e.currentTarget.setPointerCapture(e.pointerId); setDrag({ y: e.clientY, v0: value, v: value }); }}
      onPointerMove={(e) => drag && setDrag({ ...drag, v: Math.round(clamp(drag.v0 + (drag.y - e.clientY), -range, range)) })}
      onPointerUp={() => { if (drag) { setDrag(null); if (drag.v !== drag.v0) onCommit(drag.v); } }}
      onKeyDown={(e) => {
        const d = { ArrowRight: 1, ArrowUp: 1, ArrowLeft: -1, ArrowDown: -1 }[e.key];
        if (d) { e.preventDefault(); onCommit(clamp(value + d, -range, range)); }
      }}
    >
      <div className="needle" style={{ transform: `rotate(${(shown / range) * 135}deg)` }} />
      {children}
      <b>{signed(shown, 0)}°</b>
    </div>
  );
}
