import type { TelemetryState } from "./types";

type Listener = (state: TelemetryState) => void;
type ErrorListener = (message: string) => void;
type PoseListener = (pose: Record<string, number>) => void;
type TagListener = (tag: any) => void;

class ApiClient {
  private ws: WebSocket | null = null;
  private listeners: Set<Listener> = new Set();
  private errorListeners: Set<ErrorListener> = new Set();
  private poseListeners: Set<PoseListener> = new Set();
  private tagListeners: Set<TagListener> = new Set();
  private reconnectTimer: any = null;
  public currentState: TelemetryState | null = null;
  public lastError: string = "";

  constructor() {
    this.connectWs();
  }

  private connectWs() {
    if (this.ws && (this.ws.readyState === WebSocket.OPEN || this.ws.readyState === WebSocket.CONNECTING)) {
      return;
    }

    const loc = window.location;
    const wsProto = loc.protocol === "https:" ? "wss:" : "ws:";
    // If running under vite proxy, connect to loc.host
    const wsUrl = `${wsProto}//${loc.host}/ws`;

    try {
      this.ws = new WebSocket(wsUrl);

      this.ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          if (data.type === "telemetry") {
            this.currentState = data;
            this.listeners.forEach((fn) => fn(data));
          } else if (data.type === "error") {
            this.lastError = data.message || "Unknown error";
            console.error("[Robot Error]", this.lastError);
            this.errorListeners.forEach((fn) => fn(this.lastError));
          } else if (data.type === "pose_captured") {
            this.poseListeners.forEach((fn) => fn(data.pose));
          } else if (data.type === "tag_captured") {
            this.tagListeners.forEach((fn) => fn(data.tag));
          }
        } catch (e) {
          console.error("Failed to parse WS msg", e);
        }
      };

      this.ws.onclose = () => {
        this.ws = null;
        if (!this.reconnectTimer) {
          this.reconnectTimer = setTimeout(() => {
            this.reconnectTimer = null;
            this.connectWs();
          }, 1500);
        }
      };

      this.ws.onerror = () => {
        if (this.ws) {
          this.ws.close();
        }
      };
    } catch (e) {
      console.error("WS connect error", e);
    }
  }

  public subscribe(fn: Listener): () => void {
    this.listeners.add(fn);
    if (this.currentState) {
      fn(this.currentState);
    }
    return () => {
      this.listeners.delete(fn);
    };
  }

  public onError(fn: ErrorListener): () => void {
    this.errorListeners.add(fn);
    return () => {
      this.errorListeners.delete(fn);
    };
  }

  public onPoseCaptured(fn: PoseListener): () => void {
    this.poseListeners.add(fn);
    return () => {
      this.poseListeners.delete(fn);
    };
  }

  public onTagCaptured(fn: TagListener): () => void {
    this.tagListeners.add(fn);
    return () => {
      this.tagListeners.delete(fn);
    };
  }

  public send(action: string, payload: Record<string, any> = {}) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify({ action, ...payload }));
    }
  }

  // Quick Action Helpers
  public jogAxis(axis: string, delta: number) {
    this.send("jog_axis", { axis, delta });
  }

  public setAxis(axis: string, value: number) {
    this.send("set_axis", { axis, value });
  }

  public baseJog(direction: "left" | "right" | "home") {
    this.send("base_jog", { direction });
  }

  public setGripper(percent: number) {
    this.send("set_gripper", { percent });
  }

  public setWrist(pitch?: number, yaw?: number) {
    this.send("set_wrist", { pitch, yaw });
  }

  public setGripperState(state: "open" | "close") {
    this.send("set_gripper_state", { state });
  }

  public goToPoint(x: number, y: number, z: number, pitch?: number, yaw?: number) {
    this.send("go_point", { x, y, z, pitch, yaw });
  }

  public stop() {
    this.send("stop");
  }

  public toggleMotors(motors_on?: boolean) {
    this.send("toggle_motors", { motors_on });
  }

  public connectSerial(port: string, baud: number = 9600) {
    this.send("connect_serial", { port, baud });
  }

  public disconnectSerial() {
    this.send("disconnect_serial");
  }

  public clearLogs() {
    this.send("clear_logs");
  }

  public runSequence(name: string) {
    this.send("run_sequence", { name });
  }

  public stopSequence() {
    this.send("stop_sequence");
  }

  public async capturePose(): Promise<Record<string, number> | null> {
    try {
      const res = await fetch("/api/capture/pose", { method: "POST" });
      const data = await res.json();
      if (data.status === "ok" && data.pose) {
        return data.pose;
      }
    } catch {
      this.send("capture_pose");
    }
    return null;
  }

  /** Tags in view for a step at `servo` (the arm must be there and still). Errors come back as `error`. */
  public async captureTag(servo: Record<string, number>): Promise<{ tag?: any; description?: string; error?: string }> {
    try {
      const res = await fetch("/api/capture/tag", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ servo }),
      });
      const data = await res.json();
      return data.status === "ok" ? { tag: data.tag, description: data.description } : { error: data.message || "no reply" };
    } catch {
      return { error: "backend not reachable" };
    }
  }

  // REST API calls
  public async getPorts(): Promise<string[]> {
    try {
      const res = await fetch("/api/ports");
      const data = await res.json();
      return data.ports || [];
    } catch {
      return ["COM4", "COM3", "COM7"];
    }
  }

  public async getSequences(): Promise<any[]> {
    try {
      const res = await fetch("/api/sequences");
      const data = await res.json();
      return data.sequences || [];
    } catch {
      return [];
    }
  }

  public async saveSequences(sequences: any[]): Promise<boolean> {
    try {
      const res = await fetch("/api/sequences", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ sequences }),
      });
      return res.ok;
    } catch {
      return false;
    }
  }

  public async getSettings(): Promise<any> {
    try {
      const res = await fetch("/api/settings");
      return await res.json();
    } catch {
      return null;
    }
  }

  public async saveSettings(settings: any): Promise<boolean> {
    try {
      const res = await fetch("/api/settings", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ settings }),
      });
      return res.ok;
    } catch {
      return false;
    }
  }

  public async connectCamera(params: { host: string; user?: string; password?: string; res?: string; tags?: boolean }) {
    try {
      await fetch("/api/camera/connect", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(params),
      });
    } catch (e) {
      console.error(e);
    }
  }

  public async disconnectCamera() {
    try {
      await fetch("/api/camera/disconnect", { method: "POST" });
    } catch (e) {
      console.error(e);
    }
  }
}

export const api = new ApiClient();
