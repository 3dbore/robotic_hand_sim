export interface AxisState {
  name: string;
  channel: string;
  home: number;
  servo_lo: number;
  servo_hi: number;
  sim_lo: number;
  sim_hi: number;
  speed: number;
  inverted: boolean;
  trim: number;
  wire_lo: number;
  wire_hi: number;
  command_servo: number;
  board_servo: number;
  sim_angle: number;
  wire_angle: number;
  diff: number;
  status: string;
  tone: "ok" | "warn" | "info" | "bad";
}

export interface LinkHealth {
  frames_in_per_s: number;
  rejected: number;
  i2c_errors: number;
  board_resets: number;
  jumps_ignored: number;
  last_report_s: number;
  last_boot: string;
  is_healthy: boolean;
}

export interface CameraState {
  connected: boolean;
  status: string;
  status_tone: "ok" | "warn" | "info" | "bad" | "idle";
  fps: number;
  tag_count: number;
  error_msg: string;
  host: string;
  user: string;
  res: string;
  detect_tags: boolean;
}

export interface LogEntry {
  time: string;
  message: string;
  level: "stat" | "frame" | "jump" | "error" | "info";
}

export interface SequenceStep {
  name: string;
  hold: number;
  servo: Record<string, number>;
  tag?: {
    id?: number;
    tags?: Array<{ id: number; px: number[]; offset_deg: number[]; azimuth_deg: number[]; elevation_deg: number[] }>;
    camera?: { azimuth_deg: number; elevation_deg: number[] };
  };
}

export interface SequenceData {
  name: string;
  step_count: number;
  total_time_s: number;
  steps: SequenceStep[];
}

export interface Pose {
  mats: Record<string, number[]>;
  lens: number[][];
  /** fingertip centre matrix (column-major) and the jaw gap in mm */
  grip: { mat: number[]; gap: number };
}

export interface TelemetryState {
  type: string;
  time: number;
  motors_on: boolean;
  moving: boolean;
  connected: boolean;
  port: string;
  baud: number;
  link_health: LinkHealth;
  axes: AxisState[];
  gripper_percent: number;
  grip_closed_offset: number;
  ee_pos: [number, number, number];
  joints: [number, number, number][];
  target_ee: [number, number, number];
  target_pitch: number;
  target_yaw: number;
  camera: CameraState;
  sequence: {
    running: boolean;
    auto: boolean; // false: one step per Next step click
    name: string;
    step_index: number;
    step_count: number;
    holding: boolean;
    message: string;
    error: boolean;
  };
  logs: LogEntry[];
  pose: Pose;
  board_pose: Pose;
}
