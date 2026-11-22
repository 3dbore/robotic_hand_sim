# ELIOS HIL: Design System

Design spec for the ELIOS hardware-in-the-loop (HIL) console, a UI for a 5-axis waste-sorting arm. It extends the Telemetrix design system (dark, near-black cockpit, one accent) and is implemented in Figma file `lXFeH5NkApDTFkN1VTMF9Q` (3DBORE-Branding), pages "ELIOS HIL" (screens) and "HIL Components" (library).

## 1. Direction

- **Product:** HIL console for operating and testing a 5-axis arm over a serial link, with a 3D simulation and a real camera feed.
- **Character:** precise, instrument-like, calm under load.
- **World:** a test bench at night. Surfaces stay neutral; color only carries meaning.
- **Moments (the only rich areas):** the 3D arm viewport and the camera feed with AprilTag detection.
- **Signature:** the `sent ▼ / board ●` marker pair on servo ranges, and `Esc` keycap hints beside the stop control.

## 2. Principles

1. Depth comes from surfaces and 1px outlines. No shadows, gradients or glows.
2. One accent (`accent/brand`). Do not use it for decoration.
3. Red has three meanings (brand, brake/stop, disconnected). Always pair red with a text label or icon.
4. Copy is short sentence case with no emoji and no exclamation marks. Units and labels are italic.
5. Everything sits on a 4px grid. Inner radius = outer radius minus padding.
6. No raw hex or px in components. Bind to variables and text styles.

## 3. Tokens

### Color

| Token | Value | Use |
|---|---|---|
| surface/base | #050505 | App background, log well, viewport |
| surface/raised | #202020 | Panels, nav bar, cards |
| surface/selected | #1e1e1e | Active tab, selected list item |
| border/default | #686868 | Hairlines, input outlines |
| border/strong, text/primary, icon/primary | #ffffff | Focus, text, icons |
| accent/brand | #fa0549 | Primary action, selection |
| status/connected | #3cff00 | Linked, live |
| status/disconnected | #fa0549 | Offline, error text |
| data/positive | #1eb224 | Healthy, in-range |
| data/caution | #e2ff02 | Warnings, rejected frames |
| data/negative | #ff0004 | Out of range, fault |
| series | #e2ff02, #fd6af6, #615198, #02fff2 | Telemetry series only |

### Type

Plus Jakarta Sans throughout (Tomorrow Bold only for the wordmark). Line height 100%. Text styles are named `HIL/<Group>/<Size>`: `Heading/S`, `Body/S`, `Label/S Italic`, and larger steps for panel titles. Weights are 400 and 700. Italics are for labels and units. Tabular figures are not available through the Figma plugin API; use consistent decimals instead (for example `90.0°`).

### Spacing, radius, stroke, size

- Spacing: xs 4, sm 8, md 12, lg 16, xl 24.
- Radius: sm 5, md 9, pill 48.
- Stroke: thin 1, medium 2, thick 3.
- Sizes: control 36, icon-button 30, field 45, nav height 58.

## 4. Layout

Canvas 1920×1080, desktop only. The shell is a 58px nav bar over a three-region body:

| Region | Width | Content |
|---|---|---|
| Left | 463 | Serial log over a tabbed panel (Manual control / Servo telemetry) |
| Center | 990 | 3D viewport, then the Control panel (top view, side view, pitch, yaw, gripper) |
| Right | 467 | Real and simulated camera feeds, then a tabbed panel (Board link / Comms) |

The base jog cluster (Left, Home, Right) floats between the viewport and the Control panel. The Stop button with its `Esc` keycap sits in the right tab strip and is always visible.

Nav bar: BSG logo, tabs (Telemetry, Sequence, Settings), board and motor status chips, window controls. The active tab uses `surface/selected` with a white underline.

## 5. Components

Library page "HIL Components". All are bound to the tokens above.

- **Actions:** Button (Primary, Secondary, Danger; M/S; optional icon), Toggle Button, Icon Button (30/36), Keycap.
- **Status:** Status Chip (tones: Ok, Warn, Bad, Info, Idle), Status Indicator (Connected, Disconnected), Pill Label.
- **Navigation:** Tab (Default, Active), Nav Item, Window Controls, Panel Header (title, subtitle, slot for actions).
- **Inputs:** Text Field, Dropdown, Number Field (prefix, value, suffix), Checkbox, Toggle Switch, Slider, Vertical Slider, Rotary Knob.
- **Telemetry:** Joint Row, Servo Card (tones Idle to Bad), Range Bar, Stat, Base Jog Button.
- **Views:** Viewport 3D, Workspace View (Top, Side), Camera Feed (Real/Sim, Live/Offline).
- **Sequence:** Sequence List Item, Step Block (selected, tag flag).
- **Settings:** Axis Row (name, channel, min, max, home, speed, trim, invert).
- **Icons:** 21 line icons at 1.5 stroke, color from the text role.

Implementation notes:
- Slider and marker positions are driven by container padding (`Slider` left padding, `Range Bar` row padding, `Vertical Slider` rail top padding), because child positions cannot be overridden in instances.
- Variant text that differs per variant is static text, not a bound property.
- `Vertical Slider` has a fixed 240px track; shorter columns detach the instance (as with the gripper slider).

## 6. Screens

1. **Telemetry · Manual control · Board link:** per-servo jog sliders with step selector, gripper actions, 3D view, Control panel, camera feeds, link health stats.
2. **Telemetry · Servo telemetry · Comms:** servo cards with sent and board angles and state, camera setup (address, SSH, Tags, Connect), offline state with the error line.
3. **Sequence:** saved sequences, step grid with Add step and Run, step editor (name, move and hold timing, wait for tag, tag id, Capture pose).
4. **Settings:** joint configuration table, serial link, simulation and safety options.

## 7. Behavior

- Esc triggers Stop and releases the motors.
- Moves are ramped to each axis speed limit so axes arrive together.
- Frames with jumps above the limit are ignored and counted (shown in caution yellow in the log and in Link health).
- No looping motion in the UI chrome.

## 8. Do and don't

- Do pair every red element with a label: Stop, Offline, Disconnect.
- Do keep the viewport and camera feeds as the only rich areas.
- Don't add shadows, gradients, emoji or decorative accent color.
- Don't hardcode hex or px; add a variable first.
- Don't use more than one primary button per panel.