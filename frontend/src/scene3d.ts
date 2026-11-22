/* One three.js scene shared by the main viewport and the simulated camera POV,
   built like hil/scene.py: the arm STLs posed with the backend's matrices, the
   red tank keep-out box, the EE and target markers. Z is up, units are mm. */
import * as THREE from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import type { Pose } from "./types";

export const scene = new THREE.Scene();
scene.background = new THREE.Color(0x050505);

const OVERLAY = 1; // tank and markers: main view only, as the toolkit's POV render hides them

const meshes: Record<string, THREE.Mesh> = {};
let lens: number[][] | null = null;

scene.add(new THREE.AmbientLight(0xffffff, 0.55));
const key = new THREE.DirectionalLight(0xffffff, 1.6);
key.position.set(400, -600, 900);
scene.add(key);
const fill = new THREE.DirectionalLight(0xffffff, 0.5);
fill.position.set(-500, 400, 300);
scene.add(fill);

const grid = new THREE.GridHelper(700, 14, 0x686868, 0x3a3a3a);
grid.rotation.x = Math.PI / 2;
grid.position.set(0, -150, 0);
scene.add(grid);

function overlay<T extends THREE.Object3D>(o: T): T {
  o.layers.set(OVERLAY);
  scene.add(o);
  return o;
}

export const tank = new THREE.Group();
const tankGeo = new THREE.BoxGeometry(700, 350, 150);
tank.add(new THREE.Mesh(tankGeo, new THREE.MeshBasicMaterial({ color: 0xff0004, transparent: true, opacity: 0.07, depthWrite: false })));
tank.add(new THREE.LineSegments(new THREE.EdgesGeometry(tankGeo), new THREE.LineBasicMaterial({ color: 0xff0004, transparent: true, opacity: 0.35 })));
tank.position.set(0, 175, 75);
tank.traverse((o) => o.layers.set(OVERLAY));
scene.add(tank);

export const eeMarker = overlay(new THREE.Mesh(new THREE.SphereGeometry(8, 20, 14), new THREE.MeshBasicMaterial({ color: 0xff6b35, transparent: true, opacity: 0.85 })));
export const targetMarker = overlay(new THREE.Mesh(new THREE.SphereGeometry(7, 20, 14), new THREE.MeshBasicMaterial({ color: 0xff0004, transparent: true, opacity: 0.55 })));

let loading: Promise<void> | null = null;
export function loadParts(): Promise<void> {
  loading ??= (async () => {
    const res = await fetch("/api/scene/parts");
    const { parts } = (await res.json()) as { parts: { id: string; file: string; color: string }[] };
    const loader = new STLLoader();
    const geos: Record<string, Promise<THREE.BufferGeometry>> = {};
    await Promise.all(
      parts.map(async (p) => {
        geos[p.file] ??= loader.loadAsync(`/api/scene/stl/${p.file}`).then((g) => (g.computeVertexNormals(), g));
        const mat = new THREE.MeshPhongMaterial({ color: p.color, specular: 0x666666, shininess: 20 });
        const m = new THREE.Mesh(await geos[p.file], mat);
        m.matrixAutoUpdate = false;
        m.visible = false;
        meshes[p.id] = m;
        scene.add(m);
      }),
    );
  })();
  return loading;
}

export function applyPose(pose: Pose | undefined) {
  if (!pose) return;
  for (const [id, mat] of Object.entries(pose.mats)) {
    const m = meshes[id];
    if (!m) continue;
    m.matrix.fromArray(mat);
    m.matrixWorldNeedsUpdate = true;
    m.visible = true;
  }
  lens = pose.lens;
}

/** Point the POV camera through the hand camera lens (hil/scene.py camera_pov). */
export function aimPov(cam: THREE.PerspectiveCamera) {
  if (!lens) return;
  const [p, f, u] = lens;
  cam.position.set(p[0], p[1], p[2]);
  cam.up.set(u[0] - p[0], u[1] - p[1], u[2] - p[2]).normalize();
  cam.lookAt(f[0], f[1], f[2]);
}

export const POV_W = 512;
export const POV_H = 320;
export const POV_VFOV = (2 * Math.atan(Math.tan(Math.PI / 6) * (POV_H / POV_W)) * 180) / Math.PI;
export { OVERLAY };
