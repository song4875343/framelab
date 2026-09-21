/* 交叉验证：内置3D（含竖向墙膜刚度）vs OpenSees真3D（ASDShellQ4膜截面）量级一致 */
import { readFileSync, writeFileSync, mkdirSync, rmSync } from "node:fs";
import { execFileSync } from "node:child_process";
import path from "node:path";
import { fileURLToPath } from "node:url";
const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(HERE, "..");
const PY = path.join(ROOT, ".venv", "Scripts", "python.exe");
const TMP = path.join(HERE, "tmp");
(0, eval)(readFileSync(path.join(ROOT, "solver.js"), "utf8"));
(0, eval)(readFileSync(path.join(ROOT, "xara.js"), "utf8"));
const { FrameSolver, FrameXara } = globalThis;
let nFail = 0;
function check(name, cond, detail) {
  if (cond) { console.log("  PASS " + name + (detail ? "  [" + detail + "]" : "")); }
  else { nFail++; console.log("  FAIL " + name + (detail ? "  [" + detail + "]" : "")); }
}
function cleanEnv() {
  const env = Object.assign({}, process.env);
  if (env.PATH) env.PATH = env.PATH.split(";").filter((p) => !/\.venv[\\/]Library[\\/]bin/i.test(p)).join(";");
  return env;
}
const C30 = { label: "C30", E: 3.0e7, nu: 0.2 };
const colS = { type: "rect", b: 0.4, h: 0.4 };
const beamS = { type: "rect", b: 0.3, h: 0.6 };
const FALL = { ux: 1, uy: 1, uz: 1, rx: 1, ry: 1, rz: 1 };
const state = {
  NSUB: 4, AMESH: 2,
  D: { NSUB: 4, AMESH: 2 },
  model: {
    nodes: [
      { id: 1, x: 0, y: 0, z: 0, bc: FALL }, { id: 2, x: 6, y: 0, z: 0, bc: FALL },
      { id: 5, x: 0, y: 0, z: 4, bc: {} }, { id: 6, x: 6, y: 0, z: 4, bc: {} },
    ],
    members: [
      { id: 11, a: 1, b: 5, type: "col" }, { id: 12, a: 2, b: 6, type: "col" },
      { id: 15, a: 5, b: 6, type: "beam" },
    ],
    areas: [{ id: 22, etype: "shell", nodes: [1, 2, 6, 5] }], // y=0 竖向剪力墙 XZ
    cuts: [],
  },
  memStiff: {},
  memMat: { 11: C30, 12: C30, 15: C30 },
  memSec: { 11: colS, 12: colS, 15: beamS },
  memLoads: {}, nodeLoads: { 6: { fx: 50 } },
  areaMat: {}, areaSec: {}, areaLoads: {},
  memDiv: {}, areaDiv: {},
};
const builtin = FrameSolver.solve(state);
check("builtin ok", builtin.ok === true);
check("builtin 3D且墙参与", builtin.is3D === true && (builtin.skippedArea3D || 0) === 0,
  "skipped=" + builtin.skippedArea3D + " wallCells=" + builtin.areaElems.length);
let xa = null;
try {
  const built = FrameXara.buildScript(state, {});
  check("xara 路径 true3d", built.meta && built.meta.kind === "true3d", "kind=" + (built.meta && built.meta.kind));
  rmSync(TMP, { recursive: true, force: true });
  mkdirSync(TMP, { recursive: true });
  const sp = path.join(TMP, "wallx.py"), op = path.join(TMP, "wallx.json");
  writeFileSync(sp, built.script, "utf8");
  execFileSync(PY, ["run_xara.py", sp, "-o", op], { cwd: ROOT, env: cleanEnv(), stdio: "pipe" });
  const res = JSON.parse(readFileSync(op, "utf8"));
  if (!res.ok) throw new Error("xara 求解失败：" + (res.error || "?"));
  xa = FrameXara.importResults(res, built.meta, state);
  check("xara 求解+回填", true);
} catch (e) { check("xara 求解+回填", false, String((e && e.message) || e).slice(0, 300)); }
if (xa) {
  const bD = builtin.res.maxDisp, xD = xa.res.maxDisp;
  const rel = Math.abs(bD - xD) / Math.max(1e-12, Math.abs(xD));
  console.log("  builtin maxDisp=" + bD.toFixed(4) + "mm  opensees maxDisp=" + xD.toFixed(4) + "mm  rel=" + rel.toExponential(2));
  // 基线：2D悬臂墙Q4 Case D 内置vs OpenSees 位移 rel≈9.5e-2；此处竖向墙同为Q4膜，
  // 另梁端转角无膜刚度约束（ASDShell 有 drilling 刚度），放宽到 25% 仅验量级
  check("顶点/最大位移量级一致(<25%)", rel < 0.25, "rel=" + rel.toExponential(2));
  const bR = builtin.res.roof, xR = xa.res.roof;
  const relR = Math.abs(bR - xR) / Math.max(1e-12, Math.abs(xR));
  check("顶层平均侧移一致(<25%)", relR < 0.25, "rel=" + relR.toExponential(2));
}
if (nFail) { console.log("\nRESULT: " + nFail + " FAIL"); process.exit(1); }
else { console.log("\nRESULT: ALL PASS"); }
