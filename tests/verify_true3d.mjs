/* FrameLab 真 3D 路径验证 harness（Node 运行）。
 *
 * 目的：证明真 3D 链路（内置 6 自由度空间杆系求解 vs xara 真 3D 导出 +
 * OpenSees elasticBeamColumn）在数值上一致。
 *
 * 运行（仓库根目录）：
 *   node tests/verify_true3d.mjs
 *
 * 流程：
 *   state -> FrameSolver.solve（内置，is3D 应为 true，u stride=6） ──┐
 *                                                                    ├─> 对比（位移 / N/Vy/Vz/My/Mz/T）
 *   state -> FrameXara.buildScript（kind 应为 true3d） -> .venv python run_xara.py -> importResults ──┘
 *
 * 说明：
 * - 内置 3D 为欧拉梁（无剪切变形），OpenSees elasticBeamColumn 含剪切，
 *   故采用细长构件（剪切影响 ~1%），容差取 5%/8%。
 * - 梁均布按重力方向（竖向平面内横向），柱均布按水平横向；节点荷载含 fz/mx。
 */
import { readFileSync, writeFileSync, mkdirSync, rmSync } from "node:fs";
import { execFileSync } from "node:child_process";
import path from "node:path";
import { fileURLToPath } from "node:url";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(HERE, "..");
const PY = path.join(ROOT, ".venv", "Scripts", "python.exe");
const TMP = path.join(HERE, "tmp");

function loadLib(rel) {
  const code = readFileSync(path.join(ROOT, rel), "utf8");
  (0, eval)(code); // eslint-disable-line no-eval
}
loadLib("solver.js");
loadLib("xara.js");
const { FrameSolver, FrameXara } = globalThis;
if (!FrameSolver || !FrameXara) {
  console.error("FAIL: solver.js / xara.js 未能加载");
  process.exit(2);
}

let nFail = 0;
function check(name, cond, detail) {
  if (cond) { console.log("  PASS " + name + (detail ? "  [" + detail + "]" : "")); }
  else { nFail++; console.log("  FAIL " + name + (detail ? "  [" + detail + "]" : "")); }
}
function maxAbs(a) { return a.reduce((m, v) => Math.max(m, Math.abs(v)), 0); }
function relErr(got, ref) {
  const scale = Math.max(maxAbs(ref), 1e-12);
  let e = 0;
  for (let i = 0; i < got.length; i++) e = Math.max(e, Math.abs(got[i] - ref[i]) / scale);
  return e;
}
function cleanEnv() {
  const env = Object.assign({}, process.env);
  if (env.PATH) {
    env.PATH = env.PATH.split(";").filter((p) => !/\.venv[\\/]Library[\\/]bin/i.test(p)).join(";");
  }
  return env;
}
function runXara(tag, state) {
  const built = FrameXara.buildScript(state, {});
  rmSync(TMP, { recursive: true, force: true });
  mkdirSync(TMP, { recursive: true });
  const scriptPath = path.join(TMP, tag + ".py");
  const outPath = path.join(TMP, tag + ".json");
  writeFileSync(scriptPath, built.script, "utf8");
  execFileSync(PY, ["run_xara.py", scriptPath, "-o", outPath], { cwd: ROOT, env: cleanEnv(), stdio: "pipe" });
  const res = JSON.parse(readFileSync(outPath, "utf8"));
  if (!res.ok) throw new Error("xara 求解失败：" + (res.error || "?"));
  return { ana: FrameXara.importResults(res, built.meta, state), meta: built.meta };
}

// ---------- Case T1：单跨单层空间框架（X 向 6m × Z 高 4m，Y 向单榀） ----------
function stateSpaceFrame() {
  const C30 = { label: "C30", E: 3.0e7, nu: 0.2 };
  const colS = { type: "rect", b: 0.4, h: 0.4 };
  const beamS = { type: "rect", b: 0.3, h: 0.6 };
  const FALL = { ux: 1, uy: 1, uz: 1, rx: 1, ry: 1, rz: 1 };
  return {
    D: { NSUB: 5, AMESH: 2 },
    model: {
      nodes: [
        { id: 1, x: 0, y: 0, z: 0, bc: FALL },
        { id: 2, x: 6, y: 0, z: 0, bc: FALL },
        { id: 3, x: 0, y: 0, z: 4, bc: {} },
        { id: 4, x: 6, y: 0, z: 4, bc: {} },
      ],
      members: [
        { id: 11, a: 1, b: 3, type: "col" },
        { id: 12, a: 2, b: 4, type: "col" },
        { id: 13, a: 3, b: 4, type: "beam" },
      ],
      areas: [], cuts: [],
    },
    memStiff: {},
    memMat: { 11: C30, 12: C30, 13: C30 },
    memSec: { 11: colS, 12: colS, 13: beamS },
    memLoads: { 13: [{ type: "udl", q: -10 }] },
    nodeLoads: { 3: { fx: 15, fz: 2 }, 4: { fx: 15, mx: 1.5 } },
    areaMat: {}, areaSec: {}, areaLoads: {},
    memDiv: { 11: { n: 2 }, 12: { n: 2 }, 13: { n: 2 } },
    areaDiv: {},
  };
}

console.log("FrameLab 真 3D 验证");
console.log("solver=" + FrameSolver.version);
console.log("## Case T1 空间框架（内置3D vs OpenSees真3D）");
{
  const state = stateSpaceFrame();
  const builtin = FrameSolver.solve(state);
  check("builtin ok", builtin.ok === true);
  check("builtin 走 3D（stride=6）", builtin.is3D === true && builtin.u.length === builtin.all.length * 6,
    "is3D=" + builtin.is3D + " dof=" + (builtin.u.length / builtin.all.length));
  let xa;
  try { xa = runXara("true3d_t1", state); }
  catch (e) { check("xara 求解+回填", false, String(e.message || e).slice(0, 300)); }
  if (xa) {
    check("xara 求解+回填", true);
    check("路径 true3d", xa.meta && xa.meta.kind === "true3d", "kind=" + (xa.meta && xa.meta.kind));
    const sameNodes = builtin.all.length === xa.ana.all.length &&
      builtin.all.every((p, i) => Math.hypot(p[0] - xa.ana.all[i][0], p[1] - xa.ana.all[i][1]) < 1e-9);
    check("分析节点一致", sameNodes, builtin.all.length + " nodes");
    check("回填 stride=6", xa.ana.u.length === xa.ana.all.length * 6,
      "dof=" + (xa.ana.u.length / xa.ana.all.length));
    const du = [];
    for (let i = 0; i < builtin.all.length; i++) {
      for (let d = 0; d < 6; d++) du.push(xa.ana.u[6 * i + d] - builtin.u[6 * i + d]);
    }
    const eU = maxAbs(du) / Math.max(1e-12, maxAbs(builtin.u));
    check("节点位移一致（含uz/转角）", eU < 0.05, "rel=" + eU.toExponential(2));
    let eMax = 0;
    for (const mid of Object.keys(builtin.byMember)) {
      const bl = builtin.byMember[mid] || [], xl = (xa.ana.byMember[mid] || []);
      if (!xl.length) { check("杆件 M" + mid + " 回填", false, "缺失"); continue; }
      const keys = ["Vy", "Vz", "My", "Mz", "T"];
      const bA = bl.map((e) => e.axial || 0), xA = xl.map((e) => e.axial || 0);
      eMax = Math.max(eMax, relErr(xA, bA));
      keys.forEach((k) => {
        const bv = [], xv = [];
        bl.forEach((e) => e.forces.forEach((p) => bv.push(k === "Vy" ? p.S : k === "Mz" ? p.M : (p[k] || 0))));
        xl.forEach((e) => e.forces.forEach((p) => xv.push(k === "Vy" ? p.S : k === "Mz" ? p.M : (p[k] || 0))));
        if (bv.length === xv.length) eMax = Math.max(eMax, relErr(xv, bv));
      });
    }
    check("杆件 N/Vy/Vz/My/Mz/T 一致", eMax < 0.08, "rel=" + eMax.toExponential(2));
  }
}

// ---------- Case T2：Y 向双榀 + Y 向梁（双向框架，扭转耦合） ----------
console.log("## Case T2 双向空间框架（偏心荷载激起扭转）");
{
  const C30 = { label: "C30", E: 3.0e7, nu: 0.2 };
  const colS = { type: "rect", b: 0.4, h: 0.4 };
  const beamS = { type: "rect", b: 0.3, h: 0.6 };
  const FALL = { ux: 1, uy: 1, uz: 1, rx: 1, ry: 1, rz: 1 };
  const state = {
    D: { NSUB: 5, AMESH: 2 },
    model: {
      nodes: [
        { id: 1, x: 0, y: 0, z: 0, bc: FALL },
        { id: 2, x: 6, y: 0, z: 0, bc: FALL },
        { id: 3, x: 0, y: 5, z: 0, bc: FALL },
        { id: 4, x: 6, y: 5, z: 0, bc: FALL },
        { id: 5, x: 0, y: 0, z: 4, bc: {} },
        { id: 6, x: 6, y: 0, z: 4, bc: {} },
        { id: 7, x: 0, y: 5, z: 4, bc: {} },
        { id: 8, x: 6, y: 5, z: 4, bc: {} },
      ],
      members: [
        { id: 11, a: 1, b: 5, type: "col" },
        { id: 12, a: 2, b: 6, type: "col" },
        { id: 13, a: 3, b: 7, type: "col" },
        { id: 14, a: 4, b: 8, type: "col" },
        { id: 15, a: 5, b: 6, type: "beam" },
        { id: 16, a: 7, b: 8, type: "beam" },
        { id: 17, a: 5, b: 7, type: "beam" },
        { id: 18, a: 6, b: 8, type: "beam" },
      ],
      areas: [], cuts: [],
    },
    memStiff: {},
    memMat: { 11: C30, 12: C30, 13: C30, 14: C30, 15: C30, 16: C30, 17: C30, 18: C30 },
    memSec: { 11: colS, 12: colS, 13: colS, 14: colS, 15: beamS, 16: beamS, 17: beamS, 18: beamS },
    memLoads: { 15: [{ type: "udl", q: -10 }], 18: [{ type: "point", P: -20, a: 2.5 }] },
    nodeLoads: { 6: { fy: 12 }, 8: { fy: -12 } },
    areaMat: {}, areaSec: {}, areaLoads: {},
    memDiv: {},
    areaDiv: {},
  };
  const builtin = FrameSolver.solve(state);
  check("builtin ok", builtin.ok === true);
  check("builtin 走 3D", builtin.is3D === true, "maxT=" + (builtin.res.maxT || 0).toFixed(3));
  let xa;
  try { xa = runXara("true3d_t2", state); }
  catch (e) { check("xara 求解+回填", false, String(e.message || e).slice(0, 300)); }
  if (xa) {
    check("xara 求解+回填", true);
    check("路径 true3d", xa.meta && xa.meta.kind === "true3d", "kind=" + (xa.meta && xa.meta.kind));
    const du = [];
    for (let i = 0; i < builtin.all.length; i++) {
      for (let d = 0; d < 6; d++) du.push(xa.ana.u[6 * i + d] - builtin.u[6 * i + d]);
    }
    const eU = maxAbs(du) / Math.max(1e-12, maxAbs(builtin.u));
    check("节点位移一致", eU < 0.05, "rel=" + eU.toExponential(2));
    let eMax = 0;
    for (const mid of Object.keys(builtin.byMember)) {
      const bl = builtin.byMember[mid] || [], xl = (xa.ana.byMember[mid] || []);
      if (!xl.length) { check("杆件 M" + mid + " 回填", false, "缺失"); continue; }
      const bA = bl.map((e) => e.axial || 0), xA = xl.map((e) => e.axial || 0);
      eMax = Math.max(eMax, relErr(xA, bA));
      ["Vy", "Vz", "My", "Mz", "T"].forEach((k) => {
        const bv = [], xv = [];
        bl.forEach((e) => e.forces.forEach((p) => bv.push(k === "Vy" ? p.S : k === "Mz" ? p.M : (p[k] || 0))));
        xl.forEach((e) => e.forces.forEach((p) => xv.push(k === "Vy" ? p.S : k === "Mz" ? p.M : (p[k] || 0))));
        if (bv.length === xv.length) eMax = Math.max(eMax, relErr(xv, bv));
      });
    }
    check("杆件内力一致", eMax < 0.08, "rel=" + eMax.toExponential(2));
  }
}

if (nFail) { console.log("\nRESULT: " + nFail + " FAIL"); process.exit(1); }
else { console.log("\nRESULT: ALL PASS"); }
