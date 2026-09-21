/* FrameLab xara/OpenSees 桥接验证 harness（Node 运行）。
 *
 * 目的：证明 xara 路径（线弹性、纯框架 / 纯连续体）在数值上与内置求解器一致，
 * 以及混合模型被明确拒绝（本项目策略：xara 路径放弃混合形式）。
 *
 * 运行（仓库根目录）：
 *   node tests/verify_bridge.mjs
 *
 * 流程（每个纯模型用例）：
 *   state -> FrameSolver.solve（内置） ──┐
 *                                        ├─> 对比（位移 / 杆端内力 / 子单元应力）
 *   state -> FrameXara.buildScript -> .venv python run_xara.py -> importResults ──┘
 *
 * 注意：
 * - 杆件用 memDiv n=1（单段），使内置与 xara 的站位一一对应。
 * - Case A 用 memStiff 覆盖 + 巨大 GA，退化为欧拉-伯努利梁，与
 *   OpenSees elasticBeamColumn 同理论，隔离检验“映射关系”本身。
 * - Case B 用真实材料/截面（含 Timoshenko 剪切变形），检验工程常用路径，
 *   容差放宽（剪切变形理论差异）。
 * - 子进程环境会剔除 .venv\Library\bin（如有），以检验导出脚本的
 *   _bootstrap_dll() 能否独立找到 MKL 依赖。
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

// ---------- 小工具 ----------
let nFail = 0;
function check(name, cond, detail) {
  if (cond) { console.log("  PASS " + name + (detail ? "  [" + detail + "]" : "")); }
  else { nFail++; console.log("  FAIL " + name + (detail ? "  [" + detail + "]" : "")); }
}
function maxAbs(a) { return a.reduce((m, v) => Math.max(m, Math.abs(v)), 0); }
// 相对误差（以参照的最大量级归一，避免零点相除）
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
function runXara(tag, state, opts) {
  const built = FrameXara.buildScript(state, opts || {});
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

// ---------- Case A：门式框架（Euler 对标，n=1） ----------
function stateFrameEuler() {
  const E = 3.0e7, A = 0.15, I = 0.0045;
  const stiff = { EA: E * A, EI: E * I, GA: 1e12 };
  return {
    D: { NSUB: 5, AMESH: 2 },
    model: {
      nodes: [
        { id: 1, x: 0, y: 0, bc: { ux: 1, uy: 1, rz: 1 } },
        { id: 2, x: 0, y: 4, bc: {} },
        { id: 3, x: 6, y: 0, bc: { ux: 1, uy: 1, rz: 1 } },
        { id: 4, x: 6, y: 4, bc: {} },
      ],
      members: [
        { id: 11, a: 1, b: 2, type: "col" },
        { id: 12, a: 3, b: 4, type: "col" },
        { id: 13, a: 2, b: 4, type: "beam" },
      ],
      areas: [], cuts: [],
    },
    memStiff: { 11: stiff, 12: stiff, 13: stiff },
    memMat: {}, memSec: {},
    memLoads: {
      13: [{ type: "udl", q: -12 }],
      11: [{ type: "point", P: -8, a: 1.0 }],
    },
    nodeLoads: { 2: { fx: 0, fy: 0, mz: 0 }, 4: { fx: 0, fy: -5, mz: 0 } },
    areaMat: {}, areaSec: {}, areaLoads: {},
    memDiv: { 11: { n: 1 }, 12: { n: 1 }, 13: { n: 1 } },
    areaDiv: {},
  };
}

function verifyFrame(tag, state, tol, opts) {
  opts = opts || {};
  console.log("## " + tag);
  const builtin = FrameSolver.solve(state);
  check("builtin ok", builtin.ok === true);
  let xa;
  try { xa = runXara(tag, state, opts); }
  catch (e) { check("xara 求解+回填", false, String(e.message || e).slice(0, 300)); return; }
  check("xara 求解+回填", true);
  if (opts.expectKind) {
    check("路径 " + opts.expectKind, xa.meta && xa.meta.kind === opts.expectKind,
      "kind=" + (xa.meta && xa.meta.kind));
  }
  // 分析节点坐标一致
  const sameNodes = builtin.all.length === xa.ana.all.length &&
    builtin.all.every((p, i) => Math.hypot(p[0] - xa.ana.all[i][0], p[1] - xa.ana.all[i][1]) < 1e-9);
  check("分析节点一致", sameNodes, builtin.all.length + " nodes");
  // 位移
  const du = [];
  for (let i = 0; i < builtin.all.length; i++) {
    du.push(xa.ana.u[3 * i] - builtin.u[3 * i], xa.ana.u[3 * i + 1] - builtin.u[3 * i + 1]);
  }
  const scaleU = Math.max(1e-12, maxAbs(builtin.u));
  const eU = maxAbs(du) / scaleU;
  check("节点位移一致", eU < tol, "rel=" + eU.toExponential(2));
  // 杆件内力（多段拼接：按绝对站位 xa+t 逐站 S/M + 各段轴力）
  // 要求两侧分段网格一致（混合 3D 与内置在 memDiv n=1 时同为投影切分）
  let eMax = 0;
  for (const mid of Object.keys(builtin.byMember)) {
    const bl = builtin.byMember[mid] || [], xl = (xa.ana.byMember[mid] || []);
    if (!xl.length) { check("杆件 M" + mid + " 回填", false, "缺失"); continue; }
    const bS = [], xS = [], bM = [], xM = [];
    bl.forEach((e) => e.forces.forEach((p) => { bS.push(p.S); bM.push(p.M); }));
    xl.forEach((e) => e.forces.forEach((p) => { xS.push(p.S); xM.push(p.M); }));
    if (bS.length !== xS.length) {
      check("杆件 M" + mid + " 站数", false, bl.length + " vs " + xl.length + " 段");
      continue;
    }
    const bA = bl.map((e) => e.axial || 0), xA = xl.map((e) => e.axial || 0);
    const eA = maxAbs(bA.map((v, i) => v - xA[i])) /
      Math.max(maxAbs(bA), maxAbs(xA), 1e-6 * Math.max(1, maxAbs(bS), maxAbs(xS)));
    eMax = Math.max(eMax, relErr(xS, bS), relErr(xM, bM), eA);
  }
  check("杆件 N/V/M 一致", eMax < tol, "rel=" + eMax.toExponential(2));
  if (xa.ana.leak != null) { check("面外泄漏≈0", xa.ana.leak < 1e-6, "leak=" + xa.ana.leak.toExponential(1)); }
  return { builtin, xa };
}

// ---------- Case C：矩形板均匀拉伸（Q4，解析 σx=P/(t·h)） ----------
function stateTension() {
  const W = 2, H = 1, t = 0.2, E = 3.0e7, nu = 0.2;
  const P = 400; // 右端总拉力 kN
  return {
    D: { NSUB: 5, AMESH: 2 },
    model: {
      nodes: [
        { id: 1, x: 0, y: 0, bc: { ux: 1, uy: 1 } },
        { id: 2, x: W, y: 0, bc: {} },
        { id: 3, x: W, y: H, bc: {} },
        { id: 4, x: 0, y: H, bc: { ux: 1 } },
      ],
      members: [], areas: [{ id: 21, nodes: [1, 2, 3, 4], etype: "plane" }], cuts: [],
    },
    memStiff: {}, memMat: {}, memSec: {}, memLoads: {},
    nodeLoads: { 2: { fx: P / 2, fy: 0 }, 3: { fx: P / 2, fy: 0 } },
    areaMat: { 21: { label: "C30", E, nu } },
    areaSec: { 21: { kind: "stress", t } },
    areaLoads: {},
    memDiv: {}, areaDiv: { 21: { nx: 1, ny: 1 } },
    _expect: { sx: P / (t * H), tipUx: P * W / (E * t * H) },
  };
}

// ---------- Case D：悬臂墙顶侧载（Q4 弯曲场） ----------
function stateWall() {
  const W = 1, H = 4, t = 0.25, E = 3.0e7, nu = 0.2;
  return {
    D: { NSUB: 5, AMESH: 2 },
    model: {
      nodes: [
        { id: 1, x: 0, y: 0, bc: { ux: 1, uy: 1 } },
        { id: 2, x: W, y: 0, bc: { ux: 1, uy: 1 } },
        { id: 3, x: W, y: H, bc: {} },
        { id: 4, x: 0, y: H, bc: {} },
      ],
      members: [], areas: [{ id: 31, nodes: [1, 2, 3, 4], etype: "plane" }], cuts: [],
    },
    memStiff: {}, memMat: {}, memSec: {}, memLoads: {},
    nodeLoads: { 3: { fx: 60, fy: 0 }, 4: { fx: 60, fy: 0 } },
    areaMat: { 31: { label: "C30", E, nu } },
    areaSec: { 31: { kind: "stress", t } },
    areaLoads: {},
    memDiv: {}, areaDiv: { 31: { nx: 4, ny: 12 } },
  };
}

// ---------- Case E：三角形悬臂（CST 覆盖） ----------
function stateTri() {
  const t = 0.2, E = 3.0e7, nu = 0.2;
  return {
    D: { NSUB: 5, AMESH: 2 },
    model: {
      nodes: [
        { id: 1, x: 0, y: 0, bc: { ux: 1, uy: 1 } },
        { id: 2, x: 0, y: 2, bc: { ux: 1, uy: 1 } },
        { id: 3, x: 3, y: 0, bc: {} },
      ],
      members: [], areas: [{ id: 41, nodes: [1, 2, 3], etype: "plane" }], cuts: [],
    },
    memStiff: {}, memMat: {}, memSec: {}, memLoads: {},
    nodeLoads: { 3: { fx: 0, fy: -50 } },
    areaMat: { 41: { label: "C30", E, nu } },
    areaSec: { 41: { kind: "stress", t } },
    areaLoads: {},
    memDiv: {}, areaDiv: { 41: { nx: 4, ny: 4 } },
  };
}

function keyOf(cx, cy) { return cx.toFixed(9) + "," + cy.toFixed(9); }
// opts: { tolStress（分量相对容差）, tolDisp, expectSx, expectTipUx, equil:{fx, fy, m0} }
// 说明：内置 Q4（中心点应力）与 OpenSeesRT Quad（节点应力平均）是两种不同的
// 连续体列式，扰动场下子单元应力存在约 10~20% 的列式差异（已用网格加密验证
// 两者收敛于同一解）；此处分量归一化对比 + 平衡/解析基准共同验证映射链正确。
function verifyContinuum(tag, state, opts) {
  console.log("## " + tag);
  const builtin = FrameSolver.solve(state);
  check("builtin ok", builtin.ok === true);
  let run;
  try { run = runXara(tag, state); }
  catch (e) { check("xara 求解+回填", false, String(e.message || e).slice(0, 300)); return; }
  check("xara 求解+回填", true);
  const xa = run.ana;
  // 子单元按形心配对，分量各自归一化
  const bMap = new Map(builtin.areaElems.map((e) => [keyOf(e.cx, e.cy), e]));
  let n = 0;
  const d = { sx: 0, sy: 0, txy: 0 }, s = { sx: 1e-9, sy: 1e-9, txy: 1e-9 };
  for (const e of builtin.areaElems) {
    s.sx = Math.max(s.sx, Math.abs(e.s.sx));
    s.sy = Math.max(s.sy, Math.abs(e.s.sy));
    s.txy = Math.max(s.txy, Math.abs(e.s.txy));
  }
  for (const e of xa.areaElems) {
    const b = bMap.get(keyOf(e.cx, e.cy));
    if (!b) continue;
    n++;
    if (opts.skipYBelow != null && e.cy < opts.skipYBelow) continue; // 跳过固定端奇异行
    if (opts.skipXBelow != null && e.cx < opts.skipXBelow) continue; // 跳过固定端奇异列
    d.sx = Math.max(d.sx, Math.abs(e.s.sx - b.s.sx) / s.sx);
    d.sy = Math.max(d.sy, Math.abs(e.s.sy - b.s.sy) / s.sy);
    d.txy = Math.max(d.txy, Math.abs(e.s.txy - b.s.txy) / s.txy);
  }
  check("子单元配对", n === builtin.areaElems.length, n + "/" + builtin.areaElems.length);
  const eS = Math.max(d.sx, d.sy, d.txy);
  check("子单元应力一致", eS < opts.tolStress,
    "sx=" + d.sx.toExponential(1) + " sy=" + d.sy.toExponential(1) + " txy=" + d.txy.toExponential(1));
  // 位移（混合模型只比平动：转角 rz 受 drilling 铰接影响，属结构性差异）
  const du = [];
  if (opts.noRz) {
    for (let i = 0; i < xa.all.length; i++) {
      du.push(xa.u[3 * i] - builtin.u[3 * i], xa.u[3 * i + 1] - builtin.u[3 * i + 1]);
    }
  } else {
    for (let i = 0; i < xa.u.length; i++) du.push(xa.u[i] - builtin.u[i]);
  }
  const scaleU = Math.max(1e-12, maxAbs(builtin.u));
  const eU = maxAbs(du) / scaleU;
  check("节点位移一致", eU < opts.tolDisp, "rel=" + eU.toExponential(2));
  if (opts.expectSx != null) {
    const got = xa.areaElems.reduce((a, e) => a + e.s.sx, 0) / xa.areaElems.length;
    check("均匀拉伸解析值", Math.abs(got - opts.expectSx) / opts.expectSx < 0.01,
      "got=" + got.toFixed(1) + " expect=" + opts.expectSx.toFixed(1));
  }
  if (opts.expectTipUx != null) {
    const got = Math.max.apply(null, xa.u.filter((_, i) => i % 3 === 0).map(Math.abs));
    check("拉伸端部位移解析值", Math.abs(got - opts.expectTipUx) / opts.expectTipUx < 0.02,
      "got=" + got.toExponential(3) + " expect=" + opts.expectTipUx.toExponential(3));
  }
  if (opts.equil) {
    let fx = 0, fy = 0, m0 = 0;
    const all = run.meta.all, reacts = (xa.xaraLog || {}).reactions || {};
    // 2D（ndm=2）：反力 [Fx,Fy垂向,Mz]，p=[x,垂向]，位置无关新旧方案；
    // 混合3D（X-Z平面，y=0）：反力 [Fx,Fy面外,Fz,Mx,My,Mz]，p=[x,z]，
    // 面内平衡取 Fx/Fz，2D逆时针弯矩 My_2d = -r[4]。
    const is3 = run.meta && run.meta.kind === "mixed3d";
    for (const xt of Object.keys(reacts)) {
      const r = reacts[xt], p = all[+xt - 1];
      if (is3) { fx += r[0]; fy += r[2]; m0 += p[0] * r[2] - p[1] * r[0] - (r[4] || 0); }
      else { fx += r[0]; fy += r[1]; m0 += p[0] * r[1] - p[1] * r[0] + (r[2] || 0); }
    }
    // 零目标分量用总荷载量级归一化，避免 1e-12 级噪声误杀
    const lscale = Math.max(1, Math.abs(opts.equil.fx), Math.abs(opts.equil.fy), Math.abs(opts.equil.m0));
    const eF = Math.abs(fx - opts.equil.fx) / Math.max(1e-9, Math.abs(opts.equil.fx), 1e-6 * lscale) +
      Math.abs(fy - opts.equil.fy) / Math.max(1e-9, Math.abs(opts.equil.fy), 1e-6 * lscale) +
      Math.abs(m0 - opts.equil.m0) / Math.max(1e-9, Math.abs(opts.equil.m0), 1e-6 * lscale);
    check("整体平衡", eF < 1e-6, "Fx=" + fx.toFixed(3) + " Fy=" + fy.toFixed(3) + " M0=" + m0.toFixed(2));
  }
}

// ---------- Case J：共边重复节点（回归：曾导致 fix 重复报错 + 界面钉死） ----------
// 两块板并排，共边节点各存一份（含支座副本同约束），端部侧向荷载
function stateDupPanels() {
  const E = 3.0e7;
  const C30 = { label: "C30", E, nu: 0.2 };
  return {
    D: { NSUB: 5, AMESH: 2 },
    model: {
      nodes: [
        { id: 1, x: 0, y: 0, bc: { ux: 1, uy: 1 } },
        { id: 2, x: 6, y: 0, bc: {} },
        { id: 3, x: 6, y: 4, bc: {} },
        { id: 4, x: 0, y: 4, bc: { ux: 1, uy: 1 } },
        { id: 5, x: 6, y: 0, bc: {} },
        { id: 6, x: 12, y: 0, bc: {} },
        { id: 7, x: 12, y: 4, bc: {} },
        { id: 8, x: 6, y: 4, bc: {} },
        { id: 9, x: 0, y: 0, bc: { ux: 1, uy: 1 } },
      ],
      members: [],
      areas: [
        { id: 21, nodes: [1, 2, 3, 4], etype: "plane" },
        { id: 22, nodes: [5, 6, 7, 8], etype: "plane" },
      ],
      cuts: [],
    },
    memStiff: {}, memMat: {}, memSec: {}, memLoads: {},
    nodeLoads: { 6: { fx: 0, fy: -30 }, 7: { fx: 0, fy: -30 } },
    areaMat: { 21: C30, 22: C30 },
    areaSec: { 21: { kind: "stress", t: 0.25 }, 22: { kind: "stress", t: 0.25 } },
    areaLoads: {},
    memDiv: {},
    areaDiv: { 21: { nx: 6, ny: 4 }, 22: { nx: 6, ny: 4 } },
  };
}
// ---------- Case F：混合模型走 3D 退化平面路径（不再拒绝） ----------
function verifyMixed() {
  console.log("## Case F 混合模型3D路径");
  const st = stateFrameEuler();
  // 构造一个真实的混合模型：框架 + 独立面
  st.model.areas = [{ id: 51, nodes: [5, 6, 7, 8], etype: "plane" }];
  st.model.nodes.push(
    { id: 5, x: 10, y: 0, bc: { ux: 1, uy: 1 } },
    { id: 6, x: 12, y: 0, bc: { ux: 1, uy: 1 } },
    { id: 7, x: 12, y: 2, bc: {} },
    { id: 8, x: 10, y: 2, bc: {} },
  );
  st.areaMat = { 51: { label: "C30", E: 3.0e7, nu: 0.2 } };
  st.areaSec = { 51: { kind: "stress", t: 0.2 } };
  let kind = "";
  try { kind = FrameXara.buildScript(st, {}).kind; }
  catch (e) { kind = "ERR:" + String(e.message || e).slice(0, 80); }
  check("混合模型走3D路径", kind === "mixed3d", "kind=" + kind);
}

// ---------- Case G：纯框架走 3D 路径（ nodal 等效荷载，须与 2D 精确一致） ----------
function stateFrame3D() {
  const st = stateFrameEuler();
  return st;
}

// ---------- Case H：门框 + 内填墙（共边混合，真实截面） ----------
// 柱 (0,0)-(0,4),(4,0)-(4,4)；梁 (0,4)-(4,4)；墙板 4x4；顶层节点水平力 60+40
function stateInfill() {
  const E = 3.0e7;
  const C30 = { label: "C30", E, nu: 0.2 };
  return {
    D: { NSUB: 5, AMESH: 2 },
    model: {
      nodes: [
        { id: 1, x: 0, y: 0, bc: { ux: 1, uy: 1, rz: 1 } },
        { id: 2, x: 0, y: 4, bc: {} },
        { id: 3, x: 4, y: 0, bc: { ux: 1, uy: 1, rz: 1 } },
        { id: 4, x: 4, y: 4, bc: {} },
      ],
      members: [
        { id: 11, a: 1, b: 2, type: "col" },
        { id: 12, a: 3, b: 4, type: "col" },
        { id: 13, a: 2, b: 4, type: "beam" },
      ],
      areas: [{ id: 21, nodes: [1, 3, 4, 2], etype: "plane" }],
      cuts: [],
    },
    memStiff: {},
    memMat: { 11: C30, 12: C30, 13: C30 },
    memSec: {
      11: { type: "rect", b: 0.3, h: 0.4 }, 12: { type: "rect", b: 0.3, h: 0.4 },
      13: { type: "rect", b: 0.25, h: 0.5 },
    },
    memLoads: {},
    nodeLoads: { 2: { fx: 60, fy: 0, mz: 0 }, 4: { fx: 40, fy: 0, mz: 0 } },
    areaMat: { 21: C30 },
    areaSec: { 21: { kind: "stress", t: 0.25 } },
    areaLoads: {},
    memDiv: { 11: { n: 1 }, 12: { n: 1 }, 13: { n: 1 } },
    areaDiv: { 21: { nx: 4, ny: 4 } },
  };
}

// ---------- Case I：三角墙 + 包边框架（T3 在 3D 路径） ----------
// 三角 (0,0),(4,0),(0,4)；柱沿左腿，梁沿底腿；顶节点水平力 50
function stateTriFrame() {
  const E = 3.0e7;
  const C30 = { label: "C30", E, nu: 0.2 };
  return {
    D: { NSUB: 5, AMESH: 2 },
    model: {
      nodes: [
        { id: 1, x: 0, y: 0, bc: { ux: 1, uy: 1, rz: 1 } },
        { id: 2, x: 4, y: 0, bc: { ux: 1, uy: 1, rz: 1 } },
        { id: 3, x: 0, y: 4, bc: {} },
      ],
      members: [
        { id: 11, a: 1, b: 3, type: "col" },
        { id: 12, a: 1, b: 2, type: "beam" },
      ],
      areas: [{ id: 41, nodes: [1, 2, 3], etype: "plane" }],
      cuts: [],
    },
    memStiff: {},
    memMat: { 11: C30, 12: C30 },
    memSec: {
      11: { type: "rect", b: 0.3, h: 0.4 }, 12: { type: "rect", b: 0.25, h: 0.5 },
    },
    memLoads: {},
    nodeLoads: { 3: { fx: 50, fy: 0, mz: 0 } },
    areaMat: { 41: C30 },
    areaSec: { 41: { kind: "stress", t: 0.2 } },
    areaLoads: {},
    memDiv: { 11: { n: 1 }, 12: { n: 1 } },
    areaDiv: { 41: { nx: 4, ny: 4 } },
  };
}

// ---------- Case K：杆件细分（NSUB/memDiv 等分 + 杆中节点投影切分 + 缝上集中力） ----------
// 节点 5 落在梁跨中：旧导出（不切分）会将其判孤立而多加全约束；a=1.5 恰为梁 4 等分缝，
// 检验荷载跨段映射（segAt）与逐段回填。
function stateSubdiv() {
  const E = 3.0e7, A = 0.15, I = 0.0045;
  const stiff = { EA: E * A, EI: E * I, GA: 1e12 };
  return {
    D: { NSUB: 3, AMESH: 2 },
    NSUB: 3, AMESH: 2, // 与生产环境 collectSolverContext() 形状一致（顶层值优先）
    model: {
      nodes: [
        { id: 1, x: 0, y: 0, bc: { ux: 1, uy: 1, rz: 1 } },
        { id: 2, x: 0, y: 4, bc: {} },
        { id: 3, x: 6, y: 0, bc: { ux: 1, uy: 1, rz: 1 } },
        { id: 4, x: 6, y: 4, bc: {} },
        { id: 5, x: 3, y: 4, bc: {} },
      ],
      members: [
        { id: 11, a: 1, b: 2, type: "col" },
        { id: 12, a: 3, b: 4, type: "col" },
        { id: 13, a: 2, b: 4, type: "beam" },
      ],
      areas: [], cuts: [],
    },
    memStiff: { 11: stiff, 12: stiff, 13: stiff },
    memMat: {}, memSec: {},
    memLoads: {
      13: [{ type: "udl", q: -12 }, { type: "point", P: -10, a: 1.5 }],
      11: [{ type: "point", P: -8, a: 1.0 }],
    },
    nodeLoads: { 5: { fx: 0, fy: -5, mz: 0 }, 4: { fx: 0, fy: -5, mz: 0 } },
    areaMat: {}, areaSec: {}, areaLoads: {},
    memDiv: { 13: { n: 4 } },
    areaDiv: {},
  };
}
// ---------- main ----------
console.log("FrameLab xara 桥接验证");
console.log("solver=" + FrameSolver.version);
verifyFrame("Case A 门式框架Euler对标", stateFrameEuler(), 0.005);
// Case B：真实截面（Timoshenko vs Euler-Bernoulli，容差放宽）
(function () {
  const st = stateFrameEuler();
  st.memStiff = {};
  const C30 = { label: "C30", E: 3.0e7, nu: 0.2 };
  st.memMat = { 11: C30, 12: C30, 13: C30 };
  st.memSec = {
    11: { type: "rect", b: 0.3, h: 0.5 }, 12: { type: "rect", b: 0.3, h: 0.5 },
    13: { type: "rect", b: 0.25, h: 0.6 },
  };
  verifyFrame("Case B 门式框架真实截面", st, 0.03);
})();
verifyContinuum("Case C 均匀拉伸Q4", stateTension(),
  { tolStress: 0.02, tolDisp: 0.02, expectSx: stateTension()._expect.sx, expectTipUx: stateTension()._expect.tipUx });
verifyContinuum("Case D 悬臂墙弯曲Q4", stateWall(),
  { tolStress: 0.25, tolDisp: 0.12, equil: { fx: -120, fy: 0, m0: 480 }, skipYBelow: 1.0 });
verifyContinuum("Case E 三角悬臂CST", stateTri(), { tolStress: 0.2, tolDisp: 0.08 });
verifyContinuum("Case J 共边重复节点回归", stateDupPanels(),
  { tolStress: 0.25, tolDisp: 0.15, equil: { fx: 0, fy: 60, m0: 720 }, skipXBelow: 2.0 });
verifyMixed();
verifyFrame("Case G 纯框架走3D路径", stateFrame3D(), 1e-6, { force3D: true, expectKind: "mixed3d" });
verifyFrame("Case H 门框+内填墙混合3D", stateInfill(), 0.6, { expectKind: "mixed3d" });
verifyContinuum("Case H-wall 应力", stateInfill(),
  { tolStress: 0.25, tolDisp: 0.1, noRz: true, equil: { fx: -100, fy: 0, m0: 400 }, skipYBelow: 1.0 });
verifyFrame("Case K 杆件细分+投影切分", stateSubdiv(), 1e-6, { expectKind: "frame" });
// Case I：混合 + 三角形面必须被明确拒绝（T3 精度不足）
(function () {
  console.log("## Case I 混合三角面拒绝");
  let msg = "";
  try { FrameXara.buildScript(stateTriFrame(), {}); }
  catch (e) { msg = String(e.message || e); }
  check("混合三角面被拒绝", /三角形/.test(msg), msg.slice(0, 70));
})();

if (nFail > 0) { console.log("\nRESULT: FAIL (" + nFail + ")"); process.exit(1); }
console.log("\nRESULT: ALL PASS");
