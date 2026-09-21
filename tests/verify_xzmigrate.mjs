/* X-Z 统一迁移验证（Node 运行）：
 *  1. 新方案（x,z / ux,uz,ry / Fx,Fz,My / qx,qz / sx,sz,txz）悬臂梁解析值对照；
 *  2. 旧方案（x,y / ux,uy,rz / Fx,Fy,Mz）数值与新方案完全一致（求解器兼容）；
 *  3. 页面 applySnapshot 把旧快照迁移为新方案（节点 z、约束 uz/ry、荷载 fz/my、剖面 z0）；
 *  4. 简支梁均布解析值 Mmax=qL^2/8。
 * 运行：node tests/verify_xzmigrate.mjs
 */
import { readFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(HERE, "..");

function makeCtx() {
  const store = {};
  return new Proxy(store, {
    get(t, k) {
      if (k === "measureText") { return () => ({ width: 10 }); }
      if (k === "getImageData") { return () => ({ data: [] }); }
      if (k in t) { return t[k]; }
      return () => undefined;
    },
    set(t, k, v) { t[k] = v; return true; },
  });
}
function makeEl(id) {
  const el = {
    _id: id, value: "", checked: false, textContent: "", innerHTML: "", title: "",
    width: 800, height: 600, clientWidth: 800, clientHeight: 600,
    style: {}, dataset: {}, children: [], childNodes: [],
    parentNode: null, nextSibling: null,
    classList: { add() {}, remove() {}, toggle() {}, contains() { return false; } },
    _ls: {},
    addEventListener(ev, fn) { (this._ls[ev] = this._ls[ev] || []).push(fn); },
    removeEventListener() {},
    appendChild(c) { this.children.push(c); return c; },
    insertBefore(c) { this.children.push(c); return c; },
    remove() {},
    setAttribute() {}, getAttribute() { return null; },
    getElementsByTagName() { return []; },
    getBoundingClientRect() { return { left: 0, top: 0, width: 800, height: 600 }; },
    getContext() { return makeCtx(); },
    click() { (this._ls.click || []).forEach((f) => f({ target: this, preventDefault() {} })); },
    focus() {}, blur() {},
  };
  return el;
}
const elCache = {};
globalThis.window = globalThis;
globalThis.document = {
  readyState: "complete", title: "",
  body: makeEl("body"),
  getElementById(id) { return elCache[id] || (elCache[id] = makeEl(id)); },
  createElement(tag) { return makeEl(tag); },
  addEventListener() {}, removeEventListener() {},
};
globalThis.ResizeObserver = class { observe() {} unobserve() {} disconnect() {} };
globalThis.requestAnimationFrame = () => 1;
globalThis.cancelAnimationFrame = () => {};
globalThis.devicePixelRatio = 1;
globalThis.localStorage = { getItem: () => null, setItem() {}, removeItem() {} };
globalThis.alert = (m) => { throw new Error("unexpected alert: " + m); };
globalThis.addEventListener = () => {};
globalThis.removeEventListener = () => {};

function loadLib(rel) { (0, eval)(readFileSync(path.join(ROOT, rel), "utf8")); }
loadLib("design.js");
loadLib("solver.js");
{
  const html = readFileSync(path.join(ROOT, "index.html"), "utf8");
  const m = html.match(/<script>([\s\S]*?)<\/script>/);
  if (!m) { console.error("FAIL: index.html 内联脚本未找到"); process.exit(2); }
  (0, eval)(m[1]);
}
const { FrameSolver, FrameLab } = globalThis;
let nFail = 0;
function check(name, cond, detail) {
  if (cond) { console.log("  PASS " + name + (detail ? "  [" + detail + "]" : "")); }
  else { nFail++; console.log("  FAIL " + name + (detail ? "  [" + detail + "]" : "")); }
}

// ---- 1. 新方案悬臂梁：L=4, EI=150000, q=-10（局部+z，向下为负即 -Z） ----
// 解析：M固=qL^2/2=80，端部位移=qL^4/(8EI)=10*256/(8*150000)=0.0021333m=2.1333mm
function cantileverNew() {
  return {
    model: {
      nodes: [
        { id: 1, x: 0, z: 0, bc: { ux: true, uz: true, ry: true } },
        { id: 2, x: 4, z: 0, bc: {} },
      ],
      members: [{ id: 1, a: 1, b: 2, type: "beam" }], areas: [], cuts: [],
    },
    memStiff: { 1: { EA: 4.5e6, EI: 150000, GA: 1e12 } },
    memLoads: { 1: [{ type: "udl", q: -10 }] },
    nodeLoads: {}, areaLoads: {}, NSUB: 8, AMESH: 2,
  };
}
const anaNew = FrameSolver.solve(cantileverNew());
check("新方案求解 ok", anaNew.ok === true);
check("悬臂固端弯矩=80", Math.abs(anaNew.res.maxM - 80) / 80 < 1e-6, "maxM=" + anaNew.res.maxM.toFixed(4));
check("悬臂端部位移=2.1333mm", Math.abs(anaNew.res.maxDisp - 2.13333) / 2.13333 < 1e-3, "maxDisp=" + anaNew.res.maxDisp.toFixed(4) + "mm");

// ---- 2. 旧方案同模型数值一致 ----
function cantileverOld() {
  return {
    model: {
      nodes: [
        { id: 1, x: 0, y: 0, bc: { ux: true, uy: true, rz: true } },
        { id: 2, x: 4, y: 0, bc: {} },
      ],
      members: [{ id: 1, a: 1, b: 2, type: "beam" }], areas: [], cuts: [],
    },
    memStiff: { 1: { EA: 4.5e6, EI: 150000, GA: 1e12 } },
    memLoads: { 1: [{ type: "udl", q: -10 }] },
    nodeLoads: {}, areaLoads: {}, NSUB: 8, AMESH: 2,
  };
}
const anaOld = FrameSolver.solve(cantileverOld());
check("旧方案求解 ok", anaOld.ok === true);
check("新旧位移一致", Math.abs(anaNew.res.maxDisp - anaOld.res.maxDisp) < 1e-12);
check("新旧内力一致", Math.abs(anaNew.res.maxM - anaOld.res.maxM) < 1e-12);

// ---- 3. 面拉伸新旧一致 + 解析 σx=2000kPa ----
// 左缘 (x=0) 约 ux（节点1另约 uz 防刚体），右缘均布拉力 Total=σ·t·H=2000·0.2·1=400 → 每节点 200
function tensileNew() {
  return {
    model: {
      nodes: [
        { id: 1, x: 0, z: 0, bc: { ux: true, uz: true } },
        { id: 2, x: 2, z: 0, bc: {} },
        { id: 3, x: 2, z: 1, bc: {} },
        { id: 4, x: 0, z: 1, bc: { ux: true } },
      ],
      members: [], areas: [{ id: 1, nodes: [1, 2, 3, 4], etype: "plane" }], cuts: [],
    },
    areaMat: { 1: { label: "C30", E: 3e7, nu: 0.2 } },
    areaSec: { 1: { kind: "stress", t: 0.2 } },
    areaLoads: {}, memStiff: {}, memLoads: {},
    nodeLoads: { 2: { fx: 200, fz: 0, my: 0 }, 3: { fx: 200, fz: 0, my: 0 } },
    NSUB: 2, AMESH: 1,
  };
}
const anaT = FrameSolver.solve(tensileNew());
const sT = anaT.areaElems[0].s;
check("面拉伸 σx=2000", Math.abs(sT.sx - 2000) / 2000 < 1e-6, "sx=" + sT.sx.toFixed(2));
check("应力新键 sz/txz 存在", sT.sz !== undefined && sT.txz !== undefined, "sz=" + sT.sz + " txz=" + sT.txz);
check("膜内力 nz/nxz 存在", sT.nz !== undefined && sT.nxz !== undefined, "nz=" + sT.nz);
// 旧方案同模型对照
function tensileOld() {
  const o = tensileNew();
  o.model.nodes = [
    { id: 1, x: 0, y: 0, bc: { ux: true, uy: true } },
    { id: 2, x: 2, y: 0, bc: {} },
    { id: 3, x: 2, y: 1, bc: {} },
    { id: 4, x: 0, y: 1, bc: { ux: true } },
  ];
  o.nodeLoads = { 2: { fx: 200, fy: 0, mz: 0 }, 3: { fx: 200, fy: 0, mz: 0 } };
  return o;
}
const anaTO = FrameSolver.solve(tensileOld());
check("面拉伸新旧应力一致", Math.abs(anaTO.areaElems[0].s.sx - sT.sx) < 1e-9, "old=" + anaTO.areaElems[0].s.sx.toFixed(2) + " new=" + sT.sx.toFixed(2));

// ---- 4. 页面迁移：旧快照 openDocument ----
if (!FrameLab || !FrameLab.state) {
  check("页面 FrameLab 就绪", false);
} else {
  check("页面 FrameLab 就绪", true);
  const oldSnap = {
    app: "FrameLab", ver: 1, D: { NSUB: 8 },
    model: {
      nodes: [
        { id: 1, x: 0, y: 0, bc: { ux: true, uy: true, rz: true } },
        { id: 2, x: 4, y: 0, bc: {} },
      ],
      members: [{ id: 1, a: 1, b: 2, type: "beam" }], areas: [], cuts: [{ id: 0, x0: 0, y0: 0, x1: 4, y1: 0, cx: 2, cy: 0, cAuto: true }],
      nextNode: 2, nextMember: 2, nextArea: 0, nextCut: 1,
    },
    memStiff: { 1: { EA: 4.5e6, EI: 150000, GA: 1e12 } },
    memLoads: { 1: [{ type: "udl", q: -10 }] },
    nodeLoads: { 2: { fx: 0, fy: -5, mz: 3 } },
    areaMat: {}, areaSec: {}, areaLoads: {}, memDiv: {}, areaDiv: {}, memRebar: {}, areaRebar: {},
  };
  let st = null;
  try {
    if (FrameLab.openDoc) { FrameLab.openDoc("迁移测试", oldSnap); check("openDoc 接口", true); }
    else { check("openDoc 接口", false); }
    st = FrameLab.state();
  } catch (e) { check("旧快照打开", false, String(e.message || e).slice(0, 200)); }
  if (st) {
    check("旧快照打开", true);
    const n1 = st.model.nodes.find((n) => n.id === 1);
    check("节点 y→z", n1 && Math.abs(n1.z - 0) < 1e-12 && Math.abs(n1.y || 0) < 1e-12, JSON.stringify({ z: n1 && n1.z, y: n1 && n1.y }));
    check("约束 uy→uz、rz→ry", n1 && n1.bc.uz === true && n1.bc.ry === true, JSON.stringify(n1 && n1.bc));
    check("荷载 fy→fz、mz→my", st.nodeLoads[2] && st.nodeLoads[2].fz === -5 && st.nodeLoads[2].my === 3, JSON.stringify(st.nodeLoads[2]));
    const c0 = st.model.cuts[0];
    check("剖面 y→z", c0 && c0.z0 === 0 && c0.z1 === 0, JSON.stringify(c0));
    const ana = st.ana || FrameSolver.solve({ model: st.model, memStiff: st.memStiff, memLoads: st.memLoads, nodeLoads: st.nodeLoads, areaLoads: {}, NSUB: 8, AMESH: 2 });
    check("迁移后求解 ok", ana && ana.ok === true);
    if (ana && ana.ok) {
      // 等价新方案直接求解对照（含 udl -10 + 端点 fz=-5/my=3）
      const ref = FrameSolver.solve({
        model: {
          nodes: [
            { id: 1, x: 0, z: 0, bc: { ux: true, uz: true, ry: true } },
            { id: 2, x: 4, z: 0, bc: {} },
          ],
          members: [{ id: 1, a: 1, b: 2, type: "beam" }], areas: [], cuts: [],
        },
        memStiff: { 1: { EA: 4.5e6, EI: 150000, GA: 1e12 } },
        memLoads: { 1: [{ type: "udl", q: -10 }] },
        nodeLoads: { 2: { fx: 0, fz: -5, my: 3 } }, areaLoads: {}, NSUB: 8, AMESH: 2,
      });
      check("迁移后与新方案一致", Math.abs(ana.res.maxM - ref.res.maxM) < 1e-9, "maxM=" + ana.res.maxM.toFixed(3));
    }
  }
}

// ---- 5. 简支梁新方案解析值：L=6,q=-8跨中 M=36 ----
const anaSS = FrameSolver.solve({
  model: {
    nodes: [
      { id: 1, x: 0, z: 0, bc: { ux: true, uz: true, ry: false } },
      { id: 2, x: 6, z: 0, bc: { ux: false, uz: true, ry: false } },
    ],
    members: [{ id: 1, a: 1, b: 2, type: "beam" }], areas: [], cuts: [],
  },
  memStiff: { 1: { EA: 4.5e6, EI: 150000, GA: 1e12 } },
  memLoads: { 1: [{ type: "udl", q: -8 }] },
  nodeLoads: {}, areaLoads: {}, NSUB: 12, AMESH: 2,
});
check("简支梁 Mmax=36", Math.abs(anaSS.res.maxM - 36) / 36 < 1e-3, "maxM=" + anaSS.res.maxM.toFixed(3));

if (nFail > 0) { console.log("\nRESULT: FAIL (" + nFail + ")"); process.exit(1); }
console.log("\nRESULT: ALL PASS");
