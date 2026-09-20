/* FrameLab 3D 细分显示回归测试（Node 运行）。
 *
 * 运行（仓库根目录）：
 *   node tests/verify_3dsub.mjs
 *
 * 方法：用最小 DOM 桩加载真实的 index.html 内联脚本（与 solver.js 一起 eval，
 * 同 verify_phymesh.mjs），搭单柱悬臂 3D 模型 + 顶层楼面壳，驱动真实 rebuild/draw。
 * 画布 ctx 换成记录式桩，按“主窗 cv 上的绘制操作”断言。
 *
 * 覆盖的 bug：
 *  1. 位移云图按整杆两端模型节点均值着色，细分中间节点位移不参与显示
 *     （3D 只读端点，2D 按子段两端分析节点着色）→ 现按子段逐段着色；
 *  2. 3D 下杆件细分分割点无任何标记 → 现主窗画暗色小点（与白色模型节点区分）；
 *  3. 3D 下壳永远选不中（hit.areas恒空、pickInView3无面拾取、选择分支无面）
 *     → 现面多边形拾取（含切片窗过滤）；
 *  4. 壳细分线/面收缩：共面壳 areaElems 存在即绘制（含收缩），此处断言数量链路。
 */
import { readFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(HERE, "..");

// ---------- 记录式 DOM 桩 ----------
const drawOps = []; // {el, op, ...}
function makeCtx(elId) {
  const store = {};
  return new Proxy(store, {
    get(t, k) {
      if (k === "measureText") { return () => ({ width: 10 }); }
      if (k === "getImageData") { return () => ({ data: [] }); }
      if (k === "arc") {
        return (x, y, r) => { drawOps.push({ el: elId, op: "arc", r, fill: t.fillStyle, stroke: t.strokeStyle }); };
      }
      if (k in t) { return t[k]; }
      return () => undefined; // 吸收一切绘图调用
    },
    set(t, k, v) {
      t[k] = v;
      if (k === "strokeStyle" || k === "fillStyle") { drawOps.push({ el: elId, op: "style", k, v: String(v) }); }
      return true;
    },
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
    getContext() { return makeCtx(id); },
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

// ---------- 加载真实脚本 ----------
function loadLib(rel) { (0, eval)(readFileSync(path.join(ROOT, rel), "utf8")); }
loadLib("design.js");
loadLib("solver.js");
{
  const html = readFileSync(path.join(ROOT, "index.html"), "utf8");
  const m = html.match(/<script>([\s\S]*?)<\/script>/);
  if (!m) { console.error("FAIL: 内联脚本未找到"); process.exit(2); }
  (0, eval)(m[1]);
}
const FL = globalThis.window.FrameLab;
if (!FL) { console.error("FAIL: FrameLab 未加载"); process.exit(2); }
if (!FL.pick3D) { console.error("FAIL: FrameLab.pick3D 未暴露"); process.exit(2); }

let nFail = 0;
function check(name, cond, detail) {
  if (cond) { console.log("  PASS " + name + (detail ? "  [" + detail + "]" : "")); }
  else { nFail++; console.log("  FAIL " + name + (detail ? "  [" + detail + "]" : "")); }
}

// ---------- 搭 3D 单柱悬臂（NSUB=5，顶端侧向荷载 → 三次挠度曲线） ----------
let st = FL.state();
st.model.nodes.length = 0; st.model.members.length = 0;
st.model.areas.length = 0; st.model.cuts.length = 0;
st.model.nextNode = 0; st.model.nextMember = 0; st.model.nextArea = 0;
[st.memDiv, st.areaDiv, st.memLoads, st.nodeLoads, st.memStiff, st.memMat,
  st.memSec, st.areaMat, st.areaSec, st.areaLoads].forEach((o) => {
  Object.keys(o).forEach((k) => delete o[k]);
});
st.D.dim = "3d"; st.D.NSUB = 5; st.D.AMESH = 2;
st.D.view = "disp"; st.D.mag = 0; st.D.dispVar = "mag";
st.D.winAssign = ["persp", "plan", "secX", "secY"];
st.D.planeZ = 4; st.D.secY0 = 0; st.D.secX0 = 0;
st.D.showAreaShrink = false;
const FALL = { ux: true, uy: true, uz: true, rx: true, ry: true, rz: true };
const FREE = { ux: false, uy: false, uz: false, rx: false, ry: false, rz: false };
function addN(x, y, z, bc) { const n = { id: st.model.nextNode++, x, y, z, bc }; st.model.nodes.push(n); return n; }
const nBase = addN(0, 0, 0, FALL), nTop = addN(0, 0, 4, FREE);
st.model.members.push({ id: st.model.nextMember++, a: nBase.id, b: nTop.id, type: "col" });
st.nodeLoads[nTop.id] = { fx: 50, fy: 0, fz: 0, mx: 0, my: 0, mz: 0 };
drawOps.length = 0;
FL.rebuild();
st = FL.state();

console.log("## Case 1 杆位移云图读入细分数据（逐子段着色）");
check("内置 3D 求解", st.ana && st.ana.is3D === true && st.ana.ok === true,
  "elems=" + ((st.ana && st.ana.elems) || []).length);
check("单杆按 NSUB=5 切为 5 子段",
  st.ana && st.ana.byMember && st.ana.byMember[0] && st.ana.byMember[0].length === 5,
  "segs=" + ((st.ana.byMember || {})[0] || []).length);
const rgbStrokes = new Set(
  drawOps.filter((o) => o.el === "cv" && o.op === "style" && o.k === "strokeStyle" && /^rgb\(/.test(o.v)).map((o) => o.v)
);
check("主窗云图出现 ≥3 种杆段颜色（旧逻辑恒为 1 种整杆色）",
  rgbStrokes.size >= 3, "distinct=" + rgbStrokes.size);

console.log("## Case 2 细分分割点（暗黄色小点，仅几何视图）");
st.D.view = "model";
drawOps.length = 0;
FL.rebuild();
st = FL.state();
const DOTFILL = "rgba(140,110,0,0.92)";
const dotArcs = drawOps.filter((o) => o.el === "cv" && o.op === "arc" && o.r === 2 && o.fill === DOTFILL);
check("几何视图下 4 个中间细分点各绘制（每点被相邻两子段各画一次，共 8 个圆）",
  dotArcs.length === 8, "dotArcs=" + dotArcs.length);
drawOps.length = 0;
st.D.view = "disp";
FL.rebuild();
st = FL.state();
check("位移视图下细分点不再绘制",
  drawOps.filter((o) => o.el === "cv" && o.op === "arc" && o.r === 2 && o.fill === DOTFILL).length === 0,
  "dotArcs=" + drawOps.filter((o) => o.el === "cv" && o.op === "arc" && o.r === 2 && o.fill === DOTFILL).length);

// ---------- 加顶层楼面壳（共面 XY@z=4，参与内置 3D） + areaDiv 3x2 ----------
console.log("## Case 3/4 壳选中与细分线");
const n1 = addN(6, 0, 4, FREE), n2 = addN(6, 6, 4, FREE), n3 = addN(0, 6, 4, FREE);
st.model.areas.push({ id: st.model.nextArea++, nodes: [nTop.id, n1.id, n2.id, n3.id], etype: "shell" });
const shellId = st.model.areas[0].id;
st.areaDiv[shellId] = { nx: 3, ny: 2, hmax: 0 };
drawOps.length = 0;
FL.rebuild();
st = FL.state();
check("楼面壳 3x2 剖分为 6 子单元",
  st.ana && st.ana.areaElems && st.ana.areaElems.length === 6,
  "areaElems=" + ((st.ana && st.ana.areaElems) || []).length);
// 壳形心投影到主窗像素，调真实拾取函数
function proj3(kind, x, y, z) {
  if (kind === "plan") { return [x, y]; }
  if (kind === "secX") { return [x, z]; }
  if (kind === "secY") { return [y, z]; }
  const c = Math.cos(Math.PI / 6), s = Math.sin(Math.PI / 6);
  return [(x - y) * c, z + (x + y) * s * 0.5];
}
const v = globalThis.window.__view3d && globalThis.window.__view3d.cv;
check("主窗视图参数已生成", !!v, v ? v.kind : "none");
if (v) {
  const cam = (st.D.cam3 || {})[v.win] || { z: 1, px: 0, py: 0 };
  const z = Math.min(25, Math.max(0.15, cam.z || 1));
  const xb0 = (v.cw - (v.X1 - v.X0) * v.bs0) / 2, yb0 = (v.ch - (v.Y1 - v.Y0) * v.bs0) / 2;
  const SX = (u) => v.cw / 2 + (xb0 + (u - v.X0) * v.bs0 - v.cw / 2) * z + (cam.px || 0);
  const SY = (w) => v.ch / 2 + (yb0 + (v.Y1 - w) * v.bs0 - v.ch / 2) * z + (cam.py || 0);
  const pr = proj3(v.kind, 3, 3, 4);
  const hit = FL.pick3D("cv", SX(pr[0]), SY(pr[1]));
  check("壳形心点中壳（真实 pickInView3）", hit && hit.area === shellId, JSON.stringify(hit));
  const miss = FL.pick3D("cv", -1000, -1000);
  check("空白处点空", miss && miss.area == null && miss.node == null && miss.member == null, JSON.stringify(miss));
  check("selectArea 链路", FL.selectArea(shellId) === shellId);
}
// 面收缩开起重画不报错，细分线仍在
st.D.showAreaShrink = true;
drawOps.length = 0;
FL.rebuild();
st = FL.state();
check("收缩显示下仍 6 子单元", st.ana && st.ana.areaElems && st.ana.areaElems.length === 6);

if (nFail > 0) { console.log("\nRESULT: FAIL (" + nFail + ")"); process.exit(1); }
console.log("\nRESULT: ALL PASS");
