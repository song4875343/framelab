/* 3D 模型在 2D 画布下按 X-Z 平面显示（回归）。
 * 画布录制 moveTo/lineTo：X-Z 投影中柱应为竖向长段、梁为横向长段；
 * 旧逻辑（X-Y）下柱会塌成点、此处应无竖向长段。
 * 运行：node tests/verify_2dof3d.mjs
 */
import { readFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(HERE, "..");
const segs = [];
const trash = [];
function makeCtx(sink) {
  const store = {};
  return new Proxy(store, {
    get(t, k) {
      if (k === "moveTo") { return (x, y) => { t._cur = [x, y]; }; }
      if (k === "lineTo") {
        return (x, y) => {
          if (t._cur) { sink.push([t._cur, [x, y]]); }
          t._cur = [x, y];
        };
      }
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
    getContext() { return makeCtx(trash); },
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
  // 只有主画布 cv 录制路径，其余画布静默（色标条/内力小图会污染断言）
  getElementById(id) {
    if (!elCache[id]) {
      elCache[id] = makeEl(id);
      if (id === "cv") { elCache[id].getContext = () => makeCtx(segs); }
    }
    return elCache[id];
  },
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
  (0, eval)(html.match(/<script>([\s\S]*?)<\/script>/)[1]);
}
const { FrameLab } = globalThis;
let nFail = 0;
function check(name, cond, detail) {
  if (cond) { console.log("  PASS " + name + (detail ? "  [" + detail + "]" : "")); }
  else { nFail++; console.log("  FAIL " + name + (detail ? "  [" + detail + "]" : "")); }
}

// 3D 双榀框架（y=0 与 y=4 各一榀，柱高 4m，跨 6m），柱底全约束
function snap3D() {
  const nodes = [];
  let id = 1;
  const at = {};
  for (const y of [0, 4]) {
    for (const [x, z] of [[0, 0], [6, 0], [0, 4], [6, 4]]) {
      at[x + "," + y + "," + z] = id;
      nodes.push({ id: id++, x, y, z, bc: z === 0 ? { ux: 1, uy: 1, uz: 1, rx: 1, ry: 1, rz: 1 } : {} });
    }
  }
  const M = (a, b, type) => ({ id: M._i = (M._i || 0) + 1, a: at[a], b: at[b], type });
  const members = [
    M("0,0,0", "0,0,4", "col"), M("6,0,0", "6,0,4", "col"), M("0,0,4", "6,0,4", "beam"),
    M("0,4,0", "0,4,4", "col"), M("6,4,0", "6,4,4", "col"), M("0,4,4", "6,4,4", "beam"),
  ];
  const memStiff = {};
  members.forEach((m) => { memStiff[m.id] = { EA: 9e6, EI: 1e5, GA: 1e9 }; });
  return {
    app: "FrameLab", ver: 2, D: { view: "model", NSUB: 5, AMESH: 2 },
    model: { nodes, members, areas: [], cuts: [], nextNode: id, nextMember: 99, nextArea: 0, nextCut: 0 },
    memStiff, memMat: {}, memSec: {}, memLoads: {},
    nodeLoads: { [at["6,0,4"]]: { fx: 10, fy: 0, fz: 0, mx: 0, my: 0, mz: 0 } },
    areaMat: {}, areaSec: {}, areaLoads: {}, memDiv: {}, areaDiv: {}, memRebar: {}, areaRebar: {},
  };
}

segs.length = 0; // 清掉页面初始化 demo 的绘制
FrameLab.openDoc("3d-in-2d", snap3D());
const st = FrameLab.state();
const ana = st.ana;
check("3D 求解 ok", ana && ana.ok === true);
const stride = (ana.u.length >= ana.all.length * 6 - 1e-9) ? 6 : 3;
check("3D stride=6", stride === 6, "dof=" + stride);
check("all 为平面二元组", ana.all.every((p) => p.length === 2));
check("all3 为三元组", ana.all3 && ana.all3.every((p) => p.length === 3));

const long = segs.filter(([a, b]) => Math.hypot(b[0] - a[0], b[1] - a[1]) > 30);
const vert = long.filter(([a, b]) => Math.abs(b[0] - a[0]) < 5 && Math.abs(b[1] - a[1]) > 30);
const horiz = long.filter(([a, b]) => Math.abs(b[1] - a[1]) < 5 && Math.abs(b[0] - a[0]) > 30);
check("柱画成竖向长段", vert.length > 0, "vert=" + vert.length + " long=" + long.length);
check("梁画成横向长段", horiz.length > 0, "horiz=" + horiz.length);
// 两榀投影重合：横向长段应基本在同一高度（均为 z=4 顶梁）
if (horiz.length) {
  const ys = horiz.map(([a, b]) => (a[1] + b[1]) / 2);
  const spread = Math.max(...ys) - Math.min(...ys);
  check("顶梁投影重合", spread < 8, "spread=" + spread.toFixed(1) + "px");
}

if (nFail > 0) { console.log("\nRESULT: FAIL (" + nFail + ")"); process.exit(1); }
console.log("\nRESULT: ALL PASS");
