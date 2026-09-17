/* FrameLab 物理分割回归测试（Node 运行）。
 *
 * 运行（仓库根目录）：
 *   node tests/verify_phymesh.mjs
 *
 * 方法：用最小 DOM 桩加载真实的 index.html 内联脚本（与 solver.js 一起 eval），
 * 然后驱动真实 UI 链路：FrameLab.demo/demoArea → selectMember/selectArea →
 * 填细分面板参数 → 触发 dv-mesh / dv-amesh 按钮的真实 click handler。
 *
 * 覆盖的 bug：
 *  1. 物理分割无视面板参数（恒用全局 NSUB=5，“最多分 5 个”）；
 *  2. 分割一次后子段 memDiv n=1 导致再次点击无反应（“分完不能再分”）。
 */
import { readFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(HERE, "..");

// ---------- 最小 DOM 桩 ----------
function makeCtx() {
  const store = {};
  return new Proxy(store, {
    get(t, k) {
      if (k === "measureText") { return () => ({ width: 10 }); }
      if (k === "getImageData") { return () => ({ data: [] }); }
      if (k in t) { return t[k]; }
      return () => undefined; // 吸收一切绘图调用
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

function $(id) { return elCache[id] || (elCache[id] = makeEl(id)); }
function fireClick(id) { $(id).click(); }
// ---------- 断言 ----------
let nFail = 0;
function check(name, cond, detail) {
  if (cond) { console.log("  PASS " + name + (detail ? "  [" + detail + "]" : "")); }
  else { nFail++; console.log("  FAIL " + name + (detail ? "  [" + detail + "]" : "")); }
}

// ---------- Case 1：杆件物理分割采用面板参数 ----------
console.log("## Case 1 杆件物理分割用面板参数");
FL.demo();
let st = FL.state();
const targetMid = st.model.members[0].id;
const nBefore = st.model.members.length;
FL.selectMember(targetMid);
$("dv-n").value = "10"; $("dv-lmax").value = "0";
fireClick("dv-mesh");
st = FL.state();
check("面板 n=10 得 10 段（而非全局 NSUB=5）",
  st.model.members.length === nBefore - 1 + 10, nBefore + " -> " + st.model.members.length);
check("原杆件被替换", !st.model.members.some((m) => m.id === targetMid));

// ---------- Case 2：分割后可再次分割 ----------
console.log("## Case 2 杆件可重复物理分割");
const someChild = st.model.members.find((m) => m.id !== targetMid).id;
FL.selectMember(someChild);
$("dv-n").value = "4"; $("dv-lmax").value = "0";
const nBefore2 = FL.state().model.members.length;
fireClick("dv-mesh");
st = FL.state();
check("子段再次 4 等分", st.model.members.length === nBefore2 - 1 + 4,
  nBefore2 + " -> " + st.model.members.length);
check("重复分割后仍可求解", (() => {
  try { return globalThis.FrameSolver.solve(FL.solverContext()).ok === true; }
  catch (e) { return false; }
})());

// ---------- Case 3：面物理分割采用面板参数 + 可重复 ----------
console.log("## Case 3 面物理分割用面板参数且可重复");
FL.demoArea();
st = FL.state();
const aid0 = st.model.areas[0].id;
FL.selectArea(aid0);
$("dv-nx").value = "3"; $("dv-ny").value = "2"; $("dv-hmax").value = "0";
fireClick("dv-amesh");
st = FL.state();
check("面板 3x2 得 6 个子面", st.model.areas.length === 6, "areas=" + st.model.areas.length);
const childAid = st.model.areas[0].id;
FL.selectArea(childAid);
$("dv-nx").value = "2"; $("dv-ny").value = "2"; $("dv-hmax").value = "0";
fireClick("dv-amesh");
st = FL.state();
check("子面再次 2x2 分割", st.model.areas.length === 6 - 1 + 4, "areas=" + st.model.areas.length);

if (nFail > 0) { console.log("\nRESULT: FAIL (" + nFail + ")"); process.exit(1); }
console.log("\nRESULT: ALL PASS");
