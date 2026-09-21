/* FrameLab <- OpenSees 文件读取模块（独立文件，无 DOM 依赖）
 * ------------------------------------------------------------------
 * 把 OpenSees 模型文件解析为 FrameLab 模型快照（可直接 openDocument 打开，
 * 再用内置求解器或 xara/OpenSees 桥接求解）。
 *
 * 支持两种格式（按扩展名 + 内容自动判定）：
 *   OpenSeesPy (.py)：ops./op. 前缀命令；xara.Model 写法（model.）亦兼容，
 *     本页“复制脚本”导出的脚本可直接读回（roundtrip）。
 *   Tcl (.tcl)：node/fix/element/load/eleLoad 等原生命令；支持 set 变量、
 *     $var/${var} 替换与 [expr ...] 四则运算。
 *
 * 支持的命令（2D 线弹性静力子集）：
 *   wipe / model（含 ndm/ndf 识别）/ node / fix / mass(跳过)
 *   uniaxialMaterial Elastic / section Elastic
 *   geomTransf Linear（仅登记编号）/ element elasticBeamColumn |
 *     forceBeamColumn | dispBeamColumn | truss
 *   timeSeries / pattern Plain / load / eleLoad -beamUniform | -beamPoint
 * 其它命令（如 recorder/analyze/constraints/system/面单元 Quad/壳）跳过并
 * 记入 warnings，不中断导入。
 *
 * 约定（与 xara.js 导出侧对称；2D 平面统一为 X-Z，Y 为面外）：
 * - 单位：假定文件单位与 FrameLab 内部一致（m / kN / kN*m），不做换算；
 * - ndm=2 文件的第二坐标（OpenSees 称 Y，竖向）导入为 z；
 *   ndm=3 文件取全坐标（x, y, z；y 为面外）；
 * - 杆件荷载沿局部 +z 为正（udl q / beamUniform Wy 直接对应）；
 * - beamPoint 的位置参数按“相对长度（0~1）”解释（与本页导出一致）；
 *   若取值落在 (1, L] 内则按“距首节点绝对距离”解释并告警；
 * - truss 单元无抗弯刚度，导入时取 EI = EA*1e-9 并告警；
 * - 截面只记录矩形等效（由 A、Iz 反解 b、h：h=sqrt(12I/A)，b=A/h），
 *   刚度以 memStiff {EA, EI} 精确覆盖为准（GA 取 1e12，退化为 Euler 梁，
 *   与 OpenSees elasticBeamColumn 同理论）；
 * - 材料按 E 匹配内置材料库（Q235/Q345/C30/C40/AL6061），匹配不上记自定义。
 *
 * 对外接口（window.FrameOpenSees / globalThis.FrameOpenSees）：
 *   detectFormat(text, filename) -> "py" | "tcl"
 *   parse(text, filename) -> { format, snapshot, summary, warnings }
 *     snapshot 可直接传给 index.html 的 openDocument(name, snapshot)。
 */
(function (global) {
  "use strict";

  var VERSION = "1.0.0";

  /* ---------------- 通用小工具 ---------------- */
  function isNum(x) { return typeof x === "number" && isFinite(x); }
  function toNum(v) {
    if (isNum(v)) { return v; }
    if (typeof v === "string") {
      var s = v.trim().replace(/^[([{]+/, "").replace(/[)\]};,\s]+$/, "");
      if (/^[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?$/.test(s)) { return parseFloat(s); }
    }
    return NaN;
  }
  function stripQuotes(s) {
    if (typeof s !== "string" || s.length < 2) { return s; }
    var a = s[0], b = s[s.length - 1];
    if ((a === "'" && b === "'") || (a === '"' && b === '"')) { return s.slice(1, -1); }
    return s;
  }
  // Tiny 四则运算器（供 [expr]/py 变量赋值用）：数字 + 变量 + +-*/%**()，其余字符拒绝
  function evalArith(expr, env) {
    if (typeof expr !== "string") { return toNum(expr); }
    var s = expr.trim();
    if (s === "") { return NaN; }
    var out = "", i, ch, prev = "";
    // 变量名替换为数值（最长优先，避免前缀误杀）
    var names = Object.keys(env || {}).sort(function (a, b) { return b.length - a.length; });
    var done = s;
    names.forEach(function (nm) {
      if (!isNum(env[nm])) { return; }
      done = done.replace(new RegExp("\\b" + nm.replace(/[^A-Za-z0-9_]/g, "") + "\\b", "g"), "(" + env[nm] + ")");
    });
    for (i = 0; i < done.length; i++) {
      ch = done[i];
      if (/[0-9eE+\-*\/%().\s]/.test(ch)) {
        // "**" 视为 "^" 由后处理统一；此处先保留
        out += ch; prev = ch;
      } else {
        return NaN; // 非法字符：拒绝求值
      }
    }
    out = out.replace(/\*\*/g, "^");
    // ^ 幂：转成 Math.pow 的朴素处理（只处理 (a)^(b) 简单形）
    try {
      if (!/[\^a-df-zA-DF-Z_]/.test(out.replace(/e[+-]?\d/gi, ""))) {
        /* eslint-disable no-eval */
        var v = (0, eval)("(" + out + ")");
        /* eslint-enable no-eval */
        return isNum(v) ? v : NaN;
      }
      return NaN;
    } catch (e) { return NaN; }
  }

  /* ---------------- 行预处理 ---------------- */
  // py：去注释(#)、分号断句、续行符拼接
  function pyStatements(text) {
    var lines = String(text).split(/\r?\n/), buf = "", stmts = [], i, ln;
    for (i = 0; i < lines.length; i++) {
      ln = lines[i];
      // 去注释（引号外的 #）
      var q = null, k;
      for (k = 0; k < ln.length; k++) {
        var c = ln[k];
        if (q) { if (c === q && ln[k - 1] !== "\\") { q = null; } }
        else if (c === "'" || c === '"') { q = c; }
        else if (c === "#") { ln = ln.slice(0, k); break; }
      }
      ln = ln.trim();
      if (!ln) { continue; }
      if (/\\$/.test(ln)) { buf += ln.replace(/\\$/, " ") + " "; continue; }
      buf += ln;
      buf.split(";").forEach(function (p) { if (p.trim()) { stmts.push(p.trim()); } });
      buf = "";
    }
    if (buf.trim()) { stmts.push(buf.trim()); }
    return stmts;
  }
  // tcl：续行拼接、注释(# 行首或空白后)、花括号展平为普通命令
  function tclStatements(text) {
    var lines = String(text).split(/\r?\n/), buf = "", stmts = [], i, ln;
    for (i = 0; i < lines.length; i++) {
      ln = lines[i];
      if (/\\$/.test(ln)) { buf += ln.replace(/\\$/, " ") + " "; continue; }
      buf += ln; ln = buf; buf = "";
      var t = ln.trim();
      if (!t || t === "{" || t === "}") { continue; }
      if (t[0] === "#") { continue; }
      // 行内注释：空白后的 # 起（引号/花括号内除外，近似处理）
      var q = null, depth = 0, k;
      for (k = 0; k < ln.length; k++) {
        var c = ln[k];
        if (q) { if (c === q) { q = null; } }
        else if (c === '"' || c === "'") { q = c; }
        else if (c === "{") { depth++; }
        else if (c === "}") { depth = Math.max(0, depth - 1); }
        else if (c === "#" && depth === 0 && /\s/.test(ln[k - 1] || " ")) { ln = ln.slice(0, k); break; }
      }
      ln = ln.replace(/[{}]/g, " ").trim();
      if (ln) { stmts.push(ln); }
    }
    if (buf.trim()) { stmts.push(buf.trim()); }
    return stmts;
  }
  // 参数切分（尊重引号与 [] () 嵌套）
  function splitArgs(s) {
    var args = [], cur = "", q = null, dSq = 0, dRd = 0, i, c;
    for (i = 0; i < s.length; i++) {
      c = s[i];
      if (q) {
        cur += c;
        if (c === q && s[i - 1] !== "\\") { q = null; }
      } else if (c === "'" || c === '"') { q = c; cur += c; }
      else if (c === "[") { dSq++; cur += c; }
      else if (c === "]") { dSq = Math.max(0, dSq - 1); cur += c; }
      else if (c === "(") { dRd++; cur += c; }
      else if (c === ")") { dRd = Math.max(0, dRd - 1); cur += c; }
      else if ((c === " " || c === "\t" || c === ",") && dSq === 0 && dRd === 0) {
        if (cur.trim() !== "") { args.push(cur.trim()); }
        cur = "";
      } else { cur += c; }
    }
    if (cur.trim() !== "") { args.push(cur.trim()); }
    return args;
  }

  /* ---------------- 主解析 ---------------- */
  var LIB_E = [
    { key: "Q345", E: 2.06e8, nu: 0.30 }, { key: "Q235", E: 2.06e8, nu: 0.30 },
    { key: "C40", E: 3.25e7, nu: 0.20 }, { key: "C30", E: 3.00e7, nu: 0.20 },
    { key: "AL6061", E: 6.90e7, nu: 0.33 }
  ];
  function matchMat(E) {
    for (var i = 0; i < LIB_E.length; i++) {
      if (Math.abs(E - LIB_E[i].E) / LIB_E[i].E < 1e-9) {
        return { label: LIB_E[i].key, E: LIB_E[i].E, nu: LIB_E[i].nu };
      }
    }
    return { label: "自定义", E: E, nu: E >= 1e8 ? 0.30 : 0.20 };
  }
  function rectFromAI(A, I) {
    if (!(A > 0) || !(I > 0)) { return { type: "rect", b: 0.3, h: 0.5 }; }
    var h = Math.sqrt(12 * I / A), b = A / h;
    if (!isFinite(h) || !isFinite(b) || h <= 0 || b <= 0) { return { type: "rect", b: 0.3, h: 0.5 }; }
    return { type: "rect", b: b, h: h };
  }
  function memberType(x1, z1, x2, z2) {
    if (Math.abs(x1 - x2) < 1e-9) { return "col"; }
    if (Math.abs(z1 - z2) < 1e-9) { return "beam"; }
    return "user";
  }
  // 竖向坐标：z 优先（新 2D），兼容旧 y
  function vertOf(p) { return (p && isFinite(p.z)) ? p.z : ((p && isFinite(p.y)) ? p.y : 0); }

  function detectFormat(text, filename) {
    var fn = String(filename || "").toLowerCase();
    if (/\.tcl$/.test(fn)) { return "tcl"; }
    if (/\.py$/.test(fn)) { return "py"; }
    var t = String(text);
    if (/(^|\n)\s*(ops|op)\.\w+\s*\(/.test(t)) { return "py"; }
    if (/(^|\n)\s*(wipe|model\s+BasicBuilder|geomTransf|uniaxialMaterial|eleLoad)\b/.test(t)) { return "tcl"; }
    if (/(^|\n)\s*node\s+\S/.test(t)) { return "tcl"; }
    return "py";
  }

  function newCtx() {
    return {
      ndm: 2, ndf: 3,
      nodes: {}, nodeOrder: [],       // tag -> {x, y?, z?}（2D 用 x,z）
      eles: [],                        // {tag, type, i, j, A, E, Iz, secTag, matTag}
      mats: {},                        // E 材料：tag -> E
      secs: {},                        // Elastic 截面：tag -> {E, A, Iz}
      loads: {},                       // nodeTag -> {fx, fz, my}（2D；3D 为 6 分量）
      eleLoads: [],                    // {eleTag, kind, q/P..., raw}
      fixes: {},                       // nodeTag -> [f1..]
      env: {},                         // py/tcl 变量
      TclVars: {},
      warnings: [], warnSet: {},
      nSkip: {}
    };
  }
  function warn(ctx, msg) {
    if (ctx.warnSet[msg]) { return; }
    ctx.warnSet[msg] = true;
    if (ctx.warnings.length < 40) { ctx.warnings.push(msg); }
  }
  function skipCmd(ctx, name) {
    ctx.nSkip[name] = (ctx.nSkip[name] || 0) + 1;
  }

  function resolveVal(ctx, tok) {
    if (tok == null) { return NaN; }
    var s = String(tok).trim();
    // tcl [expr ...]（py 侧一般无此写法，顺手兼容）
    var m = s.match(/^\[\s*expr\s+([\s\S]+)\]$/);
    var inner = m ? m[1] : s;
    inner = stripQuotes(inner.trim());
    var v = toNum(inner);
    if (isNum(v)) { return v; }
    // 变量
    if (ctx.env && ctx.env[inner] !== undefined && isNum(ctx.env[inner])) { return ctx.env[inner]; }
    if (ctx.TclVars && ctx.TclVars[inner] !== undefined && isNum(ctx.TclVars[inner])) { return ctx.TclVars[inner]; }
    // 四则表达式（含变量）
    var merged = {};
    Object.keys(ctx.env || {}).forEach(function (k) { merged[k] = ctx.env[k]; });
    Object.keys(ctx.TclVars || {}).forEach(function (k) { merged[k] = ctx.TclVars[k]; });
    v = evalArith(inner, merged);
    return v;
  }
  // tcl $var / ${var} 替换（含 [expr] 递归求值）
  function tclSubst(ctx, tok) {
    var s = String(tok), guard = 0;
    var subVar = function (str) {
      return str.replace(/\$\{([A-Za-z_][\w]*)\}|\$([A-Za-z_][\w]*)/g, function (_, a, b) {
        var nm = a || b;
        return (ctx.TclVars[nm] !== undefined) ? ctx.TclVars[nm] : "$" + nm;
      });
    };
    s = subVar(s);
    while (/^\[.*\]$/.test(s.trim()) && guard++ < 5) {
      var m = s.trim().match(/^\[\s*expr\s+([\s\S]+)\]$/);
      if (!m) { break; }
      var v = evalArith(subVar(m[1]), ctx.TclVars);
      if (!isNum(v)) { break; }
      s = String(v);
    }
    return s;
  }

  function parsePyStatement(ctx, stmt) {
    var s = stmt.trim();
    if (!s || s[0] === "#") { return; }
    // import / from / 常量赋值：name = expr
    if (/^(import|from)\b/.test(s)) { return; }
    var mAssign = s.match(/^([A-Za-z_]\w*)\s*=\s*(.+)$/);
    if (mAssign && !/==/.test(s) && !/^\w+\s*\(/.test(mAssign[1])) {
      var rhs = mAssign[2].trim();
      if (/^[\w.]+\s*\(/.test(rhs)) {
        // x = model.node(...) 之类：仍尝试按命令执行（xara 链式少见，直接执行）
      } else {
        var v = evalArith(rhs, ctx.env);
        if (isNum(v)) { ctx.env[mAssign[1]] = v; return; }
        var v2 = toNum(stripQuotes(rhs));
        if (isNum(v2)) { ctx.env[mAssign[1]] = v2; return; }
        return;
      }
    }
    var m = s.match(/^([\w.]+)\s*\(([\s\S]*)\)\s*$/);
    if (!m) { return; }
    var dotted = m[1].split("."), fname = dotted[dotted.length - 1];
    var prefix = dotted.length > 1 ? dotted[dotted.length - 2] : "";
    if (prefix !== "ops" && prefix !== "op" && prefix !== "model" && prefix !== "") { return; }
    var rawArgs = splitArgs(m[2]), args = [];
    rawArgs.forEach(function (a) {
      var t = a.trim(), kv = t.match(/^([A-Za-z_]\w*)\s*=\s*([\s\S]+)$/); // pattern=1 等关键字参数
      if (kv) { t = kv[2].trim(); }
      if (/^\[.*\]$/.test(t)) { // 节点列表 [i, j]
        var inner = t.slice(1, -1);
        splitArgs(inner).forEach(function (p) { args.push(p.trim()); });
        return;
      }
      args.push(t);
    });
    dispatch(ctx, fname, args, false);
  }

  function parseTclStatement(ctx, stmt) {
    var toks = splitArgs(stmt);
    if (!toks.length) { return; }
    var cmd = toks[0], rest = toks.slice(1).map(function (t) { return tclSubst(ctx, t); });
    if (cmd === "set" && rest.length >= 2) {
      var v = resolveVal(ctx, rest.slice(1).join(" "));
      if (isNum(v)) { ctx.TclVars[rest[0]] = v; }
      else { ctx.TclVars[rest[0]] = stripQuotes(rest.slice(1).join(" ")); }
      return;
    }
    dispatch(ctx, cmd, rest, true);
  }

  function numArgs(ctx, args, isTcl) {
    return args.map(function (a) { return resolveVal(ctx, isTcl ? a : stripQuotes(a)); });
  }

  function dispatch(ctx, fname, args, isTcl) {
    var A = function (i) { return numArgs(ctx, args, isTcl)[i]; };
    var S = function (i) { return stripQuotes(String(args[i] == null ? "" : args[i])); };
    switch (fname) {
      case "wipe": case "wipeAnalysis": return;
      case "model": {
        // py: model('basic','-ndm',2,'-ndf',3) / tcl: model BasicBuilder -ndm 2 -ndf 3
        for (var i = 0; i < args.length; i++) {
          var t = S(i).toLowerCase();
          if (t === "-ndm" && isNum(A(i + 1))) { ctx.ndm = Math.round(A(i + 1)); }
          if (t === "-ndf" && isNum(A(i + 1))) { ctx.ndf = Math.round(A(i + 1)); }
        }
        if (ctx.ndm === 3) { warn(ctx, "文件为 3D 模型（ndm=3），取全坐标 x/y/z（含面外 y）导入。"); }
        return;
      }
      case "node": {
        var tag = A(0), x = A(1), v2 = A(2), v3 = A(3);
        if (!isNum(tag) || !isNum(x) || !isNum(v2)) { warn(ctx, "跳过无法解析的 node 定义：" + args.join(" ")); return; }
        tag = Math.round(tag);
        if (ctx.nodes[tag]) { warn(ctx, "重复的 node " + tag + "（取首次定义）。"); return; }
        // ndm=3（或带第三坐标）：node x y z 全取；ndm=2：第二坐标为竖向，存为 z
        if (ctx.ndm === 3 || isNum(v3)) { ctx.nodes[tag] = { x: x, y: v2, z: v3 }; }
        else { ctx.nodes[tag] = { x: x, z: v2 }; }
        ctx.nodeOrder.push(tag);
        return;
      }
      case "fix": {
        var ft = Math.round(A(0));
        if (!isNum(ft) || !ctx.nodes[ft]) { warn(ctx, "fix 指向未定义的节点：" + args.join(" ")); return; }
        var vals = [A(1), A(2), A(3), A(4), A(5), A(6)].filter(isNum).map(function (v) { return v ? 1 : 0; });
        ctx.fixes[ft] = vals;
        return;
      }
      case "mass": return; // 静力导入不需要质量
      case "uniaxialMaterial": {
        // Elastic tag E [eta] / py: ('Elastic', tag, E)
        if (/elastic/i.test(S(0)) && isNum(A(1)) && isNum(A(2))) { ctx.mats[Math.round(A(1))] = A(2); }
        return;
      }
      case "material": { // xara: material('ElasticIsotropic', tag, E, nu)
        if (/elastic/i.test(S(0)) && isNum(A(1)) && isNum(A(2))) { ctx.mats[Math.round(A(1))] = A(2); }
        return;
      }
      case "section": {
        // Elastic tag E A Iz (xara 膜截面等其它类型跳过)
        if (/^elastic$/i.test(S(0)) && isNum(A(1)) && isNum(A(2)) && isNum(A(3)) && isNum(A(4))) {
          ctx.secs[Math.round(A(1))] = { E: A(2), A: A(3), Iz: A(4) };
        } else { skipCmd(ctx, "section:" + S(0)); }
        return;
      }
      case "geomTransf": return; // 编号仅占位，无需处理
      case "element": {
        var et = S(0).toLowerCase(), etag = Math.round(A(1));
        if (!isNum(etag)) { warn(ctx, "跳过无法解析的 element 定义：" + args.join(" ")); return; }
        if (et === "elasticbeamcolumn" || et === "forcebeamcolumn" || et === "dispbeamcolumn") {
          var iN = Math.round(A(2)), jN = Math.round(A(3));
          var rec = { tag: etag, type: "beam", i: iN, j: jN, A: NaN, E: NaN, Iz: NaN };
          if (et === "elasticbeamcolumn") {
            rec.A = A(4); rec.E = A(5); rec.Iz = A(6);
          } else {
            // force/dispBeamColumn (i, j, transf, secTag)：由截面反查
            var sec = ctx.secs[Math.round(A(5))];
            if (sec) { rec.A = sec.A; rec.E = sec.E; rec.Iz = sec.Iz; rec.secTag = Math.round(A(5)); }
          }
          if (!isNum(rec.A) || !isNum(rec.E) || !isNum(rec.Iz)) {
            warn(ctx, "element " + etag + " 缺少截面参数（forceBeamColumn 需先定义 section Elastic），已跳过。");
            return;
          }
          ctx.eles.push(rec);
        } else if (et === "truss") {
          var ti = Math.round(A(2)), tj = Math.round(A(3)), tA = A(4), mtE = ctx.mats[Math.round(A(5))];
          if (!isNum(tA) || !isNum(mtE)) { warn(ctx, "truss " + etag + " 缺少面积/材料，已跳过。"); return; }
          ctx.eles.push({ tag: etag, type: "truss", i: ti, j: tj, A: tA, E: mtE, Iz: tA * 1e-9 });
          warn(ctx, "truss " + etag + " 按轴向杆导入（EI 取 EA*1e-9 近似），抗弯刚度极小。");
        } else if (/quad|shell|tri|brick/i.test(et)) {
          skipCmd(ctx, "element:" + S(0));
        } else {
          skipCmd(ctx, "element:" + S(0));
        }
        return;
      }
      case "timeSeries": case "pattern": return; // 仅作荷载分组标记，荷载直接累加
      case "load": {
        var lt = Math.round(A(0));
        if (!isNum(lt)) { warn(ctx, "跳过无法解析的 load 定义：" + args.join(" ")); return; }
        var e = ctx.loads[lt] || { fx: 0, fy: 0, fz: 0, mx: 0, my: 0, mz: 0 };
        if (ctx.ndm === 3) {
          if (isNum(A(1))) { e.fx += A(1); }
          if (isNum(A(2))) { e.fy += A(2); }
          if (isNum(A(3))) { e.fz += A(3); }
          if (isNum(A(4))) { e.mx += A(4); }
          if (isNum(A(5))) { e.my += A(5); }
          if (isNum(A(6))) { e.mz += A(6); }
        } else {
          // 2D：第二分量为竖向 Fz，第三分量为面内弯矩 My
          if (isNum(A(1))) { e.fx += A(1); }
          if (isNum(A(2))) { e.fz += A(2); }
          if (isNum(A(3))) { e.my += A(3); }
        }
        ctx.loads[lt] = e;
        return;
      }
      case "eleLoad": {
        // -ele tag -type -beamUniform q [qx xa xb] / -beamPoint P x
        var flag = args.map(function (a) { return stripQuotes(String(a)).toLowerCase(); });
        var ie = flag.indexOf("-ele"), it = flag.indexOf("-type");
        if (ie < 0 || it < 0) { warn(ctx, "跳过无法解析的 eleLoad：" + args.join(" ")); return; }
        var eletag = Math.round(resolveVal(ctx, args[ie + 1]));
        var kind = flag[it + 1] || "";
        var vals = [];
        for (var k = it + 2; k < args.length; k++) {
          var vv = resolveVal(ctx, args[k]);
          if (isNum(vv)) { vals.push(vv); }
        }
        if (!isNum(eletag)) { warn(ctx, "跳过无法解析的 eleLoad：" + args.join(" ")); return; }
        if (kind === "-beamuniform") {
          if (vals.length >= 4) { ctx.eleLoads.push({ eleTag: eletag, kind: "trap", q1: vals[0], q2: vals[0], c: vals[2], d: vals[3] }); }
          else if (vals.length >= 2 && Math.abs(vals[1]) > 1e-12) {
            warn(ctx, "eleLoad -beamUniform 的轴向分量（Wx=" + vals[1] + "）暂不支持，已忽略。");
            ctx.eleLoads.push({ eleTag: eletag, kind: "udl", q: vals[0] });
          }
          else { ctx.eleLoads.push({ eleTag: eletag, kind: "udl", q: vals[0] || 0 }); }
        } else if (kind === "-beampoint") {
          if (vals.length < 2) { warn(ctx, "跳过参数不足的 -beamPoint：" + args.join(" ")); return; }
          ctx.eleLoads.push({ eleTag: eletag, kind: "point", P: vals[0], x: vals[1] });
        } else { skipCmd(ctx, "eleLoad:" + kind); }
        return;
      }
      case "constraints": case "system": case "numberer": case "algorithm":
      case "integrator": case "analysis": case "analyze": case "reactions":
      case "recorder": case "print": case "printa": case "puts": case "source":
      case "eigen": case "modalproperties": case "rayleigh": case "regions":
        return; // 求解/输出类命令：静力导入无需处理
      default:
        skipCmd(ctx, fname);
    }
  }

  /* ---------------- 组装 FrameLab 快照 ---------------- */
  function defaultD() {
    return {
      bays: 1, stories: 1, W: 6, h: 4, NSUB: 5, AMESH: 2,
      mag: 1, dscale: 1, loadScale: 1, view: "disp",
      showAxial: false, showMoment: true, showShear: false,
      showForceVals: false, showExtreme: false,
      showShellBeam: false, showShellCol: false, showShellBeamF: false, showShellColF: false,
      cutSnap: true, cutVar: "sx", showCutPanel: true, showCuts: true,
      showLoadVals: false, showMat: false, showSec: false,
      showAreaMesh: true, showAreaCloud: true, cloudVar: "vm", showAreaShrink: false,
      dispVar: "mag", cloudLo: null, cloudHi: null, dispLo: null, dispHi: null,
      station: 0.5, zoom: 1, px: 0, py: 0,
      light: true, lenUnit: "m", forceUnit: "kN",
      designSteel: "GB50017-2017", designConc: "GB50010-2010", designMode: "rebar"
    };
  }

  function buildSnapshot(ctx) {
    var tag2id = {}, nodes = [], nid = 0;
    ctx.nodeOrder.forEach(function (tag) {
      var p = ctx.nodes[tag];
      tag2id[tag] = nid;
      var fx = ctx.fixes[tag] || [];
      var bc;
      if (ctx.ndm === 3) {
        bc = { ux: !!fx[0], uy: !!fx[1], uz: !!fx[2], rx: !!fx[3], ry: !!fx[4], rz: !!fx[5] };
      } else {
        // 2D：ux / uz（竖向）/ ry（面内弯曲）
        bc = { ux: !!fx[0], uz: !!fx[1], ry: !!(fx[2] || 0) };
      }
      var nd = { id: nid, x: p.x, bc: bc };
      if (ctx.ndm === 3) { nd.y = p.y || 0; nd.z = isNum(p.z) ? p.z : 0; }
      else { nd.z = vertOf(p); }
      nodes.push(nd);
      nid++;
    });
    var members = [], memStiff = {}, memMat = {}, memSec = {}, memLoads = {}, mid = 0;
    var ele2mid = {};
    ctx.eles.forEach(function (el) {
      if (tag2id[el.i] === undefined || tag2id[el.j] === undefined) {
        warn(ctx, "element " + el.tag + " 引用了未定义的节点，已跳过。");
        return;
      }
      var A = ctx.nodes[el.i], B = ctx.nodes[el.j];
      var L = Math.hypot(B.x - A.x, vertOf(B) - vertOf(A));
      if (!(L > 1e-9)) { warn(ctx, "element " + el.tag + " 为零长杆件，已跳过。"); return; }
      var m = { id: mid, a: tag2id[el.i], b: tag2id[el.j], type: memberType(A.x, vertOf(A), B.x, vertOf(B)) };
      members.push(m);
      ele2mid[el.tag] = mid;
      var EA = el.E * el.A, EI = el.E * el.Iz;
      memStiff[mid] = { EA: EA, EI: EI, GA: 1e12 };
      memMat[mid] = matchMat(el.E);
      memSec[mid] = rectFromAI(el.A, el.Iz);
      mid++;
    });
    var nodeLoads = {};
    Object.keys(ctx.loads).forEach(function (tag) {
      if (tag2id[tag] === undefined) { warn(ctx, "load 指向未定义的节点 " + tag + "，已跳过。"); return; }
      var l = ctx.loads[tag];
      if (ctx.ndm === 3) {
        if ([l.fx, l.fy, l.fz, l.mx, l.my, l.mz].every(function (v) { return Math.abs(v || 0) < 1e-12; })) { return; }
        nodeLoads[tag2id[tag]] = { fx: l.fx, fy: l.fy, fz: l.fz, mx: l.mx, my: l.my, mz: l.mz };
      } else {
        if (Math.abs(l.fx) < 1e-12 && Math.abs(l.fz) < 1e-12 && Math.abs(l.my) < 1e-12) { return; }
        nodeLoads[tag2id[tag]] = { fx: l.fx, fz: l.fz, my: l.my };
      }
    });
    ctx.eleLoads.forEach(function (el2) {
      var mm = ele2mid[el2.eleTag];
      if (mm === undefined) { warn(ctx, "eleLoad 指向未导入的单元 " + el2.eleTag + "，已跳过。"); return; }
      var mb = null, i;
      for (i = 0; i < members.length; i++) { if (members[i].id === mm) { mb = members[i]; } }
      if (!mb) { return; }
      var nA = nodes[mb.a], nB = nodes[mb.b];
      var L = Math.hypot(nB.x - nA.x, vertOf(nB) - vertOf(nA));
      var list = memLoads[mm] || (memLoads[mm] = []);
      if (el2.kind === "udl") {
        if (Math.abs(el2.q) < 1e-12) { return; }
        list.push({ type: "udl", q: el2.q });
      } else if (el2.kind === "trap") {
        list.push({ type: "trap", q1: el2.q1, q2: el2.q2, c: Math.max(0, el2.c), d: Math.min(L, el2.d) });
      } else if (el2.kind === "point") {
        if (Math.abs(el2.P) < 1e-12) { return; }
        var a;
        if (el2.x >= 0 && el2.x <= 1) { a = el2.x * L; }           // 相对长度（本页导出约定）
        else if (el2.x > 1 && el2.x <= L * (1 + 1e-9)) {           // 绝对距离
          a = el2.x;
          warn(ctx, "单元 " + el2.eleTag + " 的 -beamPoint 位置 " + el2.x + " 按距首节点绝对距离解释。");
        }
        else { warn(ctx, "单元 " + el2.eleTag + " 的 -beamPoint 位置 " + el2.x + " 超出范围，已钳制。"); a = Math.min(Math.max(el2.x, 0), L); }
        list.push({ type: "point", P: el2.P, a: a });
      }
    });
    var model = { nodes: nodes, members: members, areas: [], cuts: [], nextNode: nid, nextMember: mid, nextArea: 0, nextCut: 0 };
    Object.keys(ctx.nSkip).forEach(function (k) {
      warn(ctx, "跳过 " + ctx.nSkip[k] + " 条“" + k + "”命令（非 2D 线弹性框架子集）。");
    });
    return {
      app: "FrameLab", ver: 2, savedAt: new Date().toISOString(), D: defaultD(), model: model,
      memStiff: memStiff, memMat: memMat, memSec: memSec, memLoads: memLoads,
      nodeLoads: nodeLoads, areaMat: {}, areaSec: {}, areaLoads: {},
      memDiv: {}, areaDiv: {}, memRebar: {}, areaRebar: {}
    };
  }

  function parse(text, filename) {
    var format = detectFormat(text, filename);
    var ctx = newCtx();
    var stmts = format === "tcl" ? tclStatements(text) : pyStatements(text);
    stmts.forEach(function (st) {
      try {
        if (format === "tcl") { parseTclStatement(ctx, st); }
        else { parsePyStatement(ctx, st); }
      } catch (e) { warn(ctx, "解析失败已跳过一行：" + String(st).slice(0, 80)); }
    });
    if (!ctx.nodeOrder.length) { throw new Error("未解析到任何 node（空模型或格式不支持）。"); }
    if (!ctx.eles.length) { throw new Error("未解析到任何框架单元（仅支持 elasticBeamColumn / forceBeamColumn / dispBeamColumn / truss）。"); }
    var snapshot = buildSnapshot(ctx);
    var nLoad = Object.keys(snapshot.nodeLoads).length +
      Object.keys(snapshot.memLoads).reduce(function (a, k) { return a + snapshot.memLoads[k].length; }, 0);
    return {
      format: format, snapshot: snapshot, warnings: ctx.warnings,
      summary: {
        nodes: snapshot.model.nodes.length, members: snapshot.model.members.length,
        loads: nLoad, skipped: Object.keys(ctx.nSkip).length
      }
    };
  }

  global.FrameOpenSees = {
    version: VERSION, detectFormat: detectFormat, parse: parse
  };
})(typeof window !== "undefined" ? window : globalThis);
