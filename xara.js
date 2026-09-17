/* FrameLab -> OpenSees/xara 集成模块（独立文件）
 * ------------------------------------------------------------------
 * 把当前模型导出为 xara (https://github.com/peer-open-source/xara)
 * 的 Python 脚本（Model API），经本地桥接服务调用 OpenSees 求解，
 * 再把结果读回并映射为 FrameLab ana 结构进行后处理显示。
 *
 * 对外接口（window.FrameXara）：
 *   buildScript(state, opts)   state -> { script, meta, kind, ndf }（xara .py 文本 + 对照表）
 *   exportMeta(state, opts)    导出对照表（节点/单元标签映射，供结果回填用）
 *   runOnBridge(script, url)   POST 脚本到桥接服务，返回 Promise<resultsJSON>
 *   importResults(res, meta)   resultsJSON + meta -> ana 兼容对象
 *   runFromUI()                一键流程（按钮调用）：检查->导出->求解->回填->显示
 *   downloadScript()           下载 .py 脚本（离线/手动运行用）
 *   backend()                  当前求解器标识 "builtin" | "xara"
 *
 * 单位：直接沿用 FrameLab 内部单位（m / kN / kN*m / kPa），自洽。
 * 映射要点（均经真机 xara 0.0.34 实测）：
 *   纯框架 -> elasticBeamColumn(tag,[i,j],A,E,Iz,transf)，ndm=2,ndf=3；
 *             杆均布 beamUniform(q)、集中力 beamPoint(P,a/L)、梯形 20 点离散；
 *             杆件按 NSUB/memDiv 等分 + 杆上节点投影切分为杆段（与内置求解器
 *             同规则），逐段下单元与荷载、逐段回填 13 站内力；
 *   纯连续体 -> Quad(tag,nodes,(t,type,mat)) / Tri31(tag,n1,n2,n3,t,type,mat)，
 *             ndm=2,ndf=2；逆时针节点序
 *   混合模型 -> 走 3D 退化平面路径（ndm=3,ndf=6，z=0 平面内建模）：
 *             框架为 3D elasticBeamColumn（定向向量取 +Z，保证局部 y 在面内），
 *             连续体为 ASDShellQ4 / ASDShellT3 + ElasticMembranePlateSection 膜截面；
 *             全部节点约束面外自由度（uz,rx,ry），只施加面内荷载，退化为平面问题。
 *             杆件荷载不走 eleLoad（OpenSeesRT 的 3D 梁单元荷载向量有误，已实测），
 *             一律转为与内置求解器同式的一致等效节点力；面荷载同理。
 *             注意：膜壳的 drilling 刚度约等于零，梁以“点”方式接入墙面时转角
 *             基本不传递（相当于铰接）；共边（逐节点共享）的平动耦合是精确的，
 *             框架自身的梁-梁刚接不受影响。连梁等点连接场景的刚接处理另行立项。
 *   无框架连接的节点自动约束 rz（对应内置求解器的零刚度钻孔自由度处理）
 *   轴力（拉+）= -N1；剪力 S = V1 + A(x)；弯矩（下缘受拉+）= -(M1-V1*x-J(x))；
 *   其中 N1/V1/M1 取自 eleResponse 'localForces'（单元局部坐标）。
 *   注意：OpenSeesRT 的 'forces' 返回的是整体坐标端力，仅水平杆与局部坐标一致，
 *   故必须用 'localForces'，否则竖向/斜杆的轴力与剪力会被互换。
 */
(function (global) {
  "use strict";

  var VERSION = "1.0.0";
  var backend = "builtin";

  /* ---------------- 小工具（自包含，不依赖 index.html） ---------------- */
  function sectionProps(sec) {
    if (!sec) { return null; }
    if (sec.type === "rect") {
      var A = sec.b * sec.h;
      return { A: A, I: sec.b * Math.pow(sec.h, 3) / 12 };
    }
    if (sec.type === "circle") {
      var d = sec.d, A2 = Math.PI * d * d / 4;
      return { A: A2, I: Math.PI * Math.pow(d, 4) / 64 };
    }
    if (sec.type === "I") {
      var h = sec.h, b = sec.b, tw = sec.tw, tf = sec.tf;
      return { A: 2 * b * tf + (h - 2 * tf) * tw, I: (b * Math.pow(h, 3) - (b - tw) * Math.pow(h - 2 * tf, 3)) / 12 };
    }
    return null;
  }
  function clampInt(v, lo, hi) { v = Math.round(v); return v < lo ? lo : (v > hi ? hi : v); }
  // 杆件有效细分数（与内置求解器 effFrameDiv 同规则）：memDiv 覆盖优先，否则 NSUB；
  // 取值顺序与两处调用形状兼容：顶层 NSUB（collectSolverContext 形状）优先，D.NSUB 兜底
  function effNSUB(state) {
    if (state && state.NSUB != null) { return state.NSUB; }
    var D = (state && state.D) || {};
    return D.NSUB != null ? D.NSUB : 5;
  }
  function effFrameDiv(state, mid, L) {
    var n = clampInt(effNSUB(state), 1, 50);
    var o = (state && state.memDiv || {})[mid];
    if (o) {
      n = clampInt(o.n || n, 1, 50);
      if (o.lmax > 0 && L > 0) { n = Math.max(n, Math.ceil(L / o.lmax)); }
      n = clampInt(n, 1, 50);
    }
    return n;
  }
  function signedArea(P) {
    var s = 0, i;
    for (i = 0; i < P.length; i++) { var p = P[i], q = P[(i + 1) % P.length]; s += p[0] * q[1] - q[0] * p[1]; }
    return s / 2;
  }
  function quadSubCells(corners, nx, ny) {
    var P = corners, cells = [], r, c, grid = [];
    for (r = 0; r <= ny; r++) {
      grid[r] = [];
      for (c = 0; c <= nx; c++) {
        var s = c / nx, t = r / ny;
        grid[r][c] = [(1 - s) * (1 - t) * P[0][0] + s * (1 - t) * P[1][0] + s * t * P[2][0] + (1 - s) * t * P[3][0],
                      (1 - s) * (1 - t) * P[0][1] + s * (1 - t) * P[1][1] + s * t * P[2][1] + (1 - s) * t * P[3][1]];
      }
    }
    for (r = 0; r < ny; r++) {
      for (c = 0; c < nx; c++) {
        cells.push([r * (nx + 1) + c, r * (nx + 1) + c + 1, (r + 1) * (nx + 1) + c + 1, (r + 1) * (nx + 1) + c]);
      }
    }
    return { grid: grid, cells: cells };
  }
  function triSubCells(corners, n) {
    n = Math.max(1, Math.min(50, n));
    var A = corners[0], B = corners[1], C = corners[2], map = {}, pts = [], tris = [];
    function key(i, j, k) { return i + "," + j + "," + k; }
    function idx(i, j, k) {
      var kk = key(i, j, k);
      if (map[kk] === undefined) {
        map[kk] = pts.length;
        pts.push([(A[0] * i + B[0] * j + C[0] * k) / n, (A[1] * i + B[1] * j + C[1] * k) / n]);
      }
      return map[kk];
    }
    var i, j, k;
    for (k = 0; k <= n; k++) { for (j = 0; j <= n - k; j++) { i = n - j - k; idx(i, j, k); } }
    for (k = 0; k < n; k++) {
      for (j = 0; j < n - k; j++) {
        i = n - j - k;
        tris.push([idx(i, j, k), idx(i - 1, j + 1, k), idx(i - 1, j, k + 1)]);
        if (j + k < n - 1) { tris.push([idx(i - 1, j + 1, k), idx(i - 2, j + 1, k + 1), idx(i - 1, j, k + 1)]); }
      }
    }
    var clean = [];
    tris.forEach(function (t) {
      if (t[0] !== undefined && t[1] !== undefined && t[2] !== undefined && t[0] !== t[1] && t[1] !== t[2] && t[0] !== t[2]) { clean.push(t); }
    });
    return { pts: pts, tris: clean.length ? clean : [[idx(n, 0, 0), idx(0, n, 0), idx(0, 0, n)]] };
  }
  function num(n) {
    if (!isFinite(n)) { return "0.0"; }
    var s = String(n);
    if (s.indexOf("e") < 0 && s.indexOf(".") < 0) { s += ".0"; }
    return s;
  }
  function pyStr(s) { return "'" + String(s).replace(/\\/g, "\\\\").replace(/'/g, "\\'") + "'"; }  /* ---------------- 导出：FrameLab state -> 中间表示 meta ---------------- */
  function effAreaDiv(a, nodeById, D, areaDiv) {
    var nx = clampInt(D.AMESH != null ? D.AMESH : 2, 1, 50), ny = nx, o = (areaDiv || {})[a.id];
    if (o) { nx = clampInt(o.nx || nx, 1, 50); ny = clampInt(o.ny || ny, 1, 50); }
    if (o && o.hmax > 0) {
      var pts = a.nodes.map(nodeById).filter(Boolean), Lmax = 0, i;
      for (i = 0; i < pts.length; i++) {
        var p = pts[i], q = pts[(i + 1) % pts.length];
        Lmax = Math.max(Lmax, Math.hypot(q.x - p.x, q.y - p.y));
      }
      if (Lmax > 0) { var k = Math.min(50, Math.ceil(Lmax / o.hmax)); nx = Math.max(nx, k); ny = Math.max(ny, k); }
    }
    return { nx: nx, ny: ny };
  }

  function exportMeta(state, opts) {
    opts = opts || {};
    var model = state.model || { nodes: [], members: [], areas: [] };
    var D = state.D || {};
    var nodeById = function (id) { return (model.nodes || []).find(function (n) { return n.id === id; }); };
    var hasFrame = (model.members || []).length > 0;
    var hasCont = (model.areas || []).length > 0;
    var kind = (!hasFrame && !hasCont) ? "empty" : (hasFrame && hasCont ? "mixed" : (hasFrame ? "frame" : "continuum"));

    // 分析节点合并（与内置求解器相同的 1e-6 容差）
    var all = [], reg = {}, idx = {};
    function ensureNode(x, y) {
      var k = (Math.round(x * 1e6) / 1e6) + "," + (Math.round(y * 1e6) / 1e6);
      if (reg[k] !== undefined) { return reg[k]; }
      reg[k] = all.length; all.push([x, y]); return reg[k];
    }
    (model.nodes || []).forEach(function (n) { idx[n.id] = ensureNode(n.x, n.y); });
    var xnode = all.map(function (_, i) { return i + 1; }); // 分析节点序号 -> xara 节点号

    // 材料去重
    var matMap = {}, materials = [];
    function matTagOf(E, nu) {
      var k = E.toPrecision(12) + "|" + nu.toPrecision(12);
      if (!matMap[k]) { matMap[k] = materials.length + 1; materials.push({ tag: matMap[k], E: E, nu: nu }); }
      return matMap[k];
    }

    // 框架杆件刚度（零长杆件跳过，与内置求解器一致）；
    // 杆段切分（与内置求解器同规则）：NSUB/memDiv 等分 + 落在杆上的分析节点投影切分，
    // 否则杆中节点在 xara 侧会被判孤立而多加全约束（曾导致刚度偏大类错误）
    var DEF = { col: { EA: 4.5e6, EI: 93750 }, beam: { EA: 4.5e6, EI: 150000 }, user: { EA: 4.5e6, EI: 150000 } };
    var members = [];
    (model.members || []).forEach(function (m) {
      if (idx[m.a] === undefined || idx[m.b] === undefined) { return; }
      var A = all[idx[m.a]], B = all[idx[m.b]];
      var L = Math.hypot(B[0] - A[0], B[1] - A[1]);
      if (!(L > 1e-9)) { return; }
      var ux = (B[0] - A[0]) / L, uy = (B[1] - A[1]) / L;
      var st = (state.memStiff || {})[m.id], mat = (state.memMat || {})[m.id], sec = (state.memSec || {})[m.id];
      var E = (mat && mat.E) || 3.0e7, Asec = 0, Isec = 0;
      if (st) { Asec = st.EA / E; Isec = st.EI / E; }
      else if (mat && sec) { var p = sectionProps(sec); if (p) { Asec = p.A; Isec = p.I; } }
      else { var d0 = DEF[m.type] || DEF.user; Asec = d0.EA / E; Isec = d0.EI / E; }
      var nDiv = effFrameDiv(state, m.id, L), ts = [], ss;
      for (ss = 0; ss <= nDiv; ss++) { ts.push(ss / nDiv); }
      var gi;
      for (gi = 0; gi < all.length; gi++) {
        var px = all[gi][0] - A[0], py = all[gi][1] - A[1];
        var proj = px * ux + py * uy;
        if (proj <= 1e-7 || proj >= L - 1e-7) { continue; }
        if (Math.abs(px * (-uy) + py * ux) < 1e-6) { ts.push(proj / L); }
      }
      ts.sort(function (x, y) { return x - y; });
      var tu = [];
      ts.forEach(function (v) { if (!tu.length || v - tu[tu.length - 1] > 1e-7) { tu.push(v); } });
      tu[0] = 0; tu[tu.length - 1] = 1;
      var ids2 = tu.map(function (tt) { return ensureNode(A[0] + (B[0] - A[0]) * tt, A[1] + (B[1] - A[1]) * tt); });
      for (var k = 0; k + 1 < ids2.length; k++) {
        if (ids2[k] === ids2[k + 1]) { continue; }
        members.push({
          mid: m.id, type: m.type, i: ids2[k], j: ids2[k + 1], L: L,
          xa: tu[k] * L, Lsub: (tu[k + 1] - tu[k]) * L, ux: ux, uy: uy, E: E, A: Asec, I: Isec
        });
      }
    });

    // 连续体剖分（与内置求解器同规则；节点逆时针化）
    var quads = [], areaGrid = {};
    (model.areas || []).forEach(function (a) {
      var corners = a.nodes.map(nodeById).filter(Boolean);
      if (corners.length < 3) { return; }
      var mat = (state.areaMat || {})[a.id] || { E: 3.0e7, nu: 0.2 };
      var sec = (state.areaSec || {})[a.id] || {};
      var t = Math.max(1e-6, sec.t || 0.2);
      var ptype = (sec.kind === "strain" || sec.kind === "shell-thick") ? "PlaneStrain" : "PlaneStress";
      var mtag = matTagOf(mat.E, mat.nu == null ? 0.2 : mat.nu);
      var dv = effAreaDiv(a, nodeById, D, state.areaDiv), nx = dv.nx, ny = dv.ny;
      if (corners.length >= 4) {
        var P = [corners[0], corners[1], corners[2], corners[3]].map(function (n) { return [n.x, n.y]; });
        areaGrid[a.id] = { nx: nx, ny: ny, P: P, t: t };
        var sub = quadSubCells(P, nx, ny), gid = [], r, c;
        for (r = 0; r <= ny; r++) {
          gid[r] = [];
          for (c = 0; c <= nx; c++) {
            var onC = (r === 0 && c === 0) ? 0 : (r === 0 && c === nx) ? 1 : (r === ny && c === nx) ? 2 : (r === ny && c === 0) ? 3 : -1;
            gid[r][c] = onC >= 0 ? idx[a.nodes[onC]] : ensureNode(sub.grid[r][c][0], sub.grid[r][c][1]);
          }
        }
        sub.cells.forEach(function (cell) {
          var g = [gid[Math.floor(cell[0] / (nx + 1))][cell[0] % (nx + 1)], gid[Math.floor(cell[1] / (nx + 1))][cell[1] % (nx + 1)],
                   gid[Math.floor(cell[2] / (nx + 1))][cell[2] % (nx + 1)], gid[Math.floor(cell[3] / (nx + 1))][cell[3] % (nx + 1)]];
          var cp = g.map(function (gi) { return all[gi]; });
          if (signedArea(cp) < 0) { g = [g[0], g[3], g[2], g[1]]; }
          quads.push({ aid: a.id, etype: a.etype, kind: "q4", g: g, t: t, ptype: ptype, mtag: mtag });
        });
      } else {
        var P3 = [corners[0], corners[1], corners[2]].map(function (n) { return [n.x, n.y]; });
        var sub3 = triSubCells(P3, nx), gmap = [];
        sub3.pts.forEach(function (p) {
          var gi = -1, k;
          for (k = 0; k < 3; k++) {
            if (Math.hypot(p[0] - P3[k][0], p[1] - P3[k][1]) < 1e-9) { gi = idx[a.nodes[k]]; break; }
          }
          gmap.push(gi >= 0 ? gi : ensureNode(p[0], p[1]));
        });
        sub3.tris.forEach(function (tr) {
          var g = [gmap[tr[0]], gmap[tr[1]], gmap[tr[2]]];
          var cp = g.map(function (gi) { return all[gi]; });
          if (signedArea(cp) < 0) { g = [g[0], g[2], g[1]]; }
          quads.push({ aid: a.id, etype: a.etype, kind: "cst", g: g, t: t, ptype: ptype, mtag: mtag });
        });
      }
    });

    // 面荷载 -> 一致等效节点力（与内置求解器同式，保证合力一致）
    var extraNodal = {};
    function addN(aindex, fx, fy) {
      var e = extraNodal[aindex] || (extraNodal[aindex] = { fx: 0, fy: 0 });
      e.fx += fx; e.fy += fy;
    }
    function q4shape(xi, et) {
      return [(1 - xi) * (1 - et) / 4, (1 + xi) * (1 - et) / 4, (1 + xi) * (1 + et) / 4, (1 - xi) * (1 + et) / 4];
    }
    quads.forEach(function (qc) {
      var ld = (state.areaLoads || {})[qc.aid];
      if (!ld || (!ld.qx && !ld.qy)) { return; }
      var qx = ld.qx || 0, qy = ld.qy || 0;
      if (qc.kind === "q4") {
        var coords = qc.g.map(function (gi) { return all[gi]; });
        var g = 1 / Math.sqrt(3);
        [[-g, -g, 1], [g, -g, 1], [g, g, 1], [-g, g, 1]].forEach(function (gp) {
          var xi = gp[0], et = gp[1], w = gp[2], N = q4shape(xi, et);
          var J = [[0, 0], [0, 0]], a;
          for (a = 0; a < 4; a++) {
            var dNx = [-(1 - et) / 4, (1 - et) / 4, (1 + et) / 4, -(1 + et) / 4][a];
            var dNe = [-(1 - xi) / 4, -(1 + xi) / 4, (1 + xi) / 4, (1 - xi) / 4][a];
            J[0][0] += dNx * coords[a][0]; J[0][1] += dNx * coords[a][1];
            J[1][0] += dNe * coords[a][0]; J[1][1] += dNe * coords[a][1];
          }
          var det = Math.abs(J[0][0] * J[1][1] - J[0][1] * J[1][0]);
          for (a = 0; a < 4; a++) { addN(qc.g[a], N[a] * qx * qc.t * det * w, N[a] * qy * qc.t * det * w); }
        });
      } else {
        var p1 = all[qc.g[0]], p2 = all[qc.g[1]], p3 = all[qc.g[2]];
        var Ar = Math.abs((p2[0] - p1[0]) * (p3[1] - p1[1]) - (p3[0] - p1[0]) * (p2[1] - p1[1])) / 2;
        for (var a3 = 0; a3 < 3; a3++) { addN(qc.g[a3], qx * qc.t * Ar / 3, qy * qc.t * Ar / 3); }
      }
    });

    // 杆端集中力（a≈端点）转节点力（局部+y -> 整体）
    var endNodal = {};
    function addEndN(modelId, fx, fy) {
      var e = endNodal[modelId] || (endNodal[modelId] = { fx: 0, fy: 0 });
      e.fx += fx; e.fy += fy;
    }
    // 单元关联（按分析节点序号，约束判断用；重复/共边节点合并到同一分析节点，
    // 故必须按分析节点记录，否则副本节点会被误判为孤立而多加约束）
    var aiFrame = {}, aiQuad = {}, aiMids = {};
    Object.keys(idx).forEach(function (k) { (aiMids[idx[k]] = aiMids[idx[k]] || []).push(k); });
    members.forEach(function (mb) { aiFrame[mb.i] = true; aiFrame[mb.j] = true; });
    quads.forEach(function (qc) { qc.g.forEach(function (gi) { aiQuad[gi] = true; }); });

    return {
      kind: kind, all: all, idx: idx, xnode: xnode, materials: materials,
      members: members, quads: quads, areaGrid: areaGrid,
      aiMids: aiMids, aiFrame: aiFrame, aiQuad: aiQuad,
      extraNodal: extraNodal, endNodal: endNodal,
      nNode: (model.nodes || []).length, nMember: (model.members || []).length, nArea: (model.areas || []).length
    };
  }  /* ---------------- 导出：meta -> xara .py 脚本 ---------------- */
  function buildScript(state, opts) {
    opts = opts || {};
    var meta = exportMeta(state, opts);
    if (meta.kind === "empty") { throw new Error("模型为空，无可导出的内容。"); }
    if (meta.kind === "mixed" || opts.force3D) {
      return buildScript3D(state, opts);
    }
    var model = state.model || {};
    var ndf = meta.kind === "frame" ? 3 : 2;
    var L = [];
    L.push("# -*- coding: utf-8 -*-");
    L.push("# FrameLab -> xara 导出脚本（OpenSees 线弹性静力分析）");
    L.push("# 杆件 " + meta.nMember + "（杆段 " + meta.members.length + "，NSUB/memDiv 等分 + 杆上节点投影切分） / 面剖分子单元 " + meta.quads.length + " / 分析节点 " + meta.all.length);
    L.push("# 单位：m / kN / kN*m / kPa（与 FrameLab 内部一致）");
    L.push("import os, sys, json");
    L.push("def _bootstrap_dll():");
    L.push("    try:");
    L.push("        import site");
    L.push("        sps = list(site.getsitepackages()) + [site.getusersitepackages()]");
    L.push("    except Exception:");
    L.push("        sps = []");
    L.push("    for sp in sps:");
    L.push("        base = os.path.dirname(os.path.dirname(sp))");
    L.push("        for d in (os.path.join(base, 'Library', 'bin'), os.path.join(base, 'bin'),");
    L.push("                os.path.join(sys.prefix, 'Library', 'bin')):");
    L.push("            if os.path.isdir(d):");
    L.push("                os.environ['PATH'] = d + os.pathsep + os.environ.get('PATH', '')");
    L.push("                try:");
    L.push("                    os.add_dll_directory(d)");
    L.push("                except Exception:");
    L.push("                    pass");
    L.push("_bootstrap_dll()");
    L.push("import xara");
    L.push("model = xara.Model(ndm=2, ndf=" + ndf + ")");
    meta.materials.forEach(function (mt) {
      var nu = mt.nu == null ? 0.3 : mt.nu;
      L.push("model.material('ElasticIsotropic', " + mt.tag + ", " + num(mt.E) + ", " + num(nu) + ")");
    });
    meta.all.forEach(function (p, i) {
      L.push("model.node(" + (i + 1) + ", " + num(p[0]) + ", " + num(p[1]) + ")");
    });
    function bcOf(modelId) {
      var n = (model.nodes || []).find(function (nn) { return nn.id === modelId; });
      return (n && n.bc) || {};
    }
    // 约束按分析节点合并（或逻辑）：共边/重复建模会产生同坐标多模型节点，
    // 若逐个下 fix，同一自由度会被约束两次而报错；合并后与内置求解器一致。
    // 关联性同样按分析节点判定（某副本有关联即视为有关联），否则副本会被
    // 误判孤立而多加全约束，把共边界面钉死（曾实测悬臂墙刚度偏大 84 倍）。
    var fixMerge = {};
    Object.keys(meta.aiMids || {}).forEach(function (ai) {
      var e = fixMerge[ai] || (fixMerge[ai] = { fx: 0, fy: 0, rz: 0 });
      // 孤立分析节点（无任何单元关联）全部约束，避免刚度奇异；
      // 无框架连接节点的 rz 自动约束（对应内置的零刚度钻孔自由度处理）
      var isolated = !meta.aiFrame[ai] && !meta.aiQuad[ai];
      if (isolated) { e.fx = 1; e.fy = 1; }
      (meta.aiMids[ai] || []).forEach(function (mid) {
        var bc = bcOf(+mid);
        if (bc.ux || isolated) { e.fx = 1; }
        if (bc.uy || isolated) { e.fy = 1; }
        if (ndf === 3 && (bc.rz || isolated || !meta.aiFrame[ai])) { e.rz = 1; }
      });
    });
    Object.keys(fixMerge).forEach(function (ai) {
      var xt = (+ai) + 1, e = fixMerge[ai];
      if (ndf === 3) {
        L.push("model.fix(" + xt + ", " + e.fx + ", " + e.fy + ", " + e.rz + ")");
      } else {
        L.push("model.fix(" + xt + ", " + e.fx + ", " + e.fy + ")");
      }
    });
    var eleTag = 0;
    if (meta.kind === "frame") { L.push("model.geomTransf('Linear', 1)"); }
    meta.members.forEach(function (mb) {
      eleTag += 1; mb.eleTag = eleTag;
      L.push("model.element('elasticBeamColumn', " + eleTag + ", [" + (mb.i + 1) + ", " + (mb.j + 1) + "], " +
        num(mb.A) + ", " + num(mb.E) + ", " + num(mb.I) + ", 1)");
    });
    meta.quads.forEach(function (qc) {
      eleTag += 1; qc.eleTag = eleTag;
      var tags = qc.g.map(function (gi) { return gi + 1; });
      if (qc.kind === "q4") {
        L.push("model.element('Quad', " + eleTag + ", [" + tags.join(", ") + "], (" + num(qc.t) + ", " + pyStr(qc.ptype) + ", " + qc.mtag + "))");
      } else {
        L.push("model.element('Tri31', " + eleTag + ", [" + tags.join(", ") + "], " + num(qc.t) + ", " + pyStr(qc.ptype) + ", " + qc.mtag + ")");
      }
    });
    L.push("model.pattern('Plain', 1, 'Linear')");
    // 杆件荷载：按杆段逐段下 eleLoad，并把端点集中力折算为等效节点力（供下段使用）
    var byMid = {};
    meta.members.forEach(function (mb) { (byMid[mb.mid] = byMid[mb.mid] || []).push(mb); });
    Object.keys(byMid).forEach(function (k) {
      byMid[k].sort(function (a, b) { return a.xa - b.xa; });
    });
    function segAt(segs, a, L) {
      var k;
      for (k = 0; k < segs.length; k++) {
        var s0 = segs[k].xa, s1 = s0 + segs[k].Lsub;
        if (a >= s0 - 1e-9 * L && (a < s1 - 1e-9 * L || k === segs.length - 1)) { return segs[k]; }
      }
      return segs[segs.length - 1];
    }
    var eleLines = [];
    Object.keys((state.memLoads || {})).forEach(function (mid) {
      var segs = byMid[mid];
      if (!segs || !segs.length || !(segs[0].L > 1e-12)) { return; }
      var Lfull = segs[0].L;
      (state.memLoads[mid] || []).forEach(function (ld) {
        if (ld.type === "udl") {
          if (Math.abs(ld.q) < 1e-12) { return; }
          segs.forEach(function (sg) {
            eleLines.push("model.eleLoad('-ele', " + sg.eleTag + ", '-type', '-beamUniform', " + num(ld.q) + ", pattern=1)");
          });
        } else if (ld.type === "point") {
          if (Math.abs(ld.P) < 1e-12) { return; }
          if (ld.a <= 1e-9 * Lfull || ld.a >= Lfull * (1 - 1e-9)) {
            var mm = (model.members || []).find(function (m) { return m.id === +mid; });
            var nodeModelId = mm ? ((ld.a <= 1e-9 * Lfull) ? mm.a : mm.b) : null;
            if (nodeModelId != null) {
              var ee = meta.endNodal[nodeModelId] || (meta.endNodal[nodeModelId] = { fx: 0, fy: 0 });
              ee.fx += -segs[0].uy * ld.P; ee.fy += segs[0].ux * ld.P;
            }
          } else {
            var sg = segAt(segs, ld.a, Lfull);
            var frac = Math.min(1, Math.max(0, (ld.a - sg.xa) / sg.Lsub));
            eleLines.push("model.eleLoad('-ele', " + sg.eleTag + ", '-type', '-beamPoint', " + num(ld.P) + ", " + num(frac) + ", pattern=1)");
          }
        } else if (ld.type === "trap") {
          var K = 20, span = Math.max(0, ld.d - ld.c);
          if (!(span > 1e-12)) { return; }
          for (var k = 0; k < K; k++) {
            var xm = ld.c + span * (k + 0.5) / K;
            var qm = ld.q1 + (ld.q2 - ld.q1) * (xm - ld.c) / span;
            var Pm = qm * span / K;
            if (Math.abs(Pm) < 1e-12) { continue; }
            if (xm <= 1e-9 * Lfull || xm >= Lfull * (1 - 1e-9)) { continue; }
            var sg2 = segAt(segs, xm, Lfull);
            var frac2 = Math.min(1, Math.max(0, (xm - sg2.xa) / sg2.Lsub));
            eleLines.push("model.eleLoad('-ele', " + sg2.eleTag + ", '-type', '-beamPoint', " + num(Pm) + ", " + num(frac2) + ", pattern=1)");
          }
        }
      });
    });
    // 节点荷载（含面荷载等效节点力与杆端集中力折算）
    Object.keys(meta.idx).forEach(function (mid) {
      var ai = meta.idx[mid], xt = ai + 1;
      var nl = (state.nodeLoads || {})[mid] || {};
      var ex = (meta.extraNodal[ai] || { fx: 0, fy: 0 });
      var en = (meta.endNodal[mid] || { fx: 0, fy: 0 });
      var fx = (nl.fx || 0) + ex.fx + en.fx, fy = (nl.fy || 0) + ex.fy + en.fy, mz = nl.mz || 0;
      if (Math.abs(fx) < 1e-12 && Math.abs(fy) < 1e-12 && Math.abs(mz) < 1e-12) { return; }
      if (ndf === 3) { L.push("model.load(" + xt + ", " + num(fx) + ", " + num(fy) + ", " + num(mz) + ", pattern=1)"); }
      else { L.push("model.load(" + xt + ", " + num(fx) + ", " + num(fy) + ", pattern=1)"); }
    });
    // 单元荷载行（节点荷载之后施加，同属 pattern 1）
    eleLines.forEach(function (ln) { L.push(ln); });
    L.push("model.system('BandSPD')");
    L.push("model.numberer('RCM')");
    L.push("model.constraints('Plain')");
    L.push("model.integrator('LoadControl', 1.0)");
    L.push("model.algorithm('Linear')");
    L.push("model.analysis('Static')");
    L.push("out = {'analyze': None, 'ok': False, 'error': None, 'disps': {}, 'reactions': {}, 'beams': {}, 'conts': {}}");
    L.push("try:");
    L.push("    out['analyze'] = int(model.analyze(1))");
    L.push("    model.reactions()");
    for (var qi = 0; qi < meta.all.length; qi++) {
      var xt2 = qi + 1;
      L.push("    out['disps'][" + xt2 + "] = [float(v) for v in model.nodeDisp(" + xt2 + ")]");
      L.push("    out['reactions'][" + xt2 + "] = [float(v) for v in model.nodeReaction(" + xt2 + ")]");
    }
    meta.members.forEach(function (mb) {
      L.push("    out['beams'][" + mb.eleTag + "] = [float(v) for v in model.eleResponse(" + mb.eleTag + ", 'localForces')]");
    });
    meta.quads.forEach(function (qc) {
      L.push("    _s" + qc.eleTag + " = [float(v) for v in model.eleResponse(" + qc.eleTag + ", 'stresses')]");
      L.push("    _n" + qc.eleTag + " = len(_s" + qc.eleTag + ") // 3");
      L.push("    out['conts'][" + qc.eleTag + "] = [sum(_s" + qc.eleTag + "[i::3]) / max(1, _n" + qc.eleTag + ") for i in range(3)]");
    });
    L.push("    out['ok'] = (out['analyze'] == 0)");
    L.push("    if out['analyze'] != 0:");
    L.push("        out['error'] = 'analyze returned ' + str(out['analyze'])");
    L.push("except Exception as _ex:");
    L.push("    out['ok'] = False");
    L.push("    out['error'] = str(_ex)");
    L.push("print('FRAMELAB_JSON_BEGIN')");
    L.push("print(json.dumps(out))");
    L.push("print('FRAMELAB_JSON_END')");
    return { script: L.join("\n") + "\n", meta: meta, kind: meta.kind, ndf: ndf };
  }

  /* ============ 3D 退化平面路径（混合模型；纯模型加 opts.force3D 也可走） ============
   * ndm=3, ndf=6，全部节点 z=0；uz/rx/ry 逐节点约束，仅面内荷载。
   * 关键实测结论（xara 0.0.34 + opensees 0.1.31）：
   * - ShellMITC4 的 stresses/strains 查询返回全零，ShellDKGQ 在膜截面下奇异；
   *   采用 ASDShellQ4 / ASDShellT3 + ElasticMembranePlateSection(E,nu,t)。
   * - ASDShell 'stresses' 返回每高斯点 8 个截面广义力，前 3 个为
   *   [Nyy, Nxx, Nxy]（X/Y 向拉伸标定：Xtension 下 [8k+1]=P/H，Ytension 下 [8k]=P/W）；
   *   膜应力 = N/t。三角形单元为 3 高斯点 × 8。
   * - 3D elasticBeamColumn 的 eleLoad（beamUniform/beamPoint）结果错误
   *   （总量对、分布错：悬臂 UDL 下固端弯矩≈0、方向反），故杆件荷载一律转为
   *   一致等效节点力（与内置求解器同式，Euler 梁 + 等截面下为精确等效）。
   * - 3D 梁 localForces 为 12 分量 [N1,Vy1,Vz1,T1,My1,Mz1,N2,...]；
   *   面内映射：axial=-N1，S=Vy1+A(x)，M=-(Mz1-Vy1*x-J(x))，与 2D 同式。
   * - 平面应变 (E,nu) 按等效平面应力 (E/(1-nu^2), nu/(1-nu)) 折算。
   * - drilling 刚度≈0：点连接只传平动（铰接），此处不做刚域加强（另行立项）。
   */
  var GP4 = [-0.8611363116, -0.3399810436, 0.3399810436, 0.8611363116];
  var GW4 = [0.3478548451, 0.6521451549, 0.6521451549, 0.3478548451];
  function hermite3(x, L) {
    var t = x / L, t2 = t * t, t3 = t2 * t;
    return [1 - 3 * t2 + 2 * t3, L * (t - 2 * t2 + t3), 3 * t2 - 2 * t3, L * (-t2 + t3)];
  }
  function sideQ3(ld, x) {
    if (ld.type === "udl") { return ld.q; }
    if (ld.type === "trap") {
      if (x < ld.c || x > ld.d) { return 0; }
      return ld.q1 + (ld.q2 - ld.q1) * (x - ld.c) / ((ld.d - ld.c) || 1);
    }
    return 0;
  }
  function segLoadVector(seg, loads) {
    var p = [0, 0, 0, 0, 0, 0], xa = seg.xa, Ls = seg.Lsub, Lm = seg.L;
    (loads || []).forEach(function (ld) {
      var k;
      if (ld.type === "point") {
        if ((ld.a >= xa - 1e-9) && (ld.a < xa + Ls - 1e-9 || xa + Ls >= Lm - 1e-9)) {
          var xx = Math.min(Math.max(ld.a - xa, 0), Ls), N = hermite3(xx, Ls);
          for (k = 0; k < 6; k++) { p[k] += [0, N[0] * ld.P, N[1] * ld.P, 0, N[2] * ld.P, N[3] * ld.P][k]; }
        }
      } else {
        var s0 = xa, s1 = xa + Ls;
        if (ld.type === "trap") { s0 = Math.max(ld.c, xa); s1 = Math.min(ld.d, xa + Ls); }
        if (s1 <= s0) { return; }
        for (k = 0; k < 4; k++) {
          var x = 0.5 * (s1 - s0) * GP4[k] + 0.5 * (s0 + s1), jac = 0.5 * (s1 - s0), q = sideQ3(ld, x);
          if (q === 0) { continue; }
          var Nh = hermite3(x - xa, Ls), wj = q * jac * GW4[k];
          p[1] += Nh[0] * wj; p[2] += Nh[1] * wj; p[4] += Nh[2] * wj; p[5] += Nh[3] * wj;
        }
      }
    });
    return p;
  }
  function segResultants(mid, Lfull, x0, x1, memLoads) {
    var A = 0, Bb = 0, loads = (memLoads || {})[mid] || [];
    loads.forEach(function (ld) {
      var k;
      if (ld.type === "point") { if (ld.a > x0 && ld.a <= x1) { A += ld.P; Bb += ld.P * ld.a; } return; }
      var s0, s1;
      if (ld.type === "udl") { s0 = Math.max(0, x0); s1 = Math.min(Lfull, x1); }
      else { s0 = Math.max(ld.c, x0); s1 = Math.min(ld.d, x1); }
      if (s1 <= s0) { return; }
      for (k = 0; k < 4; k++) {
        var x = 0.5 * (s1 - s0) * GP4[k] + 0.5 * (s0 + s1), jac = 0.5 * (s1 - s0), q = sideQ3(ld, x);
        A += q * jac * GW4[k]; Bb += q * x * jac * GW4[k];
      }
    });
    return { A: A, Bb: Bb };
  }

  function exportMeta3D(state, opts) {
    opts = opts || {};
    var model = state.model || { nodes: [], members: [], areas: [] };
    var D = state.D || {};
    var nodeById = function (id) { return (model.nodes || []).find(function (n) { return n.id === id; }); };
    var all = [], reg = {};
    function ensureNode(x, y) {
      var k = (Math.round(x * 1e6) / 1e6) + "," + (Math.round(y * 1e6) / 1e6);
      if (reg[k] !== undefined) { return reg[k]; }
      reg[k] = all.length; all.push([x, y]); return reg[k];
    }
    var idx = {};
    (model.nodes || []).forEach(function (n) { idx[n.id] = ensureNode(n.x, n.y); });

    // 截面去重：(E,nu,t)；平面应变折算为等效平面应力
    var secMap = {}, sections = [];
    function secTagOf(E, nu, t, kind) {
      var e2 = E, n2 = (nu == null ? 0.2 : nu);
      if (kind === "strain" || kind === "shell-thick") { e2 = E / (1 - n2 * n2); n2 = n2 / (1 - n2); }
      var k = e2.toPrecision(12) + "|" + n2.toPrecision(12) + "|" + t.toPrecision(12);
      if (!secMap[k]) { secMap[k] = sections.length + 1; sections.push({ tag: secMap[k], E: e2, nu: n2, t: t }); }
      return secMap[k];
    }

    // 连续体剖分（与 2D 路径同规则；节点逆时针化）
    function effAreaDiv3(a) {
      var nx = clampInt(D.AMESH != null ? D.AMESH : 2, 1, 50), ny = nx, o = (state.areaDiv || {})[a.id];
      if (o) { nx = clampInt(o.nx || nx, 1, 50); ny = clampInt(o.ny || ny, 1, 50); }
      if (o && o.hmax > 0) {
        var pts = a.nodes.map(nodeById).filter(Boolean), Lmax = 0, i;
        for (i = 0; i < pts.length; i++) {
          var p = pts[i], q = pts[(i + 1) % pts.length];
          Lmax = Math.max(Lmax, Math.hypot(q.x - p.x, q.y - p.y));
        }
        if (Lmax > 0) { var k = Math.min(50, Math.ceil(Lmax / o.hmax)); nx = Math.max(nx, k); ny = Math.max(ny, k); }
      }
      return { nx: nx, ny: ny };
    }
    var quads = [], areaGrid = {};
    (model.areas || []).forEach(function (a) {
      var corners = a.nodes.map(nodeById).filter(Boolean);
      if (corners.length < 3) { return; }
      if (corners.length === 3) {
        throw new Error("混合模型的三角形面单元（面" + a.id + "）暂不支持 3D 路径：OpenSeesRT 的 ASDShellT3 单元在细分网格下偏软约 2~3 倍（已实测），为保正确请改为四边形面，或使用内置求解器。");
      }
      var mat = (state.areaMat || {})[a.id] || { E: 3.0e7, nu: 0.2 };
      var sec = (state.areaSec || {})[a.id] || {};
      var t = Math.max(1e-6, sec.t || 0.2);
      var stag = secTagOf(mat.E, mat.nu == null ? 0.2 : mat.nu, t, sec.kind);
      var dv = effAreaDiv3(a), nx = dv.nx, ny = dv.ny;
      if (corners.length >= 4) {
        var P = [corners[0], corners[1], corners[2], corners[3]].map(function (n) { return [n.x, n.y]; });
        areaGrid[a.id] = { nx: nx, ny: ny, P: P, t: t };
        var sub = quadSubCells(P, nx, ny), gid = [], r, c;
        for (r = 0; r <= ny; r++) {
          gid[r] = [];
          for (c = 0; c <= nx; c++) {
            var onC = (r === 0 && c === 0) ? 0 : (r === 0 && c === nx) ? 1 : (r === ny && c === nx) ? 2 : (r === ny && c === 0) ? 3 : -1;
            gid[r][c] = onC >= 0 ? idx[a.nodes[onC]] : ensureNode(sub.grid[r][c][0], sub.grid[r][c][1]);
          }
        }
        sub.cells.forEach(function (cell) {
          var g = [gid[Math.floor(cell[0] / (nx + 1))][cell[0] % (nx + 1)], gid[Math.floor(cell[1] / (nx + 1))][cell[1] % (nx + 1)],
                   gid[Math.floor(cell[2] / (nx + 1))][cell[2] % (nx + 1)], gid[Math.floor(cell[3] / (nx + 1))][cell[3] % (nx + 1)]];
          var cp = g.map(function (gi) { return all[gi]; });
          if (signedArea(cp) < 0) { g = [g[0], g[3], g[2], g[1]]; }
          quads.push({ aid: a.id, etype: a.etype, kind: "q4", g: g, t: t, stag: stag });
        });
      } else {
        var P3 = [corners[0], corners[1], corners[2]].map(function (n) { return [n.x, n.y]; });
        var sub3 = triSubCells(P3, nx), gmap = [];
        sub3.pts.forEach(function (p) {
          var gi = -1, k;
          for (k = 0; k < 3; k++) {
            if (Math.hypot(p[0] - P3[k][0], p[1] - P3[k][1]) < 1e-9) { gi = idx[a.nodes[k]]; break; }
          }
          gmap.push(gi >= 0 ? gi : ensureNode(p[0], p[1]));
        });
        sub3.tris.forEach(function (tr) {
          var g = [gmap[tr[0]], gmap[tr[1]], gmap[tr[2]]];
          var cp = g.map(function (gi) { return all[gi]; });
          if (signedArea(cp) < 0) { g = [g[0], g[2], g[1]]; }
          quads.push({ aid: a.id, etype: a.etype, kind: "cst", g: g, t: t, stag: stag });
        });
      }
    });

    // 杆件刚度（与 2D 同规则）+ 在落于杆上的分析节点处切分（与连续体逐节点共享）
    var DEF3 = { col: { EA: 4.5e6, EI: 93750 }, beam: { EA: 4.5e6, EI: 150000 }, user: { EA: 4.5e6, EI: 150000 } };
    var segs = [];
    (model.members || []).forEach(function (m) {
      if (idx[m.a] === undefined || idx[m.b] === undefined) { return; }
      var A = all[idx[m.a]], B = all[idx[m.b]];
      var L = Math.hypot(B[0] - A[0], B[1] - A[1]);
      if (!(L > 1e-9)) { return; }
      var ux = (B[0] - A[0]) / L, uy = (B[1] - A[1]) / L;
      var st = (state.memStiff || {})[m.id], mat = (state.memMat || {})[m.id], sec = (state.memSec || {})[m.id];
      var E, Asec, Isec, nu;
      if (st) { E = 1.0; Asec = st.EA; Isec = st.EI; nu = 0.3; }
      else if (mat && sec) {
        var p = sectionProps(sec);
        if (!p) { return; }
        E = mat.E; Asec = p.A; Isec = p.I; nu = (mat.nu == null ? 0.3 : mat.nu);
      }
      else { var d0 = DEF3[m.type] || DEF3.user; E = 1.0; Asec = d0.EA; Isec = d0.EI; nu = 0.3; }
      var nDiv = effFrameDiv(state, m.id, L), ts = [], sDiv;
      for (sDiv = 0; sDiv <= nDiv; sDiv++) { ts.push(sDiv / nDiv); }
      var gi;
      for (gi = 0; gi < all.length; gi++) {
        var px = all[gi][0] - A[0], py = all[gi][1] - A[1];
        var proj = px * ux + py * uy;
        if (proj <= 1e-7 || proj >= L - 1e-7) { continue; }
        if (Math.abs(px * (-uy) + py * ux) < 1e-6) { ts.push(proj / L); }
      }
      ts.sort(function (x, y) { return x - y; });
      var tu = [];
      ts.forEach(function (v) { if (!tu.length || v - tu[tu.length - 1] > 1e-7) { tu.push(v); } });
      tu[0] = 0; tu[tu.length - 1] = 1;
      var ids2 = tu.map(function (tt) { return ensureNode(A[0] + (B[0] - A[0]) * tt, A[1] + (B[1] - A[1]) * tt); });
      for (var k = 0; k + 1 < ids2.length; k++) {
        if (ids2[k] === ids2[k + 1]) { continue; }
        segs.push({
          mid: m.id, type: m.type, i: ids2[k], j: ids2[k + 1], L: L,
          xa: tu[k] * L, Lsub: (tu[k + 1] - tu[k]) * L, ux: ux, uy: uy,
          E: E, A: Asec, I: Isec, nu: nu
        });
      }
    });

    // 荷载 -> 一致等效节点力（面内 Fx, Fy, Mz）
    var nodal = {};
    function addN3(ai, fx, fy, mz) {
      var e = nodal[ai] || (nodal[ai] = { fx: 0, fy: 0, mz: 0 });
      e.fx += fx; e.fy += fy; e.mz += mz;
    }
    Object.keys(idx).forEach(function (mid) {
      var nl = (state.nodeLoads || {})[mid] || {};
      if ((nl.fx || 0) || (nl.fy || 0) || (nl.mz || 0)) { addN3(idx[mid], nl.fx || 0, nl.fy || 0, nl.mz || 0); }
    });
    var byMid = {};
    segs.forEach(function (sg) { (byMid[sg.mid] = byMid[sg.mid] || []).push(sg); });
    Object.keys((state.memLoads || {})).forEach(function (mid) {
      var mm = (model.members || []).find(function (x) { return x.id === +mid; });
      var list = byMid[mid] || [];
      if (!mm || !list.length) { return; }
      var Lm = list[0].L;
      (state.memLoads[mid] || []).forEach(function (ld) {
        var isEnd = (ld.type === "point" && (ld.a <= 1e-9 * Lm || ld.a >= Lm * (1 - 1e-9)));
        if (isEnd) {
          // 端点集中力折为等效节点力（局部+y -> 整体）
          var nm = (ld.a <= 1e-9 * Lm) ? mm.a : mm.b;
          var sg0 = list[0];
          if (nm != null && idx[nm] !== undefined) { addN3(idx[nm], -sg0.uy * ld.P, sg0.ux * ld.P, 0); }
          return;
        }
        list.forEach(function (sg) {
          if (ld.type === "point" && !(ld.a > sg.xa - 1e-9 * Lm && (ld.a < sg.xa + sg.Lsub - 1e-9 * Lm || sg.xa + sg.Lsub >= Lm - 1e-9 * Lm))) { return; }
          var p = segLoadVector(sg, [ld]);
          // 局部 (0,Fy1,M1,0,Fy2,M2) -> 整体
          addN3(sg.i, -sg.uy * p[1], sg.ux * p[1], p[2]);
          addN3(sg.j, -sg.uy * p[4], sg.ux * p[4], p[5]);
        });
      });
    });
    function q4shape3(xi, et) {
      return [(1 - xi) * (1 - et) / 4, (1 + xi) * (1 - et) / 4, (1 + xi) * (1 + et) / 4, (1 - xi) * (1 + et) / 4];
    }
    quads.forEach(function (qc) {
      var ld = (state.areaLoads || {})[qc.aid];
      if (!ld || (!ld.qx && !ld.qy)) { return; }
      var qx = ld.qx || 0, qy = ld.qy || 0;
      if (qc.kind === "q4") {
        var coords = qc.g.map(function (gi) { return all[gi]; });
        var g = 1 / Math.sqrt(3);
        [[-g, -g, 1], [g, -g, 1], [g, g, 1], [-g, g, 1]].forEach(function (gp) {
          var xi = gp[0], et = gp[1], w = gp[2], NN = q4shape3(xi, et);
          var J = [[0, 0], [0, 0]], a;
          for (a = 0; a < 4; a++) {
            var dNx = [-(1 - et) / 4, (1 - et) / 4, (1 + et) / 4, -(1 + et) / 4][a];
            var dNe = [-(1 - xi) / 4, -(1 + xi) / 4, (1 + xi) / 4, (1 - xi) / 4][a];
            J[0][0] += dNx * coords[a][0]; J[0][1] += dNx * coords[a][1];
            J[1][0] += dNe * coords[a][0]; J[1][1] += dNe * coords[a][1];
          }
          var det = Math.abs(J[0][0] * J[1][1] - J[0][1] * J[1][0]);
          for (a = 0; a < 4; a++) { addN3(qc.g[a], NN[a] * qx * qc.t * det * w, NN[a] * qy * qc.t * det * w, 0); }
        });
      } else {
        var p1 = all[qc.g[0]], p2 = all[qc.g[1]], p3 = all[qc.g[2]];
        var Ar = Math.abs((p2[0] - p1[0]) * (p3[1] - p1[1]) - (p3[0] - p1[0]) * (p2[1] - p1[1])) / 2;
        for (var a3 = 0; a3 < 3; a3++) { addN3(qc.g[a3], qx * qc.t * Ar / 3, qy * qc.t * Ar / 3, 0); }
      }
    });

    var ai2mid = {};
    Object.keys(idx).forEach(function (k) { ai2mid[idx[k]] = k; });
    return {
      kind: "mixed3d", all: all, idx: idx, ai2mid: ai2mid, sections: sections,
      members: segs, quads: quads, areaGrid: areaGrid, nodal: nodal,
      nNode: (model.nodes || []).length, nMember: (model.members || []).length, nArea: (model.areas || []).length
    };
  }

  function buildScript3D(state, opts) {
    opts = opts || {};
    var meta = exportMeta3D(state, opts);
    if (!meta.members.length && !meta.quads.length) { throw new Error("模型为空，无可导出的内容。"); }
    var model = state.model || {};
    var L = [];
    L.push("# -*- coding: utf-8 -*-");
    L.push("# FrameLab -> xara 导出脚本（3D 退化平面：混合模型专用，线弹性静力）");
    L.push("# 杆段 " + meta.members.length + " / 面剖分子单元 " + meta.quads.length + " / 分析节点 " + meta.all.length);
    L.push("# 单位：m / kN / kN*m / kPa（与 FrameLab 内部一致）；z=0 平面，面外自由度全约束");
    L.push("import os, sys, json");
    L.push("def _bootstrap_dll():");
    L.push("    try:");
    L.push("        import site");
    L.push("        sps = list(site.getsitepackages()) + [site.getusersitepackages()]");
    L.push("    except Exception:");
    L.push("        sps = []");
    L.push("    for sp in sps:");
    L.push("        base = os.path.dirname(os.path.dirname(sp))");
    L.push("        for d in (os.path.join(base, 'Library', 'bin'), os.path.join(base, 'bin'),");
    L.push("                os.path.join(sys.prefix, 'Library', 'bin')):");
    L.push("            if os.path.isdir(d):");
    L.push("                os.environ['PATH'] = d + os.pathsep + os.environ.get('PATH', '')");
    L.push("                try:");
    L.push("                    os.add_dll_directory(d)");
    L.push("                except Exception:");
    L.push("                    pass");
    L.push("_bootstrap_dll()");
    L.push("import xara");
    L.push("model = xara.Model(ndm=3, ndf=6)");
    meta.sections.forEach(function (s) {
      L.push("model.section('ElasticMembranePlateSection', " + s.tag + ", " + num(s.E) + ", " + num(s.nu) + ", " + num(s.t) + ")");
    });
    meta.all.forEach(function (p, i) {
      L.push("model.node(" + (i + 1) + ", " + num(p[0]) + ", " + num(p[1]) + ", 0.0)");
    });
    function bcOf3(modelId) {
      var n = (model.nodes || []).find(function (nn) { return nn.id === modelId; });
      return (n && n.bc) || {};
    }
    // 约束按分析节点合并（或逻辑），同 2D 路径：重复节点不再重复下 fix；
    // 无任何单元关联的孤立模型节点全约束，避免刚度奇异。
    var used3 = {};
    (meta.members || []).forEach(function (sg) { used3[sg.i] = true; used3[sg.j] = true; });
    (meta.quads || []).forEach(function (qc) { qc.g.forEach(function (gi) { used3[gi] = true; }); });
    var fixMerge3 = {};
    Object.keys(meta.idx).forEach(function (mid) {
      var ai = meta.idx[mid], bc = bcOf3(+mid);
      var e = fixMerge3[ai] || (fixMerge3[ai] = { fx: 0, fy: 0, rz: 0 });
      if (used3[ai] !== true) { e.fx = 1; e.fy = 1; e.rz = 1; }
      if (bc.ux) { e.fx = 1; }
      if (bc.uy) { e.fy = 1; }
      if (bc.rz) { e.rz = 1; }
    });
    Object.keys(fixMerge3).forEach(function (ai) {
      var xt = (+ai) + 1, e = fixMerge3[ai];
      L.push("model.fix(" + xt + ", " + e.fx + ", " + e.fy + ", 1, 1, 1, " + e.rz + ")");
    });
    // 非模型节点（细分/投影新增）：面内自由，面外全约束
    var inIdx = {};
    Object.keys(meta.idx).forEach(function (k) { inIdx[meta.idx[k]] = true; });
    for (var ai0 = 0; ai0 < meta.all.length; ai0++) {
      if (!inIdx[ai0]) { L.push("model.fix(" + (ai0 + 1) + ", 0, 0, 1, 1, 1, 0)"); }
    }
    var eleTag = 0;
    L.push("model.geomTransf('Linear', 1, 0.0, 0.0, 1.0)");
    meta.members.forEach(function (sg) {
      eleTag += 1; sg.eleTag = eleTag;
      var G = sg.E / (2 * (1 + sg.nu));
      L.push("model.element('elasticBeamColumn', " + eleTag + ", [" + (sg.i + 1) + ", " + (sg.j + 1) + "], " +
        num(sg.A) + ", " + num(sg.E) + ", " + num(G) + ", " + num(sg.I) + ", " + num(sg.I) + ", " + num(sg.I) + ", 1)");
    });
    meta.quads.forEach(function (qc) {
      eleTag += 1; qc.eleTag = eleTag;
      var tags = qc.g.map(function (gi) { return gi + 1; });
      if (qc.kind === "q4") {
        L.push("model.element('ASDShellQ4', " + eleTag + ", [" + tags.join(", ") + "], " + qc.stag + ")");
      } else {
        // ASDShellT3 只接受散参节点（列表形式报 unknown type），与 Q4 不同
        L.push("model.element('ASDShellT3', " + eleTag + ", " + tags.join(", ") + ", " + qc.stag + ")");
      }
    });
    L.push("model.pattern('Plain', 1, 'Linear')");
    Object.keys(meta.nodal).forEach(function (ai) {
      var e = meta.nodal[ai], xt = (+ai) + 1;
      if (!e.fx && !e.fy && !e.mz) { return; }
      L.push("model.load(" + xt + ", " + num(e.fx) + ", " + num(e.fy) + ", 0.0, 0.0, 0.0, " + num(e.mz) + ", pattern=1)");
    });
    L.push("model.system('BandSPD')");
    L.push("model.numberer('RCM')");
    L.push("model.constraints('Plain')");
    L.push("model.integrator('LoadControl', 1.0)");
    L.push("model.algorithm('Linear')");
    L.push("model.analysis('Static')");
    L.push("out = {'analyze': None, 'ok': False, 'error': None, 'disps': {}, 'reactions': {}, 'beams': {}, 'conts': {}}");
    L.push("try:");
    L.push("    out['analyze'] = int(model.analyze(1))");
    L.push("    model.reactions()");
    for (var qi = 0; qi < meta.all.length; qi++) {
      var xt2 = qi + 1;
      L.push("    out['disps'][" + xt2 + "] = [float(v) for v in model.nodeDisp(" + xt2 + ")]");
      L.push("    out['reactions'][" + xt2 + "] = [float(v) for v in model.nodeReaction(" + xt2 + ")]");
    }
    meta.members.forEach(function (sg) {
      L.push("    out['beams'][" + sg.eleTag + "] = [float(v) for v in model.eleResponse(" + sg.eleTag + ", 'localForces')]");
    });
    meta.quads.forEach(function (qc) {
      L.push("    out['conts'][" + qc.eleTag + "] = [float(v) for v in model.eleResponse(" + qc.eleTag + ", 'stresses')]");
    });
    L.push("    out['ok'] = (out['analyze'] == 0)");
    L.push("    if out['analyze'] != 0:");
    L.push("        out['error'] = 'analyze returned ' + str(out['analyze'])");
    L.push("except Exception as _ex:");
    L.push("    out['ok'] = False");
    L.push("    out['error'] = str(_ex)");
    L.push("print('FRAMELAB_JSON_BEGIN')");
    L.push("print(json.dumps(out))");
    L.push("print('FRAMELAB_JSON_END')");
    return { script: L.join("\n") + "\n", meta: meta, kind: meta.kind, ndf: 6 };
  }

  function importResults3D(res, meta, state) {
    if (!res || !res.ok) { throw new Error("xara 求解失败：" + ((res && res.error) || "未知错误")); }
    var model = (state && state.model) || { nodes: [], members: [], areas: [] };
    var all = meta.all;
    var u = new Array(all.length * 3).fill(0);
    var leakD = 0;
    Object.keys(res.disps || {}).forEach(function (xt) {
      var ai = (+xt) - 1, d = res.disps[xt] || [];
      if (ai < 0 || ai >= all.length) { return; }
      u[3 * ai] = d[0] || 0; u[3 * ai + 1] = d[1] || 0; u[3 * ai + 2] = d[5] || 0;
      leakD = Math.max(leakD, Math.abs(d[2] || 0), Math.abs(d[3] || 0), Math.abs(d[4] || 0));
    });
    var idx = {};
    Object.keys(meta.idx || {}).forEach(function (mid) { idx[mid] = meta.idx[mid]; });
    var memLoads = (state && state.memLoads) || {};

    var elems = [], byMember = {}, maxM = 0, maxV = 0, maxN = 0, leakB = 0;
    (meta.members || []).forEach(function (sg) {
      var f = (res.beams || {})[sg.eleTag];
      if (!f || f.length < 12) { return; }
      var N1 = f[0], V1 = f[1], M1 = f[5];
      leakB = Math.max(leakB, Math.abs(f[2] || 0), Math.abs(f[3] || 0), Math.abs(f[4] || 0),
        Math.abs(f[8] || 0), Math.abs(f[9] || 0), Math.abs(f[10] || 0));
      var m = (model.members || []).find(function (mm) { return mm.id === sg.mid; });
      var el = {
        key: sg.mid, member: m || { id: sg.mid, L: sg.L }, a: sg.i, b: sg.j,
        xa: sg.xa, Lsub: sg.Lsub, axial: -N1, forces: [], fromXara: true
      };
      maxN = Math.max(maxN, Math.abs(el.axial));
      // 3D 路径杆件荷载以节点力施加（无 eleLoad），单元端力量测 q=k·u 未扣除
      // 分布荷载固端力 p，故站内力须补扣：S=(V1-p1)+A，M=-((M1-p2)-(V1-p1)*t-J)
      var pseg = segLoadVector(sg, memLoads[sg.mid] || []);
      var V1a = V1 - (pseg[1] || 0), M1a = M1 - (pseg[2] || 0);
      var NN = 12;
      for (var k = 0; k <= NN; k++) {
        var t = sg.Lsub * k / NN;
        var R2 = segResultants(sg.mid, sg.L, sg.xa, sg.xa + t, memLoads);
        var S = V1a + R2.A;
        var J = (sg.xa + t) * R2.A - R2.Bb;
        var Mo = -(M1a - V1a * t - J);
        el.forces.push({ t: t, S: S, M: Mo });
        maxM = Math.max(maxM, Math.abs(Mo)); maxV = Math.max(maxV, Math.abs(S));
      }
      elems.push(el);
      if (!byMember[el.key]) { byMember[el.key] = []; }
      byMember[el.key].push(el);
    });
    // 杆段按 xa 排序，保证 byMember 内顺序与内置一致
    Object.keys(byMember).forEach(function (k) {
      byMember[k].sort(function (a, b) { return a.xa - b.xa; });
    });

    var areaElems = [], areaNodal = [], maxVM = 0, maxSX = 0;
    function nodalAcc(i) {
      if (!areaNodal[i]) { areaNodal[i] = { sx: 0, sy: 0, txy: 0, vm: 0, s1: 0, nx: 0, n: 0 }; }
      return areaNodal[i];
    }
    (meta.quads || []).forEach(function (qc) {
      var s = (res.conts || {})[qc.eleTag];
      if (!s || s.length < 8) { return; }
      var nGP = Math.floor(s.length / 8), k, Nxx = 0, Nyy = 0, Nxy = 0;
      for (k = 0; k < nGP; k++) { Nyy += s[8 * k] || 0; Nxx += s[8 * k + 1] || 0; Nxy += s[8 * k + 2] || 0; }
      // 注意：实测 ASDShell 的 Nxy 符号与 2D（标准等参元）相反，此处取反以对齐
      // FrameLab 约定（门框+内填墙全场反相关验证：量级一致、符号全反）
      var sx = Nxx / nGP / qc.t, sy = Nyy / nGP / qc.t, txy = -Nxy / nGP / qc.t;
      var avg = (sx + sy) / 2, R = Math.hypot((sx - sy) / 2, txy);
      var s1 = avg + R, s2 = avg - R;
      var vm = Math.sqrt(sx * sx - sx * sy + sy * sy + 3 * txy * txy);
      var ae = {
        aid: qc.aid, etype: qc.etype, kind: qc.kind, g: qc.g.slice(), B: null, Dmat: null, t: qc.t,
        s: { sx: sx, sy: sy, txy: txy, s1: s1, s2: s2, vm: vm, nx: sx * qc.t, ny: sy * qc.t, nxy: txy * qc.t },
        cx: 0, cy: 0, fromXara: true
      };
      qc.g.forEach(function (gi) { ae.cx += all[gi][0]; ae.cy += all[gi][1]; });
      ae.cx /= qc.g.length; ae.cy /= qc.g.length;
      maxVM = Math.max(maxVM, Math.abs(vm)); maxSX = Math.max(maxSX, Math.abs(sx), Math.abs(sy));
      qc.g.forEach(function (gi) {
        var acc = nodalAcc(gi);
        acc.sx += sx; acc.sy += sy; acc.txy += txy; acc.vm += vm; acc.s1 += s1; acc.nx += sx * qc.t; acc.n++;
      });
      areaElems.push(ae);
    });

    var maxDisp = 0;
    for (var i2 = 0; i2 < all.length; i2++) { maxDisp = Math.max(maxDisp, Math.hypot(u[3 * i2], u[3 * i2 + 1])); }
    var nodes = model.nodes || [];
    var ys = nodes.map(function (n) { return n.y; });
    var ymax = ys.length ? Math.max.apply(null, ys) : 0;
    var roof = 0, nr = 0;
    nodes.forEach(function (n) { if (Math.abs(n.y - ymax) < 1e-6 && idx[n.id] !== undefined) { roof += u[3 * idx[n.id]]; nr++; } });
    roof = nr ? roof / nr : 0;
    var scaleL = Math.max(1e-12, maxDisp / 1000, maxM / 1e4);
    var leak = Math.max(leakD / Math.max(1e-12, maxDisp / 1000), leakB / scaleL);

    return {
      nodes: nodes, all: all, elems: elems, u: u, ok: true, idx: idx, byMember: byMember,
      areaElems: areaElems, areaNodal: areaNodal, areaGrid: meta.areaGrid || {},
      shellCuts: [], cutResults: {},
      fromXara: true, xaraLog: res, leak: leak,
      res: { roof: roof * 1000, maxDisp: maxDisp * 1000, maxM: maxM, maxV: maxV, maxN: maxN, maxVM: maxVM, maxSX: maxSX, nNode: nodes.length, nMember: (model.members || []).length, nArea: (model.areas || []).length }
    };
  }
  /* ---------------- 回填：resultsJSON + meta -> ana ---------------- */
  function importResults(res, meta, state) {
    if (meta && meta.kind === "mixed3d") { return importResults3D(res, meta, state); }
    if (!res || !res.ok) { throw new Error("xara 求解失败：" + ((res && res.error) || "未知错误")); }
    var model = (state && state.model) || { nodes: [], members: [], areas: [] };
    var all = meta.all;
    var u = new Array(all.length * 3).fill(0);
    Object.keys(res.disps || {}).forEach(function (xt) {
      var ai = (+xt) - 1, d = res.disps[xt] || [];
      if (ai < 0 || ai >= all.length) { return; }
      u[3 * ai] = d[0] || 0; u[3 * ai + 1] = d[1] || 0; u[3 * ai + 2] = d[2] || 0;
    });
    var idx = {};
    Object.keys(meta.idx || {}).forEach(function (mid) { idx[mid] = meta.idx[mid]; });

    // 杆件：按杆段端力 + 分布荷载积分 -> 每段 13 站内力（与内置求解器同符号、同分段结构）
    var elems = [], byMember = {}, maxM = 0, maxV = 0, maxN = 0;
    function loadAB(loads, L, x) {
      var A = 0, Bb = 0;
      loads.forEach(function (ld) {
        // 端点集中力在导出时已折为等效节点力，此处跳过（判据与导出一致）
        if (ld.type === "point") { if (ld.a > 1e-9 * L && ld.a < L * (1 - 1e-9) && ld.a <= x) { A += ld.P; Bb += ld.P * ld.a; } return; }
        var s0, s1;
        if (ld.type === "udl") { s0 = 0; s1 = x; }
        else { s0 = Math.max(ld.c, 0); s1 = Math.min(ld.d, x); }
        if (s1 <= s0) { return; }
        // 梯形/均布：解析积分（线性分布精确）
        if (ld.type === "udl") { var q = ld.q || 0; A += q * (s1 - s0); Bb += q * (s1 * s1 - s0 * s0) / 2; }
        else {
          var span = (ld.d - ld.c) || 1, g0 = Math.max(s0, ld.c), g1 = Math.min(s1, ld.d);
          if (g1 > g0) {
            var qa = ld.q1 + (ld.q2 - ld.q1) * (g0 - ld.c) / span, qb = ld.q1 + (ld.q2 - ld.q1) * (g1 - ld.c) / span;
            A += (qa + qb) / 2 * (g1 - g0);
            Bb += (qa * (2 * g0 + g1) + qb * (g0 + 2 * g1)) / 6 * (g1 - g0);
          }
        }
      });
      return { A: A, Bb: Bb };
    }
    var segsByMid = {};
    (meta.members || []).forEach(function (mb) { (segsByMid[mb.mid] = segsByMid[mb.mid] || []).push(mb); });
    Object.keys(segsByMid).forEach(function (mid) {
      var segs = segsByMid[mid];
      segs.sort(function (a, b) { return a.xa - b.xa; });
      var m = (model.members || []).find(function (mm) { return mm.id === +mid; });
      var Lfull = segs[0].L;
      var loads = (state.memLoads || {})[mid] || [];
      segs.forEach(function (sg) {
        var f = (res.beams || {})[sg.eleTag];
        if (!f || f.length < 6) { return; }
        var N1 = f[0], V1 = f[1], M1 = f[2];
        var el = {
          key: sg.mid, member: m || { id: sg.mid, L: Lfull }, a: sg.i, b: sg.j,
          xa: sg.xa, Lsub: sg.Lsub, axial: -N1, forces: [], fromXara: true
        };
        maxN = Math.max(maxN, Math.abs(el.axial));
        var R0 = loadAB(loads, Lfull, sg.xa);
        var NN = 12;
        for (var k = 0; k <= NN; k++) {
          var t = sg.Lsub * k / NN;
          var R1 = loadAB(loads, Lfull, sg.xa + t);
          var dA = R1.A - R0.A, dBb = R1.Bb - R0.Bb;
          // 端点集中力已折为节点力，不再重复计入；
          // J 取 (xa+t)*dA-dBb（与内置 loadResultants+J 同式，xa=0 时退化为旧式）
          var S = V1 + dA;
          var J = (sg.xa + t) * dA - dBb;
          var Mo = -(M1 - V1 * t - J);
          el.forces.push({ t: t, S: S, M: Mo });
          maxM = Math.max(maxM, Math.abs(Mo)); maxV = Math.max(maxV, Math.abs(S));
        }
        elems.push(el);
        if (!byMember[el.key]) { byMember[el.key] = []; }
        byMember[el.key].push(el);
      });
    });

    // 连续体：子单元应力直接回填
    var areaElems = [], areaNodal = [], maxVM = 0, maxSX = 0;
    function nodalAcc(i) {
      if (!areaNodal[i]) { areaNodal[i] = { sx: 0, sy: 0, txy: 0, vm: 0, s1: 0, nx: 0, n: 0 }; }
      return areaNodal[i];
    }
    (meta.quads || []).forEach(function (qc) {
      var s = (res.conts || {})[qc.eleTag];
      if (!s) { return; }
      var sx = s[0] || 0, sy = s[1] || 0, txy = s[2] || 0;
      var avg = (sx + sy) / 2, R = Math.hypot((sx - sy) / 2, txy);
      var s1 = avg + R, s2 = avg - R;
      var vm = Math.sqrt(sx * sx - sx * sy + sy * sy + 3 * txy * txy);
      var ae = {
        aid: qc.aid, etype: qc.etype, kind: qc.kind, g: qc.g.slice(), B: null, Dmat: null, t: qc.t,
        s: { sx: sx, sy: sy, txy: txy, s1: s1, s2: s2, vm: vm, nx: sx * qc.t, ny: sy * qc.t, nxy: txy * qc.t },
        cx: 0, cy: 0, fromXara: true
      };
      qc.g.forEach(function (gi) { ae.cx += all[gi][0]; ae.cy += all[gi][1]; });
      ae.cx /= qc.g.length; ae.cy /= qc.g.length;
      maxVM = Math.max(maxVM, Math.abs(vm)); maxSX = Math.max(maxSX, Math.abs(sx), Math.abs(sy));
      qc.g.forEach(function (gi) {
        var acc = nodalAcc(gi);
        acc.sx += sx; acc.sy += sy; acc.txy += txy; acc.vm += vm; acc.s1 += s1; acc.nx += sx * qc.t; acc.n++;
      });
      areaElems.push(ae);
    });

    var maxDisp = 0;
    for (var i2 = 0; i2 < all.length; i2++) { maxDisp = Math.max(maxDisp, Math.hypot(u[3 * i2], u[3 * i2 + 1])); }
    var nodes = model.nodes || [];
    var ys = nodes.map(function (n) { return n.y; });
    var ymax = ys.length ? Math.max.apply(null, ys) : 0;
    var roof = 0, nr = 0;
    nodes.forEach(function (n) { if (Math.abs(n.y - ymax) < 1e-6 && idx[n.id] !== undefined) { roof += u[3 * idx[n.id]]; nr++; } });
    roof = nr ? roof / nr : 0;

    return {
      nodes: nodes, all: all, elems: elems, u: u, ok: true, idx: idx, byMember: byMember,
      areaElems: areaElems, areaNodal: areaNodal, areaGrid: meta.areaGrid || {},
      shellCuts: [], cutResults: {},
      fromXara: true, xaraLog: res,
      res: { roof: roof * 1000, maxDisp: maxDisp * 1000, maxM: maxM, maxV: maxV, maxN: maxN, maxVM: maxVM, maxSX: maxSX, nNode: nodes.length, nMember: (model.members || []).length, nArea: (model.areas || []).length }
    };
  }  /* ---------------- 桥接调用 ---------------- */
  var BRIDGE_DEFAULT = "http://127.0.0.1:8007";
  var BRIDGE_OLD_DEFAULT = "http://127.0.0.1:8000";
  function bridgeUrl() {
    try {
      var v = (typeof localStorage !== "undefined") && localStorage.getItem("framelab_xara_url");
      if (v && v.replace(/\/+$/, "") === BRIDGE_OLD_DEFAULT) {
        // 旧版本默认端口迁移：8000 -> 8007（8000 易与其它程序冲突）
        try { localStorage.removeItem("framelab_xara_url"); } catch (e2) { /* ignore */ }
        v = null;
      }
      if (v) { return v.replace(/\/+$/, ""); }
    } catch (e) { /* ignore */ }
    return BRIDGE_DEFAULT;
  }
  function runOnBridge(script, url) {
    url = (url || bridgeUrl()).replace(/\/+$/, "");
    return fetch(url + "/api/analyze", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ script: script })
    }).then(function (resp) {
      if (!resp.ok) { throw new Error("桥接服务 HTTP " + resp.status); }
      return resp.json();
    }).then(function (data) {
      if (!data.ok) {
        var tail = ((data.stderr || "") + "\n" + (data.stdout || "")).split("\n").filter(function (ln) {
          return ln && ln.indexOf("FRAMELAB_JSON") < 0;
        }).slice(-6).join("\n");
        throw new Error("桥接执行失败：" + (data.error || "未知错误") + (tail ? " ｜ " + tail.slice(-400) : ""));
      }
      return data.results;
    });
  }
  function checkBridge(url) {
    url = (url || bridgeUrl()).replace(/\/+$/, "");
    return fetch(url + "/api/health").then(function (r) { return r.json(); });
  }

  /* ---------------- UI ---------------- */
  function $(id) { return (typeof document !== "undefined") ? document.getElementById(id) : null; }
  function setStatus(html) {
    var el = $("xara-status");
    if (el) { el.innerHTML = html; }
  }
  function currentState() {
    if (!global.FrameLab || !global.FrameLab.state) { throw new Error("FrameLab 未就绪。"); }
    return global.FrameLab.state();
  }
  function runFromUI() {
    var st;
    try { st = currentState(); }
    catch (e) { setStatus("采集模型失败：" + e.message); return; }
    var built;
    try {
      built = buildScript(st, {});
    } catch (e) { setStatus("无法走 xara 路径：" + e.message); return; }
    setStatus("已导出 xara 脚本（" + built.script.split("\n").length + " 行），正在调用 OpenSees 求解…");
    runOnBridge(built.script).then(function (res) {
      var ana;
      try { ana = importResults(res, built.meta, st); }
      catch (e) { setStatus("结果回填失败：" + e.message); return; }
      if (global.FrameLab.setExternalAna) {
        global.FrameLab.setExternalAna(ana, { backend: "xara", ndf: built.ndf, kind: built.kind, log: "analyze=" + res.analyze });
        backend = "xara";
        var tag3d = built.kind === "mixed3d" ? "（3D 退化平面）" : "";
        setStatus("xara 求解完成" + tag3d + "：analyze=" + res.analyze + "，最大位移 " + ana.res.maxDisp.toFixed(3) +
          " mm，最大弯矩 " + ana.res.maxM.toFixed(2) + " kN·m。点“回到内置求解器”可切回。");
      } else {
        setStatus("结果已取回，但页面缺少 setExternalAna 接口（请更新 index.html）。");
      }
    }).catch(function (e) {
      setStatus("调用失败：" + e.message + "。请确认已在项目目录执行 <code>uv run --no-sync xara_server.py --port 8007</code> 启动桥接（" + bridgeUrl() + "），或用“复制脚本”离线运行。");
    });
  }
  function backToBuiltin() {
    if (global.FrameLab && global.FrameLab.clearExternalAna) { global.FrameLab.clearExternalAna(); }
    backend = "builtin";
    setStatus("已切回内置求解器（默认）。");
  }
  function downloadScript() {
    var st = currentState();
    var built = buildScript(st, {});
    var blob = new Blob([built.script], { type: "text/x-python;charset=utf-8" });
    var a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = "framelab_xara.py";
    document.body.appendChild(a); a.click();
    setTimeout(function () { URL.revokeObjectURL(a.href); a.remove(); }, 500);
    setStatus("已下载 framelab_xara.py（可用 <code>uv run --no-sync run_xara.py framelab_xara.py</code> 运行）。");
  }
  function copyScript() {
    var st = currentState();
    var built = buildScript(st, {});
    function done() { setStatus("脚本已复制到剪贴板。"); }
    if (navigator.clipboard && navigator.clipboard.writeText) {
      navigator.clipboard.writeText(built.script).then(done, function () { setStatus("复制失败，请用下载方式。"); });
    } else { setStatus("浏览器不支持剪贴板，请用下载方式。"); }
  }
  function probeServer() {
    // 只探测固定地址（8007），不在界面暴露地址输入
    setStatus("正在探测求解服务器…");
    checkBridge().then(function (h) {
      if (h && h.xara_ready) {
        setStatus("OpenSees 求解服务器连接成功（" + bridgeUrl() + (h.xara ? "，xara " + h.xara : "") + "）。默认使用内置求解器，点击“用 OpenSees 分析”切换。");
      } else if (h) {
        setStatus("求解服务器有响应，但 xara 未就绪（未安装 xara 包），请检查服务器环境。");
      }
    }).catch(function () {
      setStatus("求解服务器未开启；请在项目目录执行 <code>uv run --no-sync xara_server.py --port 8007</code> 启动（首次需先 <code>uv pip install -r requirements-xara.txt</code>）。");
    });
  }
  function init() {
    if (typeof document === "undefined") { return; }
    function on(id, fn) {
      var el = document.getElementById(id);
      if (el && !el._xaraWired) { el._xaraWired = true; el.addEventListener("click", fn); }
    }
    on("btn-xara-run", function () { runFromUI(); });
    on("btn-xara-back", function () { backToBuiltin(); });
    on("btn-xara-dl", function () { try { downloadScript(); } catch (e) { setStatus("导出失败：" + e.message); } });
    on("btn-xara-copy", function () { try { copyScript(); } catch (e) { setStatus("导出失败：" + e.message); } });
    probeServer();
  }

  global.FrameXara = {
    version: VERSION, backend: function () { return backend; },
    exportMeta: exportMeta, buildScript: buildScript, importResults: importResults,
    exportMeta3D: exportMeta3D, buildScript3D: buildScript3D, importResults3D: importResults3D,
    runOnBridge: runOnBridge, checkBridge: checkBridge, bridgeUrl: bridgeUrl, probeServer: probeServer,
    runFromUI: runFromUI, backToBuiltin: backToBuiltin,
    downloadScript: downloadScript, copyScript: copyScript, init: init
  };
  if (typeof document !== "undefined") {
    if (document.readyState === "loading") { document.addEventListener("DOMContentLoaded", init); }
    else { init(); }
  }
})(typeof window !== "undefined" ? window : globalThis);