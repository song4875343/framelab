/* FrameLab 内置求解器（独立模块）
 * ------------------------------------------------------------------
 * 本文件是从 index.html 拆出的自编求解器：二维平面框架 / 平面 / 壳
 * （膜）线弹性直接刚度法，Timoshenko 梁 + Q4/CST 面单元，零 DOM 依赖，
 * 可被 index.html、Node 脚本或其他页面复用。
 *
 * 接口
 *   FrameSolver.version            字符串版本号
 *   FrameSolver.solve(ctx)         执行一次线弹性分析，返回 ana 结果对象
 *   FrameSolver.info()             返回模块能力说明（含单位约定与局限）
 *
 * ctx（分析上下文，纯数据对象）字段：
 *   model     { nodes:[{id,x,y,bc:{ux,uy,rz}}], members:[{id,a,b,type}],
 *               areas:[{id,nodes:[ids],etype:"plane"|"shell"}], cuts:[...] }
 *   NSUB      杆件全局默认细分数（默认 5）
 *   AMESH     面全局默认细分 nx=ny（默认 2）
 *   memStiff  {mid:{EA,EI,GA}}        杆件刚度手动覆盖（优先）
 *   memMat    {mid:{label,E,nu}}      杆件材料（E 单位 kN/m^2）
 *   memSec    {mid:{type:"rect"|"circle"|"I",...}}  杆件截面（m）
 *   memLoads  {mid:[{type:"udl",q}|{type:"point",P,a}|{type:"trap",q1,q2,c,d}]}
 *             杆件荷载：局部 +y 为正（水平杆向上），a/c/d 为沿杆米数
 *   nodeLoads {nid:{fx,fy,mz}}        节点荷载（整体坐标，kN / kN*m）
 *   areaMat   {aid:{label,E,nu}}      面材料
 *   areaSec   {aid:{kind:"stress"|"strain"|"shell-thin"|"shell-thick",t}}
 *   areaLoads {aid:{qx,qy}}           面荷载（整体坐标，kN/m^2）
 *   memDiv    {mid:{n,lmax}}          杆件细分覆盖
 *   areaDiv   {aid:{nx,ny,hmax}}      面细分覆盖
 *   MATERIALS {label:{E,nu}}          材料库（可省略，用内置默认）
 *   DEF_STIFF {col|beam|user:{EA,EI,GA}}  缺省杆件刚度（可省略）
 *
 * 返回 ana（与原 index.html analyze() 完全同形）：
 *   { nodes, all, elems, u, ok, idx, byMember, areaElems, areaNodal,
 *     areaGrid, shellCuts, cutResults,
 *     res:{roof,maxDisp,maxM,maxV,maxN,maxVM,maxSX,nNode,nMember,nArea} }
 *   长度 m、力 kN、弯矩 kN*m、应力 kPa；位移 res.roof/maxDisp 单位 mm。
 * 注意：solve() 会像原实现一样回写 member.L/ux/uy/x1/y1。
 */
(function (global) {
  "use strict";

  var VERSION = "1.0.0";

  var BUILTIN_MATERIALS = {
    Q235: { E: 2.06e8, nu: 0.30 }, Q345: { E: 2.06e8, nu: 0.30 },
    C30: { E: 3.00e7, nu: 0.20 }, C40: { E: 3.25e7, nu: 0.20 },
    AL6061: { E: 6.90e7, nu: 0.33 }, CUSTOM: { E: 2.06e8, nu: 0.30 }
  };
  var BUILTIN_DEF_STIFF = {
    col: { EA: 4.5e6, EI: 93750, GA: 1.0e9 },
    beam: { EA: 4.5e6, EI: 150000, GA: 1.0e9 },
    user: { EA: 4.5e6, EI: 150000, GA: 1.0e9 }
  };

  /* ---------------- 通用数学 ---------------- */
  function zeros(n, m) { var a = []; for (var i = 0; i < n; i++) { a.push(new Array(m).fill(0)); } return a; }
  function clamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); }
  function clampInt(v, lo, hi) { return Math.round(clamp(v, lo, hi)); }
  function matVec(M, v) { var o = []; for (var i = 0; i < M.length; i++) { var s = 0; for (var j = 0; j < v.length; j++) { s += M[i][j] * v[j]; } o.push(s); } return o; }
  function transpose(M) { var r = M.length, c = M[0].length, T = zeros(c, r); for (var i = 0; i < r; i++) { for (var j = 0; j < c; j++) { T[j][i] = M[i][j]; } } return T; }
  /* 列主元高斯消元 */
  function solveLin(A, b) {
    var n = b.length, M = A.map(function (row, i) { return row.concat([b[i]]); });
    for (var col = 0; col < n; col++) {
      var piv = col;
      for (var r = col + 1; r < n; r++) { if (Math.abs(M[r][col]) > Math.abs(M[piv][col])) { piv = r; } }
      var t = M[col]; M[col] = M[piv]; M[piv] = t;
      var d = M[col][col] || 1e-12;
      for (var r2 = col + 1; r2 < n; r2++) { var f = M[r2][col] / d; if (f === 0) { continue; } for (var c = col; c <= n; c++) { M[r2][c] -= f * M[col][c]; } }
    }
    var x = new Array(n).fill(0);
    for (var i = n - 1; i >= 0; i--) { var s = M[i][n]; for (var j = i + 1; j < n; j++) { s -= M[i][j] * x[j]; } x[i] = s / (M[i][i] || 1e-12); }
    return x;
  }

  /* ---------------- 截面几何 ---------------- */
  function sectionProps(sec) {
    if (!sec) { return null; }
    if (sec.type === "rect") {
      var A = sec.b * sec.h;
      return { A: A, I: sec.b * Math.pow(sec.h, 3) / 12, As: 5 / 6 * A };
    }
    if (sec.type === "circle") {
      var d = sec.d, A2 = Math.PI * d * d / 4;
      return { A: A2, I: Math.PI * Math.pow(d, 4) / 64, As: 0.9 * A2 };
    }
    if (sec.type === "I") {
      var h = sec.h, b = sec.b, tw = sec.tw, tf = sec.tf;
      var A3 = 2 * b * tf + (h - 2 * tf) * tw;
      var I3 = (b * Math.pow(h, 3) - (b - tw) * Math.pow(h - 2 * tf, 3)) / 12;
      return { A: A3, I: I3, As: h * tw };
    }
    return null;
  }
  /* 3D 截面特性：双向抗弯 Iy/Iz + 抗扭 J（内置 3D 杆系用；Iz 为 2D 面内弯曲惯性矩，保持与 2D 一致） */
  function sectionProps3D(sec) {
    if (!sec) { return null; }
    if (sec.type === "rect") {
      var b = sec.b, h = sec.h, A = b * h;
      var Iz = b * Math.pow(h, 3) / 12, Iy = h * Math.pow(b, 3) / 12;
      var mx = Math.max(b, h), mn = Math.min(b, h);
      var J = mx * Math.pow(mn, 3) * (1 / 3 - 0.21 * mn / mx);
      return { A: A, Iy: Iy, Iz: Iz, J: J, As: 5 / 6 * A };
    }
    if (sec.type === "circle") {
      var d = sec.d, A2 = Math.PI * d * d / 4, I2 = Math.PI * Math.pow(d, 4) / 64;
      return { A: A2, Iy: I2, Iz: I2, J: 2 * I2, As: 0.9 * A2 };
    }
    if (sec.type === "I") {
      var h2 = sec.h, b2 = sec.b, tw = sec.tw, tf = sec.tf;
      var A3 = 2 * b2 * tf + (h2 - 2 * tf) * tw;
      var Iz3 = (b2 * Math.pow(h2, 3) - (b2 - tw) * Math.pow(h2 - 2 * tf, 3)) / 12;
      var Iy3 = (2 * tf * Math.pow(b2, 3) + (h2 - 2 * tf) * Math.pow(tw, 3)) / 12;
      var J3 = (2 * b2 * Math.pow(tf, 3) + (h2 - 2 * tf) * Math.pow(tw, 3)) / 3;
      return { A: A3, Iy: Iy3, Iz: Iz3, J: J3, As: h2 * tw };
    }
    return null;
  }
  /* 3D 梁局部坐标架：非竖向杆 y 取“竖向平面内横向”（含整体 +Z 分量，向上为正，梁均布重力向下取负；
   * 竖向杆 y 取水平横向（风荷载方向）。注意：与 2D 局部 +y（面内）不同，3D 路径下
   * 杆件横向荷载一律按此局部 y 施加，xara 真 3D 导出采用同一坐标架以保证一致。 */
  function beamAxes3D(ax, ay, az, bx, by, bz) {
    var ex = bx - ax, ey = by - ay, ez = bz - az;
    var L = Math.hypot(ex, ey, ez) || 1;
    ex /= L; ey /= L; ez /= L;
    var yx, yy, yz;
    if (Math.abs(ez) <= 0.99) {
      var s = ex * 0 + ey * 0 + ez * 1;
      yx = 0 - s * ex; yy = 0 - s * ey; yz = 1 - s * ez;
    } else {
      var up = [0, 1, 0];
      if (Math.abs(ex * up[0] + ey * up[1] + ez * up[2]) > 0.99) { up = [1, 0, 0]; }
      yx = up[1] * ez - up[2] * ey; yy = up[2] * ex - up[0] * ez; yz = up[0] * ey - up[1] * ex;
    }
    var yl = Math.hypot(yx, yy, yz) || 1;
    yx /= yl; yy /= yl; yz /= yl;
    var zx = ey * yz - ez * yy, zy = ez * yx - ex * yz, zz = ex * yy - ey * yx;
    return { x: [ex, ey, ez], y: [yx, yy, yz], z: [zx, zy, zz], L: L };
  }
  function rot12(ax3) {
    var R = zeros(12, 12), B = [ax3.x, ax3.y, ax3.z], r, c, k;
    for (k = 0; k < 4; k++) {
      for (r = 0; r < 3; r++) { for (c = 0; c < 3; c++) { R[3 * k + r][3 * k + c] = B[r][c]; } }
    }
    return R;
  }
  /* 3D 欧拉梁局部刚度 12x12（轴向 + 扭转 + 双向弯曲；自由度序 ux,uy,uz,rx,ry,rz × 两端） */
  function beamKe3D(EA, EIz, EIy, GJ, L) {
    var k = zeros(12, 12);
    function add(i, j, v) { k[i][j] += v; }
    var ea = EA / L, gj = GJ / L;
    add(0, 0, ea); add(0, 6, -ea); add(6, 0, -ea); add(6, 6, ea);
    add(3, 3, gj); add(3, 9, -gj); add(9, 3, -gj); add(9, 9, gj);
    var a = 12 * EIz / (L * L * L), b = 6 * EIz / (L * L), c = 4 * EIz / L, d = 2 * EIz / L;
    add(1, 1, a); add(1, 5, b); add(1, 7, -a); add(1, 11, b);
    add(5, 1, b); add(5, 5, c); add(5, 7, -b); add(5, 11, d);
    add(7, 1, -a); add(7, 5, -b); add(7, 7, a); add(7, 11, -b);
    add(11, 1, b); add(11, 5, d); add(11, 7, -b); add(11, 11, c);
    var a2 = 12 * EIy / (L * L * L), b2 = 6 * EIy / (L * L), c2 = 4 * EIy / L, d2 = 2 * EIy / L;
    add(2, 2, a2); add(2, 4, -b2); add(2, 8, -a2); add(2, 10, -b2);
    add(4, 2, -b2); add(4, 4, c2); add(4, 8, b2); add(4, 10, d2);
    add(8, 2, -a2); add(8, 4, b2); add(8, 8, a2); add(8, 10, b2);
    add(10, 2, -b2); add(10, 4, d2); add(10, 8, b2); add(10, 10, c2);
    return k;
  }  /* ---------------- 网格剖分 ---------------- */
  function quadSubCells(corners, nx, ny) {
    var P = corners, cells = [], r, c;
    var grid = [];
    for (r = 0; r <= ny; r++) {
      grid[r] = [];
      for (c = 0; c <= nx; c++) {
        var s = c / nx, t = r / ny;
        var x = (1 - s) * (1 - t) * P[0][0] + s * (1 - t) * P[1][0] + s * t * P[2][0] + (1 - s) * t * P[3][0];
        var y = (1 - s) * (1 - t) * P[0][1] + s * (1 - t) * P[1][1] + s * t * P[2][1] + (1 - s) * t * P[3][1];
        grid[r][c] = [x, y];
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
    for (k = 0; k <= n; k++) {
      for (j = 0; j <= n - k; j++) {
        i = n - j - k; idx(i, j, k);
      }
    }
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
  function planeD(E, nu, kind) {
    if (kind === "strain") {
      var f = E * (1 - nu) / ((1 + nu) * (1 - 2 * nu));
      return [[f, f * nu / (1 - nu), 0], [f * nu / (1 - nu), f, 0], [0, 0, f * (1 - 2 * nu) / (2 * (1 - nu))]];
    }
    var f2 = E / (1 - nu * nu);
    return [[f2, f2 * nu, 0], [f2 * nu, f2, 0], [0, 0, f2 * (1 - nu) / 2]];
  }
  function q4Stiffness(coords, Dmat, t) {
    var g = 1 / Math.sqrt(3), ke = zeros(8, 8), B0 = null;
    [[-g, -g, 1], [g, -g, 1], [g, g, 1], [-g, g, 1]].forEach(function (gp) {
      var xi = gp[0], et = gp[1], w = gp[2];
      var dN = [
        [-(1 - et) / 4, -(1 - xi) / 4], [(1 - et) / 4, -(1 + xi) / 4],
        [(1 + xi) / 4, (1 + et) / 4], [-(1 + xi) / 4, (1 - et) / 4]
      ];
      var J = [[0, 0], [0, 0]], a;
      for (a = 0; a < 4; a++) {
        J[0][0] += dN[a][0] * coords[a][0]; J[0][1] += dN[a][0] * coords[a][1];
        J[1][0] += dN[a][1] * coords[a][0]; J[1][1] += dN[a][1] * coords[a][1];
      }
      var det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
      if (Math.abs(det) < 1e-12) { return; }
      var inv = [[J[1][1] / det, -J[0][1] / det], [-J[1][0] / det, J[0][0] / det]];
      var B = zeros(3, 8);
      for (a = 0; a < 4; a++) {
        var dx = inv[0][0] * dN[a][0] + inv[0][1] * dN[a][1];
        var dy = inv[1][0] * dN[a][0] + inv[1][1] * dN[a][1];
        B[0][2 * a] = dx; B[1][2 * a + 1] = dy; B[2][2 * a] = dy; B[2][2 * a + 1] = dx;
      }
      var DB = zeros(3, 8), r2, c2;
      for (r2 = 0; r2 < 3; r2++) { for (c2 = 0; c2 < 8; c2++) { DB[r2][c2] = Dmat[r2][0] * B[0][c2] + Dmat[r2][1] * B[1][c2] + Dmat[r2][2] * B[2][c2]; } }
      for (r2 = 0; r2 < 8; r2++) {
        for (c2 = 0; c2 < 8; c2++) {
          var s = B[0][r2] * DB[0][c2] + B[1][r2] * DB[1][c2] + B[2][r2] * DB[2][c2];
          ke[r2][c2] += s * t * Math.abs(det) * w;
        }
      }
    });
    (function () {
      var dN = [[-0.25, -0.25], [0.25, -0.25], [0.25, 0.25], [-0.25, 0.25]];
      var J = [[0, 0], [0, 0]], a;
      for (a = 0; a < 4; a++) {
        J[0][0] += dN[a][0] * coords[a][0]; J[0][1] += dN[a][0] * coords[a][1];
        J[1][0] += dN[a][1] * coords[a][0]; J[1][1] += dN[a][1] * coords[a][1];
      }
      var det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
      if (Math.abs(det) < 1e-12) { B0 = zeros(3, 8); return; }
      var inv = [[J[1][1] / det, -J[0][1] / det], [-J[1][0] / det, J[0][0] / det]];
      B0 = zeros(3, 8);
      for (a = 0; a < 4; a++) {
        var dx = inv[0][0] * dN[a][0] + inv[0][1] * dN[a][1];
        var dy = inv[1][0] * dN[a][0] + inv[1][1] * dN[a][1];
        B0[0][2 * a] = dx; B0[1][2 * a + 1] = dy; B0[2][2 * a] = dy; B0[2][2 * a + 1] = dx;
      }
    })();
    return { ke: ke, B0: B0 };
  }
  function cstStiffness(coords, Dmat, t) {
    var x1 = coords[0][0], y1 = coords[0][1], x2 = coords[1][0], y2 = coords[1][1], x3 = coords[2][0], y3 = coords[2][1];
    var det = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1), A = Math.abs(det) / 2;
    if (A < 1e-12) { return { ke: zeros(6, 6), B: zeros(3, 6), area: 0 }; }
    var b1 = y2 - y3, b2 = y3 - y1, b3 = y1 - y2, c1 = x3 - x2, c2 = x1 - x3, c3 = x2 - x1;
    var B = [[b1, 0, b2, 0, b3, 0], [0, c1, 0, c2, 0, c3], [c1, b1, c2, b2, c3, b3]].map(function (row) { return row.map(function (v) { return v / det; }); });
    var DB = zeros(3, 6), r, c;
    for (r = 0; r < 3; r++) { for (c = 0; c < 6; c++) { DB[r][c] = Dmat[r][0] * B[0][c] + Dmat[r][1] * B[1][c] + Dmat[r][2] * B[2][c]; } }
    var ke = zeros(6, 6);
    for (r = 0; r < 6; r++) { for (c = 0; c < 6; c++) { ke[r][c] = (B[0][r] * DB[0][c] + B[1][r] * DB[1][c] + B[2][r] * DB[2][c]) * t * A; } }
    return { ke: ke, B: B, area: A };
  }
  function q4N(xi, et) {
    return [(1 - xi) * (1 - et) / 4, (1 + xi) * (1 - et) / 4, (1 + xi) * (1 + et) / 4, (1 - xi) * (1 + et) / 4];
  }  /* ---------------- 杆单元矩阵 / 荷载 ---------------- */
  var GP = [-0.8611363116, -0.3399810436, 0.3399810436, 0.8611363116];
  var GW = [0.3478548451, 0.6521451549, 0.6521451549, 0.3478548451];
  function elemMatrices(x1, y1, x2, y2, EA, EI, GA) {
    var dx = x2 - x1, dy = y2 - y1, L = Math.hypot(dx, dy), c = dx / L, s = dy / L;
    var ea = EA / L;
    var phi = (GA > 0) ? 12 * EI / (GA * L * L) : 0, d1 = 1 + phi;
    var a = 12 * EI / (L * L * L * d1), b = 6 * EI / (L * L * d1), c1 = (4 + phi) * EI / (L * d1), c2 = (2 - phi) * EI / (L * d1);
    var kl = [
      [ea, 0, 0, -ea, 0, 0], [0, a, b, 0, -a, b], [0, b, c1, 0, -b, c2],
      [-ea, 0, 0, ea, 0, 0], [0, -a, -b, 0, a, -b], [0, b, c2, 0, -b, c1]
    ];
    var T = [
      [c, s, 0, 0, 0, 0], [-s, c, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0],
      [0, 0, 0, c, s, 0], [0, 0, 0, -s, c, 0], [0, 0, 0, 0, 0, 1]
    ];
    var kg = zeros(6, 6);
    for (var i = 0; i < 6; i++) { for (var j = 0; j < 6; j++) { var sum = 0; for (var p = 0; p < 6; p++) { for (var q = 0; q < 6; q++) { sum += T[p][i] * kl[p][q] * T[q][j]; } } kg[i][j] = sum; } }
    return { kg: kg, kl: kl, T: T, L: L };
  }
  function hermiteShapes(x, L) { var t = x / L, t2 = t * t, t3 = t2 * t; return [1 - 3 * t2 + 2 * t3, L * (t - 2 * t2 + t3), 3 * t2 - 2 * t3, L * (-t2 + t3)]; }
  function hermiteTrans(x, L, p) { var N = hermiteShapes(x, L); return [0, N[0] * p, N[1] * p, 0, N[2] * p, N[3] * p]; }
  function intensity(ld, x) {
    if (ld.type === "udl") { return ld.q; }
    if (ld.type === "trap") { if (x < ld.c || x > ld.d) { return 0; } return (ld.q1 + (ld.q2 - ld.q1) * (x - ld.c) / ((ld.d - ld.c) || 1)); }
    return 0;
  }
  function elemLoadVector(el, memLoads) {
    var p = [0, 0, 0, 0, 0, 0], loads = (memLoads || {})[el.key] || [], xa = el.xa, Ls = el.Lsub;
    loads.forEach(function (ld) {
      if (ld.type === "point") {
        if ((ld.a >= xa - 1e-9) && (ld.a < xa + Ls - 1e-9 || xa + Ls >= el.member.L - 1e-9)) {
          var hp = hermiteTrans(clamp(ld.a - xa, 0, Ls), Ls, ld.P);
          for (var i = 0; i < 6; i++) { p[i] += hp[i]; }
        }
      } else {
        var s0 = xa, s1 = xa + Ls;
        if (ld.type === "trap") { s0 = Math.max(ld.c, xa); s1 = Math.min(ld.d, xa + Ls); }
        if (s1 <= s0) { return; }
        for (var k = 0; k < 4; k++) {
          var x = 0.5 * (s1 - s0) * GP[k] + 0.5 * (s0 + s1), jac = 0.5 * (s1 - s0), q = intensity(ld, x);
          if (q === 0) { continue; }
          var N = hermiteShapes(x - xa, Ls), wj = q * jac * GW[k];
          p[1] += N[0] * wj; p[2] += N[1] * wj; p[4] += N[2] * wj; p[5] += N[3] * wj;
        }
      }
    });
    return p;
  }
  function loadResultants(mem, x0, x1, memLoads) {
    var A = 0, Bb = 0, loads = (memLoads || {})[mem.id] || [];
    loads.forEach(function (ld) {
      if (ld.type === "point") { if (ld.a > x0 && ld.a <= x1) { A += ld.P; Bb += ld.P * ld.a; } return; }
      var s0, s1;
      if (ld.type === "udl") { s0 = Math.max(0, x0); s1 = Math.min(mem.L, x1); } else { s0 = Math.max(ld.c, x0); s1 = Math.min(ld.d, x1); }
      if (s1 <= s0) { return; }
      for (var k = 0; k < 4; k++) {
        var x = 0.5 * (s1 - s0) * GP[k] + 0.5 * (s0 + s1), jac = 0.5 * (s1 - s0), q = intensity(ld, x);
        A += q * jac * GW[k]; Bb += q * x * jac * GW[k];
      }
    });
    return { A: A, Bb: Bb };
  }

  /* ---------------- 壳剖面切割 / 壳梁壳柱（分析侧） ---------------- */
  function clipSegPoly(ax, ay, bx, by, poly) {
    var t0 = 0, t1 = 1, dx = bx - ax, dy = by - ay, i;
    var cx = 0, cy = 0;
    for (i = 0; i < poly.length; i++) { cx += poly[i][0]; cy += poly[i][1]; }
    cx /= poly.length; cy /= poly.length;
    for (i = 0; i < poly.length; i++) {
      var p = poly[i], q = poly[(i + 1) % poly.length];
      var ex = q[0] - p[0], ey = q[1] - p[1];
      var nx = -ey, ny = ex;
      if (nx * (cx - p[0]) + ny * (cy - p[1]) < 0) { nx = -nx; ny = -ny; }
      var denom = nx * dx + ny * dy, dist = nx * (ax - p[0]) + ny * (ay - p[1]);
      if (Math.abs(denom) < 1e-12) { if (dist < 0) { return null; } continue; }
      var t = -dist / denom;
      if (denom > 0) { if (t > t0) { t0 = t; } }
      else { if (t < t1) { t1 = t; } }
      if (t0 > t1) { return null; }
    }
    if (t1 < 0 || t0 > 1) { return null; }
    return [clamp(t0, 0, 1), clamp(t1, 0, 1)];
  }
  function cutGeom(cut) {
    var dx = cut.x1 - cut.x0, dy = cut.y1 - cut.y0, L = Math.hypot(dx, dy);
    if (!(L > 1e-12)) { return null; }
    var sx = dx / L, sy = dy / L;
    return { L: L, sx: sx, sy: sy, nx: -sy, ny: sx };
  }
  function integrateCuts(cuts, elems, all) {
    var out = {};
    (cuts || []).forEach(function (cut) {
      var g = cutGeom(cut);
      var res = { L: 0, Fn: 0, Fs: 0, M: 0, segs: [], hit: false, cax: cut.cx, cay: cut.cy };
      if (!g || !elems || !all) { out[cut.id] = res; return; }
      res.L = g.L;
      elems.forEach(function (ae) {
        if (!ae.s) { return; }
        var poly = ae.g.map(function (gi) { return all[gi]; });
        var tt = clipSegPoly(cut.x0, cut.y0, cut.x1, cut.y1, poly);
        if (!tt) { return; }
        var a = tt[0] * g.L, b = tt[1] * g.L;
        if (!(b - a > 1e-12)) { return; }
        res.segs.push({ a: a, b: b, s: ae.s, t: ae.t || 0.2 });
        res.hit = true;
      });
      res.segs.sort(function (p, q) { return p.a - q.a; });
      if (res.hit && cut.cAuto !== false) {
        var mid = (res.segs[0].a + res.segs[res.segs.length - 1].b) / 2;
        cut.cx = cut.x0 + g.sx * mid; cut.cy = cut.y0 + g.sy * mid;
      }
      res.cax = cut.cx; res.cay = cut.cy;
      res.segs.forEach(function (sg) {
        var len = sg.b - sg.a, s = sg.s, t = sg.t;
        var tx = s.sx * g.nx + s.txy * g.ny;
        var ty = s.txy * g.nx + s.sy * g.ny;
        res.Fn += (tx * g.nx + ty * g.ny) * t * len;
        res.Fs += (tx * g.sx + ty * g.sy) * t * len;
        var mx = cut.x0 + g.sx * (sg.a + sg.b) / 2, my = cut.y0 + g.sy * (sg.a + sg.b) / 2;
        res.M += ((mx - cut.cx) * ty - (my - cut.cy) * tx) * t * len;
      });
      out[cut.id] = res;
    });
    return out;
  }
  function rectInfo(P) {
    if (!P || P.length < 4) { return null; }
    function dd(i, j) { return Math.hypot(P[j][0] - P[i][0], P[j][1] - P[i][1]); }
    var L01 = dd(0, 1), L12 = dd(1, 2), L23 = dd(2, 3), L30 = dd(3, 0);
    if (L01 < 1e-9 || L12 < 1e-9) { return null; }
    var tol = 0.02 * Math.max(L01, L12);
    if (Math.abs(L01 - L23) > tol || Math.abs(L12 - L30) > tol) { return null; }
    var D02 = Math.hypot(P[2][0] - P[0][0], P[2][1] - P[0][1]);
    var D13 = Math.hypot(P[3][0] - P[1][0], P[3][1] - P[1][1]);
    if (Math.abs(D02 - D13) > 0.02 * Math.max(D02, D13)) { return null; }
    var ux = (P[1][0] - P[0][0]) / L01, uy = (P[1][1] - P[0][1]) / L01;
    var vx = (P[3][0] - P[0][0]) / L30, vy = (P[3][1] - P[0][1]) / L30;
    if (Math.abs(ux * vx + uy * vy) > 0.05) { return null; }
    return { Lu: L01, Lv: L30, ux: ux, uy: uy, vx: vx, vy: vy };
  }
  function linInterp(xs, ys, x) {
    var n = xs.length, i, t;
    if (n === 0) { return 0; }
    if (n === 1) { return ys[0]; }
    if (x <= xs[0]) { i = 0; t = (x - xs[0]) / ((xs[1] - xs[0]) || 1e-12); return ys[0] + (ys[1] - ys[0]) * t; }
    if (x >= xs[n - 1]) { i = n - 2; t = (x - xs[n - 2]) / ((xs[n - 1] - xs[n - 2]) || 1e-12); return ys[n - 2] + (ys[n - 1] - ys[n - 2]) * t; }
    for (i = 0; i + 1 < n; i++) {
      if (x >= xs[i] && x <= xs[i + 1]) {
        t = (x - xs[i]) / ((xs[i + 1] - xs[i]) || 1e-12);
        return ys[i] + (ys[i + 1] - ys[i]) * t;
      }
    }
    return ys[n - 1];
  }
  function buildShellCuts(areas, grid, elems) {
    var cuts = [];
    areas.forEach(function (a) {
      if (!a || a.etype !== "shell" || !a.nodes || a.nodes.length !== 4) { return; }
      var g = grid[a.id];
      if (!g || !g.nx || !g.ny) { return; }
      var ri = rectInfo(g.P);
      if (!ri) { return; }
      var list = elems.filter(function (e) { return e.aid === a.id && e.s; });
      if (list.length !== g.nx * g.ny) { return; }
      var nx = g.nx, ny = g.ny, t = g.t || 0.2, Lu = ri.Lu, Lv = ri.Lv;
      var du = Lu / nx, dv = Lv / ny, r, c;
      function cs(r2, c2) { var e = list[r2 * nx + c2]; return e ? e.s : null; }
      var bu = [], bN = [], bV = [], bM = [];
      for (c = 0; c < nx; c++) {
        var N = 0, V = 0, M = 0;
        for (r = 0; r < ny; r++) {
          var s = cs(r, c);
          if (!s) { continue; }
          N += s.sx * t * dv; V += s.txy * t * dv;
          M += -s.sx * t * ((r + 0.5) * dv - Lv / 2) * dv;
        }
        bu.push((c + 0.5) * du); bN.push(N); bV.push(V); bM.push(M);
      }
      var cv = [], cN = [], cV = [], cM = [];
      for (r = 0; r < ny; r++) {
        var N2 = 0, V2 = 0, M2 = 0;
        for (c = 0; c < nx; c++) {
          var s2 = cs(r, c);
          if (!s2) { continue; }
          N2 += s2.sy * t * du; V2 += -s2.txy * t * du;
          M2 += s2.sy * t * ((c + 0.5) * du - Lu / 2) * du;
        }
        cv.push((r + 0.5) * dv); cN.push(N2); cV.push(V2); cM.push(M2);
      }
      var P = g.P;
      var b0 = [P[0][0] + ri.vx * Lv / 2, P[0][1] + ri.vy * Lv / 2];
      var b1 = [P[1][0] + ri.vx * Lv / 2, P[1][1] + ri.vy * Lv / 2];
      var c0 = [P[0][0] + ri.ux * Lu / 2, P[0][1] + ri.uy * Lu / 2];
      var c1 = [P[3][0] + ri.ux * Lu / 2, P[3][1] + ri.uy * Lu / 2];
      function series(xs, Ns, Vs, Ms, L) {
        var nS = Math.max(49, xs.length * 4), pts = [], i;
        var mN = 0, mV = 0, mM = 0;
        for (i = 0; i < nS; i++) {
          var x = L * i / (nS - 1);
          var qN = xs.length > 1 ? linInterp(xs, Ns, x) : (Ns[0] || 0);
          var qV = xs.length > 1 ? linInterp(xs, Vs, x) : (Vs[0] || 0);
          var qM = xs.length > 1 ? linInterp(xs, Ms, x) : (Ms[0] || 0);
          pts.push({ x: x, N: qN, V: qV, M: qM });
          mN = Math.max(mN, Math.abs(qN)); mV = Math.max(mV, Math.abs(qV)); mM = Math.max(mM, Math.abs(qM));
        }
        return { pts: pts, maxN: mN, maxV: mV, maxM: mM };
      }
      var bs = series(bu, bN, bV, bM, Lu), cs2 = series(cv, cN, cV, cM, Lv);
      function ends(pts) {
        if (!pts.length) { return { a: { N: 0, V: 0, M: 0 }, m: { N: 0, V: 0, M: 0 }, b: { N: 0, V: 0, M: 0 } }; }
        return { a: pts[0], m: pts[Math.floor(pts.length / 2)], b: pts[pts.length - 1] };
      }
      cuts.push({ aid: a.id, kind: "beam", L: Lu, t: t, p0: b0, p1: b1, pts: bs.pts, maxN: bs.maxN, maxV: bs.maxV, maxM: bs.maxM, ends: ends(bs.pts) });
      cuts.push({ aid: a.id, kind: "col", L: Lv, t: t, p0: c0, p1: c1, pts: cs2.pts, maxN: cs2.maxN, maxV: cs2.maxV, maxM: cs2.maxM, ends: ends(cs2.pts) });
    });
    return cuts;
  }  /* ---------------- 主入口：线弹性分析 ---------------- */
  function solve(ctx) {
    ctx = ctx || {};
    var model = ctx.model || { nodes: [], members: [], areas: [], cuts: [] };
    var C = {
      NSUB: ctx.NSUB != null ? ctx.NSUB : 5,
      AMESH: ctx.AMESH != null ? ctx.AMESH : 2,
      memStiff: ctx.memStiff || {}, memMat: ctx.memMat || {}, memSec: ctx.memSec || {},
      memLoads: ctx.memLoads || {}, nodeLoads: ctx.nodeLoads || {},
      areaMat: ctx.areaMat || {}, areaSec: ctx.areaSec || {}, areaLoads: ctx.areaLoads || {},
      memDiv: ctx.memDiv || {}, areaDiv: ctx.areaDiv || {},
      MATERIALS: ctx.MATERIALS || BUILTIN_MATERIALS,
      DEF_STIFF: ctx.DEF_STIFF || BUILTIN_DEF_STIFF
    };
    function nodeById(id) { return (model.nodes || []).find(function (n) { return n.id === id; }); }
    function areaById(id) { return (model.areas || []).find(function (a) { return a.id === id; }); }
    function areaMatOf(aid) {
      return C.areaMat[aid] || { label: "C30", E: (C.MATERIALS.C30 || BUILTIN_MATERIALS.C30).E, nu: (C.MATERIALS.C30 || BUILTIN_MATERIALS.C30).nu };
    }
    function areaSecOf(aid) {
      var s = C.areaSec[aid];
      if (s) { return s; }
      var a = areaById(aid);
      return { kind: (a && a.etype === "shell") ? "shell-thin" : "stress", t: 0.2 };
    }
    function effFrameDiv(m) {
      var n = clampInt(C.NSUB, 1, 50), o = C.memDiv[m.id];
      if (o) {
        n = clampInt(o.n || n, 1, 50);
        if (o.lmax > 0 && m.L > 0) { n = Math.max(n, Math.ceil(m.L / o.lmax)); }
        n = clampInt(n, 1, 50);
      }
      return n;
    }
    function nzS(n) { return (n && isFinite(n.z)) ? n.z : 0; }
    function d3(a, b) { return Math.hypot(a.x - b.x, a.y - b.y, nzS(a) - nzS(b)); }
    function effAreaDiv(a) {
      var nx = clampInt(C.AMESH, 1, 50), ny = nx, o = C.areaDiv[a.id];
      if (o) { nx = clampInt(o.nx || nx, 1, 50); ny = clampInt(o.ny || ny, 1, 50); }
      if (o && o.hmax > 0) {
        var pts = a.nodes.map(nodeById).filter(Boolean), Lmax = 0, i;
        for (i = 0; i < pts.length; i++) {
          var p = pts[i], q = pts[(i + 1) % pts.length];
          Lmax = Math.max(Lmax, d3(p, q));
        }
        if (Lmax > 0) { var k = Math.min(50, Math.ceil(Lmax / o.hmax)); nx = Math.max(nx, k); ny = Math.max(ny, k); }
      }
      return { nx: nx, ny: ny };
    }
    function stiffOf(m) {
      if (C.memStiff[m.id]) { return C.memStiff[m.id]; }
      var mat = C.memMat[m.id], sec = C.memSec[m.id];
      if (mat && sec) {
        var p = sectionProps(sec);
        if (p && p.A > 0) {
          var G = mat.E / (2 * (1 + (mat.nu || 0.3)));
          return { EA: mat.E * p.A, EI: mat.E * p.I, GA: G * p.As };
        }
      }
      return C.DEF_STIFF[m.type] || C.DEF_STIFF.user;
    }
    /* 3D 判定：任一节点 z≠0、任一杆件跨越 z、任一 6 自由度边界/荷载分量、任一面单元不共 XY 平面 */
    function is3DModel() {
      var ms = model.nodes || [], i;
      for (i = 0; i < ms.length; i++) {
        var n = ms[i];
        if (Math.abs(nzS(n)) > 1e-9) { return true; }
        var bc = n.bc || {};
        if (bc.uz || bc.rx || bc.ry) { return true; }
      }
      var nl;
      for (var k in C.nodeLoads) {
        nl = C.nodeLoads[k] || {};
        if ((nl.fz || 0) || (nl.mx || 0) || (nl.my || 0)) { return true; }
      }
      function nb(id) { return (model.nodes || []).find(function (x) { return x.id === id; }); }
      for (i = 0; i < (model.members || []).length; i++) {
        var m = model.members[i], A = nb(m.a), B = nb(m.b);
        if (A && B && Math.abs(nzS(A) - nzS(B)) > 1e-9) { return true; }
      }
      for (i = 0; i < (model.areas || []).length; i++) {
        var a = model.areas[i], zs = (a.nodes || []).map(function (id) { var t = nb(id); return t ? nzS(t) : 0; });
        var z0 = zs.length ? zs[0] : 0;
        for (var j = 1; j < zs.length; j++) { if (Math.abs(zs[j] - z0) > 1e-9) { return true; } }
      }
      return false;
    }
    if (is3DModel()) { return solve3D(ctx, C, model); }

    var nodes = model.nodes || [];
    var all = [];
    var reg = {};
    // 分析节点合并：x,y,z 三向（兼容 2D 老数据 z 缺省 0）；精度 1e-6 与内置一致
    function ensureNode(x, y, z) {
      var zz = isFinite(z) ? z : 0;
      var k = (Math.round(x * 1e6) / 1e6) + "," + (Math.round(y * 1e6) / 1e6) + "," + (Math.round(zz * 1e6) / 1e6);
      if (reg[k] !== undefined) { return reg[k]; }
      reg[k] = all.length; all.push([x, y]); return reg[k];
    }
    var idx = {};
    nodes.forEach(function (n) { idx[n.id] = ensureNode(n.x, n.y, nzS(n)); });
    var elems = [];
    var areaElems = [];
    var areaGrid = {};
    var areaNodal = [];
    function nodalAcc(i) {
      if (!areaNodal[i]) { areaNodal[i] = { sx: 0, sy: 0, txy: 0, vm: 0, s1: 0, nx: 0, n: 0 }; }
      return areaNodal[i];
    }

    (model.members || []).forEach(function (m) {
      var ia = idx[m.a], ib = idx[m.b];
      if (ia === undefined || ib === undefined) { return; }
      var A = all[ia], Bp = all[ib];
      var na = nodeById(m.a), nb = nodeById(m.b);
      var L3 = (na && nb) ? d3(na, nb) : Math.hypot(Bp[0] - A[0], Bp[1] - A[1]);
      var Lm = Math.hypot(Bp[0] - A[0], Bp[1] - A[1]);
      if (L3 > 1e-9) { m.L = L3; }
      if (Lm > 1e-9) { m.ux = (Bp[0] - A[0]) / Lm; m.uy = (Bp[1] - A[1]) / Lm; m.x1 = A[0]; m.y1 = A[1]; }
    });

    var areaDivUse = {}, memTarget = {};
    (model.areas || []).forEach(function (a) { var d = effAreaDiv(a); areaDivUse[a.id] = { nx: d.nx, ny: d.ny }; });
    (model.members || []).forEach(function (m) { memTarget[m.id] = effFrameDiv(m); });
    function edgeSameMember(p, q, m) {
      var mp = nodeById(m.a), mq = nodeById(m.b);
      if (!mp || !mq) { return false; }
      return (d3(mp, p) < 1e-6 && d3(mq, q) < 1e-6) ||
             (d3(mp, q) < 1e-6 && d3(mq, p) < 1e-6);
    }
    (model.areas || []).forEach(function (a) {
      var ids = a.nodes, cn = ids.length;
      for (var e = 0; e < cn; e++) {
        var p = nodeById(ids[e]), q = nodeById(ids[(e + 1) % cn]);
        if (!p || !q) { continue; }
        var isU = cn < 4 || e === 0 || e === 2;
        (model.members || []).forEach(function (m) {
          if (!edgeSameMember(p, q, m)) { return; }
          var n = Math.max(isU ? areaDivUse[a.id].nx : areaDivUse[a.id].ny, memTarget[m.id] || effFrameDiv(m));
          if (isU) { areaDivUse[a.id].nx = n; } else { areaDivUse[a.id].ny = n; }
          memTarget[m.id] = n;
        });
      }
    });

    (model.areas || []).forEach(function (a) {
      var corners = a.nodes.map(nodeById).filter(Boolean);
      if (corners.length < 3) { return; }
      var mat = areaMatOf(a.id), sec = areaSecOf(a.id);
      var Dmat = planeD(mat.E, mat.nu, sec.kind === "strain" || sec.kind === "shell-thick" ? "strain" : "stress");
      var t = Math.max(1e-6, sec.t || 0.2);
      var dv = areaDivUse[a.id] || effAreaDiv(a), nx = dv.nx, ny = dv.ny;
      if (corners.length >= 4) {
        var P = [corners[0], corners[1], corners[2], corners[3]].map(function (n) { return [n.x, n.y]; });
        var za = nzS(corners[0]);
        areaGrid[a.id] = { nx: nx, ny: ny, P: P, t: t };
        var sub = quadSubCells(P, nx, ny), gid = [], r, c;
        for (r = 0; r <= ny; r++) {
          gid[r] = [];
          for (c = 0; c <= nx; c++) {
            var onCorner = (r === 0 && c === 0) ? 0 : (r === 0 && c === nx) ? 1 : (r === ny && c === nx) ? 2 : (r === ny && c === 0) ? 3 : -1;
            if (onCorner >= 0) { gid[r][c] = idx[a.nodes[onCorner]]; }
            else { gid[r][c] = ensureNode(sub.grid[r][c][0], sub.grid[r][c][1], za); }
          }
        }
        sub.cells.forEach(function (cell) {
          var g = [gid[Math.floor(cell[0] / (nx + 1))][cell[0] % (nx + 1)], gid[Math.floor(cell[1] / (nx + 1))][cell[1] % (nx + 1)], gid[Math.floor(cell[2] / (nx + 1))][cell[2] % (nx + 1)], gid[Math.floor(cell[3] / (nx + 1))][cell[3] % (nx + 1)]];
          var coords = g.map(function (gi) { return all[gi]; });
          var q = q4Stiffness(coords, Dmat, t);
          areaElems.push({ aid: a.id, etype: a.etype, kind: "q4", g: g, B: q.B0, Dmat: Dmat, t: t });
        });
      } else {
        var P3 = [corners[0], corners[1], corners[2]].map(function (n) { return [n.x, n.y]; });
        var za3 = nzS(corners[0]);
        var sub3 = triSubCells(P3, nx), gmap = [];
        sub3.pts.forEach(function (p) {
          var gi = -1, k;
          for (k = 0; k < 3; k++) {
            if (Math.hypot(p[0] - P3[k][0], p[1] - P3[k][1]) < 1e-9) { gi = idx[a.nodes[k]]; break; }
          }
          gmap.push(gi >= 0 ? gi : ensureNode(p[0], p[1], za3));
        });
        sub3.tris.forEach(function (tr) {
          var g = [gmap[tr[0]], gmap[tr[1]], gmap[tr[2]]];
          var coords = g.map(function (gi) { return all[gi]; });
          var cs = cstStiffness(coords, Dmat, t);
          areaElems.push({ aid: a.id, etype: a.etype, kind: "cst", g: g, B: cs.B, Dmat: Dmat, t: t });
        });
      }
    });

    (model.members || []).forEach(function (m) {
      var ia = idx[m.a], ib = idx[m.b];
      if (ia === undefined || ib === undefined) { return; }
      var A = all[ia], Bp = all[ib];
      var Lm = Math.hypot(Bp[0] - A[0], Bp[1] - A[1]);
      if (Lm < 1e-9) { return; }   // 面外（Z向）杆件不参与 2D 内置求解（走 OpenSees 3D 路径），m.L 保留 3D 真长
      var na0 = nodeById(m.a), zm = na0 ? nzS(na0) : 0;
      m.L = Lm; m.ux = (Bp[0] - A[0]) / Lm; m.uy = (Bp[1] - A[1]) / Lm; m.x1 = A[0]; m.y1 = A[1];
      var n = clampInt(memTarget[m.id] || effFrameDiv(m), 1, 60);
      var ts = [], s;
      for (s = 0; s <= n; s++) { ts.push(s / n); }
      for (var gi = 0; gi < all.length; gi++) {
        var px = all[gi][0] - A[0], py = all[gi][1] - A[1];
        var proj = px * m.ux + py * m.uy;
        if (proj <= 1e-7 || proj >= Lm - 1e-7) { continue; }
        if (Math.abs(px * (-m.uy) + py * m.ux) < 1e-6) { ts.push(proj / Lm); }
      }
      ts.sort(function (x, y) { return x - y; });
      var tsU = [];
      for (s = 0; s < ts.length; s++) { if (!tsU.length || ts[s] - tsU[tsU.length - 1] > 1e-7) { tsU.push(ts[s]); } }
      if (tsU[0] > 1e-9) { tsU.unshift(0); }
      tsU[0] = 0;
      if (tsU[tsU.length - 1] < 1 - 1e-9) { tsU.push(1); }
      tsU[tsU.length - 1] = 1;
      var ids2 = tsU.map(function (tt) { return ensureNode(A[0] + (Bp[0] - A[0]) * tt, A[1] + (Bp[1] - A[1]) * tt, zm); });
      for (var k = 0; k + 1 < ids2.length; k++) {
        if (ids2[k] === ids2[k + 1]) { continue; }
        elems.push({ key: m.id, member: m, a: ids2[k], b: ids2[k + 1], xa: tsU[k] * Lm, Lsub: (tsU[k + 1] - tsU[k]) * Lm });
      }
    });

    var ndof = all.length * 3;
    var K = zeros(ndof, ndof);
    elems.forEach(function (el) {
      var p1 = all[el.a], p2 = all[el.b];
      var st = stiffOf(el.member);
      var mm = elemMatrices(p1[0], p1[1], p2[0], p2[1], st.EA, st.EI, st.GA);
      el.kl = mm.kl; el.T = mm.T;
      el.p = elemLoadVector(el, C.memLoads);
      var map = [3 * el.a, 3 * el.a + 1, 3 * el.a + 2, 3 * el.b, 3 * el.b + 1, 3 * el.b + 2];
      for (var i = 0; i < 6; i++) { for (var j = 0; j < 6; j++) { K[map[i]][map[j]] += mm.kg[i][j]; } }
    });

    areaElems.forEach(function (ae) {
      var coords = ae.g.map(function (gi) { return all[gi]; });
      var ke;
      if (ae.kind === "q4") { ke = q4Stiffness(coords, ae.Dmat, ae.t).ke; }
      else { ke = cstStiffness(coords, ae.Dmat, ae.t).ke; }
      ae.ke = ke;
      var nn = ae.g.length, map2 = [];
      for (var a2 = 0; a2 < nn; a2++) { map2.push(3 * ae.g[a2], 3 * ae.g[a2] + 1); }
      for (var i2 = 0; i2 < 2 * nn; i2++) { for (var j2 = 0; j2 < 2 * nn; j2++) { K[map2[i2]][map2[j2]] += ke[i2][j2]; } }
    });

    var F = new Array(ndof).fill(0);
    nodes.forEach(function (n) {
      var nl = C.nodeLoads[n.id], ni = idx[n.id];
      if (nl) { F[3 * ni] += nl.fx || 0; F[3 * ni + 1] += nl.fy || 0; F[3 * ni + 2] += nl.mz || 0; }
    });
    elems.forEach(function (el) {
      var pg = matVec(transpose(el.T), el.p);
      var map = [3 * el.a, 3 * el.a + 1, 3 * el.a + 2, 3 * el.b, 3 * el.b + 1, 3 * el.b + 2];
      for (var i = 0; i < 6; i++) { F[map[i]] += pg[i]; }
    });

    areaElems.forEach(function (ae) {
      var ld = C.areaLoads[ae.aid];
      if (!ld || (!ld.qx && !ld.qy)) { return; }
      var qx = ld.qx || 0, qy = ld.qy || 0, nn = ae.g.length;
      if (ae.kind === "q4") {
        var coords = ae.g.map(function (gi) { return all[gi]; });
        var g = 1 / Math.sqrt(3);
        [[-g, -g, 1], [g, -g, 1], [g, g, 1], [-g, g, 1]].forEach(function (gp) {
          var xi = gp[0], et = gp[1], w = gp[2], N = q4N(xi, et);
          var J = [[0, 0], [0, 0]], a;
          for (a = 0; a < 4; a++) {
            var dNx = [-(1 - et) / 4, (1 - et) / 4, (1 + et) / 4, -(1 + et) / 4][a];
            var dNe = [-(1 - xi) / 4, -(1 + xi) / 4, (1 + xi) / 4, (1 - xi) / 4][a];
            J[0][0] += dNx * coords[a][0]; J[0][1] += dNx * coords[a][1];
            J[1][0] += dNe * coords[a][0]; J[1][1] += dNe * coords[a][1];
          }
          var det = Math.abs(J[0][0] * J[1][1] - J[0][1] * J[1][0]);
          for (a = 0; a < 4; a++) {
            F[3 * ae.g[a]] += N[a] * qx * ae.t * det * w;
            F[3 * ae.g[a] + 1] += N[a] * qy * ae.t * det * w;
          }
        });
      } else {
        var p1 = all[ae.g[0]], p2 = all[ae.g[1]], p3 = all[ae.g[2]];
        var A3 = Math.abs((p2[0] - p1[0]) * (p3[1] - p1[1]) - (p3[0] - p1[0]) * (p2[1] - p1[1])) / 2;
        for (var a3 = 0; a3 < 3; a3++) {
          F[3 * ae.g[a3]] += qx * ae.t * A3 / 3;
          F[3 * ae.g[a3] + 1] += qy * ae.t * A3 / 3;
        }
      }
    });

    var fixed = {}, free = [];
    nodes.forEach(function (n) {
      var ni = idx[n.id];
      var bc = n.bc || {};
      if (bc.ux) { fixed[3 * ni] = true; }
      if (bc.uy) { fixed[3 * ni + 1] = true; }
      if (bc.rz) { fixed[3 * ni + 2] = true; }
    });
    (function () {
      var md = 0, d;
      for (d = 0; d < ndof; d++) { md = Math.max(md, Math.abs(K[d][d])); }
      var eps = md * 1e-12;
      for (d = 0; d < ndof; d++) { if (!fixed[d] && Math.abs(K[d][d]) <= eps) { fixed[d] = true; } }
    })();
    for (var d = 0; d < ndof; d++) { if (!fixed[d]) { free.push(d); } }

    var Kr = zeros(free.length, free.length);
    for (var r = 0; r < free.length; r++) { for (var cc = 0; cc < free.length; cc++) { Kr[r][cc] = K[free[r]][free[cc]]; } }
    var ur = solveLin(Kr, free.map(function (d) { return F[d]; }));
    var u = new Array(ndof).fill(0);
    free.forEach(function (d, k) { u[d] = ur[k]; });
    var ok = u.every(function (v) { return isFinite(v); });

    var maxM = 0, maxV = 0, maxN = 0, byMember = {};
    elems.forEach(function (el) {
      var ue = [u[3 * el.a], u[3 * el.a + 1], u[3 * el.a + 2], u[3 * el.b], u[3 * el.b + 1], u[3 * el.b + 2]];
      var f = matVec(el.kl, matVec(el.T, ue));
      var fe = [f[0] - el.p[0], f[1] - el.p[1], f[2] - el.p[2], f[3] - el.p[3], f[4] - el.p[4], f[5] - el.p[5]];
      el.axial = -fe[0];
      maxN = Math.max(maxN, Math.abs(el.axial));
      var N = 12, pts = [];
      for (var k = 0; k <= N; k++) {
        var t = el.Lsub * k / N;
        var R2 = loadResultants(el.member, el.xa, el.xa + t, C.memLoads);
        var J = (el.xa + t) * R2.A - R2.Bb;
        var S = fe[1] + R2.A;
        var Mo = -(fe[2] - fe[1] * t - J);
        pts.push({ t: t, S: S, M: Mo });
        maxM = Math.max(maxM, Math.abs(Mo)); maxV = Math.max(maxV, Math.abs(S));
      }
      el.forces = pts;
      if (!byMember[el.key]) { byMember[el.key] = []; }
      byMember[el.key].push(el);
    });

    var maxVM = 0, maxSX = 0;
    areaElems.forEach(function (ae) {
      var nn = ae.g.length, ue = [];
      for (var a4 = 0; a4 < nn; a4++) { ue.push(u[3 * ae.g[a4]], u[3 * ae.g[a4] + 1]); }
      var eps = [0, 0, 0], r3, c3;
      for (r3 = 0; r3 < 3; r3++) { var s3 = 0; for (c3 = 0; c3 < 2 * nn; c3++) { s3 += ae.B[r3][c3] * ue[c3]; } eps[r3] = s3; }
      var st = [0, 0, 0];
      for (r3 = 0; r3 < 3; r3++) { st[r3] = ae.Dmat[r3][0] * eps[0] + ae.Dmat[r3][1] * eps[1] + ae.Dmat[r3][2] * eps[2]; }
      var sx = st[0], sy = st[1], txy = st[2];
      var avg = (sx + sy) / 2, R = Math.hypot((sx - sy) / 2, txy);
      var s1 = avg + R, s2 = avg - R;
      var vm = Math.sqrt(sx * sx - sx * sy + sy * sy + 3 * txy * txy);
      ae.s = { sx: sx, sy: sy, txy: txy, s1: s1, s2: s2, vm: vm, nx: sx * ae.t, ny: sy * ae.t, nxy: txy * ae.t };
      ae.cx = 0; ae.cy = 0;
      for (var a5 = 0; a5 < nn; a5++) { ae.cx += all[ae.g[a5]][0]; ae.cy += all[ae.g[a5]][1]; }
      ae.cx /= nn; ae.cy /= nn;
      maxVM = Math.max(maxVM, Math.abs(vm)); maxSX = Math.max(maxSX, Math.abs(sx), Math.abs(sy));
      for (var a6 = 0; a6 < nn; a6++) {
        var acc = nodalAcc(ae.g[a6]);
        acc.sx += sx; acc.sy += sy; acc.txy += txy; acc.vm += vm; acc.s1 += s1; acc.nx += sx * ae.t; acc.n++;
      }
    });

    var shellCuts = buildShellCuts(model.areas || [], areaGrid, areaElems);
    var cutResults = integrateCuts(model.cuts || [], areaElems, all);

    var maxDisp = 0;
    for (var i2 = 0; i2 < all.length; i2++) { maxDisp = Math.max(maxDisp, Math.hypot(u[3 * i2], u[3 * i2 + 1])); }
    var ys = nodes.map(function (n) { return n.y; });
    var ymax = ys.length ? Math.max.apply(null, ys) : 0;
    var roof = 0, nr = 0;
    nodes.forEach(function (n) { if (Math.abs(n.y - ymax) < 1e-6) { roof += u[3 * idx[n.id]]; nr++; } });
    roof = nr ? roof / nr : 0;

    return {
      nodes: nodes, all: all, elems: elems, u: u, ok: ok, idx: idx, byMember: byMember,
      areaElems: areaElems, areaNodal: areaNodal, areaGrid: areaGrid, shellCuts: shellCuts,
      cutResults: cutResults,
      res: { roof: roof * 1000, maxDisp: maxDisp * 1000, maxM: maxM, maxV: maxV, maxN: maxN, maxVM: maxVM, maxSX: maxSX, nNode: nodes.length, nMember: (model.members || []).length, nArea: (model.areas || []).length }
    };
  }

  /* ---------------- 3D 杆系求解（6 自由度/节点：ux,uy,uz,rx,ry,rz） ----------------
   * 触发条件见 solve() 内 is3DModel()。杆件为 3D 欧拉梁（轴向+扭转+双向弯曲），
   * 非竖向杆局部 y 取竖向平面内横向（梁均布即重力），竖向杆 y 取水平横向；
   * 杆件横向荷载（udl/point/trap）一律沿局部 y 施加（与 xara 真 3D 同坐标架）；
   * 面单元仅支承共 XY 平面（等 z）者（膜行为耦合 ux/uy），空间斜面跳过并计数
   * （精确解走 OpenSees 真 3D 路径）。位移 u stride=6，内力站含 N/Vy/Vz/T/My/Mz，
   * 其中 S=Vy、M=Mz 保持与 2D 同名字段兼容。
   */
  function solve3D(ctx, C, model) {
    function nodeById(id) { return (model.nodes || []).find(function (n) { return n.id === id; }); }
    function areaById(id) { return (model.areas || []).find(function (a) { return a.id === id; }); }
    function nzS(n) { return (n && isFinite(n.z)) ? n.z : 0; }
    function d3(a, b) { return Math.hypot(a.x - b.x, a.y - b.y, nzS(a) - nzS(b)); }
    function effFrameDiv(m) {
      var n = clampInt(C.NSUB, 1, 50), o = C.memDiv[m.id];
      if (o) {
        n = clampInt(o.n || n, 1, 50);
        if (o.lmax > 0 && m.L > 0) { n = Math.max(n, Math.ceil(m.L / o.lmax)); }
        n = clampInt(n, 1, 50);
      }
      return n;
    }
    function effAreaDiv(a) {
      var nx = clampInt(C.AMESH, 1, 50), ny = nx, o = C.areaDiv[a.id];
      if (o) { nx = clampInt(o.nx || nx, 1, 50); ny = clampInt(o.ny || ny, 1, 50); }
      if (o && o.hmax > 0) {
        var pts = a.nodes.map(nodeById).filter(Boolean), Lmax = 0, i;
        for (i = 0; i < pts.length; i++) {
          var p = pts[i], q = pts[(i + 1) % pts.length];
          Lmax = Math.max(Lmax, d3(p, q));
        }
        if (Lmax > 0) { var k = Math.min(50, Math.ceil(Lmax / o.hmax)); nx = Math.max(nx, k); ny = Math.max(ny, k); }
      }
      return { nx: nx, ny: ny };
    }
    function areaMatOf(aid) {
      return C.areaMat[aid] || { label: "C30", E: (C.MATERIALS.C30 || BUILTIN_MATERIALS.C30).E, nu: (C.MATERIALS.C30 || BUILTIN_MATERIALS.C30).nu };
    }
    function areaSecOf(aid) {
      var s = C.areaSec[aid];
      if (s) { return s; }
      var a = areaById(aid);
      return { kind: (a && a.etype === "shell") ? "shell-thin" : "stress", t: 0.2 };
    }
    function stiff3Of(m) {
      var ov = C.memStiff[m.id];
      if (ov) { return { EA: ov.EA, EIz: ov.EI, EIy: ov.EI, GJ: 0.77 * ov.EI }; }
      var mat = C.memMat[m.id], sec = C.memSec[m.id];
      if (mat && sec) {
        var p = sectionProps3D(sec);
        if (p && p.A > 0) {
          var G = mat.E / (2 * (1 + (mat.nu || 0.3)));
          return { EA: mat.E * p.A, EIz: mat.E * p.Iz, EIy: mat.E * p.Iy, GJ: G * p.J };
        }
      }
      var d = C.DEF_STIFF[m.type] || C.DEF_STIFF.user;
      return { EA: d.EA, EIz: d.EI, EIy: d.EI, GJ: 0.77 * d.EI };
    }

    var nodes = model.nodes || [];
    var all = [], reg = {};
    function ensureNode(x, y, z) {
      var zz = isFinite(z) ? z : 0;
      var k = (Math.round(x * 1e6) / 1e6) + "," + (Math.round(y * 1e6) / 1e6) + "," + (Math.round(zz * 1e6) / 1e6);
      if (reg[k] !== undefined) { return reg[k]; }
      reg[k] = all.length; all.push([x, y, zz]); return reg[k];
    }
    var idx = {};
    nodes.forEach(function (n) { idx[n.id] = ensureNode(n.x, n.y, nzS(n)); });
    var elems = [], areaElems = [], areaGrid = {}, areaNodal = [], skippedArea3D = 0;
    function nodalAcc(i) {
      if (!areaNodal[i]) { areaNodal[i] = { sx: 0, sy: 0, txy: 0, vm: 0, s1: 0, nx: 0, n: 0 }; }
      return areaNodal[i];
    }

    /* 杆件几何（3D 真长）+ 切分（NSUB/memDiv 等分 + 落杆分析节点投影） */
    (model.members || []).forEach(function (m) {
      var A = nodeById(m.a), B = nodeById(m.b);
      if (!A || !B) { return; }
      var L3 = d3(A, B);
      if (!(L3 > 1e-9)) { return; }
      m.L = L3;
      var Lm = Math.hypot(B.x - A.x, B.y - A.y);
      if (Lm > 1e-9) { m.ux = (B.x - A.x) / Lm; m.uy = (B.y - A.y) / Lm; m.x1 = A.x; m.y1 = A.y; }
    });
    /* 杆—面共边细分协调（对照 2D solve：共边杆件与面单元取最大细分，保证节点共享、计算一致） */
    var areaDivUse3 = {}, memTarget3 = {};
    (model.areas || []).forEach(function (a) { var d = effAreaDiv(a); areaDivUse3[a.id] = { nx: d.nx, ny: d.ny }; });
    (model.members || []).forEach(function (m) { memTarget3[m.id] = effFrameDiv(m); });
    function edgeSameMember3(p, q, m) {
      var mp = nodeById(m.a), mq = nodeById(m.b);
      if (!mp || !mq) { return false; }
      return (d3(mp, p) < 1e-6 && d3(mq, q) < 1e-6) ||
             (d3(mp, q) < 1e-6 && d3(mq, p) < 1e-6);
    }
    (model.areas || []).forEach(function (a) {
      var ids = a.nodes, cn = ids.length;
      for (var e = 0; e < cn; e++) {
        var p = nodeById(ids[e]), q = nodeById(ids[(e + 1) % cn]);
        if (!p || !q) { continue; }
        var isU = cn < 4 || e === 0 || e === 2;
        (model.members || []).forEach(function (m) {
          if (!edgeSameMember3(p, q, m)) { return; }
          var n = Math.max(isU ? areaDivUse3[a.id].nx : areaDivUse3[a.id].ny, memTarget3[m.id] || effFrameDiv(m));
          if (isU) { areaDivUse3[a.id].nx = n; } else { areaDivUse3[a.id].ny = n; }
          memTarget3[m.id] = n;
        });
      }
    });
    (model.members || []).forEach(function (m) {
      var A = nodeById(m.a), B = nodeById(m.b);
      if (!A || !B) { return; }
      var L3 = d3(A, B);
      if (!(L3 > 1e-9)) { return; }
      var ex = (B.x - A.x) / L3, ey = (B.y - A.y) / L3, ez = (nzS(B) - nzS(A)) / L3;
      var n = clampInt(memTarget3[m.id] || effFrameDiv(m), 1, 60), ts = [], s;
      for (s = 0; s <= n; s++) { ts.push(s / n); }
      var gi;
      for (gi = 0; gi < all.length; gi++) {
        var px = all[gi][0] - A.x, py = all[gi][1] - A.y, pz = all[gi][2] - nzS(A);
        var proj = px * ex + py * ey + pz * ez;
        if (proj <= 1e-7 || proj >= L3 - 1e-7) { continue; }
        var qx = px - proj * ex, qy = py - proj * ey, qz = pz - proj * ez;
        if (Math.hypot(qx, qy, qz) < 1e-6) { ts.push(proj / L3); }
      }
      ts.sort(function (x, y) { return x - y; });
      var tu = [];
      for (s = 0; s < ts.length; s++) { if (!tu.length || ts[s] - tu[tu.length - 1] > 1e-7) { tu.push(ts[s]); } }
      tu[0] = 0; tu[tu.length - 1] = 1;
      var zm = nzS(A);
      var ids2 = tu.map(function (tt) { return ensureNode(A.x + (B.x - A.x) * tt, A.y + (B.y - A.y) * tt, zm + (nzS(B) - zm) * tt); });
      for (var k = 0; k + 1 < ids2.length; k++) {
        if (ids2[k] === ids2[k + 1]) { continue; }
        elems.push({ key: m.id, member: m, a: ids2[k], b: ids2[k + 1], xa: tu[k] * L3, Lsub: (tu[k + 1] - tu[k]) * L3 });
      }
    });

    /* 面单元：共面（XY 等 z / XZ 等 y / YZ 等 x）者以膜刚度参与 3D 内置求解
     *（竖向墙按自身平面耦合面内自由度，与共边杆件共享分析节点）；
     * 真斜面（avis 不共任一坐标平面）仍跳过，需 OpenSees 真 3D 路径。 */
    function wallPlane3(corners) {
      var xs = corners.map(function (c) { return c.x; });
      var ys = corners.map(function (c) { return c.y; });
      var zs = corners.map(function (c) { return nzS(c); });
      function same(v) { var v0 = v[0]; return v.every(function (x) { return Math.abs(x - v0) < 1e-9; }); }
      var sx = same(xs), sy = same(ys), sz = same(zs);
      if (sz && !sx && !sy) { return "xy"; }
      if (sy && !sx && !sz) { return "xz"; }
      if (sx && !sy && !sz) { return "yz"; }
      if (sz) { return "xy"; }
      if (sy) { return "xz"; }
      if (sx) { return "yz"; }
      return null;
    }
    function locOf3(plane, x, y, z) {
      if (plane === "xz") { return [x, z]; }
      if (plane === "yz") { return [y, z]; }
      return [x, y];
    }
    function nodeOf3(plane, u, v, fx) {
      if (plane === "xz") { return ensureNode(u, fx, v); }
      if (plane === "yz") { return ensureNode(fx, u, v); }
      return ensureNode(u, v, fx);
    }
    function dofsOf3(plane) {
      if (plane === "xz") { return [0, 2]; }
      if (plane === "yz") { return [1, 2]; }
      return [0, 1];
    }
    (model.areas || []).forEach(function (a) {
      var corners = a.nodes.map(nodeById).filter(Boolean);
      if (corners.length < 3) { return; }
      var plane = wallPlane3(corners);
      if (!plane) { skippedArea3D++; return; }
      var dofs = dofsOf3(plane);
      var fix = plane === "xy" ? nzS(corners[0]) : plane === "xz" ? corners[0].y : corners[0].x;
      var mat = areaMatOf(a.id), sec = areaSecOf(a.id);
      var Dmat = planeD(mat.E, mat.nu, sec.kind === "strain" || sec.kind === "shell-thick" ? "strain" : "stress");
      var t = Math.max(1e-6, sec.t || 0.2);
      var dv = areaDivUse3[a.id] || effAreaDiv(a), nx = dv.nx, ny = dv.ny;
      if (corners.length >= 4) {
        var P = [corners[0], corners[1], corners[2], corners[3]].map(function (n) { return locOf3(plane, n.x, n.y, nzS(n)); });
        areaGrid[a.id] = { nx: nx, ny: ny, P: P, t: t, plane: plane };
        var sub = quadSubCells(P, nx, ny), gid = [], r, c;
        for (r = 0; r <= ny; r++) {
          gid[r] = [];
          for (c = 0; c <= nx; c++) {
            var onC = (r === 0 && c === 0) ? 0 : (r === 0 && c === nx) ? 1 : (r === ny && c === nx) ? 2 : (r === ny && c === 0) ? 3 : -1;
            gid[r][c] = onC >= 0 ? idx[a.nodes[onC]] : nodeOf3(plane, sub.grid[r][c][0], sub.grid[r][c][1], fix);
          }
        }
        sub.cells.forEach(function (cell) {
          var g = [gid[Math.floor(cell[0] / (nx + 1))][cell[0] % (nx + 1)], gid[Math.floor(cell[1] / (nx + 1))][cell[1] % (nx + 1)], gid[Math.floor(cell[2] / (nx + 1))][cell[2] % (nx + 1)], gid[Math.floor(cell[3] / (nx + 1))][cell[3] % (nx + 1)]];
          var coords = g.map(function (gi2) { return locOf3(plane, all[gi2][0], all[gi2][1], all[gi2][2]); });
          var q = q4Stiffness(coords, Dmat, t);
          areaElems.push({ aid: a.id, etype: a.etype, kind: "q4", g: g, B: q.B0, Dmat: Dmat, t: t, plane: plane, dofs: dofs });
        });
      } else {
        var P3 = [corners[0], corners[1], corners[2]].map(function (n) { return locOf3(plane, n.x, n.y, nzS(n)); });
        var sub3 = triSubCells(P3, nx), gmap = [];
        sub3.pts.forEach(function (p) {
          var gi3 = -1, k2;
          for (k2 = 0; k2 < 3; k2++) {
            if (Math.hypot(p[0] - P3[k2][0], p[1] - P3[k2][1]) < 1e-9) { gi3 = idx[a.nodes[k2]]; break; }
          }
          gmap.push(gi3 >= 0 ? gi3 : nodeOf3(plane, p[0], p[1], fix));
        });
        sub3.tris.forEach(function (tr) {
          var g = [gmap[tr[0]], gmap[tr[1]], gmap[tr[2]]];
          var coords = g.map(function (gi2) { return locOf3(plane, all[gi2][0], all[gi2][1], all[gi2][2]); });
          var cs = cstStiffness(coords, Dmat, t);
          areaElems.push({ aid: a.id, etype: a.etype, kind: "cst", g: g, B: cs.B, Dmat: Dmat, t: t, plane: plane, dofs: dofs });
        });
      }
    });

    /* 杆件局部荷载向量（局部 +y 横向，与 2D 同语义） */
    var GP3 = [-0.8611363116, -0.3399810436, 0.3399810436, 0.8611363116];
    var GW3 = [0.3478548451, 0.6521451549, 0.6521451549, 0.3478548451];
    function hShapes(x, L) { var t = x / L, t2 = t * t, t3 = t2 * t; return [1 - 3 * t2 + 2 * t3, L * (t - 2 * t2 + t3), 3 * t2 - 2 * t3, L * (-t2 + t3)]; }
    function inten(ld, x) {
      if (ld.type === "udl") { return ld.q; }
      if (ld.type === "trap") { if (x < ld.c || x > ld.d) { return 0; } return (ld.q1 + (ld.q2 - ld.q1) * (x - ld.c) / ((ld.d - ld.c) || 1)); }
      return 0;
    }
    function elemLoadLocal(el) {
      var p = new Array(12).fill(0), loads = (C.memLoads || {})[el.key] || [], xa = el.xa, Ls = el.Lsub, Lm = el.member.L;
      loads.forEach(function (ld) {
        if (ld.type === "point") {
          if ((ld.a >= xa - 1e-9) && (ld.a < xa + Ls - 1e-9 || xa + Ls >= Lm - 1e-9)) {
            var xx = clamp(ld.a - xa, 0, Ls), N = hShapes(xx, Ls);
            p[1] += N[0] * ld.P; p[5] += N[1] * ld.P; p[7] += N[2] * ld.P; p[11] += N[3] * ld.P;
          }
        } else {
          var s0 = xa, s1 = xa + Ls;
          if (ld.type === "trap") { s0 = Math.max(ld.c, xa); s1 = Math.min(ld.d, xa + Ls); }
          if (s1 <= s0) { return; }
          for (var k = 0; k < 4; k++) {
            var x = 0.5 * (s1 - s0) * GP3[k] + 0.5 * (s0 + s1), jac = 0.5 * (s1 - s0), q = inten(ld, x);
            if (q === 0) { continue; }
            var Nh = hShapes(x - xa, Ls), wj = q * jac * GW3[k];
            p[1] += Nh[0] * wj; p[5] += Nh[1] * wj; p[7] += Nh[2] * wj; p[11] += Nh[3] * wj;
          }
        }
      });
      return p;
    }
    function loadRes(mem, x0, x1) {
      var A = 0, Bb = 0, loads = (C.memLoads || {})[mem.id] || [];
      loads.forEach(function (ld) {
        if (ld.type === "point") { if (ld.a > x0 && ld.a <= x1) { A += ld.P; Bb += ld.P * ld.a; } return; }
        var s0, s1;
        if (ld.type === "udl") { s0 = Math.max(0, x0); s1 = Math.min(mem.L, x1); } else { s0 = Math.max(ld.c, x0); s1 = Math.min(ld.d, x1); }
        if (s1 <= s0) { return; }
        for (var k = 0; k < 4; k++) {
          var x = 0.5 * (s1 - s0) * GP3[k] + 0.5 * (s0 + s1), jac = 0.5 * (s1 - s0), q = inten(ld, x);
          A += q * jac * GW3[k]; Bb += q * x * jac * GW3[k];
        }
      });
      return { A: A, Bb: Bb };
    }

    /* 组装（6 自由度/节点：ux,uy,uz,rx,ry,rz） */
    var ndof = all.length * 6;
    var K = zeros(ndof, ndof);
    elems.forEach(function (el) {
      var p1 = all[el.a], p2 = all[el.b];
      var st = stiff3Of(el.member);
      var ax3 = beamAxes3D(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]);
      var Lsub = Math.hypot(p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]) || el.Lsub;
      var kl = beamKe3D(st.EA, st.EIz, st.EIy, st.GJ, Lsub);
      var R = rot12(ax3);
      var Rt = transpose(R);
      var kg = zeros(12, 12), i, j, a, b;
      for (i = 0; i < 12; i++) { for (j = 0; j < 12; j++) { var s = 0; for (a = 0; a < 12; a++) { for (b = 0; b < 12; b++) { s += Rt[i][a] * kl[a][b] * R[b][j]; } } kg[i][j] = s; } }
      el.R = R; el.kl = kl;
      el.pLocal = elemLoadLocal(el);
      el.p = matVec(Rt, el.pLocal);
      var map = [6 * el.a, 6 * el.a + 1, 6 * el.a + 2, 6 * el.a + 3, 6 * el.a + 4, 6 * el.a + 5,
                 6 * el.b, 6 * el.b + 1, 6 * el.b + 2, 6 * el.b + 3, 6 * el.b + 4, 6 * el.b + 5];
      for (i = 0; i < 12; i++) { for (j = 0; j < 12; j++) { K[map[i]][map[j]] += kg[i][j]; } }
    });
    areaElems.forEach(function (ae) {
      var pl = ae.plane || "xy", df = ae.dofs || [0, 1];
      var coords = ae.g.map(function (gi) { return locOf3(pl, all[gi][0], all[gi][1], all[gi][2]); });
      var ke = ae.kind === "q4" ? q4Stiffness(coords, ae.Dmat, ae.t).ke : cstStiffness(coords, ae.Dmat, ae.t).ke;
      var nn = ae.g.length, map2 = [], a2;
      for (a2 = 0; a2 < nn; a2++) { map2.push(6 * ae.g[a2] + df[0], 6 * ae.g[a2] + df[1]); }
      for (var i2 = 0; i2 < 2 * nn; i2++) { for (var j2 = 0; j2 < 2 * nn; j2++) { K[map2[i2]][map2[j2]] += ke[i2][j2]; } }
    });

    var F = new Array(ndof).fill(0);
    nodes.forEach(function (n) {
      var nl = C.nodeLoads[n.id], ni = idx[n.id];
      if (nl) {
        F[6 * ni] += nl.fx || 0; F[6 * ni + 1] += nl.fy || 0; F[6 * ni + 2] += nl.fz || 0;
        F[6 * ni + 3] += nl.mx || 0; F[6 * ni + 4] += nl.my || 0; F[6 * ni + 5] += nl.mz || 0;
      }
    });
    elems.forEach(function (el) {
      var map = [6 * el.a, 6 * el.a + 1, 6 * el.a + 2, 6 * el.a + 3, 6 * el.a + 4, 6 * el.a + 5,
                 6 * el.b, 6 * el.b + 1, 6 * el.b + 2, 6 * el.b + 3, 6 * el.b + 4, 6 * el.b + 5];
      for (var i = 0; i < 12; i++) { F[map[i]] += el.p[i]; }
    });
    areaElems.forEach(function (ae) {
      var ld = C.areaLoads[ae.aid];
      if (!ld || (!ld.qx && !ld.qy)) { return; }
      var pl = ae.plane || "xy", df = ae.dofs || [0, 1];
      // 面荷载为整体坐标 qx/qy：竖向墙只取面内分量（XZ 取 qx，YZ 取 qy；z 向输入暂无）
      var qu = pl === "yz" ? (ld.qy || 0) : (ld.qx || 0);
      var qv = pl === "xy" ? (ld.qy || 0) : 0;
      if (!qu && !qv) { return; }
      var qN = ae.g.map(function (gi) { return locOf3(pl, all[gi][0], all[gi][1], all[gi][2]); });
      if (ae.kind === "q4") {
        var coords = qN;
        var g = 1 / Math.sqrt(3);
        [[-g, -g, 1], [g, -g, 1], [g, g, 1], [-g, g, 1]].forEach(function (gp) {
          var xi = gp[0], et = gp[1], w = gp[2], N = q4N(xi, et);
          var J = [[0, 0], [0, 0]], a;
          for (a = 0; a < 4; a++) {
            var dNx = [-(1 - et) / 4, (1 - et) / 4, (1 + et) / 4, -(1 + et) / 4][a];
            var dNe = [-(1 - xi) / 4, -(1 + xi) / 4, (1 + xi) / 4, (1 - xi) / 4][a];
            J[0][0] += dNx * coords[a][0]; J[0][1] += dNx * coords[a][1];
            J[1][0] += dNe * coords[a][0]; J[1][1] += dNe * coords[a][1];
          }
          var det = Math.abs(J[0][0] * J[1][1] - J[0][1] * J[1][0]);
          for (a = 0; a < 4; a++) {
            F[6 * ae.g[a] + df[0]] += N[a] * qu * ae.t * det * w;
            F[6 * ae.g[a] + df[1]] += N[a] * qv * ae.t * det * w;
          }
        });
      } else {
        var p1 = qN[0], p2 = qN[1], p3 = qN[2];
        var A3 = Math.abs((p2[0] - p1[0]) * (p3[1] - p1[1]) - (p3[0] - p1[0]) * (p2[1] - p1[1])) / 2;
        for (var a3 = 0; a3 < 3; a3++) {
          F[6 * ae.g[a3] + df[0]] += qu * ae.t * A3 / 3;
          F[6 * ae.g[a3] + df[1]] += qv * ae.t * A3 / 3;
        }
      }
    });

    var fixed = {}, free = [];
    var BCKEY = ["ux", "uy", "uz", "rx", "ry", "rz"];
    nodes.forEach(function (n) {
      var ni = idx[n.id], bc = n.bc || {};
      for (var d = 0; d < 6; d++) { if (bc[BCKEY[d]]) { fixed[6 * ni + d] = true; } }
    });
    (function () {
      var md = 0, d;
      for (d = 0; d < ndof; d++) { md = Math.max(md, Math.abs(K[d][d])); }
      var eps = md * 1e-12;
      for (d = 0; d < ndof; d++) { if (!fixed[d] && Math.abs(K[d][d]) <= eps) { fixed[d] = true; } }
    })();
    for (var dd = 0; dd < ndof; dd++) { if (!fixed[dd]) { free.push(dd); } }
    var Kr = zeros(free.length, free.length);
    for (var r = 0; r < free.length; r++) { for (var cc = 0; cc < free.length; cc++) { Kr[r][cc] = K[free[r]][free[cc]]; } }
    var ur = solveLin(Kr, free.map(function (d) { return F[d]; }));
    var u = new Array(ndof).fill(0);
    free.forEach(function (d, k) { u[d] = ur[k]; });
    var ok = u.every(function (v) { return isFinite(v); });

    var maxM = 0, maxV = 0, maxN = 0, maxT = 0, byMember = {};
    elems.forEach(function (el) {
      var ue = [u[6 * el.a], u[6 * el.a + 1], u[6 * el.a + 2], u[6 * el.a + 3], u[6 * el.a + 4], u[6 * el.a + 5],
                u[6 * el.b], u[6 * el.b + 1], u[6 * el.b + 2], u[6 * el.b + 3], u[6 * el.b + 4], u[6 * el.b + 5]];
      var ul = matVec(el.R, ue);
      var fl = matVec(el.kl, ul);
      for (var i = 0; i < 12; i++) { fl[i] -= el.pLocal[i]; }
      var N1 = -fl[0], Vy1 = fl[1], Vz1 = fl[2], T1 = fl[3], My1 = fl[4], Mz1 = fl[5];
      el.axial = N1; el.torsion = T1;
      maxN = Math.max(maxN, Math.abs(N1)); maxT = Math.max(maxT, Math.abs(T1));
      var N = 12, pts = [];
      for (var k = 0; k <= N; k++) {
        var t = el.Lsub * k / N;
        var R2 = loadRes(el.member, el.xa, el.xa + t, C.memLoads);
        var J = (el.xa + t) * R2.A - R2.Bb;
        var Vy = Vy1 + R2.A;
        var Mz = -(Mz1 - Vy1 * t - J);
        var Vz = Vz1;
        // 注意右手系下 My=-EI·w''（与 Mz=+EI·v'' 反号），故此处为 + 号；
        // 若误写成 - 号，My 沿线会在每个子单元交界处跳变（锯齿状），Mz 则不受影响
        var My = -(My1 + Vz1 * t);
        // 有限元惯例：局部 1=杆轴, 2=局部y, 3=局部z；M2=My(绕2轴), M3=Mz(绕3轴)。
        // M/S 字段仅为 2D 兼容别名(M=M3, S=Vy=V2)；3D 显示必须用 M2/M3。
        pts.push({ t: t, S: Vy, M: Mz, Vy: Vy, Vz: Vz, My: My, Mz: Mz, T: T1, V2: Vy, V3: Vz, M2: My, M3: Mz });
        maxM = Math.max(maxM, Math.abs(Mz), Math.abs(My)); maxV = Math.max(maxV, Math.abs(Vy), Math.abs(Vz));
      }
      el.forces = pts;
      if (!byMember[el.key]) { byMember[el.key] = []; }
      byMember[el.key].push(el);
    });

    var maxVM = 0, maxSX = 0;
    areaElems.forEach(function (ae) {
      var nn = ae.g.length, ue = [], a4, df = ae.dofs || [0, 1], pl = ae.plane || "xy";
      for (a4 = 0; a4 < nn; a4++) { ue.push(u[6 * ae.g[a4] + df[0]], u[6 * ae.g[a4] + df[1]]); }
      var eps = [0, 0, 0], r3, c3;
      for (r3 = 0; r3 < 3; r3++) { var s3 = 0; for (c3 = 0; c3 < 2 * nn; c3++) { s3 += ae.B[r3][c3] * ue[c3]; } eps[r3] = s3; }
      var stt = [0, 0, 0];
      for (r3 = 0; r3 < 3; r3++) { stt[r3] = ae.Dmat[r3][0] * eps[0] + ae.Dmat[r3][1] * eps[1] + ae.Dmat[r3][2] * eps[2]; }
      var sx = stt[0], sy = stt[1], txy = stt[2];
      var avg = (sx + sy) / 2, R = Math.hypot((sx - sy) / 2, txy);
      var s1 = avg + R, s2 = avg - R;
      var vm = Math.sqrt(sx * sx - sx * sy + sy * sy + 3 * txy * txy);
      ae.s = { sx: sx, sy: sy, txy: txy, s1: s1, s2: s2, vm: vm, nx: sx * ae.t, ny: sy * ae.t, nxy: txy * ae.t };
      ae.cx = 0; ae.cy = 0;
      for (var a5 = 0; a5 < nn; a5++) { var _lp = locOf3(pl, all[ae.g[a5]][0], all[ae.g[a5]][1], all[ae.g[a5]][2]); ae.cx += _lp[0]; ae.cy += _lp[1]; }
      ae.cx /= nn; ae.cy /= nn;
      maxVM = Math.max(maxVM, Math.abs(vm)); maxSX = Math.max(maxSX, Math.abs(sx), Math.abs(sy));
      for (var a6 = 0; a6 < nn; a6++) {
        var acc = nodalAcc(ae.g[a6]);
        acc.sx += sx; acc.sy += sy; acc.txy += txy; acc.vm += vm; acc.s1 += s1; acc.nx += sx * ae.t; acc.n++;
      }
    });

    // 剖面切割/壳梁柱参考线仅处理 XY 平面单元（竖向墙在 2D 投影退化，保持原有行为）
    var xyAreas3 = (model.areas || []).filter(function (a) { var g = areaGrid[a.id]; return !g || !g.plane || g.plane === "xy"; });
    var xyElems3 = areaElems.filter(function (e) { return !e.plane || e.plane === "xy"; });
    var shellCuts = buildShellCuts(xyAreas3, areaGrid, xyElems3);
    var cutResults = integrateCuts(model.cuts || [], xyElems3, all.map(function (p) { return [p[0], p[1]]; }));

    var maxDisp = 0;
    for (var i2 = 0; i2 < all.length; i2++) { maxDisp = Math.max(maxDisp, Math.hypot(u[6 * i2], u[6 * i2 + 1], u[6 * i2 + 2])); }
    var hasZ = nodes.some(function (n) { return Math.abs(nzS(n)) > 1e-9; });
    var hl = nodes.map(function (n) { return hasZ ? nzS(n) : n.y; });
    var hmax = hl.length ? Math.max.apply(null, hl) : 0;
    var roof = 0, nr = 0;
    nodes.forEach(function (n) {
      var h = hasZ ? nzS(n) : n.y;
      if (Math.abs(h - hmax) < 1e-6) { roof += u[6 * idx[n.id]]; nr++; }
    });
    roof = nr ? roof / nr : 0;

    return {
      nodes: nodes, all: all.map(function (p) { return [p[0], p[1]]; }), all3: all, elems: elems, u: u, ok: ok, idx: idx, byMember: byMember,
      areaElems: areaElems, areaNodal: areaNodal, areaGrid: areaGrid, shellCuts: shellCuts,
      cutResults: cutResults, dof: 6, is3D: true, skippedArea3D: skippedArea3D,
      warn3D: skippedArea3D ? ("3D 内置求解跳过 " + skippedArea3D + " 个真斜面单元（仅共面 XY/XZ/YZ 者以膜刚度参与；精确解请用 OpenSees 真 3D 路径）") : null,
      res: { roof: roof * 1000, maxDisp: maxDisp * 1000, maxM: maxM, maxV: maxV, maxN: maxN, maxT: maxT, maxVM: maxVM, maxSX: maxSX, nNode: nodes.length, nMember: (model.members || []).length, nArea: (model.areas || []).length }
    };
  }

  function info() {
    return {
      name: "FrameLab built-in solver", version: VERSION,
      method: "direct stiffness, linear elastic",
      frame: "2D: Timoshenko beam-column (u,v,rz); 3D(auto): Euler beam 12x12 (ux,uy,uz,rx,ry,rz) + torsion + biaxial bending",
      plane: "Q4 (2x2 Gauss, center stress) / CST, plane stress or plane strain",
      shell: "membrane only (N=sigma*t), no plate bending",
      units: "m, kN, kN*m, kPa; roof/maxDisp in mm",
      limits: ["linear only", "no buckling/dynamics", "shell has no out-of-plane bending", "3D skewed shells skipped (use OpenSees)"]
    };
  }

  global.FrameSolver = {
    version: VERSION, solve: solve, info: info,
    utils: { sectionProps: sectionProps, sectionProps3D: sectionProps3D, planeD: planeD, solveLin: solveLin }
  };
})(typeof window !== "undefined" ? window : globalThis);