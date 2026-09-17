#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FrameLab xara/OpenSees 本地桥接服务（仅标准库，无第三方依赖）。

启动：
    python xara_server.py [--port 8007] [--dir .]

作用：
  1. 提供 HTTP 桥：接收浏览器（xara.js）POST 来的 xara 脚本，调用本机
     Python + xara 执行 OpenSees 运算，把结果 JSON 回传给页面做后处理；
  2. 顺带静态托管本目录（用 http://127.0.0.1:8007/index.html 打开页面，
     可避免 file:// 页面的 fetch 限制）；
  3. 示例文件中转（GET /api/file?path=example/xxx）：file:// 方式打开页面时
     fetch 不可用，前端自动经本接口读取 example/ 下的示例文件。

前置：pip install xara   （https://github.com/peer-open-source/xara）

接口：
  GET  /api/health    -> {"ok": true, "xara": "<版本或null>", ...}
  GET  /api/file?path=example/foo.json -> {"ok": true, "path": ..., "text": "..."}
       （仅允许 example/ 目录下的文本文件，防目录穿越，单文件上限 2MB）
  POST /api/analyze   {"script": "<xara.py文本>"} ->
       {"ok": true, "results": {...}, "stdout": "...", "stderr": "..."}
       {"ok": false, "error": "...", "stdout": "...", "stderr": "..."}
"""
import argparse
import functools
import http.server
import json
import os
import socketserver
import subprocess
import sys
import tempfile
from urllib.parse import urlsplit, parse_qs

BEGIN = "FRAMELAB_JSON_BEGIN"
END = "FRAMELAB_JSON_END"
MAX_BODY = 20 * 1024 * 1024


def run_script(script_text, timeout=300):
    """执行 xara 脚本， 추출 FRAMELAB_JSON 结果块。"""
    with tempfile.TemporaryDirectory(prefix="framelab_xara_") as td:
        fp = os.path.join(td, "model.py")
        with open(fp, "w", encoding="utf-8") as f:
            f.write(script_text)
        try:
            p = subprocess.run(
                [sys.executable, fp],
                cwd=td, capture_output=True, text=True, timeout=timeout,
            )
        except subprocess.TimeoutExpired:
            return {"ok": False, "error": "xara 执行超时（>%ss）" % timeout,
                    "stdout": "", "stderr": ""}
        out, err = p.stdout or "", p.stderr or ""
        i, j = out.find(BEGIN), out.find(END)
        if i < 0 or j < 0 or j < i:
            return {"ok": False, "error": "脚本未输出结果块（returncode=%s）" % p.returncode,
                    "stdout": out[-4000:], "stderr": err[-4000:]}
        try:
            results = json.loads(out[i + len(BEGIN):j].strip())
        except Exception as ex:
            return {"ok": False, "error": "结果 JSON 解析失败：%s" % ex,
                    "stdout": out[-4000:], "stderr": err[-4000:]}
        if not isinstance(results, dict) or not results.get("ok"):
            return {"ok": False,
                    "error": "xara 求解失败：%s" % ((results or {}).get("error") or "analyze != 0"),
                    "results": results, "stdout": out[-4000:], "stderr": err[-4000:]}
        return {"ok": True, "results": results,
                "stdout": out[-4000:], "stderr": err[-4000:]}


def xara_version():
    try:
        import importlib.metadata as md
        return md.version("xara")
    except Exception:
        return None


class Handler(http.server.SimpleHTTPRequestHandler):
    server_version = "FrameLabXaraBridge/1.0"

    def _cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def _json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self._cors()
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors()
        self.end_headers()

    def do_GET(self):
        parts = urlsplit(self.path)
        if parts.path == "/api/health":
            xv = xara_version()
            return self._json({"ok": True, "xara": xv,
                               "xara_ready": xv is not None,
                               "python": sys.version.split()[0]})
        if parts.path == "/api/file":
            return self._serve_example_file(parse_qs(parts.query).get("path", [""])[0])
        return super().do_GET()

    def _serve_example_file(self, rel):
        """示例文件中转：仅允许 <dir>/example/ 下的文本文件（防目录穿越）。"""
        base = os.path.realpath(os.path.join(os.getcwd(), "example"))
        if not isinstance(rel, str) or not rel:
            return self._json({"ok": False, "error": "缺少 path 参数"}, 400)
        # 仅接受 example/ 开头的相对路径，拒绝绝对路径与 ..
        norm = rel.replace("\\", "/").lstrip("/")
        if norm != rel.replace("\\", "/") or norm.startswith("/") or ".." in norm.split("/"):
            return self._json({"ok": False, "error": "非法路径"}, 403)
        if not norm.startswith("example/"):
            return self._json({"ok": False, "error": "仅允许读取 example/ 目录下的文件"}, 403)
        if len(norm) > 200:
            return self._json({"ok": False, "error": "路径过长"}, 400)
        fp = os.path.realpath(os.path.join(os.getcwd(), norm))
        if not (fp == base or fp.startswith(base + os.sep)) or not os.path.isfile(fp):
            return self._json({"ok": False, "error": "文件不存在"}, 404)
        if os.path.getsize(fp) > 2 * 1024 * 1024:
            return self._json({"ok": False, "error": "文件过大（>2MB）"}, 413)
        try:
            with open(fp, "r", encoding="utf-8") as f:
                text = f.read()
        except (OSError, UnicodeDecodeError) as ex:
            return self._json({"ok": False, "error": "读取失败：%s" % ex}, 500)
        return self._json({"ok": True, "path": norm, "text": text})

    def do_POST(self):
        if self.path.split("?")[0] != "/api/analyze":
            return self._json({"ok": False, "error": "unknown endpoint"}, 404)
        try:
            n = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            n = 0
        if n <= 0 or n > MAX_BODY:
            return self._json({"ok": False, "error": "请求体长度非法"}, 400)
        try:
            data = json.loads(self.rfile.read(n).decode("utf-8"))
        except Exception as ex:
            return self._json({"ok": False, "error": "JSON 解析失败：%s" % ex}, 400)
        script = data.get("script") or ""
        timeout = data.get("timeout") or 300
        if not isinstance(script, str) or "xara" not in script:
            return self._json({"ok": False, "error": "缺少 script 字段"}, 400)
        try:
            timeout = max(5, min(1800, int(timeout)))
        except (TypeError, ValueError):
            timeout = 300
        self._json(run_script(script, timeout=timeout))


def main(argv=None):
    ap = argparse.ArgumentParser(description="FrameLab xara bridge server")
    ap.add_argument("--port", type=int, default=8007)
    ap.add_argument("--dir", default=os.path.dirname(os.path.abspath(__file__)))
    args = ap.parse_args(argv)
    os.chdir(args.dir)
    socketserver.TCPServer.allow_reuse_address = True
    handler = functools.partial(Handler, directory=args.dir)
    with socketserver.ThreadingTCPServer(("127.0.0.1", args.port), handler) as httpd:
        print("FrameLab xara bridge on http://127.0.0.1:%d/" % args.port)
        print("  page  : http://127.0.0.1:%d/index.html" % args.port)
        print("  health: http://127.0.0.1:%d/api/health" % args.port)
        print("  xara  : %s" % (xara_version() or "NOT INSTALLED (pip install xara)"))
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()