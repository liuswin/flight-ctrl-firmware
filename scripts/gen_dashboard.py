#!/usr/bin/env python3
"""从 Arduino 版面板 HTML 重新生成 IDF 版 dashboard.h。

源头文件: flight_ctrl/dashboard_html.h（R"HTML(...)HTML" 原始字面量）
目标文件: idf/main/dashboard.h（C 字符串数组）

修改面板样式时：改 Arduino 版 HTML，然后在 firmware/ 目录运行:
    python3 scripts/gen_dashboard.py
两个固件的面板内容即保持同步。
"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent          # firmware/
SRC = ROOT / "flight_ctrl" / "dashboard_html.h"
DST = ROOT / "idf" / "main" / "dashboard.h"

html = re.search(r'R"HTML\((.*)\)HTML"', SRC.read_text(), re.S).group(1)

lines = []
for line in html.split("\n"):
    escaped = line.replace("\\", "\\\\").replace('"', '\\"')
    lines.append(f'  "{escaped}\\n"')

out = (
    "// 自动生成：板载调试面板页面（勿手改）\n"
    "// 修改请编辑 ../flight_ctrl/dashboard_html.h 后运行 scripts/gen_dashboard.py\n"
    "#pragma once\n\n"
    "static const char DASHBOARD_HTML[] =\n" + "\n".join(lines) + ";\n"
)
DST.write_text(out)
print(f"OK: {DST}（HTML {len(html)} 字节）")
