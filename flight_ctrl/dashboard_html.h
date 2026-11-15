#pragma once

// 板载调试面板页面（纯内联，无外部依赖——设备热点下没有互联网）
// 压到一个 C 字符串里，用 R"HTML(...)HTML" 原始字面量避免转义地狱
static const char DASHBOARD_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>FlightCtrl 调试面板</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: -apple-system, "PingFang SC", sans-serif; background: #0b0e14;
         color: #e8eaf0; min-height: 100vh; display: flex; flex-direction: column;
         align-items: center; padding: 16px; }
  h1 { font-size: 18px; margin-bottom: 4px; letter-spacing: 1px; }
  .sub { color: #7d8696; font-size: 12px; margin-bottom: 16px; }
  .status { display: flex; align-items: center; gap: 8px; font-size: 13px; margin-bottom: 16px; }
  .dot { width: 10px; height: 10px; border-radius: 50%; background: #e5484d; transition: background .3s; }
  .dot.on { background: #46d68c; box-shadow: 0 0 8px #46d68c88; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; width: 100%; max-width: 640px; }
  .card { background: #141925; border: 1px solid #232b3d; border-radius: 12px; padding: 14px; }
  .card label { font-size: 12px; color: #8b94a7; display: block; margin-bottom: 6px; }
  .val { font-size: 34px; font-variant-numeric: tabular-nums; font-weight: 600; }
  .val small { font-size: 16px; color: #8b94a7; font-weight: 400; }
  .bar { position: relative; height: 8px; background: #1c2333; border-radius: 4px; margin-top: 10px; }
  .bar::after { content: ""; position: absolute; left: 50%; top: -3px; bottom: -3px; width: 1px; background: #3a445c; }
  .bar i { position: absolute; top: 0; bottom: 0; left: 50%; width: 0; background: #5aa2ff; border-radius: 4px; }
  .card.pitch .bar i { background: #ffb454; }
  canvas { border-radius: 12px; margin-top: 12px; width: 100%; max-width: 640px; background: #141925;
           border: 1px solid #232b3d; }
  .btn { margin-top: 14px; padding: 12px 32px; border: 0; border-radius: 10px; background: #2b6cff;
         color: #fff; font-size: 15px; }
  .btn:active { transform: scale(.97); }
  .meta { color: #7d8696; font-size: 12px; margin-top: 12px; font-variant-numeric: tabular-nums; }
</style>
</head>
<body>
  <h1>FlightCtrl 调试面板</h1>
  <div class="sub">ESP32-C3 + GY-61 倾角数据（40Hz 轮询）</div>
  <div class="status"><span class="dot" id="dot"></span><span id="stxt">连接中…</span></div>
  <div class="grid">
    <div class="card"><label>横滚 ROLL（°）</label>
      <div class="val"><span id="roll">--</span><small>°</small></div>
      <div class="bar"><i id="rollbar"></i></div></div>
    <div class="card pitch"><label>俯仰 PITCH（°）</label>
      <div class="val"><span id="pitch">--</span><small>°</small></div>
      <div class="bar"><i id="pitchbar"></i></div></div>
  </div>
  <canvas id="horizon" width="640" height="280"></canvas>
  <button class="btn" id="center">以当前姿势重新居中</button>
  <div class="meta" id="meta">-- msg/s</div>
<script>
(function () {
  var timer = null, count = 0, rateT = performance.now(), retry = 0;
  var el = {
    dot: document.getElementById('dot'), stxt: document.getElementById('stxt'),
    roll: document.getElementById('roll'), pitch: document.getElementById('pitch'),
    rollbar: document.getElementById('rollbar'), pitchbar: document.getElementById('pitchbar'),
    meta: document.getElementById('meta'),
    horizon: document.getElementById('horizon'),
  };
  function setConn(on) {
    el.dot.classList.toggle('on', on);
    el.stxt.textContent = on ? '已连接' : '连接中…';
  }
  function poll() {
    fetch('/tilt').then(function (r) { return r.json(); }).then(function (d) {
      setConn(true); retry = 0;
      el.roll.textContent = d.roll.toFixed(1);
      el.pitch.textContent = d.pitch.toFixed(1);
      el.rollbar.style.width = Math.abs(d.cr) * 50 + '%';
      el.rollbar.style.left = d.cr < 0 ? (50 - Math.abs(d.cr) * 50) + '%' : '50%';
      el.pitchbar.style.width = Math.abs(d.cp) * 50 + '%';
      el.pitchbar.style.left = d.cp < 0 ? (50 - Math.abs(d.cp) * 50) + '%' : '50%';
      drawHorizon(d.roll, d.pitch);
      count++;
      var now = performance.now();
      if (now - rateT > 1000) {
        el.meta.textContent = (count * 1000 / (now - rateT)).toFixed(0) + ' 次/秒';
        count = 0; rateT = now;
      }
    }).catch(function () {
      setConn(false);
      if (++retry > 3) { clearInterval(timer); setTimeout(start, 2000); }
    });
  }
  function start() {
    setConn(false);
    timer = setInterval(poll, 25);   // 40Hz 轮询
    poll();
  }
  // 简易地平仪：天地分界线随横滚旋转、随俯仰上下移动
  var ctx = el.horizon.getContext('2d');
  function drawHorizon(roll, pitch) {
    var w = el.horizon.width, h = el.horizon.height, cx = w / 2, cy = h / 2;
    ctx.save();
    ctx.clearRect(0, 0, w, h);
    ctx.beginPath(); ctx.rect(0, 0, w, h); ctx.clip();
    ctx.translate(cx, cy);
    ctx.rotate(-roll * Math.PI / 180);
    var off = pitch * 2.2;
    ctx.fillStyle = '#2a4d8f'; ctx.fillRect(-w, -h * 2 + off, w * 3, h * 2);      // 天
    ctx.fillStyle = '#7a5230'; ctx.fillRect(-w, off, w * 3, h * 2);               // 地
    ctx.strokeStyle = '#e8eaf0'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(-w, off); ctx.lineTo(w, off); ctx.stroke();       // 地平线
    ctx.restore();
    // 固定的机身参考线
    ctx.strokeStyle = '#ffd75e'; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(cx - 70, cy); ctx.lineTo(cx - 20, cy);
    ctx.lineTo(cx - 8, cy + 10); ctx.lineTo(cx, cy); ctx.lineTo(cx + 8, cy + 10);
    ctx.lineTo(cx + 20, cy); ctx.lineTo(cx + 70, cy); ctx.stroke();
  }
  document.getElementById('center').onclick = function () {
    fetch('/center');
  };
  start();
})();
</script>
</body>
</html>)HTML";
