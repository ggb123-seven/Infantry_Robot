# -*- coding: utf-8 -*-
"""pitch 重力前馈离线拟合工具

输入 EmberProbe 导出的 CSV（原始列名可带 " [f32]"/" [u8]" 类型后缀），
按角度分段提取静止稳态电流，线性最小二乘拟合

    i(theta) = a*cos(theta) + b*sin(theta) + c

再换算成固件字段

    A  = hypot(a, b)          -> gimbal.config.pitch_gravity_current_a
    t0 = atan2(b, a) (度)     -> gimbal.config.pitch_gravity_angle_deg
    i0 = c                    -> gimbal.config.pitch_gravity_bias_a

本脚本只读 CSV、只写本地报告，不连接探针、不写目标板

用法：
    py tools/pitch_gravity_fit.py <csv> [--outdir docs/validation]

依赖：numpy
"""
import argparse
import csv
import datetime
import json
import math
import os
import re
import sys

import numpy as np

# 与 gimbal.h 的轴状态、模式枚举保持一致
PITCH_STATE_RUNNING = 1
GIMBAL_MODE_ANGLE = 2

TYPE_SUFFIX = re.compile(r"\s*\[(?:u8|u16|u32|i8|i16|i32|f32|f64)\]\s*$")


def normalize(name):
    """去掉 EmberProbe 导出的类型后缀和首尾空白"""
    return TYPE_SUFFIX.sub("", name.strip())


def find_col(columns, exact, keywords):
    """先按精确名匹配，再退回关键字全包含匹配"""
    for name in exact:
        if name in columns:
            return name
    for name in columns:
        low = name.lower()
        if all(k in low for k in keywords):
            return name
    return None


def find_suffix(columns, suffix, must_contain):
    """按列名后缀匹配，避免 "i" 这类单字符关键字误命中其他字段"""
    for name in columns:
        if name.endswith(suffix) and all(k in name for k in must_contain):
            return name
    return None


def to_float(text):
    """空值和非数字统一按 NaN 处理"""
    try:
        value = float(text)
    except (TypeError, ValueError):
        return float("nan")
    return value


def load_csv(path):
    """读取 CSV 并返回归一化列名、按列存放的浮点数组和原始字符串行"""
    with open(path, "r", encoding="utf-8-sig", newline="") as fp:
        reader = csv.reader(fp)
        raw_header = next(reader)
        names = [normalize(h) for h in raw_header]
        rows = [r for r in reader if r]
    data = {}
    for idx, name in enumerate(names):
        values = np.array([to_float(r[idx]) if idx < len(r) else float("nan") for r in rows], dtype=float)
        # 同名重复列只保留第一份
        if name not in data:
            data[name] = values
    return names, data, rows


def parse_time_column(raw_rows, index):
    """把时间列解析成秒，支持 EmberProbe 的 ISO 8601 与纯数字秒"""
    seconds = np.full(len(raw_rows), float("nan"))
    formats = ("%Y-%m-%dT%H:%M:%S.%fZ", "%Y-%m-%dT%H:%M:%SZ",
               "%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S")
    for i, row in enumerate(raw_rows):
        if index >= len(row):
            continue
        text = row[index].strip()
        for fmt in formats:
            try:
                seconds[i] = datetime.datetime.strptime(text, fmt).timestamp()
                break
            except ValueError:
                continue
        else:
            try:
                seconds[i] = float(text)
            except ValueError:
                continue
    return seconds


def resolve_columns(columns):
    """解析后续处理需要的列，缺失的记为 None"""
    return {
        "angle": find_col(columns, ["gimbal.pitch.angle_deg"], ["pitch", "angle_deg"]),
        "current": find_col(columns, ["gimbal.pitch.output_current_a"], ["pitch", "output_current"]),
        "target_speed": find_col(columns, ["gimbal.pitch.target_speed_rad_s"], ["pitch", "target_speed"]),
        "speed": find_col(columns, ["gimbal.pitch.speed_rad_s"], ["pitch", "speed_rad_s"]),
        "target_angle": find_col(columns, ["gimbal.pitch.target_angle_deg"], ["pitch", "target_angle"]),
        "max_current": find_col(columns, ["gimbal.config.pitch.max_current_a"], ["pitch", "max_current_a"]),
        "state": find_col(columns, ["gimbal.pitch.state"], ["pitch", "state"]),
        "mode": find_col(columns, ["gimbal.mode"], ["gimbal", "mode"]),
        "config_valid": find_col(columns, ["gimbal.config_valid"], ["config_valid"]),
        "integral": find_suffix(columns, ".speed_pid.i", ("pitch",)),
        "i_limit": find_suffix(columns, ".speed_pid.i_limit", ("pitch",)),
    }


def steady_mask(data, cols, args):
    """按运行状态、角速度与目标速度筛出静止稳态样本"""
    n = len(data[cols["angle"]])
    mask = np.ones(n, dtype=bool)
    if cols["state"]:
        mask &= data[cols["state"]] == PITCH_STATE_RUNNING
    if cols["mode"]:
        mask &= data[cols["mode"]] == GIMBAL_MODE_ANGLE
    if cols["config_valid"]:
        mask &= data[cols["config_valid"]] == 1
    if cols["target_speed"]:
        mask &= np.abs(data[cols["target_speed"]]) <= args.speed_tol
    if cols["speed"]:
        mask &= np.abs(data[cols["speed"]]) <= args.rate_tol
    mask &= np.isfinite(data[cols["angle"]]) & np.isfinite(data[cols["current"]])
    return mask


def compute_direction(angles, window):
    """用平滑差分判断每个采样点的运动方向，静止段继承逼近方向

    转向点上行末段与下行首段目标角相同，只有运动方向能把它们分开
    """
    filled = np.array(angles, dtype=float)
    last = float("nan")
    for i in range(len(filled)):
        if math.isfinite(filled[i]):
            last = filled[i]
        else:
            filled[i] = last
    dirs = np.zeros(len(filled), dtype=float)
    for i in range(len(filled)):
        lo = max(0, i - window)
        hi = min(len(filled) - 1, i + window)
        delta = filled[hi] - filled[lo]
        if abs(delta) > 0.3:
            dirs[i] = 1.0 if delta > 0.0 else -1.0
    return dirs


def assign_runs(direction, persist):
    """只有新方向连续保持 persist 个采样点才切换运行段，避免静止噪声反复切段"""
    runs = np.zeros(len(direction), dtype=int)
    run = 0
    sign = 0.0
    pending = 0.0
    count = 0
    for i in range(len(direction)):
        value = direction[i]
        if value != 0.0 and value != sign:
            if value == pending:
                count += 1
            else:
                pending = value
                count = 1
            if count >= persist:
                sign = value
                run += 1
                count = 0
        else:
            pending = 0.0
            count = 0
        runs[i] = run
    return runs


def split_segments(data, cols, mask, args):
    """按目标角与逼近方向切分静止段，目标角抖动用滞回吸收"""
    key_col = cols["target_angle"] if cols["target_angle"] else cols["angle"]
    direction = compute_direction(data[cols["angle"]], args.dir_window)
    run_ids = assign_runs(direction, args.dir_persist)
    segments = []
    current = None
    for i in range(len(direction)):
        if not mask[i]:
            continue
        key = data[key_col][i]
        if not math.isfinite(key):
            continue
        continued = (current is not None and int(run_ids[i]) == current["run"]
                     and abs(key - current["target"]) <= args.seg_hysteresis)
        if not continued:
            current = {"run": int(run_ids[i]), "target": float(key),
                       "direction": int(np.sign(direction[i])), "rows": []}
            segments.append(current)
        current["rows"].append(i)
    return segments


def summarize_segments(data, cols, segments, elapsed, settle_samples, args):
    """每段丢弃进入稳定前的过渡样本后取均值，逼近方向用于上下行迟滞抵消"""
    points = []
    usable_time = elapsed is not None and math.isfinite(float(elapsed[0]))
    for seg in segments:
        rows = seg["rows"]
        # 驻留时间短于设定过渡时长时按段长的一半截断，避免整段被丢空
        if usable_time:
            start = elapsed[rows[0]]
            duration = elapsed[rows[-1]] - start
            settle = min(args.settle_s, 0.5 * duration) if math.isfinite(duration) else args.settle_s
            steady = [r for r in rows if elapsed[r] - start >= settle]
        else:
            steady = rows[:int(len(rows) * 0.5)] if len(rows) * 0.5 >= settle_samples else rows[settle_samples:]
        if len(steady) < args.min_samples:
            continue
        currents = data[cols["current"]][steady]
        angle = float(np.mean(data[cols["angle"]][steady]))
        current = float(np.mean(currents))
        current_std = float(np.std(currents))
        # 拟合量是电流，窗口内电流仍在爬升时该段不可用，即使角度和速度看起来已稳定
        half = len(currents) // 2
        drift = abs(float(np.mean(currents[half:])) - float(np.mean(currents[:half]))) if half else float("inf")
        if current_std > args.current_std_max or drift > args.current_drift_max:
            continue
        target = seg["target"]
        direction = seg["direction"]
        points.append({
            "target_deg": float(target),
            "angle_deg": angle,
            "current_a": current,
            "current_std_a": current_std,
            "samples": len(steady),
            "direction": direction,
            "integral": float(np.mean(data[cols["integral"]][steady])) if cols["integral"] else None,
        })
    return points


def merge_by_target(points):
    """同一目标角的上行与下行取平均，抵消静摩擦迟滞"""
    grouped = {}
    for p in points:
        key = round(p["target_deg"] * 2.0) / 2.0
        grouped.setdefault(key, []).append(p)
    merged = []
    for key in sorted(grouped):
        items = grouped[key]
        angles = [p["angle_deg"] for p in items]
        currents = [p["current_a"] for p in items]
        merged.append({
            "target_deg": key,
            "angle_deg": float(np.mean(angles)),
            "current_a": float(np.mean(currents)),
            "spread_a": float(max(currents) - min(currents)) if len(items) > 1 else 0.0,
            "current_std_a": float(np.mean([p["current_std_a"] for p in items])),
            "samples": int(sum(p["samples"] for p in items)),
            "integral": float(np.mean([p["integral"] for p in items if p["integral"] is not None]))
            if any(p["integral"] is not None for p in items) else None,
        })
    return merged


def fit(angles_deg, currents, fix_angle_deg=None):
    """线性最小二乘解 i = a*cos(theta) + b*sin(theta) + c

    固定弧度 theta0 时退化为两参数 A*cos(theta - theta0) + i0，用于行程过短、
    三参数不可辨识但已知相位机械位置的场合
    """
    theta = np.deg2rad(np.asarray(angles_deg, dtype=float))
    y = np.asarray(currents, dtype=float)
    if fix_angle_deg is None:
        design = np.column_stack([np.cos(theta), np.sin(theta), np.ones_like(theta)])
        coef = np.linalg.lstsq(design, y, rcond=None)[0]
        a, b, c = float(coef[0]), float(coef[1]), float(coef[2])
        A = math.hypot(a, b)
        theta0 = math.degrees(math.atan2(b, a))
    else:
        phase = np.deg2rad(fix_angle_deg)
        design = np.column_stack([np.cos(theta - phase), np.ones_like(theta)])
        coef = np.linalg.lstsq(design, y, rcond=None)[0]
        A = float(coef[0])
        c = float(coef[1])
        a = A * math.cos(phase)
        b = A * math.sin(phase)
        theta0 = float(fix_angle_deg)
    pred = design @ coef
    residual = y - pred
    ss_res = float(np.sum(residual ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0.0 else float("nan")
    rms = float(np.sqrt(ss_res / len(y)))
    # 归一化条件数，用于提示行程过短时 cos 与常数项强共线
    scale = np.sqrt(np.sum(design ** 2, axis=0))
    cond = float(np.linalg.cond((design / scale).T @ (design / scale)))
    # 参数标准误差由残差方差与设计矩阵协方差给出，用于标注短行程下的不可辨识程度
    dof = max(len(y) - design.shape[1], 1)
    se = {"A": float("nan"), "theta0_deg": float("nan"), "i0": float("nan")}
    try:
        cov = (ss_res / dof) * np.linalg.inv(design.T @ design)
        if fix_angle_deg is None:
            var_a, var_b, var_c = float(cov[0, 0]), float(cov[1, 1]), float(cov[2, 2])
            cov_ab = float(cov[0, 1])
            se["i0"] = math.sqrt(max(var_c, 0.0))
            se["A"] = math.sqrt(max((a * a * var_a + b * b * var_b + 2.0 * a * b * cov_ab), 0.0)) / A if A > 0.0 else float("nan")
            var_t = (b * b * var_a + a * a * var_b - 2.0 * a * b * cov_ab) / (A ** 4) if A > 0.0 else float("nan")
            se["theta0_deg"] = math.degrees(math.sqrt(max(var_t, 0.0))) if math.isfinite(var_t) else float("nan")
        else:
            se["i0"] = math.sqrt(max(float(cov[1, 1]), 0.0))
            se["A"] = math.sqrt(max(float(cov[0, 0]), 0.0))
            se["theta0_deg"] = 0.0
    except np.linalg.LinAlgError:
        pass
    return {
        "a": a, "b": b, "c": c, "A": A, "theta0_deg": theta0, "i0": c,
        "se_A": se["A"], "se_theta0_deg": se["theta0_deg"], "se_i0": se["i0"],
        "dof": int(dof),
        "r2": r2, "rms_a": rms,
        "max_abs_residual_a": float(np.max(np.abs(residual))) if len(residual) else float("nan"),
        "cond": cond,
        "fixed_angle_deg": fix_angle_deg,
        "residual": residual.tolist(),
        "predicted": pred.tolist(),
    }


def build_warnings(points, result, max_current, i_limit, args):
    """收集会让拟合系数不可用的现场条件"""
    warnings = []
    span = max(p["angle_deg"] for p in points) - min(p["angle_deg"] for p in points) if points else 0.0
    if span < args.min_span:
        warnings.append("角度跨度仅 %.1f 度，小于 %.1f 度，参数不可辨识，只能得到局部等效曲线" % (span, args.min_span))
    if result["cond"] > 100.0:
        warnings.append("归一化条件数 %.1f 偏大：行程短时 cos 与常数项强共线" % result["cond"])
    if math.isfinite(result["se_theta0_deg"]) and result["se_theta0_deg"] > 5.0:
        warnings.append("theta0 标准误差 %.1f 度偏大，A 与 i0 会互相补偿；区间内曲线仍可用，"
                        "但不要拿 A 或 theta0 的绝对值做物理推断" % result["se_theta0_deg"])
    if math.isfinite(result["se_A"]) and result["A"] > 0.0 and result["se_A"] > 0.2 * result["A"]:
        warnings.append("A 相对标准误差 %.0f%%，建议加长行程，或先用 --fix-angle-deg 固定相位"
                        % (100.0 * result["se_A"] / result["A"]))
    if max_current > 0.0:
        saturated = [p for p in points if abs(p["current_a"]) >= 0.98 * max_current]
        if saturated:
            warnings.append("%d 个点电流接近 max_current_a=%.3f A，可能被限幅，拟合会偏小" % (len(saturated), max_current))
    if i_limit > 0.0:
        clamped = [p for p in points if p["integral"] is not None and abs(p["integral"]) >= 0.98 * i_limit]
        if clamped:
            warnings.append("%d 个点速度环积分贴近 i_limit=%.3f，该点数据右截断，不可作为重力依据"
                            % (len(clamped), i_limit))
    if len(points) >= 2:
        ordered = sorted(points, key=lambda p: p["angle_deg"])
        edges = [p for p in (ordered[0], ordered[-1]) if p["spread_a"] <= 1e-6]
        if edges:
            warnings.append("%d 个端点只有单方向数据，静摩擦迟滞在端点无法抵消，端点电流可能整体偏移"
                            % len(edges))
    if result["A"] + abs(result["i0"]) > max_current > 0.0:
        warnings.append("A + |i0| = %.3f 超过 max_current_a=%.3f，写入后配置校验会拒绝，两轴进故障"
                        % (result["A"] + abs(result["i0"]), max_current))
    if result["rms_a"] > 0.0 and result["A"] < 3.0 * result["rms_a"]:
        warnings.append("拟合幅值 A 与残差同量级，说明本组数据里重力项不显著，可先只写常值项")
    return warnings


def render_svg(points, result, path, title):
    """无第三方依赖的散点与拟合曲线图"""
    width, height = 860, 460
    left, right, top, bottom = 80, 40, 50, 60
    angles = [p["angle_deg"] for p in points]
    currents = [p["current_a"] for p in points]
    x_min, x_max = min(angles) - 1.0, max(angles) + 1.0
    y_min, y_max = min(currents) - 0.05, max(currents) + 0.05
    if y_max - y_min < 0.1:
        mid = (y_max + y_min) / 2.0
        y_min, y_max = mid - 0.05, mid + 0.05

    def sx(x):
        return left + (x - x_min) / (x_max - x_min) * (width - left - right)

    def sy(y):
        return height - bottom - (y - y_min) / (y_max - y_min) * (height - top - bottom)

    out = ['<?xml version="1.0" encoding="UTF-8"?>']
    out.append('<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" font-family="Microsoft YaHei, sans-serif">'
               % (width, height))
    out.append('<rect width="100%%" height="100%%" fill="#ffffff"/>')
    out.append('<text x="%d" y="28" font-size="16" fill="#222">%s</text>' % (left, title))
    out.append('<rect x="%d" y="%d" width="%d" height="%d" fill="none" stroke="#999"/>'
               % (left, top, width - left - right, height - top - bottom))
    for k in range(6):
        xg = x_min + (x_max - x_min) * k / 5.0
        out.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="#eee"/>'
                   % (sx(xg), top, sx(xg), height - bottom))
        out.append('<text x="%.1f" y="%d" font-size="11" fill="#666" text-anchor="middle">%.0f</text>'
                   % (sx(xg), height - bottom + 16, xg))
    for k in range(6):
        yg = y_min + (y_max - y_min) * k / 5.0
        out.append('<line x1="%d" y1="%.1f" x2="%d" y2="%.1f" stroke="#eee"/>'
                   % (left, sy(yg), width - right, sy(yg)))
        out.append('<text x="%d" y="%.1f" font-size="11" fill="#666" text-anchor="end">%.3f</text>'
                   % (left - 8, sy(yg) + 4, yg))
    # 拟合曲线
    curve = []
    steps = 200
    for k in range(steps + 1):
        xg = x_min + (x_max - x_min) * k / steps
        yg = result["a"] * math.cos(math.radians(xg)) + result["b"] * math.sin(math.radians(xg)) + result["c"]
        curve.append("%.1f,%.1f" % (sx(xg), sy(yg)))
    out.append('<polyline points="%s" fill="none" stroke="#1a73e8" stroke-width="2"/>' % " ".join(curve))
    # 实测点，迟滞区间用竖线表示
    for p in points:
        if p["spread_a"] > 0.0:
            out.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#e8710a" stroke-width="2"/>'
                       % (sx(p["angle_deg"]), sy(p["current_a"] - p["spread_a"] / 2.0),
                          sx(p["angle_deg"]), sy(p["current_a"] + p["spread_a"] / 2.0)))
        out.append('<circle cx="%.1f" cy="%.1f" r="3.5" fill="#d93025"/>'
                   % (sx(p["angle_deg"]), sy(p["current_a"])))
    out.append('<text x="%d" y="%d" font-size="12" fill="#444" text-anchor="middle">大地系俯仰角 / 度</text>'
               % ((left + width - right) // 2, height - 18))
    out.append('<text x="18" y="%d" font-size="12" fill="#444" transform="rotate(-90 18 %d)" '
               'text-anchor="middle">维持电流 / A</text>' % ((top + height - bottom) // 2, (top + height - bottom) // 2))
    out.append('<text x="%d" y="%d" font-size="12" fill="#1a73e8">A=%.4f  theta0=%.1f°  i0=%.4f  R2=%.4f  '
               'RMS=%.4f A</text>' % (left, top - 12, result["A"], result["theta0_deg"], result["i0"],
                                      result["r2"], result["rms_a"]))
    out.append("</svg>")
    with open(path, "w", encoding="utf-8", newline="\n") as fp:
        fp.write("\n".join(out))


def main(argv):
    parser = argparse.ArgumentParser(description="pitch 重力前馈拟合：由静止稳态电流反推 A/theta0/i0")
    parser.add_argument("csv", help="EmberProbe 导出的 CSV 路径")
    parser.add_argument("--outdir", default="docs/validation", help="报告输出目录，默认 docs/validation")
    parser.add_argument("--prefix", default="", help="报告文件名前缀，默认取 CSV 文件名")
    parser.add_argument("--speed-tol", type=float, default=0.02, help="目标角速度稳态阈值 rad/s，默认 0.02")
    parser.add_argument("--rate-tol", type=float, default=0.05, help="实测角速度稳态阈值 rad/s，默认 0.05")
    parser.add_argument("--settle-s", type=float, default=1.0, help="每段进入稳定后丢弃的过渡时长 秒，默认 1.0")
    parser.add_argument("--current-std-max", type=float, default=0.02, help="窗口内电流标准差上限 A，默认 0.02")
    parser.add_argument("--current-drift-max", type=float, default=0.02, help="窗口前半与后半电流均值差上限 A，默认 0.02")
    parser.add_argument("--min-samples", type=int, default=20, help="每段最少稳态样本数，默认 20")
    parser.add_argument("--seg-hysteresis", type=float, default=1.0, help="目标角分段滞回宽度 度，默认 1.0")
    parser.add_argument("--dir-window", type=int, default=10, help="运动方向平滑差分半窗，单位采样点，默认 10")
    parser.add_argument("--dir-persist", type=int, default=20, help="方向切换需连续保持的采样点数，默认 20")
    parser.add_argument("--min-span", type=float, default=10.0, help="最小角度跨度 度，默认 10")
    parser.add_argument("--sign", type=float, default=1.0, help="电流符号修正，取 -1 可翻转，默认 1")
    parser.add_argument("--fix-angle-deg", type=float, default=None,
                        help="固定相位 theta0 退化为两参数拟合，行程过短时用于拿到可解释幅值")
    parser.add_argument("--max-current", type=float, default=0.0, help="覆盖 max_current_a，0 表示按 CSV 或 1.0")
    parser.add_argument("--no-svg", action="store_true", help="不输出 SVG 图")
    parser.add_argument("--allow-sparse", action="store_true", help="角度点少于 4 个时仍输出结果，用于诊断")
    args = parser.parse_args(argv)

    columns, data, raw_rows = load_csv(args.csv)
    cols = resolve_columns(columns)
    missing = [k for k in ("angle", "current") if not cols[k]]
    if missing:
        print("缺少必要列: %s" % ", ".join(missing), file=sys.stderr)
        print("可用列: %s" % ", ".join(columns), file=sys.stderr)
        return 2

    max_current = args.max_current
    if max_current <= 0.0 and cols["max_current"]:
        values = data[cols["max_current"]]
        values = values[np.isfinite(values)]
        max_current = float(np.median(values)) if len(values) else 1.0
    if max_current <= 0.0:
        max_current = 1.0
    i_limit = 0.0
    if cols["i_limit"]:
        values = data[cols["i_limit"]]
        values = values[np.isfinite(values)]
        i_limit = float(np.median(values)) if len(values) else 0.0

    # 时间轴只用于按真实秒数丢弃过渡过程，采样率变化时不用改阈值
    elapsed = None
    for idx, name in enumerate(columns):
        if name.lower() in ("time", "timestamp"):
            parsed = parse_time_column(raw_rows, idx)
            if math.isfinite(float(parsed[0])):
                elapsed = parsed - parsed[0]
            break
    settle_samples = 60
    if elapsed is not None and len(elapsed) > 1:
        diffs = np.diff(elapsed)
        diffs = diffs[np.isfinite(diffs) & (diffs > 0.0)]
        if len(diffs):
            settle_samples = max(3, int(round(args.settle_s / float(np.median(diffs)))))

    mask = steady_mask(data, cols, args)
    if int(np.sum(mask)) == 0:
        print("没有满足稳态条件的样本，检查阈值或采集质量", file=sys.stderr)
        return 3
    segments = split_segments(data, cols, mask, args)
    points = summarize_segments(data, cols, segments, elapsed, settle_samples, args)
    if not points:
        print("没有可用分段，每段样本数不足 --min-samples", file=sys.stderr)
        return 3
    merged = merge_by_target(points)
    if len(merged) < 4 and not args.allow_sparse:
        print("合并后只有 %d 个角度点，不足以拟合三参数，检查驻留时间或 --settle-s；"
              "已知相位时可加 --fix-angle-deg 并用 --allow-sparse" % len(merged), file=sys.stderr)
        return 3
    for p in merged:
        # 符号修正只用于电流方向约定不一致的场合，积分列同步翻转保持可对照
        p["current_a"] *= args.sign
        if p["integral"] is not None:
            p["integral"] *= args.sign
    result = fit([p["angle_deg"] for p in merged], [p["current_a"] for p in merged], args.fix_angle_deg)
    warnings = build_warnings(merged, result, max_current, i_limit, args)

    os.makedirs(args.outdir, exist_ok=True)
    stem = args.prefix or os.path.splitext(os.path.basename(args.csv))[0]
    report_path = os.path.join(args.outdir, stem + "_pitch_gravity_fit.json")
    svg_path = os.path.join(args.outdir, stem + "_pitch_gravity_fit.svg")

    print("输入: %s" % args.csv)
    print("列绑定: " + ", ".join("%s=%s" % (k, v) for k, v in cols.items() if v))
    print("稳态样本 %d / 总行 %d，分段 %d，合并后角度点 %d" % (int(np.sum(mask)), len(data[cols["angle"]]),
                                                        len(segments), len(merged)))
    print("")
    print("%-10s %-12s %-12s %-10s %-8s %s" % ("目标角", "实测角", "电流/A", "迟滞/A", "样本", "积分"))
    for p in merged:
        print("%-10.2f %-12.3f %-12.4f %-10.4f %-8d %s" % (
            p["target_deg"], p["angle_deg"], p["current_a"], p["spread_a"], p["samples"],
            "-" if p["integral"] is None else "%.4f" % p["integral"]))
    print("")
    if result["fixed_angle_deg"] is None:
        print("拟合: i = (%+.5f)*cos(theta) + (%+.5f)*sin(theta) + (%+.5f)"
              % (result["a"], result["b"], result["c"]))
    else:
        print("拟合: i = (%+.5f)*cos(theta %+.2f deg) + (%+.5f)（theta0 由 --fix-angle-deg 固定）"
              % (result["A"], -result["fixed_angle_deg"], result["i0"]))
    print("      A = %.5f +- %.5f A, theta0 = %.2f +- %.2f deg, i0 = %.5f +- %.5f A"
          % (result["A"], result["se_A"], result["theta0_deg"], result["se_theta0_deg"],
             result["i0"], result["se_i0"]))
    print("      区间内最大预测偏差 = %.5f A, R2 = %.4f, RMS = %.5f A, 归一化条件数 = %.1f"
          % (result["max_abs_residual_a"], result["r2"], result["rms_a"], result["cond"]))
    print("")
    print("写入建议（需 A + |i0| <= %.3f）:" % max_current)
    print("  gimbal.config.pitch_gravity_current_a = %.4f" % result["A"])
    print("  gimbal.config.pitch_gravity_angle_deg = %.2f" % result["theta0_deg"])
    print("  gimbal.config.pitch_gravity_bias_a    = %.4f" % result["i0"])
    print("  gimbal.config.pitch_gravity_enabled   = 1")
    if warnings:
        print("")
        print("警告:")
        for w in warnings:
            print("  - " + w)

    payload = {
        "source_csv": os.path.abspath(args.csv),
        "columns": cols,
        "steady_samples": int(np.sum(mask)),
        "segments": len(segments),
        "points": merged,
        "fit": {k: v for k, v in result.items() if k not in ("residual", "predicted")},
        "residual_a": result["residual"],
        "predicted_a": result["predicted"],
        "max_current_a": max_current,
        "speed_pid_i_limit": i_limit,
        "sign": args.sign,
        "warnings": warnings,
    }
    with open(report_path, "w", encoding="utf-8", newline="\n") as fp:
        json.dump(payload, fp, ensure_ascii=False, indent=2)
    print("")
    print("报告: %s" % report_path)
    if not args.no_svg:
        render_svg(merged, result, svg_path,
                   "pitch 重力前馈拟合  A=%.4f A  theta0=%.1f deg" % (result["A"], result["theta0_deg"]))
        print("配图: %s" % svg_path)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
