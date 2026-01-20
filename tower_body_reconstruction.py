"""
tower_body_reconstruction.py - 输电塔塔身三维坐标转换主入口

统一调度双视图 (A 线) 与单正面 (B 线) 处理流程，自动分类数据并输出合并结果。

使用方法:
    python tower_body_reconstruction.py                          # 使用默认数据目录
    python tower_body_reconstruction.py <数据目录路径>            # 指定数据目录
"""

import glob
import json
import math
import os
import re
import shutil
import sys
from typing import Dict, List, Optional, Tuple

import dual_view_processor as dual_main
import io_utils as rw
import single_view_processor as sv
import sv_class1_transform as t1

Point3D = Tuple[float, float, float]


# =========================
# 基础工具函数
# =========================

def _is_num(x) -> bool:
    try:
        float(x)
        return True
    except Exception:
        return False


def _get_xyz(node: dict) -> Optional[Point3D]:
    if all(k in node for k in ("X", "Y", "Z")) and all(_is_num(node[k]) for k in ("X", "Y", "Z")):
        return float(node["X"]), float(node["Y"]), float(node["Z"])
    if all(k in node for k in ("x", "y", "z")) and all(_is_num(node[k]) for k in ("x", "y", "z")):
        return float(node["x"]), float(node["y"]), float(node["z"])
    return None


def _dist3(a: Point3D, b: Point3D) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _prefix(s: str) -> str:
    return s[:-2] if isinstance(s, str) and len(s) >= 2 else s


def _promote_xyz_uppercase(obj):
    """统一将坐标键提升为大写 X/Y/Z，便于最终输出。"""

    if isinstance(obj, dict):
        for lower, upper in (("x", "X"), ("y", "Y"), ("z", "Z")):
            if lower in obj:
                obj[upper] = obj.pop(lower)
        for key in list(obj.keys()):
            _promote_xyz_uppercase(obj[key])
    elif isinstance(obj, list):
        for item in obj:
            _promote_xyz_uppercase(item)


# =========================
# 自动判别 (正侧面 vs 单正面)
# =========================

_BLOCK_PATTERN = re.compile(
    r"coordinates(?P<name>Front|Overhead|Bottom)_data\s*=\s*\{(?P<body>.*?)\}",
    re.IGNORECASE | re.DOTALL,
)
_ENTRY_PATTERN = re.compile(r"([0-9A-Za-z_]+)\s*:\s*\[")


def _count_block_entries(text: str) -> dict:
    counts = {"front": 0, "overhead": 0, "bottom": 0}
    for match in _BLOCK_PATTERN.finditer(text):
        name = match.group("name").lower()
        body = match.group("body")
        counts[name] = len(_ENTRY_PATTERN.findall(body))
    return counts


def detect_dataset_type(txt_path: str) -> str:
    try:
        with open(txt_path, "r", encoding="utf-8", errors="ignore") as f:
            text = f.read(1024 * 1024)
    except Exception:
        return "single"

    counts = _count_block_entries(text)
    front_cnt = counts.get("front", 0)
    over_cnt = counts.get("overhead", 0)
    bottom_cnt = counts.get("bottom", 0)

    if bottom_cnt > 0:
        return "other"

    if front_cnt == 0 and over_cnt == 0:
        return "other"

    if over_cnt == 0 and front_cnt > 0:
        return "single"

    if front_cnt == 0:
        return "other"

    bigger = max(front_cnt, over_cnt)
    smaller = max(1, min(front_cnt, over_cnt))
    ratio = bigger / smaller
    diff = abs(front_cnt - over_cnt)

    if ratio >= 3.0 or diff >= 8:
        return "single"

    return "dual"


def _empty_dir(path: str):
    if os.path.isdir(path):
        shutil.rmtree(path)
    os.makedirs(path, exist_ok=True)


def autodetect_and_stage(root_dir: str) -> Tuple[str, str]:
    dual_dir = os.path.join(root_dir, "_auto_dual")
    single_dir = os.path.join(root_dir, "_auto_single")
    other_dir = os.path.join(root_dir, "_auto_skip")
    _empty_dir(dual_dir)
    _empty_dir(single_dir)
    _empty_dir(other_dir)

    txts = sorted(glob.glob(os.path.join(root_dir, "*.txt")))
    for p in txts:
        mode = detect_dataset_type(p)
        basename = os.path.basename(p)
        if mode == "dual":
            shutil.copy2(p, os.path.join(dual_dir, basename))
            print(f"[auto] {basename} -> 双视图(A线)")
        elif mode == "single":
            shutil.copy2(p, os.path.join(single_dir, basename))
            print(f"[auto] {basename} -> 单视图(B线)")
        else:
            shutil.copy2(p, os.path.join(other_dir, basename))
            print(f"[auto] {basename} -> 跳过 (其它流程)")
    return dual_dir, single_dir


def _build_family_id_map(B0_id: str, A0_id: str) -> Dict[str, str]:
    """根据后缀族关系为重合点构建整套映射表。"""

    def suf(s):
        return s[-2:] if isinstance(s, str) and len(s) >= 2 else s

    def pref(s):
        return s[:-2] if isinstance(s, str) and len(s) >= 2 else ""

    sb, sa = suf(B0_id), suf(A0_id)
    pb, pa = pref(B0_id), pref(A0_id)

    top = {"10", "11", "12", "13"}
    bot = {"20", "21", "22", "23"}

    id_map: Dict[str, str] = {}

    if (sb in top) and (sa in bot):
        mapping = {"10": "20", "11": "21", "12": "22", "13": "23"}
        for k, v in mapping.items():
            id_map[pb + k] = pa + v
    else:
        id_map[B0_id] = A0_id

    return id_map


# =========================
# A 线：双视图 (塔身主体)
# =========================

def run_dual_view(dual_dir: str):
    """
    执行 A 线流程：处理双视图数据（通常是塔身）。
    调用 dual_view_processor.main() 完成从 2D 到 3D 的重建及内部拼接。
    """
    ganjian_A, jiedian_A, pinjie_A = dual_main.main(data_dir=dual_dir)
    # 确保 ID 为字符串格式，避免后续处理出错
    for m in ganjian_A:
        m["member_id"] = str(m.get("member_id"))
        m["node1_id"] = str(m.get("node1_id"))
        m["node2_id"] = str(m.get("node2_id"))
    for n in jiedian_A:
        n["node_id"] = str(n.get("node_id"))
    return ganjian_A, jiedian_A, pinjie_A


# =========================
# B 线：单正面 (塔头/地线支架)
# =========================

def _build_axis_reference_map(jiedian_B: List[dict]) -> Dict[str, Dict[str, str]]:
    axis_map: Dict[str, Dict[str, str]] = {}
    for nd in jiedian_B:
        if int(nd.get("node_type", 0)) != 12:
            continue
        node_id = str(nd.get("node_id", ""))
        if not node_id:
            continue
        axis_info: Dict[str, str] = {}
        for axis in ("X", "Y", "Z"):
            val = nd.get(axis)
            if val is None:
                continue
            axis_info[axis] = "ref" if isinstance(val, str) else "num"
        for axis in ("x", "y", "z"):
            val = nd.get(axis)
            if val is None:
                continue
            axis_info[axis.upper()] = "ref" if isinstance(val, str) else "num"
        if axis_info:
            axis_map[node_id] = axis_info
    return axis_map



def run_single_view_B(single_dir: str, file_glob: str = "*.txt"):
    """
    执行 B 线流程：处理单视图数据（通常是塔头）。
    1. 读取所有单视图文件。
    2. 调用 single_view_processor 进行三维化（基于分类假设）。
    3. 寻找最佳的“桥接点”（通常是塔头底部的最宽处），用于后续与塔身对接。
    """
    paths = sorted(glob.glob(os.path.join(single_dir, file_glob)))
    filelist: List[int | str] = []
    for p in paths:
        stem = os.path.splitext(os.path.basename(p))[0]
        try:
            filelist.append(int(stem))  # "07" -> 7
        except Exception:
            filelist.append(stem)

    filelist.sort()

    if not filelist:
        return [], [], None, None, None, {}

    # 调用单视图核心处理逻辑
    ganjian_B, jiedian_B = sv.single_view(filelist, single_dir)
    axis_map = _build_axis_reference_map(jiedian_B)

    # 寻找 B 线底部的最佳桥接特征点 (z_top, UL, UR, special_id, base_id)
    # 这里逻辑是遍历所有文件，找到顶部跨度最大的那个文件作为“接口”候选
    best = None  # (z_top, UL, UR, special_id, base_id)
    for p in paths:
        line_coord = rw.read_coords(p)
        if not isinstance(line_coord, dict):
            print(f"[警告] 单视图文件 {os.path.basename(p)} 解析失败，已跳过")
            continue
        fm_sv, special_id = t1.build_final_map_single_view(line_coord)
        UL, UR = t1.extract_top_span_points_single(line_coord)
        if not UL or not UR or not special_id:
            continue
        z_top = max(UL[2], UR[2])
        base_id = _prefix(str(special_id))
        cand = (z_top, UL, UR, special_id, base_id)
        if (best is None) or (cand[0] > best[0]):
            best = cand
    if not best:
        return ganjian_B, jiedian_B, None, None, None, axis_map
    _, UL, UR, special_id, _ = best
    return ganjian_B, jiedian_B, UL, UR, special_id, axis_map


# =========================
# 几何桥接与 ID 复用 (Merging)
# =========================

def apply_numeric_transform_B(jiedian_B: List[dict], s: float, t: Point3D, axis_map: Optional[Dict[str, Dict[str, str]]] = None):
    """
    对 B 线（塔头）的所有节点执行 缩放(s) + 平移(t) 变换。
    目的是将塔头“安装”到塔身顶部。
    """
    axis_map = axis_map or {}

    for nd in jiedian_B:
        nt = int(nd.get("node_type", 0))
        if nt == 11:
            if all(k in nd for k in ("X", "Y", "Z")):
                nd["X"] = round(float(nd["X"]) * s + t[0], 6)
                nd["Y"] = round(float(nd["Y"]) * s + t[1], 6)
                nd["Z"] = round(float(nd["Z"]) * s + t[2], 6)
            elif all(k in nd for k in ("x", "y", "z")):
                nd["x"] = round(float(nd["x"]) * s + t[0], 6)
                nd["y"] = round(float(nd["y"]) * s + t[1], 6)
                nd["z"] = round(float(nd["z"]) * s + t[2], 6)
        elif nt == 12:
            node_id = str(nd.get("node_id", ""))
            axis_info = axis_map.get(node_id, {})

            def _should_skip(axis_key: str) -> bool:
                tag = axis_info.get(axis_key.upper())
                return tag == "ref"

            def _transform_axis(axis_key: str, idx: int):
                if _should_skip(axis_key):
                    return
                val = nd.get(axis_key)
                if val is None:
                    return
                try:
                    num_val = float(val)
                except (TypeError, ValueError):
                    return
                nd[axis_key] = round(num_val * s + t[idx], 6)

            for axis_key, idx in (("X", 0), ("Y", 1), ("Z", 2)):
                _transform_axis(axis_key, idx)
            for axis_key, idx in (("x", 0), ("y", 1), ("z", 2)):
                _transform_axis(axis_key, idx)


def _collect_11_nodes(jiedian: List[dict]) -> Dict[str, Point3D]:
    out = {}
    for nd in jiedian:
        if int(nd.get("node_type", 0)) != 11:
            continue
        p = _get_xyz(nd)
        if p is None:
            continue
        out[str(nd.get("node_id"))] = p
    return out


def pick_bottom_support_point_for_bridge_A(jiedian_A: List[dict]) -> Tuple[str, Point3D]:
    """
    在 A 线（塔身）中寻找用于桥接的“顶部”基准点。
    策略：找 Z 最大（最高）、且 X/Y 最小（最靠中心/左前）的 11 类节点。
    """
    candidates = _collect_11_nodes(jiedian_A)
    if not candidates:
        for nd in jiedian_A:
            xyz = _get_xyz(nd)
            if xyz is None:
                continue
            candidates[str(nd.get("node_id"))] = xyz
    if not candidates:
        raise RuntimeError("A线：未找到可用于桥接的节点")
    best_id, best_xyz = max(
        candidates.items(),
        key=lambda item: (item[1][2], -abs(item[1][0]), -abs(item[1][1]), item[0]),
    )
    return best_id, best_xyz


def pick_top_member_point_for_bridge_B(jiedian_B: List[dict]) -> Tuple[str, Point3D]:
    """
    在 B 线（塔头）中寻找用于桥接的“底部”基准点。
    策略：找 Z 最小（最低）、且 X/Y 最大（最靠外）的 11 类节点。
    """
    candidates = _collect_11_nodes(jiedian_B)
    if not candidates:
        for nd in jiedian_B:
            xyz = _get_xyz(nd)
            if xyz is None:
                continue
            candidates[str(nd.get("node_id"))] = xyz
    if not candidates:
        raise RuntimeError("B线：未找到可用于桥接的节点")
    best_id, best_xyz = min(
        candidates.items(),
        key=lambda item: (item[1][2], abs(item[1][0]), abs(item[1][1]), item[0]),
    )
    return best_id, best_xyz


def _apply_id_map_family_to_B(id_map: Dict[str, str], ganjian_B: List[dict], jiedian_B: List[dict], B0_id: str):
    """
    将 B 线中与接口相关的节点 ID 替换为 A 线的对应 ID（节点复用）。
    同时移除 B 线中重复定义的接口节点。
    """
    if not id_map:
        return

    # 移除 B 线中那个重合的基准点（因为它将使用 A 线的点）
    jiedian_B[:] = [nd for nd in jiedian_B if str(nd.get("node_id")) != B0_id]

    # 更新 B 线节点中的引用 (12类节点)
    for nd in jiedian_B:
        if int(nd.get("node_type", 0)) == 12:
            for key in ("X", "Y"):
                v = nd.get(key)
                if not isinstance(v, str):
                    continue
                if v in id_map:
                    nd[key] = id_map[v]
                elif v.startswith("1") and v[1:] in id_map:
                    nd[key] = "1" + id_map[v[1:]]

    # 更新 B 线杆件的连接关系
    for m in ganjian_B:
        n1, n2 = str(m.get("node1_id", "")), str(m.get("node2_id", ""))
        if n1 in id_map:
            m["node1_id"] = id_map[n1]
        if n2 in id_map:
            m["node2_id"] = id_map[n2]


def reuse_ids_at_final(
    ganjian_A: List[dict],
    jiedian_A: List[dict],
    ganjian_B: List[dict],
    jiedian_B: List[dict],
    eps: float = 1e-6,
):
    def collect_A_body_base_11(jiedian_A, ganjian_A):
        # 找到被 12 类节点引用的 11（塔头接口）
        interface_11 = set()

        # 只排除：被 12 类节点引用的 11
        for nd in jiedian_A:
            if int(nd.get("node_type", 0)) != 12:
                continue
            for k in ("X", "Y"):
                v = nd.get(k)
                if isinstance(v, str):
                    interface_11.add(v)

        out = {}
        for nd in jiedian_A:
            if int(nd.get("node_type", 0)) != 11:
                continue
            nid = str(nd.get("node_id"))
            if nid in interface_11:
                continue  # 排除塔头接口层
            p = _get_xyz(nd)
            if p:
                out[nid] = p
        return out

    """
    最终合并步骤：检测 A 线顶部和 B 线底部的重合节点，建立 ID 映射并替换。
    """
    def collect_11(jiedian):
        out = {}
        for nd in jiedian:
            if int(nd.get("node_type", 0)) != 11:
                continue
            p = _get_xyz(nd)
            if p is not None:
                out[str(nd.get("node_id"))] = p
        return out

##    A11 = collect_11(jiedian_A)
    # ==== 新增：找 A 线塔身基部 Z ====
    A11 = collect_A_body_base_11(jiedian_A, ganjian_A)
    if not A11:
        print("[reuse_ids] 未检测到塔身基部节点，跳过基部 ID 复用")
        return

    min_Z_A = min(p[2] for p in A11.values())

    B11 = collect_11(jiedian_B)


    if not B11:
        return

    min_Z_B = min(p[2] for p in B11.values())





    # 寻找距离最近的一对点作为锚点
    # best = (float("inf"), "", "")
    # for bid, bp in B11.items():
    #     for aid, ap in A11.items():
    #         d = _dist3(bp, ap)
    #         if d <= eps and d < best[0]:
    #             best = (d, aid, bid)
    # if not best[1] or not best[2]:
    #     return
    best = (float("inf"), "", "")
    for bid, bp in B11.items():
        for aid, ap in A11.items():

            # ====== 关键新增规则 ======
            # 如果 B 点在基部附近，只允许匹配 A 线的基部节点

            # if abs(bp[2] - min(p[2] for p in B11.values())) < 1e-3:
            #     if abs(ap[2] - min_Z_A) > 1e-3:
            #         continue
            # =========================
            if abs(bp[2] - min_Z_B) < 1e-3:
                if abs(ap[2] - min_Z_A) > 1e-3:
                    continue

            d = _dist3(bp, ap)
            if d <= eps and d < best[0]:
                best = (d, aid, bid)

    A0, B0 = best[1], best[2]
    if not A0 or not B0:
        print("[reuse_ids] 未找到满足基部规则的匹配点，跳过 ID 复用")
        return

    # 构建整个“家族”的映射（例如 10->20, 11->21 等对称点）
    fam_map = _build_family_id_map(B0, A0)
    _apply_id_map_family_to_B(fam_map, ganjian_B, jiedian_B, B0_id=B0)


# =========================
# 合并输出
# =========================

def merge_and_print(ganjian_A, jiedian_A, pinjie_A, ganjian_B, jiedian_B):
    """
    将 A 线和 B 线的数据合并，去重，并打印最终 JSON。
    """
    reuse_ids_at_final(ganjian_A, jiedian_A, ganjian_B, jiedian_B, eps=1e-6)

    node_ids: set[str] = set()
    jiedian: List[dict] = []
    # 先加入 A 线节点
    for nd in jiedian_A:
        jiedian.append(nd)
        node_ids.add(str(nd.get("node_id")))
    # 再加入 B 线节点（去重）
    for nd in jiedian_B:
        nid = str(nd.get("node_id"))
        if nid in node_ids:
            continue
        jiedian.append(nd)
        node_ids.add(nid)

    def _key(m):
        n1, n2 = str(m.get("node1_id")), str(m.get("node2_id"))
        a, b = (n1, n2) if n1 <= n2 else (n2, n1)
        return (str(m.get("member_id")), a, b, int(m.get("symmetry_type", 0)))

    seen: set[tuple] = set()
    ganjian: List[dict] = []
    # 合并杆件并去重
    for m in list(ganjian_A) + list(ganjian_B):
        k = _key(m)
        if k in seen:
            continue
        seen.add(k)
        ganjian.append(m)

    # 处理 member_id，移除 "_" 及其后缀
    for m in ganjian:
        mid = str(m.get("member_id", ""))
        if "_" in mid:
            m["member_id"] = mid.split("_")[0]

    pinjie = list(pinjie_A)

    _promote_xyz_uppercase(ganjian)
    _promote_xyz_uppercase(jiedian)
    _promote_xyz_uppercase(pinjie)

    print("\n" + "=" * 60)
    print("## 最终合并输出（已对齐 & 单点复用）：")
    print("=" * 60)
    print("\n--- ganjian ---")
    print(json.dumps(ganjian, ensure_ascii=False, indent=2))
    print("\n--- jiedian ---")
    print(json.dumps(jiedian, ensure_ascii=False, indent=2))
    print("\n--- pinjie ---")
    print(json.dumps(pinjie, ensure_ascii=False, indent=2))
    return ganjian, jiedian, pinjie


# =========================
# 主调度流程
# =========================

def run(dual_dir: str, single_dir: str):
    """
    总控流程：
    1. 运行 A 线（双视图） -> 得到塔身模型
    2. 运行 B 线（单视图） -> 得到塔头模型
    3. 计算变换矩阵，将 B 线模型对齐到 A 线顶部
    4. 合并输出
    """

    # 检查是否有双视图文件
    dual_files = glob.glob(os.path.join(dual_dir, "*.txt"))
    has_dual = len(dual_files) > 0

    # 检查是否有单视图文件
    single_files = glob.glob(os.path.join(single_dir, "*.txt"))
    has_single = len(single_files) > 0

    print(f"检测到数据类型：双视图={has_dual}, 单视图={has_single}")

    # 场景1：只有双视图
    if has_dual and not has_single:
        print("场景：仅双视图(A线) - 输出塔身模型")
        ganjian_A, jiedian_A, pinjie_A = run_dual_view(dual_dir)
        return merge_and_print(ganjian_A, jiedian_A, pinjie_A, [], [])

    # 场景2：只有单视图
    if not has_dual and has_single:
        print("场景：仅单视图(B线) - 输出塔头模型")
        ganjian_B, jiedian_B, _, _, _, axis_map = run_single_view_B(single_dir)
        if ganjian_B or jiedian_B:
            return merge_and_print([], [], [], ganjian_B, jiedian_B)
        else:
            return [], [], []

    # 场景3：双视图+单视图（完整流程）
    if has_dual and has_single:
        print("场景：双视图+单视图 - 桥接合并处理")
    ganjian_A, jiedian_A, pinjie_A = run_dual_view(dual_dir)
    # ======================================================
    # 【后处理】最低层节点 snapping 到最近基部节点（仅 A 线）
    # 目的：修复 1905 / 1906 这类“基部端点漂移”的问题
    # ======================================================

    def snap_lowest_nodes_to_base(
        ganjian: List[dict],
        jiedian: List[dict],
        z_eps: float = 1e-6,
        xy_tol: float = 50.0,   # ⚠️ 可根据图纸尺度调整，建议 30~80
    ):
        # 1️⃣ 收集所有 node_id -> (x,y,z)
        node_xyz = {}
        for nd in jiedian:
            p = _get_xyz(nd)
            if p is not None:
                node_xyz[str(nd["node_id"])] = p

        if not node_xyz:
            return

        # 2️⃣ 找最低 Z
        z_min = min(p[2] for p in node_xyz.values())

        # 3️⃣ 基部节点（node_type == 11 且 Z≈z_min）
        base_nodes = {
            nid: p
            for nid, p in node_xyz.items()
            if abs(p[2] - z_min) < z_eps
            and any(
                str(nd.get("node_id")) == nid and int(nd.get("node_type", 0)) == 11
                for nd in jiedian
            )
        }

        if len(base_nodes) < 2:
            return  # 没有可靠基部，直接跳过

        # 4️⃣ 找“可疑节点”：Z≈z_min，但不是基部 11 节点
        snap_map = {}  # bad_node_id -> base_node_id

        for nid, (x, y, z) in node_xyz.items():
            target_nodes = set()

            for m in ganjian:
                if str(m.get("member_id")) in {"1905", "1906"}:
                    target_nodes.add(str(m.get("node1_id")))
                    target_nodes.add(str(m.get("node2_id")))

            for nid in target_nodes:
                x, y, z = node_xyz[nid]
                best = None
                for bid, (bx, by, bz) in base_nodes.items():
                    dxy = math.hypot(x - bx, y - by)
                    if dxy < xy_tol and (best is None or dxy < best[0]):
                        best = (dxy, bid)
                if best:
                    snap_map[nid] = best[1]

            if nid in base_nodes:
                continue

            # 找最近的基部节点（XY 平面）
            best = None
            for bid, (bx, by, bz) in base_nodes.items():
                dxy = math.hypot(x - bx, y - by)
                if dxy <= xy_tol and (best is None or dxy < best[0]):
                    best = (dxy, bid)

            if best is not None:
                snap_map[nid] = best[1]

        if not snap_map:
            return

        print(f"[snap-base] 基部吸附节点映射: {snap_map}")

        # 5️⃣ 重写杆件端点
        for m in ganjian:
            n1 = str(m.get("node1_id"))
            n2 = str(m.get("node2_id"))
            if n1 in snap_map:
                m["node1_id"] = snap_map[n1]
            if n2 in snap_map:
                m["node2_id"] = snap_map[n2]

        # 6️⃣ 删除被吸附的冗余节点
        jiedian[:] = [
            nd for nd in jiedian
            if str(nd.get("node_id")) not in snap_map
        ]

    # 👉 执行 snapping（只对 A 线）
    snap_lowest_nodes_to_base(ganjian_A, jiedian_A)

    ganjian_B, jiedian_B, _, _, _, axis_map = run_single_view_B(single_dir)
    if not ganjian_B and not jiedian_B:
        return merge_and_print(ganjian_A, jiedian_A, pinjie_A, [], [])

    # 计算对齐参数
    _, A_point = pick_bottom_support_point_for_bridge_A(jiedian_A)
    _, B_point = pick_top_member_point_for_bridge_B(jiedian_B)

    # 假设：B 线底部宽度应匹配 A 线顶部宽度
    # 这里用 X 坐标估算缩放比例
    denom = B_point[0] * 2.0
    if abs(denom) <= 1e-9:
        raise RuntimeError("B线：桥接参考点的 X*2 为 0，无法计算缩放因子")
    scale = (A_point[0] * 2.0) / denom

    # 计算平移向量
    B_point_scaled = (B_point[0] * scale, B_point[1] * scale, B_point[2] * scale)
    translation = (
        A_point[0] - B_point_scaled[0],
        A_point[1] - B_point_scaled[1],
        A_point[2] - B_point_scaled[2],
    )

    # 应用变换到 B 线
    apply_numeric_transform_B(jiedian_B, scale, translation, axis_map=axis_map)

    return merge_and_print(ganjian_A, jiedian_A, pinjie_A, ganjian_B, jiedian_B)


def run_autodetect(root_dir: str):
    dual_dir, single_dir = autodetect_and_stage(root_dir)
    return run(dual_dir, single_dir)


def build_tower_body(tashen_dir):
    ganjian, jiedian, pinjie = run_autodetect(tashen_dir)
    return ganjian, jiedian, pinjie






# =========================
# CLI 入口
# =========================

def main():
    """命令行入口，可选传入数据目录。"""

    default_data_directory = r"D:\SanWei\TaShen\test"

    if len(sys.argv) > 1:
        data_directory = sys.argv[1]
        print(f"使用命令行指定的数据目录: {data_directory}")
    else:
        data_directory = default_data_directory
        print(f"使用默认数据目录: {data_directory}")

    if not os.path.exists(data_directory):
        print(f"错误: 数据目录不存在: {data_directory}")
        print("请检查路径是否正确，或通过命令行参数指定正确的目录")
        print("用法: python main.py <数据目录路径>")
        return

    print("=" * 60)
    print("开始执行三维重建...")
    print("=" * 60)

    try:
        ganjian, jiedian, pinjie = run_autodetect(data_directory)

        print("\n" + "=" * 60)
        print("处理完成！")
        print(f"生成杆件数量: {len(ganjian)}")
        print(f"生成节点数量: {len(jiedian)}")
        print(f"拼接信息数量: {len(pinjie)}")
        print("=" * 60)
        
        # 保存结果到文件供可视化使用
        if ganjian or jiedian:
            output_data = {
                "ganjian": ganjian,
                "jiedian": jiedian,
                "pinjie": pinjie
            }
            output_file = "tower_output.json"
            with open(output_file, 'w', encoding='utf-8') as f:
                json.dump(output_data, f, ensure_ascii=False, indent=2)
            print(f"\n✅ 结果已保存到 {output_file} (共 {len(ganjian)} 条杆件, {len(jiedian)} 个节点)")


        return ganjian, jiedian, pinjie

    except Exception as e:
        print(f"\n错误: 处理过程中出现异常: {e}")
        import traceback

        traceback.print_exc()
        return None, None, None

if __name__ == "__main__":
    main()