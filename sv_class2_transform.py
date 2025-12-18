import math
import io_utils as rw

# ================= 杆件提取相关子函数 =================

def extract_lines01(lines_dict):
    target_members = {}
    for member_id, coordinates in lines_dict.items():
        member_str = str(member_id)
        if "_" in member_str:
            int_part = member_str.split("_", 1)[0]
        else:
            int_part = member_str
        end_suffix = int_part[-2:] if len(int_part) >= 2 else ""
        if end_suffix in ["01", "02"]:
            target_members[member_id] = coordinates
    return target_members


def parameter_extract(lines_dict):
    lines = list(lines_dict.values())
    kb_list = []
    for line in lines:
        (x1, y1), (x2, y2) = line
        if abs(x2 - x1) < 1e-9:
            raise ValueError("杆件端点 X 相同，无法计算斜率。")
        k = (y2 - y1) / (x2 - x1)
        b = y1 - k * x1
        kb_list.append([k, b])

    if kb_list[0][0] < 0:
        left_line = 0
        right_line = 1
    else:
        left_line = 1
        right_line = 0

    (x1_l, y1_l), (x2_l, y2_l) = lines[left_line]
    if y1_l <= y2_l:
        x_left_ymin, x_left_ymax = x1_l, x2_l
    else:
        x_left_ymin, x_left_ymax = x2_l, x1_l

    (x1_r, y1_r), (x2_r, y2_r) = lines[right_line]
    if y1_r <= y2_r:
        x_right_ymin, x_right_ymax = x1_r, x2_r
    else:
        x_right_ymin, x_right_ymax = x2_r, x1_r

    avg_min = min((x_left_ymax + x_right_ymin), (x_left_ymin + x_right_ymax)) / 2.0
    avg_max = max((x_left_ymax + x_right_ymin), (x_left_ymin + x_right_ymax)) / 2.0
    return avg_min, avg_max


def extract_lines0201(line_coord, avg_min, avg_max):
    result = {}
    for key, coords in line_coord.items():
        (x1, y1), (x2, y2) = coords
        mid_x = (x1 + x2) / 2
        if (avg_min - 1) <= mid_x <= (avg_max + 1):
            result[key] = coords
    return result


def extract_lines0202(line_coord, lines01, lines0201, line_groups):
    if not lines01 or not lines0201:
        return {}, {}

    horizontal_lines = {}
    for group in line_groups:
        if len(group) == 1:
            candidate_id = group[0]
            if candidate_id in lines0201:
                horizontal_lines[candidate_id] = lines0201[candidate_id]

    classified_ids = set(lines01.keys()).union(lines0201.keys())
    unclassified_lines = {
        line_id: coords for line_id, coords in line_coord.items() if line_id not in classified_ids
    }

    def point_to_line_distance(point, seg_start, seg_end):
        x0, y0 = point
        x1, y1 = seg_start
        x2, y2 = seg_end
        if abs(x2 - x1) < 1e-9:
            return abs(x0 - x1)
        k = (y2 - y1) / (x2 - x1)
        b = y1 - k * x1
        return abs(k * x0 - y0 + b) / math.sqrt(k ** 2 + 1)

    result = {}
    for line_id, coords in unclassified_lines.items():
        if len(coords) != 2:
            continue
        end1, end2 = coords
        end1_near_lines01 = any(point_to_line_distance(end1, seg[0], seg[1]) < 20 for seg in lines01.values())
        end1_near_horizontal = any(point_to_line_distance(end1, seg[0], seg[1]) < 20 for seg in horizontal_lines.values())
        end2_near_lines01 = any(point_to_line_distance(end2, seg[0], seg[1]) < 20 for seg in lines01.values())
        end2_near_horizontal = any(point_to_line_distance(end2, seg[0], seg[1]) < 20 for seg in horizontal_lines.values())

        disqualify_prefix = str(line_id).startswith("1914")
        meets_distance = (
            (end1_near_lines01 and end2_near_horizontal)
            or (end2_near_lines01 and end1_near_horizontal)
        )
        if not disqualify_prefix and meets_distance:
            result[line_id] = coords

    return result, horizontal_lines


# ================= 杆件分组相关子函数 =================

def calculate_y_midpoint(points):
    (x1, y1), (x2, y2) = points
    return (y1 + y2) / 2


def find_y_proximity_bars(bars, threshold=100):
    y_midpoints = {bar_id: calculate_y_midpoint(points) for bar_id, points in bars.items()}
    processed = set()
    result = []

    for bar1_id in bars:
        if bar1_id in processed:
            continue
        bar1_ym = y_midpoints[bar1_id]
        group = [bar1_id]
        processed.add(bar1_id)

        for bar2_id in bars:
            if bar1_id == bar2_id or bar2_id in processed:
                continue
            bar2_ym = y_midpoints[bar2_id]
            if abs(bar1_ym - bar2_ym) <= threshold:
                group.append(bar2_id)
                processed.add(bar2_id)

        result.append(group)
    return result


# ================= 杆件修正相关子函数 =================

def correct_lines(line_dict, line_groups):
    res = {}
    group_mapping = {}
    for group in line_groups:
        for member in group:
            group_mapping[member] = group

    for key, coords in line_dict.items():
        (x1, y1), (x2, y2) = coords
        if key not in group_mapping:
            y_avg = round((y1 + y2) / 2)
            res[key] = [(x1, y_avg), (x2, y_avg)]
            continue

        group = group_mapping[key]
        ymin_values = []
        ymax_values = []

        def append_pair(y_a, y_b):
            if y_a <= y_b:
                ymin_values.append(y_a)
                ymax_values.append(y_b)
            else:
                ymin_values.append(y_b)
                ymax_values.append(y_a)

        append_pair(y1, y2)
        for member in group:
            if member == key or member not in line_dict:
                continue
            (mx1, my1), (mx2, my2) = line_dict[member]
            append_pair(my1, my2)

        y1_avg = round(sum(ymin_values) / len(ymin_values))
        y2_avg = round(sum(ymax_values) / len(ymax_values))

        if (y2 - y1) * (x2 - x1) > 0:
            if x1 < x2:
                res[key] = [(x1, y1_avg), (x2, y2_avg)]
            else:
                res[key] = [(x1, y2_avg), (x2, y1_avg)]
        else:
            if x1 < x2:
                res[key] = [(x1, y2_avg), (x2, y1_avg)]
            else:
                res[key] = [(x1, y1_avg), (x2, y2_avg)]
    return res


# ================= 杆件转换相关子函数 =================

def parameter_extract01(lines_dict):
    lines = list(lines_dict.values())
    if len(lines) < 2:
        raise ValueError("计算二类参数失败：需要至少两根一类杆件。")

    kb_list = []
    for line in lines:
        (x1, y1), (x2, y2) = line
        if abs(x2 - x1) < 1e-9:
            raise ValueError("杆件端点 X 相同，无法计算斜率。")
        k = (y2 - y1) / (x2 - x1)
        b = y1 - k * x1
        kb_list.append([k, b])

    if kb_list[0][0] < 0:
        left_line = 0
        right_line = 1
    else:
        left_line = 1
        right_line = 0

    (x1_l, y1_l), (x2_l, y2_l) = lines[left_line]
    (x1_r, y1_r), (x2_r, y2_r) = lines[right_line]

    y_left_min = min(y1_l, y2_l)
    y_left_max = max(y1_l, y2_l)
    y_right_min = min(y1_r, y2_r)
    y_right_max = max(y1_r, y2_r)

    y_avg_min = (y_left_min + y_right_min) / 2.0
    y_avg_max = (y_left_max + y_right_max) / 2.0

    k_avg = (abs(kb_list[0][0]) + abs(kb_list[1][0])) / 2.0

    x1_left = (y_avg_min - kb_list[left_line][1]) / (-k_avg)
    x2_left = (y_avg_max - kb_list[left_line][1]) / (-k_avg)
    x1_right = (y_avg_min - kb_list[right_line][1]) / k_avg
    x2_right = (y_avg_max - kb_list[right_line][1]) / k_avg

    h = round(abs(y_avg_max - y_avg_min))
    a = round(abs(x1_right - x1_left))
    b = round(abs(x2_right - x2_left))
    return h, a, b, y_avg_min, y_avg_max


def _correct_all(y_avg_min, y_avg_max, lines_dict):
    res = {}
    span = round(y_avg_max - y_avg_min)
    for key, coordinates in lines_dict.items():
        (x1, y1), (x2, y2) = coordinates
        y1_correct = round(min(max(y1 - y_avg_min, 0), span))
        y2_correct = round(min(max(y2 - y_avg_min, 0), span))
        if span - 20 <= y1_correct <= span + 20:
            y1_correct = span
        if span - 20 <= y2_correct <= span + 20:
            y2_correct = span
        res[key] = [(x1, y1_correct), (x2, y2_correct)]
    return res


def correct_all0201(y_avg_min, y_avg_max, lines_dict):
    return _correct_all(y_avg_min, y_avg_max, lines_dict)


def correct_all0202(y_avg_min, y_avg_max, lines_dict):
    return _correct_all(y_avg_min, y_avg_max, lines_dict)


def trans_lines02(h, a, b, line_dict):
    res = {}
    for key, line in line_dict.items():
        (x1, y1), (x2, y2) = line
        k1 = y1 / h if h else 0.0
        k2 = y2 / h if h else 0.0
        span = math.sqrt(max(h ** 2 - ((b - a) / 2.0) ** 2, 0.0))
        res[key] = [
            round(span * k1 / 1000.0, 6),
            round(span * k2 / 1000.0, 6),
        ]
    return res


# ================= 格式转换相关子函数 =================

def _find_special_bar(lines01):
    for bar_id in lines01.keys():
        if str(bar_id).endswith("01"):
            return str(bar_id)
    return str(next(iter(lines01.keys()))) if lines01 else ""


def convert_data0201(line_z_dict, line_group, lines01):
    ganjian = []
    jiedian = []
    special_bar_id = _find_special_bar(lines01)

    for group in line_group:
        if not group:
            continue
        if len(group) == 1:
            bar_id = group[0]
            if bar_id not in line_z_dict:
                continue
            z_value = line_z_dict[bar_id][0]
            node_id = f"{bar_id}10"
            x_val = f"{special_bar_id}10"
            y_val = f"{special_bar_id}20"
            jiedian.append({
                "node_id": node_id,
                "node_type": 12,
                "symmetry_type": 4,
                "X": x_val,
                "Y": y_val,
                "Z": z_value,
            })
            ganjian.append({
                "member_id": str(bar_id),
                "node1_id": f"{bar_id}10",
                "node2_id": f"{bar_id}11",
                "symmetry_type": 2,
            })
            ganjian.append({
                "member_id": str(bar_id),
                "node1_id": f"{bar_id}10",
                "node2_id": f"{bar_id}12",
                "symmetry_type": 1,
            })
        elif len(group) == 2:
            bar1_id, bar2_id = group
            if bar1_id not in line_z_dict:
                continue
            z_values = line_z_dict[bar1_id]
            z_min = min(z_values)
            z_max = max(z_values)

            node1_id = f"{bar1_id}10"
            x1_val = f"{special_bar_id}10"
            y1_val = f"{special_bar_id}20"
            jiedian.append({
                "node_id": node1_id,
                "node_type": 12,
                "symmetry_type": 4,
                "X": x1_val,
                "Y": y1_val,
                "Z": z_min,
            })

            node2_id = f"{bar1_id}20"
            x2_val = f"{special_bar_id}11"
            y2_val = f"{special_bar_id}21"
            jiedian.append({
                "node_id": node2_id,
                "node_type": 12,
                "symmetry_type": 4,
                "X": x2_val,
                "Y": y2_val,
                "Z": z_max,
            })

            ganjian.append({
                "member_id": bar1_id,
                "node1_id": f"{bar1_id}10",
                "node2_id": f"{bar1_id}20",
                "symmetry_type": 4,
            })
            ganjian.append({
                "member_id": bar2_id,
                "node1_id": f"{bar1_id}10",
                "node2_id": f"{bar1_id}23",
                "symmetry_type": 4,
            })

    return ganjian, jiedian


def convert_data0202(line_z_dict, lines02, line_group, lines01, horizontal_lines, midpoint):
    ganjian = []
    jiedian = []
    special_bar_id = _find_special_bar(lines01)

    for group in line_group:
        if len(group) < 2:
            continue
        primary_id = group[0]
        partner_id = group[1]
        if primary_id not in lines02 or primary_id not in line_z_dict:
            continue

        (x1, y1), (x2, y2) = lines02[primary_id]
        if abs(x1 - midpoint) <= abs(x2 - midpoint):
            near_zero_point = (x1, y1)
            other_point = (x2, y2)
        else:
            near_zero_point = (x2, y2)
            other_point = (x1, y1)

        near_zero_y = near_zero_point[1]
        matched_horizontal_id = None
        min_y_diff = float("inf")
        for h_line_id, (h_end1, h_end2) in horizontal_lines.items():
            h_y = (h_end1[1] + h_end2[1]) / 2
            diff = abs(near_zero_y - h_y)
            if diff < min_y_diff:
                min_y_diff = diff
                matched_horizontal_id = h_line_id

        if matched_horizontal_id is None:
            continue

        far_end_point = near_zero_y < other_point[1]
        z_values = line_z_dict[primary_id]
        z_far = max(z_values) if far_end_point else min(z_values)

        node101_id = f"{primary_id}10"
        jiedian.append({
            "node_id": node101_id,
            "node_type": 12,
            "symmetry_type": 2,
            "X": 0,
            "Y": f"{matched_horizontal_id}10",
            "Z": f"{matched_horizontal_id}11",
        })

        node102_id = f"{primary_id}20"
        jiedian.append({
            "node_id": node102_id,
            "node_type": 12,
            "symmetry_type": 1,
            "X": f"{matched_horizontal_id}10",
            "Y": 0,
            "Z": f"{matched_horizontal_id}12",
        })

        node1_id = f"{primary_id}30"
        jiedian.append({
            "node_id": node1_id,
            "node_type": 12,
            "symmetry_type": 4,
            "X": f"{special_bar_id}10",
            "Y": f"{special_bar_id}20",
            "Z": z_far,
        })

        ganjian.append({
            "member_id": primary_id,
            "node1_id": f"{primary_id}10",
            "node2_id": f"{primary_id}30",
            "symmetry_type": 4,
        })
        ganjian.append({
            "member_id": partner_id,
            "node1_id": f"{primary_id}20",
            "node2_id": f"{primary_id}30",
            "symmetry_type": 4,
        })

    return ganjian, jiedian


def correct_lines02(ganjian, jiedian):
    res_jiedian = []
    node_id_mapping = {}

    for node in jiedian:
        current_x = node["X"]
        current_y = node["Y"]
        current_z = node["Z"]
        current_id = node["node_id"]
        current_x_type = type(current_x)
        current_y_type = type(current_y)
        current_z_type = type(current_z)

        try:
            current_z_float = float(current_z)
        except (ValueError, TypeError):
            current_z_float = None

        is_match = False
        for existing_node in res_jiedian:
            existing_x = existing_node["X"]
            existing_y = existing_node["Y"]
            existing_z = existing_node["Z"]
            existing_id = existing_node["node_id"]
            if (
                type(existing_x) != current_x_type
                or type(existing_y) != current_y_type
                or type(existing_z) != current_z_type
            ):
                continue
            if existing_x != current_x or existing_y != current_y:
                continue

            if current_z_float is not None:
                try:
                    existing_z_float = float(existing_z)
                except (ValueError, TypeError):
                    continue
                if abs(existing_z_float - current_z_float) < 0.02:
                    is_match = True
            else:
                if existing_z == current_z:
                    is_match = True

            if is_match:
                node_id_mapping[current_id] = existing_id
                break

        if not is_match:
            res_jiedian.append(node.copy())

    for member in ganjian:
        if member["node1_id"] in node_id_mapping:
            member["node1_id"] = node_id_mapping[member["node1_id"]]
        if member["node2_id"] in node_id_mapping:
            member["node2_id"] = node_id_mapping[member["node2_id"]]

    return ganjian, res_jiedian


# ================= 总函数 =================

def single_view0201(line_coord):
    lines01 = extract_lines01(line_coord)
    avg_min, avg_max = parameter_extract(lines01)
    lines0201 = extract_lines0201(line_coord, avg_min, avg_max)
    inter_lines0201 = find_y_proximity_bars(lines0201)
    lines0202, horizontal_lines = extract_lines0202(line_coord, lines01, lines0201, inter_lines0201)
    inter_lines0202 = find_y_proximity_bars(lines0202)

    corrected_lines0201 = correct_lines(lines0201, inter_lines0201)
    corrected_lines0202 = correct_lines(lines0202, inter_lines0202)

    h, a, b, y_avg_min, y_avg_max = parameter_extract01(lines01)
    final_corrected_lines0201 = correct_all0201(y_avg_min, y_avg_max, corrected_lines0201)
    final_corrected_lines0202 = correct_all0202(y_avg_min, y_avg_max, corrected_lines0202)

    res_z0201 = trans_lines02(h, a, b, final_corrected_lines0201)
    res_z0202 = trans_lines02(h, a, b, final_corrected_lines0202)

    res_ganjian0201, res_jiedian0201 = convert_data0201(res_z0201, inter_lines0201, lines01)
    res_ganjian0202, res_jiedian0202 = convert_data0202(
        res_z0202,
        final_corrected_lines0202,
        inter_lines0202,
        lines01,
        horizontal_lines,
        (avg_min + avg_max) / 2.0,
    )

    res_ganjian = res_ganjian0201 + res_ganjian0202
    res_jiedian = res_jiedian0201 + res_jiedian0202
    res_ganjian, res_jiedian = correct_lines02(res_ganjian, res_jiedian)
    return res_ganjian, res_jiedian