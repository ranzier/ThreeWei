import math
import json

def dist_points(p1, p2):
    """计算点之间距离"""
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def angle_between_lines(line1_pt1, line1_pt2, line2_pt1, line2_pt2):
    """
    计算两条直线的锐角夹角（0-90度）的简化版本

    参数:
        line1_pt1: 第一条直线的第一个端点 (x1, y1)
        line1_pt2: 第一条直线的第二个端点 (x2, y2)
        line2_pt1: 第二条直线的第一个端点 (x3, y3)
        line2_pt2: 第二条直线的第二个端点 (x4, y4)

    返回:
        两条直线的锐角夹角（单位：度，范围0-90度）
    """
    # 计算方向向量
    v1 = (line1_pt2[0] - line1_pt1[0], line1_pt2[1] - line1_pt1[1])
    v2 = (line2_pt2[0] - line2_pt1[0], line2_pt2[1] - line2_pt1[1])

    # 计算点积和模
    dot = v1[0] * v2[0] + v1[1] * v2[1]
    mag1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
    mag2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)

    # 检查零向量
    if mag1 == 0 or mag2 == 0:
        return 0

    # 计算夹角
    cos_angle = max(-1.0, min(1.0, dot / (mag1 * mag2)))
    angle_deg = math.degrees(math.acos(cos_angle))

    # 返回锐角
    return min(angle_deg, 180 - angle_deg)

def dist_point_to_line(point, line_point1, line_point2):
    """
    计算点到直线的垂直距离（垂线段长度）

    参数:
        point: 点的坐标 (x, y)
        line_point1: 直线上的第一个点 (x1, y1)
        line_point2: 直线上的第二个点 (x2, y2)

    返回:
        点到直线的垂直距离
    """
    x, y = point
    x1, y1 = line_point1
    x2, y2 = line_point2

    # 如果直线是垂直的
    if x1 == x2:
        return abs(x - x1)

    # 如果直线是水平的
    if y1 == y2:
        return abs(y - y1)

    # 计算直线的一般式方程 Ax + By + C = 0
    A = y2 - y1
    B = x1 - x2
    C = x2 * y1 - x1 * y2

    # 计算点到直线的距离
    distance = abs(A * x + B * y + C) / math.sqrt(A ** 2 + B ** 2)

    return distance

def line_intersection(line1, line2):
    """
    计算两条直线的交点

    参数:
        line1: 第一条直线，格式为 [(x1, y1), (x2, y2)]
        line2: 第二条直线，格式为 [(x3, y3), (x4, y4)]

    返回:
        交点坐标 (x, y)，如果直线平行则返回 None
    """
    # 提取坐标
    (x1, y1), (x2, y2) = line1
    (x3, y3), (x4, y4) = line2

    # 计算分母
    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

    # 如果分母为0，则直线平行或重合
    if denominator == 0:
        return None

    # 计算交点坐标
    x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator
    y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator

    return (x, y)

def dist_z(p1, p2):
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

def compute_l1_B(coordinatesBottom_data, drawing_id):
    """
    自动计算底视图中的 l1 和 l2 杆件编号。

    返回:
        l1, l2 （两个int编号）
    """
    rod_ids = [k for k in coordinatesBottom_data.keys() if k // 100 == drawing_id]
    rod_ids = sorted(rod_ids)[-2:]  # 取最大两个编号

    id1, id2 = rod_ids[-2], rod_ids[-1]
    ma = max(id1, id2)
    return ma % 100

def compute_l1_O(coordinatesOverhead_data, drawing_id):
    """
    自动计算俯视图中的 l1 和 l2 杆件编号。

    返回:
        l1, l2 （两个int编号）
    """
    rod_ids = [k for k in coordinatesOverhead_data.keys() if k // 100 == drawing_id]
    rod_ids = sorted(rod_ids)[-2:]  # 取最大两个编号

    id1, id2 = rod_ids[-2], rod_ids[-1]
    ma = max(id1, id2)
    return ma % 100

def transform_data(input_data):
    """
    将输入的节点数据按每4个一组进行分组

    参数:
        input_data: 输入的节点数据列表

    返回:
        分组后的数据
    """
    # 将数据分成4组，每组4个节点
    groups = []
    for i in range(0, len(input_data), 4):
        group = input_data[i:i + 4]
        groups.append(group)

    return groups

def cluster_points(points, threshold=150.0):
    """
    对点进行聚类，如果当前点与前一个点距离小于阈值，则合并到前一个聚类

    参数:
        points: 点列表，每个点为 (x, y) 格式
        threshold: 聚类阈值，默认10.0

    返回:
        聚类后的点列表
    """
    clusters = []

    for point in points:
        if not clusters:
            # 第一个点，直接作为第一个聚类
            clusters.append({
                "points": [point],
                "centroid": point
            })
        else:
            # 计算与最后一个聚类中心的距离
            last_centroid = clusters[-1]["centroid"]
            distance = dist_points(point, last_centroid)

            if distance < threshold:
                # 合并到最后一个聚类
                clusters[-1]["points"].append(point)
                # 更新聚类中心为所有点的平均值
                all_points = clusters[-1]["points"]
                avg_x = sum(p[0] for p in all_points) / len(all_points)
                avg_y = sum(p[1] for p in all_points) / len(all_points)
                clusters[-1]["centroid"] = (avg_x, avg_y)
            else:
                # 创建新聚类
                clusters.append({
                    "points": [point],
                    "centroid": point
                })

    # 返回聚类中心点
    return [cluster["centroid"] for cluster in clusters]

def remove_endpoint_clusters(clustered_points, rod_endpoints, threshold=10):
    """
    去除与杆件端点重合的聚类点

    参数:
        clustered_points: 聚类后的点列表，每个点为 (x, y) 格式
        rod_endpoints: 杆件的端点列表，每个点为 (x, y) 格式
        threshold: 重合判断阈值，默认1.0

    返回:
        去除重合点后的聚类点列表
    """
    filtered_points = []

    for point in clustered_points:
        is_endpoint = False
        for endpoint in rod_endpoints:
            distance = dist_points(point, endpoint)
            if distance < threshold:
                is_endpoint = True
                break

        if not is_endpoint:
            filtered_points.append(point)

    return filtered_points

jiedian = []
ganjian = []
def trans(file_path, drawing_id, data1):
    """
    参数:
        file_path: 包含三视图坐标数据的文件路径
        drawing_id: 图纸序号
        l1, l2: 对称轴附近的杆件编号
    """
    # 读取字典数据
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # 获取二维坐标点
    namespace = {}
    exec(content, namespace)
    coordinatesFront_data = namespace.get('coordinatesFront_data', {})
    coordinatesBottom_data = namespace.get('coordinatesBottom_data', {})
    coordinatesOverhead_data = namespace.get('coordinatesOverhead_data', {})

    yuzhi = 50#是否在直线上距离阈值

    # 将塔身组传来的担架与塔身相交的端点，一共16个端点，每两个担架有4个端点，所以将16个端点转成四组，每组4个
    data = transform_data(data1)

    # 将16个端点按照x轴的正负进行分组，【【【担架1的上端点】，【担架1的下端点】】，【【担架2的上端点】，【担架2的下端点】】，【【担架3的上端点】，【担架3的下端点】】，【】，【】...】
    pj = []
    for array in data:
        positive_group = []
        negative_group = []
        for point in array:
            x_coord = point[1][0]
            #print(point[1][0])
            if x_coord >= 0:
                positive_group.append(point)
            else:
                negative_group.append(point)
        pj.append(positive_group)
        pj.append(negative_group)

    # print("pj =", pj)



    # node1_id就是101、201、301...，然后再×100：10100、20100、30100...
    node1_id = (drawing_id * 100 + 1) * 100
    # node2_id就是103、203、303...，然后再×100：10300、20300、30300...
    node2_id = (drawing_id * 100 + 3) * 100

    # 底面是仰视图：也就是生成3、4、5、6、7、8号担架
    # 这个判断语句是：如果101、201、301、401在底视图的二维坐标数据里面
    if(drawing_id * 100 + 1 in coordinatesBottom_data):
        l1 = compute_l1_B(coordinatesBottom_data, drawing_id)
        node3_id = (drawing_id * 100 + l1) * 100
        start_01x = coordinatesBottom_data[drawing_id * 100 + 1][0][0]
        start_01y = coordinatesBottom_data[drawing_id * 100 + 1][0][1]
        start_02x = coordinatesBottom_data[drawing_id * 100 + 2][0][0]
        start_02y = coordinatesBottom_data[drawing_id * 100 + 2][0][1]
        end_01x = coordinatesBottom_data[drawing_id * 100 + 1][1][0]
        end_01y = coordinatesBottom_data[drawing_id * 100 + 1][1][1]
        end_02x = coordinatesBottom_data[drawing_id * 100 + 2][1][0]
        end_02y = coordinatesBottom_data[drawing_id * 100 + 2][1][1]
        midr = ((end_01x + end_02x) / 2, (end_01y + end_02y) / 2) # 计算301和302两个右端点的中点
        midl = ((start_01x + start_02x) / 2, (start_01y + start_02y) / 2)  # 计算301和302两个左端点的中点
        h = dist_points(midl, midr)  # 计算左中点到右中点的距离
        p_01l = (start_01x, start_01y)
        a1 = dist_points(p_01l, midl)   # 计算301左端点到左中点的距离
        shiji = abs(pj[drawing_id - 1][1][1][1])  # 计算担架3与塔身相加的下端点的三维x坐标的值，也就是实际值
        bili = shiji / a1 # 计算实际值/像素值的比例
        p_01r = (end_01x, end_01y)
        a2 = dist_points(p_01r, midr)


        #生成尖点
        if(pj[drawing_id - 1][1][1][0] > 0): # if里面生成右边担架的尖点
            newx = pj[drawing_id - 1][1][1][0] + h * bili
            newy = -a2 * bili
            newz = pj[drawing_id - 1][1][1][2]
        else:  # else生成左边担架的尖点
            newx = pj[drawing_id - 1][1][1][0] - h * bili
            newy = -a2 * bili
            newz = pj[drawing_id - 1][1][1][2]
        new_node = {
            "node_id": f"{node1_id + 20}",
            "node_type": 11,  # 根据实际情况设置节点类型
            "symmetry_type": 2,  # 根据实际情况设置对称类型
            "X": round(newx,3),
            "Y": round(newy,3),
            "Z": round(newz,3)
        }
        jiedian.append(new_node)


        #正视图
        intersections_101 = []
        intersections_102 = []
        intersections_103 = []

        rod_101_id = drawing_id * 100 + 1
        rod_102_id = drawing_id * 100 + 2
        rod_103_id = drawing_id * 100 + 3
        rod_104_id = drawing_id * 100 + 4

        rod_101_points = coordinatesFront_data[rod_101_id]
        rod_103_points = coordinatesFront_data[rod_103_id]

        # 正视图以301、303两个杆件为参考杆件
        ref_points_101 = rod_101_points
        ref_points_103 = rod_103_points

        mi = 199999 #二类杆件的起始编号

        # 遍历所有杆件
        # rod_id：杆件编号（如 301、302、303 …），points：该杆件的两个端点（二维）
        for rod_id, points in coordinatesFront_data.items():
            if rod_id in [rod_101_id, rod_103_id]:  # 如果是301、303杆件就跳过
                continue
            flag1 = 0 # flag1 用来统计该杆件有多少端点靠近参考杆件
            for i, point in enumerate(points):
                distance_to_101 = dist_point_to_line(point, ref_points_101[0], ref_points_101[1])
                distance_to_103 = dist_point_to_line(point, ref_points_103[0], ref_points_103[1])
                if(distance_to_103 < yuzhi or distance_to_101 < yuzhi): # 如果端点只要靠近任意一根参考杆，flag1就加1
                    flag1 += 1
            if flag1 == 2: # 该杆件的两个端点都靠近参考杆件
                intersection_101 = line_intersection(points, rod_101_points)
                mi = min(mi, rod_id) # 记录最小杆件编号
                if intersection_101 is not None:
                    intersections_101.append(intersection_101) # 求杆件与 301 的交点坐标

                # 计算目标杆件与303杆件的交点坐标
                intersection_103 = line_intersection(points, rod_103_points)
                if intersection_103 is not None:
                    intersections_103.append(intersection_103)
        # 对在301杆件上的交点这个数组的元素进行排序，按照二维坐标的x轴从小到大进行排序
        if (pj[drawing_id - 1][1][1][0] > 0):
            intersections_101.sort(key=lambda point: point[0])
        else:
            intersections_101.sort(key=lambda point: point[0], reverse=True)
        node_101 = cluster_points(intersections_101, threshold=150.0)  # 对杆件上的交点进行聚类得到node_101[]这个聚类后的交点数组
        if (pj[drawing_id - 1][1][1][0] > 0):
            intersections_103.sort(key=lambda point: point[0])
        else:
            intersections_103.sort(key=lambda point: point[0], reverse=True)
        node_103 = cluster_points(intersections_103, threshold=150.0)
        filtered_101 = remove_endpoint_clusters(node_101, rod_101_points) # 去除聚类后交点数组中的301的端点
        filtered_103 = remove_endpoint_clusters(node_103, rod_103_points)

        # 得到301杆件的两个端点的二维X坐标
        min_2d_x = rod_101_points[0][0]
        max_2d_x = rod_101_points[1][0]

        # 得到301 杆件在三维中的两端的 X 坐标
        if (pj[drawing_id - 1][1][1][0] > 0):
            min_3d_x = pj[drawing_id - 1][1][1][0]  # 这个是301杆件与塔身相交的端点的三维X坐标
            max_3d_x = newx  # 这个是301杆件尖点的三维X坐标
        else:
            min_3d_x = newx
            max_3d_x = pj[drawing_id - 1][1][1][0]

        # 计算比例并转换为真实x坐标
        real_101 = []
        for point in filtered_101:
            # 计算点在二维杆件上的x坐标比例
            x_ratio = (point[0] - min_2d_x) / (max_2d_x - min_2d_x)
            # 根据比例计算真实x坐标
            real_x = min_3d_x + x_ratio * (max_3d_x - min_3d_x)
            real_101.append(real_x)

        min_2d_x = rod_103_points[0][0]
        max_2d_x = rod_103_points[1][0]
        if (pj[drawing_id - 1][0][1][0] > 0):
            min_3d_x = pj[drawing_id - 1][0][1][0]
            max_3d_x = newx
        else:
            min_3d_x = newx
            max_3d_x = pj[drawing_id - 1][0][1][0]

        # 计算比例并转换为真实x坐标
        real_103 = []
        for point in filtered_103:
            # 计算点在二维杆件上的x坐标比例
            x_ratio = (point[0] - min_2d_x) / (max_2d_x - min_2d_x)
            # 根据比例计算真实x坐标
            real_x = min_3d_x + x_ratio * (max_3d_x - min_3d_x)
            real_103.append(real_x)
        min_len = min(len(real_101), len(real_103))

        # 为交点创建节点
        node_101_nodes = []
        node_103_nodes = []

        # 为101杆件上的交点创建节点
        for i, point in enumerate(real_101):
            node_id = f"{drawing_id}101{i + 1}0"
            node_101_nodes.append(node_id)
            jiedian.append({
                "node_id": node_id,
                "node_type": 12,
                "symmetry_type": 2,
                "X": point,
                "Y": f"1{pj[drawing_id - 1][1][0]}",
                "Z": f"1{node1_id + 20}"
            })

        # 为103杆件上的交点创建节点
        for i, point in enumerate(real_103):
            node_id = f"{drawing_id}103{i + 1}0"
            node_103_nodes.append(node_id)
            jiedian.append({
                "node_id": node_id,
                "node_type": 12,
                "symmetry_type": 2,
                "X": point,
                "Y": f"1{pj[drawing_id - 1][0][0]}",
                "Z": f"1{node1_id + 20}"
            })

        # 将前视图的杆件编号提取出来放到一个数组中，并从小到大排序
        Front_ganjian_ID_list = sorted(coordinatesFront_data.keys())
        member_id_base = mi  # 新杆件的基准编号
        index_address = Front_ganjian_ID_list.index(member_id_base) # 基准杆件在杆件数组中的索引位置
        current_offset = 0  # 用于依次取后续编号

        # 按照指定规则连接交点，开口左右分别处理方便编号
        if(drawing_id != 7 and drawing_id != 8):
            ganjian.append({
                "member_id": f"{member_id_base}",
                "node1_id": pj[drawing_id - 1][1][0],
                "node2_id": node_103_nodes[0],
                "symmetry_type": 2
            })

        for i in range(min_len):
            # 连接101杆件上的第i个交点与103杆件上的第i个交点
            if i < len(node_101_nodes) and i < len(node_103_nodes):
                current_offset += 1
                ganjian.append({
                    "member_id": str(Front_ganjian_ID_list[index_address + current_offset]),
                    "node1_id": node_101_nodes[i],
                    "node2_id": node_103_nodes[i],
                    "symmetry_type": 2
                })

            # 连接103杆件上的第i+1个交点与101杆件上的第i个交点
            if i + 1 < len(node_103_nodes) and i < len(node_101_nodes):
                current_offset += 1
                ganjian.append({
                    "member_id": str(Front_ganjian_ID_list[index_address + current_offset]),
                    "node1_id": node_103_nodes[i + 1],
                    "node2_id": node_101_nodes[i],
                    "symmetry_type": 2
                })


        #底视图
        intersections_101.clear()

        rod_101_points = coordinatesBottom_data[rod_101_id]
        rod_102_points = coordinatesBottom_data[rod_102_id]

        # 提取参考杆件的所有端点
        ref_points_101 = rod_101_points
        ref_points_102 = rod_102_points

        mi = 199999  # 二类杆件的起始编号

        # 遍历所有杆件
        for rod_id, points in coordinatesBottom_data.items():
            if rod_id in [rod_101_id, rod_102_id]:
                continue
            flag1 = 0
            for i, point in enumerate(points):
                distance_to_101 = dist_point_to_line(point, ref_points_101[0], ref_points_101[1])
                distance_to_102 = dist_point_to_line(point, ref_points_102[0], ref_points_102[1])
                if (distance_to_102 < yuzhi or distance_to_101 < yuzhi):
                    flag1 += 1
            if flag1 == 2:
                intersection_101 = line_intersection(points, rod_101_points)
                if intersection_101 is not None:
                    mi = min(mi,rod_id)
                    intersections_101.append(intersection_101)
        if (pj[drawing_id - 1][1][1][0] > 0):
            intersections_101.sort(key=lambda point: point[0])
        else:
            intersections_101.sort(key=lambda point: point[0], reverse=True)
        node_101 = cluster_points(intersections_101, threshold=150.0)
        filtered_101 = remove_endpoint_clusters(node_101, rod_101_points)
        min_2d_x = rod_101_points[0][0]
        max_2d_x = rod_101_points[1][0]
        if (pj[drawing_id - 1][1][1][0] > 0):
            min_3d_x = pj[drawing_id - 1][1][1][0]
            max_3d_x = newx
        else:
            min_3d_x = newx
            max_3d_x = pj[drawing_id - 1][1][1][0]

        # 计算比例并转换为真实x坐标
        real_101.clear()
        for point in filtered_101:
            # 计算点在二维杆件上的x坐标比例
            x_ratio = (point[0] - min_2d_x) / (max_2d_x - min_2d_x)
            # 根据比例计算真实x坐标
            real_x = min_3d_x + x_ratio * (max_3d_x - min_3d_x)
            real_101.append(real_x)

        node_101_ids = []
        for i, point in enumerate(real_101):
            node_id = f"{drawing_id}201{i + 1}0"
            node_101_ids.append(node_id)
            duichenzuo = int(pj[drawing_id - 1][1][0]) + 2
            jiedian.append({
                "node_id": node_id,
                "node_type": 12,
                "symmetry_type": 2,
                "X": point,
                "Y": f"1{duichenzuo}",
                "Z": f"1{node1_id + 22}"
            })
        member_id_base = mi  # 新杆件的基准编号
        ganjian.append({
            "member_id": f"{member_id_base}",
            "node1_id": pj[drawing_id - 1][1][0],
            "node2_id": node_101_ids[0],
            "symmetry_type": 2
        })
        for i in range(0, len(real_101) - 1):
            if i % 2 == 0:  # 偶数索引：当前点与下一个点的对称点相连
                ganjian.append({
                    "member_id": f"{member_id_base + i * 2 + 2}",
                    "node1_id": f"{drawing_id}201{i + 2}2",
                    "node2_id": node_101_ids[i],
                    "symmetry_type": 2
                })
            else:  # 奇数索引：当前点的对称点与下一个点相连
                ganjian.append({
                    "member_id": f"{member_id_base + i * 2 + 2}",
                    "node1_id": node_101_ids[i + 1],
                    "node2_id": f"{drawing_id}201{i + 1}2",
                    "symmetry_type": 2
                })
        ganjian.append({
            "member_id": f"{member_id_base + (len(real_101) - 1) * 2 + 2}",
            "node1_id": f"{node1_id + 20}",
            "node2_id": node_101_ids[len(real_101) - 1],
            "symmetry_type": 2
        })


        #顶视图
        jdyuzhi = 5
        intersections_103.clear()
        rod_103_points = coordinatesOverhead_data[rod_103_id]
        rod_104_points = coordinatesOverhead_data[rod_104_id]

        # 提取参考杆件的所有端点
        ref_points_103 = rod_103_points
        ref_points_104 = rod_104_points

        flag = 0#1代表平行情况，0交叉
        mi = 199999
        # 遍历所有杆件
        for rod_id, points in coordinatesOverhead_data.items():
            if rod_id in [rod_103_id, rod_104_id]:
                continue
            flag1 = 0
            for i, point in enumerate(points):
                distance_to_103 = dist_point_to_line(point, ref_points_103[0], ref_points_103[1])
                distance_to_104 = dist_point_to_line(point, ref_points_104[0], ref_points_104[1])
                if (distance_to_103 < yuzhi or distance_to_104 < yuzhi):
                    flag1 += 1
            if flag1 == 2:
                jiaodu = angle_between_lines(points[0],points[1],ref_points_103[0],ref_points_104[0])
                if jiaodu < jdyuzhi:
                    flag = 1
                intersection_103 = line_intersection(points, rod_103_points)
                if intersection_103 is not None:
                    mi = min(mi,rod_id)
                    intersections_103.append(intersection_103)

        if (pj[drawing_id - 1][1][1][0] > 0):
            intersections_103.sort(key=lambda point: point[0])
        else:
            intersections_103.sort(key=lambda point: point[0], reverse=True)
        node_103 = cluster_points(intersections_103, threshold=150.0)
        filtered_103 = remove_endpoint_clusters(node_103, rod_103_points)
        min_2d_x = rod_103_points[0][0]
        max_2d_x = rod_103_points[1][0]
        if (pj[drawing_id - 1][1][1][0] > 0):
            min_3d_x = pj[drawing_id - 1][0][1][0]
            max_3d_x = newx
        else:
            min_3d_x = newx
            max_3d_x = pj[drawing_id - 1][0][1][0]

        # 计算比例并转换为真实x坐标
        real_103.clear()
        for point in filtered_103:
            # 计算点在二维杆件上的x坐标比例
            x_ratio = (point[0] - min_2d_x) / (max_2d_x - min_2d_x)
            # 根据比例计算真实x坐标
            real_x = min_3d_x + x_ratio * (max_3d_x - min_3d_x)
            real_103.append(real_x)
        node_103_ids = []
        for i, point in enumerate(real_103):
            node_id = f"{drawing_id}301{i + 1}0"
            node_103_ids.append(node_id)
            jiedian.append({
                "node_id": node_id,
                "node_type": 12,
                "symmetry_type": 2,
                "X": point,
                "Y": f"1{pj[drawing_id - 1][0][0]}",
                "Z": f"1{node1_id + 20}"
            })
        member_id_base = mi
        if flag == 1:
            for i in range(0, len(real_103)):
                ganjian.append({
                    "member_id": f"{member_id_base + i}",
                    "node1_id": node_103_ids[i],
                    "node2_id": f"{drawing_id}301{i + 1}2",
                    "symmetry_type": 0
                })
        # else是俯视图交叉的情况


        # 添加301，303和一个特殊的杆件
        new_ganjian = {
            "member_id": f"{drawing_id * 100 + 1}",
            "node1_id": pj[drawing_id - 1][1][0],
            "node2_id": f"{node1_id + 20}",
            "symmetry_type": 2
        }
        ganjian.append(new_ganjian)
        new_ganjian = {
            "member_id": f"{drawing_id * 100 + 3}",
            "node1_id": pj[drawing_id - 1][0][0],
            "node2_id": f"{node1_id + 20}",
            "symmetry_type": 2
        }
        ganjian.append(new_ganjian)
        new_ganjian = {
            "member_id": f"{drawing_id * 100 + l1}",
            "node1_id": f"{node1_id + 20}",
            "node2_id": f"{node1_id + 22}",
            "symmetry_type": 0
        }
        ganjian.append(new_ganjian)


  #底面是俯视图：也就是生成1、2号担架
    else:
        l1 = compute_l1_O(coordinatesOverhead_data, drawing_id)
        start_01x = coordinatesOverhead_data[drawing_id * 100 + 1][0][0]
        start_01y = coordinatesOverhead_data[drawing_id * 100 + 1][0][1]
        start_02x = coordinatesOverhead_data[drawing_id * 100 + 2][0][0]
        start_02y = coordinatesOverhead_data[drawing_id * 100 + 2][0][1]
        end_01x = coordinatesOverhead_data[drawing_id * 100 + 1][1][0]
        end_01y = coordinatesOverhead_data[drawing_id * 100 + 1][1][1]
        end_02x = coordinatesOverhead_data[drawing_id * 100 + 2][1][0]
        end_02y = coordinatesOverhead_data[drawing_id * 100 + 2][1][1]
        midr = ((end_01x + end_02x) / 2, (end_01y + end_02y) / 2)
        midl = ((start_01x + start_02x) / 2, (start_01y + start_02y) / 2)
        h = dist_points(midl, midr)
        p_01l = (start_01x, start_01y)
        a1 = dist_points(p_01l, midl)
        shiji = abs(pj[drawing_id - 1][0][1][1])
        bili = shiji / a1
        p_01r = (end_01x, end_01y)
        a2 = dist_points(p_01r, midr)
        node3_id = (drawing_id * 100 + l1) * 100


        # 生成尖点
        if (pj[drawing_id - 1][0][1][0] > 0):
            newx = pj[drawing_id - 1][0][1][0] + h * bili
            newy = -a2 * bili
            newz = pj[drawing_id - 1][0][1][2]
        else:
            newx = pj[drawing_id - 1][0][1][0] - h * bili
            newy = -a2 * bili
            newz = pj[drawing_id - 1][0][1][2]
        new_node = {
            "node_id": f"{node1_id + 20}",
            "node_type": 11,  # 根据实际情况设置节点类型
            "symmetry_type": 2,  # 根据实际情况设置对称类型
            "X": round(newx,3),
            "Y": round(newy,3),
            "Z": round(newz,3)
        }
        jiedian.append(new_node)



        # 正视图
        intersections_101 = []
        intersections_102 = []
        intersections_103 = []

        rod_101_id = drawing_id * 100 + 1
        rod_102_id = drawing_id * 100 + 2
        rod_103_id = drawing_id * 100 + 3
        rod_104_id = drawing_id * 100 + 4

        rod_101_points = coordinatesFront_data[rod_101_id]
        rod_103_points = coordinatesFront_data[rod_103_id]

        # 提取参考杆件的所有端点
        ref_points_101 = rod_101_points
        ref_points_103 = rod_103_points

        mi = 199999
        # 遍历所有杆件
        for rod_id, points in coordinatesFront_data.items():
            if rod_id in [rod_101_id, rod_103_id]:
                continue
            flag1 = 0
            for i, point in enumerate(points):
                distance_to_101 = dist_point_to_line(point, ref_points_101[0], ref_points_101[1])
                distance_to_103 = dist_point_to_line(point, ref_points_103[0], ref_points_103[1])
                if (distance_to_103 < yuzhi or distance_to_101 < yuzhi):
                    flag1 += 1
            if flag1 == 2:
                intersection_101 = line_intersection(points, rod_101_points)
                mi = min(mi, rod_id)
                if intersection_101 is not None:
                    intersections_101.append(intersection_101)

                # 计算目标杆件与103杆件的交点
                intersection_103 = line_intersection(points, rod_103_points)
                if intersection_103 is not None:
                    intersections_103.append(intersection_103)

        if (pj[drawing_id - 1][1][1][0] > 0):
            intersections_101.sort(key=lambda point: point[0])
        else:
            intersections_101.sort(key=lambda point: point[0], reverse=True)
        node_101 = cluster_points(intersections_101, threshold=150.0)
        if (pj[drawing_id - 1][1][1][0] > 0):
            intersections_103.sort(key=lambda point: point[0])
        else:
            intersections_103.sort(key=lambda point: point[0], reverse=True)
        node_103 = cluster_points(intersections_103, threshold=150.0)
        filtered_101 = remove_endpoint_clusters(node_101, rod_101_points)
        filtered_103 = remove_endpoint_clusters(node_103, rod_103_points)

        min_2d_x = rod_101_points[0][0]
        max_2d_x = rod_101_points[1][0]
        if (pj[drawing_id - 1][1][1][0] > 0):
            min_3d_x = pj[drawing_id - 1][0][1][0]
            max_3d_x = newx
        else:
            min_3d_x = newx
            max_3d_x = pj[drawing_id - 1][0][1][0]

        # 计算比例并转换为真实x坐标
        real_101 = []
        for point in filtered_101:
            # 计算点在二维杆件上的x坐标比例
            x_ratio = (point[0] - min_2d_x) / (max_2d_x - min_2d_x)
            # 根据比例计算真实x坐标
            real_x = min_3d_x + x_ratio * (max_3d_x - min_3d_x)
            real_101.append(real_x)

        min_2d_x = rod_103_points[0][0]
        max_2d_x = rod_103_points[1][0]
        if (pj[drawing_id - 1][0][1][0] > 0):
            min_3d_x = pj[drawing_id - 1][1][1][0]
            max_3d_x = newx
        else:
            min_3d_x = newx
            max_3d_x = pj[drawing_id - 1][1][1][0]

        # 计算比例并转换为真实x坐标
        real_103 = []
        for point in filtered_103:
            # 计算点在二维杆件上的x坐标比例
            x_ratio = (point[0] - min_2d_x) / (max_2d_x - min_2d_x)
            # 根据比例计算真实x坐标
            real_x = min_3d_x + x_ratio * (max_3d_x - min_3d_x)
            real_103.append(real_x)
        min_len = min(len(real_101), len(real_103))

        # 为交点创建节点
        node_101_nodes = []
        node_103_nodes = []

        # 为101杆件上的交点创建节点
        for i, point in enumerate(real_101):
            node_id = f"{drawing_id}101{i + 1}0"
            node_101_nodes.append(node_id)
            jiedian.append({
                "node_id": node_id,
                "node_type": 12,
                "symmetry_type": 2,
                "X": point,
                "Y": f"1{pj[drawing_id - 1][0][0]}",
                "Z": f"1{node1_id + 20}"
            })

        # 为103杆件上的交点创建节点
        for i, point in enumerate(real_103):
            node_id = f"{drawing_id}103{i + 1}0"
            node_103_nodes.append(node_id)
            jiedian.append({
                "node_id": node_id,
                "node_type": 12,
                "symmetry_type": 2,
                "X": point,
                "Y": f"1{pj[drawing_id - 1][1][0]}",
                "Z": f"1{node1_id + 20}"
            })

            # 将前视图的杆件编号提取出来放到一个数组中，并从小到大排序
        Front_ganjian_ID_list = sorted(coordinatesFront_data.keys())
        member_id_base = mi  # 新杆件的基准编号
        index_address = Front_ganjian_ID_list.index(member_id_base)  # 基准杆件在杆件数组中的索引位置
        current_offset = 0  # 用于依次取后续编号

        ganjian.append({
            "member_id": f"{member_id_base}",
            "node1_id": pj[drawing_id - 1][0][0],
            "node2_id": node_103_nodes[0],
            "symmetry_type": 2
        })

        for i in range(min_len):
            # 连接101杆件上的第i个交点与103杆件上的第i个交点
            if i < len(node_101_nodes) and i < len(node_103_nodes):
                current_offset += 1
                ganjian.append({
                    "member_id": str(Front_ganjian_ID_list[index_address + current_offset]),
                    "node1_id": node_101_nodes[i],
                    "node2_id": node_103_nodes[i],
                    "symmetry_type": 2
                })

            # 连接103杆件上的第i+1个交点与101杆件上的第i个交点
            if i + 1 < len(node_103_nodes) and i < len(node_101_nodes):
                current_offset += 1
                ganjian.append({
                    "member_id": str(Front_ganjian_ID_list[index_address + current_offset]),
                    "node1_id": node_103_nodes[i + 1],
                    "node2_id": node_101_nodes[i],
                    "symmetry_type": 2
                })



        #顶视图
        intersections_102.clear()

        rod_101_points = coordinatesOverhead_data[rod_101_id]
        rod_102_points = coordinatesOverhead_data[rod_102_id]

        # 提取参考杆件的所有端点
        ref_points_101 = rod_101_points
        ref_points_102 = rod_102_points

        mi = 199999
        # 遍历所有杆件
        for rod_id, points in coordinatesOverhead_data.items():
            if rod_id in [rod_101_id, rod_102_id]:
                continue
            flag1 = 0
            for i, point in enumerate(points):
                distance_to_101 = dist_point_to_line(point, ref_points_101[0], ref_points_101[1])
                distance_to_102 = dist_point_to_line(point, ref_points_102[0], ref_points_102[1])
                if (distance_to_102 < yuzhi or distance_to_101 < yuzhi):
                    flag1 += 1
            if flag1 == 2:
                mi = min(mi,rod_id)
                intersection_102 = line_intersection(points, rod_102_points)
                if intersection_102 is not None:
                    intersections_102.append(intersection_102)

        if (pj[drawing_id - 1][1][1][0] > 0):
            intersections_102.sort(key=lambda point: point[0])
        else:
            intersections_102.sort(key=lambda point: point[0], reverse=True)
        node_102 = cluster_points(intersections_102, threshold=150.0)
        filtered_102 = remove_endpoint_clusters(node_102, rod_102_points)
        min_2d_x = rod_102_points[0][0]
        max_2d_x = rod_102_points[1][0]
        if (pj[drawing_id - 1][1][1][0] > 0):
            min_3d_x = pj[drawing_id - 1][0][1][0]
            max_3d_x = newx
        else:
            min_3d_x = newx
            max_3d_x = pj[drawing_id - 1][0][1][0]

        # 计算比例并转换为真实x坐标
        real_102 = []
        for point in filtered_102:
            # 计算点在二维杆件上的x坐标比例
            x_ratio = (point[0] - min_2d_x) / (max_2d_x - min_2d_x)
            # 根据比例计算真实x坐标
            real_x = min_3d_x + x_ratio * (max_3d_x - min_3d_x)
            real_102.append(real_x)

        node_102_ids = []
        for i, point in enumerate(real_102):
            node_id = f"{drawing_id}201{i + 1}0"
            node_102_ids.append(node_id)
            duichenzuo = int(pj[drawing_id - 1][0][0]) + 2
            jiedian.append({
                "node_id": node_id,
                "node_type": 12,
                "symmetry_type": 2,
                "X": point,
                "Y": f"1{duichenzuo}",
                "Z": f"1{node1_id + 22}"
            })
        member_id_base = mi  # 新杆件的基准编号
        ganjian.append({
            "member_id": f"{member_id_base}",
            "node1_id": pj[drawing_id - 1][0][0],
            "node2_id": node_102_ids[0],
            "symmetry_type": 2
        })
        for i in range(0, len(real_102) - 1):
            if i % 2 == 0:  # 偶数索引：交点与下一个对称点相连
                ganjian.append({
                    "member_id": f"{member_id_base + i * 2 + 2}",
                    "node1_id": f"{drawing_id}201{i + 2}2",
                    "node2_id": node_102_ids[i],
                    "symmetry_type": 2
                })
            else:  # 奇数索引：下一个交点与对称点相连
                ganjian.append({
                    "member_id": f"{member_id_base + i * 2 + 2}",
                    "node1_id": node_102_ids[i + 1],
                    "node2_id": f"{drawing_id}201{i + 1}2",
                    "symmetry_type": 2
                })
        ganjian.append({
            "member_id": f"{member_id_base + (len(real_102) - 1) * 2 + 2}",
            "node1_id": f"{node1_id + 20}",
            "node2_id": node_102_ids[len(real_102) - 1],
            "symmetry_type": 2
        })



        #仰视图
        intersections_103.clear()

        rod_103_points = coordinatesBottom_data[rod_103_id]
        rod_104_points = coordinatesBottom_data[rod_104_id]

        # 提取参考杆件的所有端点
        ref_points_103 = rod_103_points
        ref_points_104 = rod_104_points

        mi = 199999
        # 遍历所有杆件
        for rod_id, points in coordinatesBottom_data.items():
            if rod_id in [rod_103_id, rod_104_id]:
                continue
            flag1 = 0
            for i, point in enumerate(points):
                distance_to_103 = dist_point_to_line(point, ref_points_103[0], ref_points_103[1])
                distance_to_104 = dist_point_to_line(point, ref_points_104[0], ref_points_104[1])
                if (distance_to_104 < yuzhi or distance_to_103 < yuzhi):
                    flag1 += 1
            if flag1 == 2:
                mi = min(mi,rod_id)
                intersection_103 = line_intersection(points, rod_103_points)
                if intersection_103 is not None:
                    intersections_103.append(intersection_103)

        if (pj[drawing_id - 1][1][1][0] > 0):
            intersections_103.sort(key=lambda point: point[0])
        else:
            intersections_103.sort(key=lambda point: point[0], reverse=True)
        node_103 = cluster_points(intersections_103)
        filtered_103 = remove_endpoint_clusters(node_103, rod_103_points)
        min_2d_x = rod_103_points[0][0]
        max_2d_x = rod_103_points[1][0]
        if (pj[drawing_id - 1][1][1][0] > 0):
            min_3d_x = pj[drawing_id - 1][0][1][0]
            max_3d_x = newx
        else:
            min_3d_x = newx
            max_3d_x = pj[drawing_id - 1][0][1][0]

        # 计算比例并转换为真实x坐标
        real_103 = []
        for point in filtered_103:
            # 计算点在二维杆件上的x坐标比例
            x_ratio = (point[0] - min_2d_x) / (max_2d_x - min_2d_x)
            # 根据比例计算真实x坐标
            real_x = min_3d_x + x_ratio * (max_3d_x - min_3d_x)
            real_103.append(real_x)

        node_103_ids = []
        for i, point in enumerate(real_103):
            node_id = f"{drawing_id}301{i + 1}0"
            node_103_ids.append(node_id)
            duichenzuo = int(pj[drawing_id - 1][0][0]) + 2
            jiedian.append({
                "node_id": node_id,
                "node_type": 12,
                "symmetry_type": 2,
                "X": point,
                "Y": f"1{duichenzuo}",
                "Z": f"1{node1_id + 22}"
            })
        member_id_base = mi  # 新杆件的基准编号
        ganjian.append({
            "member_id": f"{member_id_base}",
            "node1_id": pj[drawing_id - 1][1][0],
            "node2_id": node_103_ids[0],
            "symmetry_type": 2
        })
        for i in range(0, len(real_103) - 1):
            if i % 2 == 0:  # 偶数索引
                ganjian.append({
                    "member_id": f"{member_id_base + i * 2 + 2}",
                    "node1_id": f"{drawing_id}301{i + 2}2",
                    "node2_id": node_103_ids[i],
                    "symmetry_type": 2
                })
            else:  # 奇数索引
                ganjian.append({
                    "member_id": f"{member_id_base + i * 2 + 2}",
                    "node1_id": node_103_ids[i + 1],
                    "node2_id": f"{drawing_id}301{i + 1}2",
                    "symmetry_type": 2
                })
        ganjian.append({
            "member_id": f"{member_id_base + (len(real_103) - 1) * 2 + 2}",
            "node1_id": f"{node1_id + 20}",
            "node2_id": node_103_ids[len(real_103) - 1],
            "symmetry_type": 2
        })



        new_ganjian = {
           "member_id": f"{drawing_id * 100 + 1}",
           "node1_id": pj[drawing_id - 1][0][0],
           "node2_id": f"{node1_id + 20}",
            "symmetry_type": 2
        }
        ganjian.append(new_ganjian)
        new_ganjian = {
            "member_id": f"{drawing_id * 100 + 3}",
            "node1_id": pj[drawing_id - 1][1][0],
            "node2_id": f"{node1_id + 20}",
            "symmetry_type": 2
        }
        ganjian.append(new_ganjian)
        new_ganjian = {
            "member_id": f"{drawing_id * 100 + l1}",
            "node1_id": f"{node1_id + 20}",
            "node2_id": f"{node1_id + 22}",
            "symmetry_type": 0
        }
        ganjian.append(new_ganjian)

def work(file_path, data):
    for i in range(1,5):
        specific_file_path = f"{file_path}\\0{i}.txt"
        trans(specific_file_path, i, data)
    return jiedian, ganjian

