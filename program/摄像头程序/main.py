"""
全自动几何拼矩形算法 - MaixCAM (MaixPy v4)  
"""

import cv2
import numpy as np
import math
from maix import camera, display, image, time, uart

# ====================== 可调参数 ======================
CAM_W       = 800
CAM_H       = 600
GAUSS_K     = (5, 5)

AREA_MIN    = 500
AREA_MAX    = 60000
A4_AREA_MIN = 30000

A4_WIDTH_MM  = 297.0
A4_HEIGHT_MM = 210.0
A4_HALF_X    = 148.5

SAT_DIFF_THRESH = 40
EDGE_MARGIN = 4
MORPH_SIZE  = 7
SKIP_FRAME  = 0

DRAW_PIECES_TOGGLE = {'#1': True, '#2': True, '#3': True, '#4': True}
DRAW_COMBINED_OUTLINE_TOGGLE = True
 
RIGHT_ZONE_LEFT_LIMIT = 158.0     
RIGHT_ZONE_RIGHT_LIMIT = 287.0    
RIGHT_ZONE_TOP_LIMIT = 15.0       
RIGHT_ZONE_BOTTOM_LIMIT = 195.0   
# =====================================================


def order_points(pts):
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect


def find_a4_paper_smart(img_np):
    hsv = cv2.cvtColor(img_np, cv2.COLOR_RGB2HSV)
    sat = hsv[:, :, 1]
    blur_sat = cv2.GaussianBlur(sat, GAUSS_K, 0)
    _, th_a4 = cv2.threshold(blur_sat, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    line_bridge_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (35, 3))
    th_a4_bridged = cv2.morphologyEx(th_a4, cv2.MORPH_CLOSE, line_bridge_kernel)
    contours, _ = cv2.findContours(th_a4_bridged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in sorted(contours, key=cv2.contourArea, reverse=True):
        area = cv2.contourArea(cnt)
        if area < A4_AREA_MIN:
            continue
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.025 * peri, True)
        if len(approx) == 4:
            pts = approx.reshape(4, 2)
            return order_points(pts)
    return None


def enforce_ccw(pts):
    area = 0
    n = len(pts)
    for i in range(n):
        area += pts[i][0] * pts[(i+1)%n][1] - pts[(i+1)%n][0] * pts[i][1]
    if area > 0:
        return list(reversed(pts))
    return pts


def wrap_angle_180(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return round(float(angle), 1)


def extract_edges_and_vertex_angles(pts, right_angle_thresh=5.0):
    n = len(pts)
    if n < 3:
        return []
    v_angles = []

    for i in range(n):
        p_prev = np.array(pts[i - 1], dtype=np.float32)
        p_curr = np.array(pts[i], dtype=np.float32)
        p_next = np.array(pts[(i + 1) % n], dtype=np.float32)

        v1 = p_prev - p_curr
        v2 = p_next - p_curr
        norm1 = np.linalg.norm(v1)
        norm2 = np.linalg.norm(v2)
        if norm1 < 1e-3 or norm2 < 1e-3:
            v_angles.append(90.0)
            continue

        cos_a = np.clip(np.dot(v1, v2) / (norm1 * norm2), -1.0, 1.0)
        v_angles.append(round(float(np.degrees(np.arccos(cos_a))), 1))

    edges = []
    for i in range(n):
        p1 = pts[i]
        p2 = pts[(i + 1) % n]
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        length = math.hypot(dx, dy)
        angle = math.degrees(math.atan2(dy, dx))

        ang1, ang2 = v_angles[i], v_angles[(i + 1) % n]
        is_right_edge = (abs(ang1 - 90.0) <= right_angle_thresh) or (abs(ang2 - 90.0) <= right_angle_thresh)

        edges.append({
            'edge_idx': i,
            'len': round(length, 1),
            'angle': angle,
            'p1': (round(float(p1[0]), 1), round(float(p1[1]), 1)),
            'p2': (round(float(p2[0]), 1), round(float(p2[1]), 1)),
            'ang1': ang1,
            'ang2': ang2,
            'is_right': is_right_edge,
            'matched': False
        })
    return edges


def calc_base_correct_rotation(edges):
    n = len(edges)
    for i in range(n):
        ang = edges[i]['ang1']
        if abs(ang - 90.0) <= 5.0:
            e_in = edges[i - 1]
            e_out = edges[i]
            
            v_in_angle = wrap_angle_180(e_in['angle'] + 180.0)
            v_out_angle = e_out['angle']
            
            rot_a = wrap_angle_180(0.0 - v_out_angle)
            v_in_rot = wrap_angle_180(v_in_angle + rot_a)
            if abs(v_in_rot - (-90.0)) > 15.0:
                rot_a = wrap_angle_180(-90.0 - v_out_angle)

            return i, rot_a, e_in['len'], e_out['len']
            
    return None, 0.0, 0.0, 0.0


def is_combined_shape_valid_rectangle(assembled_pieces, M_inv):

    assembled_true_pieces = [p for p in assembled_pieces if p.get('is_assembled', False)]
    if len(assembled_true_pieces) < 2:
        return False

    if M_inv is None:
        return len(assembled_true_pieces) >= 2
 
    total_pieces_area_mm2 = 0.0
    all_px_pts = []

    for item in assembled_true_pieces:
        pts_mm = item.get('placed_pts', [])
        if len(pts_mm) >= 3:
            pts_mm_np = np.array(pts_mm, dtype=np.float32).reshape(-1, 1, 2)
            pts_px_np = cv2.perspectiveTransform(pts_mm_np, M_inv).reshape(-1, 2).astype(int)
            all_px_pts.append(pts_px_np)

            contour_mm_np = np.array(pts_mm, dtype=np.float32)
            total_pieces_area_mm2 += cv2.contourArea(contour_mm_np)

    if not all_px_pts or total_pieces_area_mm2 == 0:
        return False
 
    mask_canvas = np.zeros((CAM_H, CAM_W), dtype=np.uint8)
    for pts in all_px_pts:
        cv2.fillPoly(mask_canvas, [pts], 255)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask_canvas = cv2.morphologyEx(mask_canvas, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask_canvas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False

    main_cnt = max(contours, key=cv2.contourArea)
    cnt_area_px = cv2.contourArea(main_cnt)
 
    rect_px = cv2.minAreaRect(main_cnt)
    box_px_area = rect_px[1][0] * rect_px[1][1]
    
    if box_px_area == 0:
        return False
 
    solidity = cnt_area_px / box_px_area
    if solidity < 0.78:
        return False
 
    box_px = cv2.boxPoints(rect_px)
    box_px = order_points(box_px)

    M_trans = cv2.invert(M_inv)[1]
    box_px_float = box_px.astype("float32").reshape(-1, 1, 2)
    pts_mm_4 = cv2.perspectiveTransform(box_px_float, M_trans).reshape(-1, 2)

    side_a = np.linalg.norm(pts_mm_4[0] - pts_mm_4[1])
    side_b = np.linalg.norm(pts_mm_4[1] - pts_mm_4[2])
    bounding_box_area_mm2 = side_a * side_b
 
    area_diff_ratio = abs(bounding_box_area_mm2 - total_pieces_area_mm2) / total_pieces_area_mm2

    if area_diff_ratio <= 0.15:
        return True

    return False


def solve_fourth_piece_with_outline(combined_edges, target_piece):
    if not combined_edges or not target_piece:
        return 0.0, target_piece['x_mm'], target_piece['y_mm'], []

    p4_pts = enforce_ccw(target_piece['contour_mm'])
    p4_edges = extract_edges_and_vertex_angles(p4_pts, right_angle_thresh=5.0)

    if not p4_edges:
        return 0.0, target_piece['x_mm'], target_piece['y_mm'], []

    target_edges_non_right = [e for e in combined_edges if e['len'] >= 10.0 and not e.get('is_right', False)]
    target_edges_right = [e for e in combined_edges if e['len'] >= 10.0 and e.get('is_right', False)]

    target_edges_non_right.sort(key=lambda x: x['len'], reverse=True)
    target_edges_right.sort(key=lambda x: x['len'], reverse=True)

    p4_edges_non_right = [e for e in p4_edges if not e.get('is_right', False)]
    p4_edges_right = [e for e in p4_edges if e.get('is_right', False)]

    p4_edges_non_right.sort(key=lambda x: x['len'], reverse=True)
    p4_edges_right.sort(key=lambda x: x['len'], reverse=True)

    best_match = None
    min_cost = float('inf')

    match_candidate_pairs = [
        (target_edges_non_right, p4_edges_non_right, 0),
        (target_edges_non_right, p4_edges, 50),
        (target_edges_right, p4_edges_non_right, 100),
        (target_edges_right, p4_edges_right, 150)
    ]

    for t_list, u_list, tier_penalty in match_candidate_pairs:
        for te in t_list:
            for ue in u_list:
                len_diff = abs(te['len'] - ue['len'])
                if len_diff > 8.0:
                    continue

                ang_sum_1 = abs((te['ang1'] + ue['ang2']) - 180.0)
                ang_sum_2 = abs((te['ang2'] + ue['ang1']) - 180.0)
                complement_cost = min(ang_sum_1, ang_sum_2)

                if complement_cost > 22.0:
                    continue

                cost = tier_penalty + len_diff * 3.0 + complement_cost - (te['len'] * 0.2)

                if cost < min_cost:
                    min_cost = cost
                    best_match = (te, ue)

    if best_match is not None:
        te, ue = best_match
        final_rot = wrap_angle_180(te['angle'] - ue['angle'] + 180.0)

        rad = math.radians(final_rot)
        cos_a, sin_a = math.cos(rad), math.sin(rad)

        cx, cy = target_piece['x_mm'], target_piece['y_mm']
        m1x, m1y = ue['p1']

        m1_rot_x = cx + (m1x - cx) * cos_a - (m1y - cy) * sin_a
        m1_rot_y = cy + (m1x - cx) * sin_a + (m1y - cy) * cos_a

        s_x = te['p2'][0] - m1_rot_x
        s_y = te['p2'][1] - m1_rot_y

        final_cx = round(cx + s_x, 1)
        final_cy = round(cy + s_y, 1)

        new_placed_pts = []
        for p in p4_pts:
            px_rot = cx + (p[0] - cx) * cos_a - (p[1] - cy) * sin_a + s_x
            py_rot = cy + (p[0] - cx) * sin_a + (p[1] - cy) * cos_a + s_y
            new_placed_pts.append((round(px_rot, 1), round(py_rot, 1)))

        return final_rot, final_cx, final_cy, new_placed_pts

    return 0.0, target_piece['x_mm'], target_piece['y_mm'], p4_pts


def find_all_right_angle_bases(p):
    pts = enforce_ccw(p['contour_mm'])
    edges = extract_edges_and_vertex_angles(pts, right_angle_thresh=5.0)
    n = len(edges)
    
    candidate_rotations = []
    for i in range(n):
        ang = edges[i]['ang1']
        if abs(ang - 90.0) <= 5.0:
            e_in = edges[i - 1]
            e_out = edges[i]
            
            v_in_angle = wrap_angle_180(e_in['angle'] + 180.0)
            v_out_angle = e_out['angle']
            
            rot_a = wrap_angle_180(0.0 - v_out_angle)
            v_in_rot = wrap_angle_180(v_in_angle + rot_a)
            if abs(v_in_rot - (-90.0)) > 15.0:
                rot_a = wrap_angle_180(-90.0 - v_out_angle)
            
            candidate_rotations.append({
                'vertex_idx': i,
                'rot_angle': rot_a,
                'edges': edges
            })
            
    return candidate_rotations


def relocate_assembled_pieces_to_right_zone(assembled_pieces):
    valid_pieces = [p for p in assembled_pieces if p.get('is_assembled', False)]
    if not valid_pieces:
        return assembled_pieces

    all_x_coords = []
    all_y_coords = []
    for item in valid_pieces:
        pts = item.get('placed_pts', [])
        for px, py in pts:
            all_x_coords.append(px)
            all_y_coords.append(py)

    if not all_x_coords:
        return assembled_pieces

    min_x = min(all_x_coords)
    max_x = max(all_x_coords)
    min_y = min(all_y_coords)
    max_y = max(all_y_coords)

    shift_x = 0.0
    shift_y = 0.0

    if min_x < RIGHT_ZONE_LEFT_LIMIT:
        shift_x = RIGHT_ZONE_LEFT_LIMIT - min_x
        if max_x + shift_x > RIGHT_ZONE_RIGHT_LIMIT:
            shift_x = ((RIGHT_ZONE_LEFT_LIMIT + RIGHT_ZONE_RIGHT_LIMIT) / 2.0) - ((min_x + max_x) / 2.0)

    if min_y < RIGHT_ZONE_TOP_LIMIT:
        shift_y = RIGHT_ZONE_TOP_LIMIT - min_y
    elif max_y > RIGHT_ZONE_BOTTOM_LIMIT:
        shift_y = RIGHT_ZONE_BOTTOM_LIMIT - max_y

    if abs(shift_x) > 0.1 or abs(shift_y) > 0.1:
        for item in assembled_pieces:
            if 'final_cx' in item:
                item['final_cx'] = round(item['final_cx'] + shift_x, 1)
            if 'final_cy' in item:
                item['final_cy'] = round(item['final_cy'] + shift_y, 1)
            
            if 'placed_pts' in item and item['placed_pts']:
                new_pts = []
                for px, py in item['placed_pts']:
                    new_pts.append((round(px + shift_x, 1), round(py + shift_y, 1)))
                item['placed_pts'] = new_pts
                item['placed_edges'] = extract_edges_and_vertex_angles(new_pts, right_angle_thresh=5.0)

    return assembled_pieces


def run_single_stitching_pass(upper_pieces, relax_mask):
    targets = []
    if not upper_pieces:
        return targets, []

    unplaced_raw = []
    for p in upper_pieces:
        pts = p["geo"]["contour_mm"]
        if len(pts) >= 3:
            unplaced_raw.append({
                "id": p["id"],
                "x_mm": p["x_mm"],
                "y_mm": p["y_mm"],
                "contour_mm": pts,
                "is_assembled": False
            })
    if not unplaced_raw:
        return targets, []

    error_stages = [
        {'max_len_diff': 3.0, 'max_ang_diff': 3.0},
        {'max_len_diff': 5.0, 'max_ang_diff': 6.0},
        {'max_len_diff': 6.0, 'max_ang_diff': 8.0},
        {'max_len_diff': 8.0, 'max_ang_diff': 10.0}
    ]

    candidate_bases = []
    for idx, p in enumerate(unplaced_raw):
        right_angle_configs = find_all_right_angle_bases(p)
        for config in right_angle_configs:
            candidate_bases.append({
                'raw_idx': idx,
                'piece': p,
                'edges': config['edges'],
                'rot_angle': config['rot_angle'],
                'v_idx': config['vertex_idx']
            })

    selected_base_idx = -1
    base_info = None

    for cand in candidate_bases:
        c_idx = cand['raw_idx']
        cand_p = cand['piece']
        cand_edges = cand['edges']
        rot_b = cand['rot_angle']

        other_pieces = [p for i, p in enumerate(unplaced_raw) if i != c_idx]
        non_right_edges = [e for e in cand_edges if not e.get('is_right', False)]
        
        can_fit_next = False
        for oth in other_pieces:
            oth_pts = enforce_ccw(oth['contour_mm'])
            oth_edges = extract_edges_and_vertex_angles(oth_pts, right_angle_thresh=5.0)
            
            oth_id_idx = oth['id'] - 1
            allow_oth_right = relax_mask[oth_id_idx] if oth_id_idx < len(relax_mask) else False
            
            oth_candidate_edges = [e for e in oth_edges if (not e.get('is_right', False)) or allow_oth_right]
            
            for te in non_right_edges:
                for ue in oth_candidate_edges:
                    len_diff = abs(te['len'] - ue['len'])
                    ang_sum_1 = abs((te['ang1'] + ue['ang2']) - 180.0)
                    ang_sum_2 = abs((te['ang2'] + ue['ang1']) - 180.0)
                    complement_cost = min(ang_sum_1, ang_sum_2)

                    for stage in error_stages:
                        if len_diff <= stage['max_len_diff'] and complement_cost <= stage['max_ang_diff']:
                            can_fit_next = True
                            break
                    if can_fit_next:
                        break
                if can_fit_next:
                    break
            if can_fit_next:
                break

        if can_fit_next:
            selected_base_idx = c_idx
            base_info = (cand_p, cand_edges, rot_b)
            break

    if base_info is None:
        if candidate_bases:
            selected_base_idx = candidate_bases[0]['raw_idx']
            base_info = (candidate_bases[0]['piece'], candidate_bases[0]['edges'], candidate_bases[0]['rot_angle'])
        else:
            selected_base_idx = 0
            base_p = unplaced_raw[0]
            b_pts = enforce_ccw(base_p['contour_mm'])
            b_edges = extract_edges_and_vertex_angles(b_pts, right_angle_thresh=5.0)
            _, rot_b, _, _ = calc_base_correct_rotation(b_edges)
            base_info = (base_p, b_edges, rot_b)

    base = base_info[0]
    rot_base = base_info[2]
    unplaced = [p for i, p in enumerate(unplaced_raw) if i != selected_base_idx]

    base_pts = enforce_ccw(base['contour_mm'])
    target_base_cx, target_base_cy = 200.0, 110.0 

    rot_rad = math.radians(rot_base)
    cos_b, sin_b = math.cos(rot_rad), math.sin(rot_rad)
    bcx, bcy = base['x_mm'], base['y_mm']

    rot_base_pts = []
    for p in base_pts:
        px = bcx + (p[0] - bcx) * cos_b - (p[1] - bcy) * sin_b
        py = bcy + (p[0] - bcx) * sin_b + (p[1] - bcy) * cos_b
        rot_base_pts.append((px, py))

    shift_x = target_base_cx - bcx
    shift_y = target_base_cy - bcy
    placed_pts = [(p[0] + shift_x, p[1] + shift_y) for p in rot_base_pts]

    base['placed_pts'] = placed_pts
    base['placed_edges'] = extract_edges_and_vertex_angles(placed_pts, right_angle_thresh=5.0)
    base['rot_relative'] = rot_base
    base['final_cx'] = target_base_cx
    base['final_cy'] = target_base_cy
    base['is_assembled'] = True
    assembled = [base]

    total_piece_count = len(upper_pieces)

    while unplaced:
        true_assembled_cnt = len([p for p in assembled if p.get('is_assembled', False)])
        if true_assembled_cnt >= total_piece_count - 1 and total_piece_count >= 3:
            last_piece = unplaced.pop(0)
            last_pts = enforce_ccw(last_piece['contour_mm'])
            last_piece['placed_pts'] = last_pts
            last_piece['placed_edges'] = extract_edges_and_vertex_angles(last_pts, right_angle_thresh=5.0)
            last_piece['rot_relative'] = 0.0
            last_piece['final_cx'] = last_piece['x_mm']
            last_piece['final_cy'] = last_piece['y_mm']
            last_piece['is_assembled'] = False  
            assembled.append(last_piece)
            break

        best_match = None
        min_cost = float('inf')

        for ass_p in assembled:
            if not ass_p.get('is_assembled', False):
                continue

            target_edges = [e for e in ass_p.get('placed_edges', []) if not e['matched']]
            target_edges.sort(key=lambda x: (x.get('is_right', False), -x['len']))

            for i, up in enumerate(unplaced):
                up_pts = enforce_ccw(up['contour_mm'])
                up_edges = extract_edges_and_vertex_angles(up_pts, right_angle_thresh=5.0)
                up_edges.sort(key=lambda x: (x.get('is_right', False), -x['len']))

                up_id_idx = up['id'] - 1
                allow_up_right = relax_mask[up_id_idx] if up_id_idx < len(relax_mask) else False

                for te in target_edges:
                    for ue in up_edges:
                        len_diff = abs(te['len'] - ue['len'])

                        is_te_right = te.get('is_right', False)
                        is_ue_right = ue.get('is_right', False)

                        if len_diff <= 8.0:
                            ang_sum_1 = abs((te['ang1'] + ue['ang2']) - 180.0)
                            ang_sum_2 = abs((te['ang2'] + ue['ang1']) - 180.0)
                            complement_cost = min(ang_sum_1, ang_sum_2)

                            if (is_te_right and not allow_up_right) or (is_ue_right and not allow_up_right):
                                 continue

                            matched_stage_level = None
                            for stage_idx, stage in enumerate(error_stages):
                                if len_diff <= stage['max_len_diff'] and complement_cost <= stage['max_ang_diff']:
                                    matched_stage_level = stage_idx
                                    break

                            if matched_stage_level is None:
                                continue

                            cost = matched_stage_level * 100.0 + len_diff * 3.0 + complement_cost - (te['len'] * 0.1)
                            if cost < min_cost:
                                min_cost = cost
                                best_match = (i, te, ue, up, up_pts, up_edges)
                        else:
                            if (is_te_right and not allow_up_right) or (is_ue_right and not allow_up_right):
                                continue

        if best_match:
            i, te, ue, up, up_pts, up_edges = best_match
            unplaced.pop(i)

            rot_angle = wrap_angle_180(te['angle'] - ue['angle'] + 180.0)

            rad = math.radians(rot_angle)
            cos_a, sin_a = math.cos(rad), math.sin(rad)

            cx, cy = up['x_mm'], up['y_mm']
            m1x, m1y = ue['p1']

            m1_rot_x = cx + (m1x - cx) * cos_a - (m1y - cy) * sin_a
            m1_rot_y = cy + (m1x - cx) * sin_a + (m1y - cy) * cos_a

            s_x = te['p2'][0] - m1_rot_x
            s_y = te['p2'][1] - m1_rot_y

            final_cx = cx + s_x
            final_cy = cy + s_y

            new_placed_pts = []
            for p in up_pts:
                px_rot = cx + (p[0] - cx) * cos_a - (p[1] - cy) * sin_a + s_x
                py_rot = cy + (p[0] - cx) * sin_a + (p[1] - cy) * cos_a + s_y
                new_placed_pts.append((round(px_rot, 1), round(py_rot, 1)))

            up['placed_pts'] = new_placed_pts
            up_placed_edges = extract_edges_and_vertex_angles(new_placed_pts, right_angle_thresh=5.0)

            te['matched'] = True
            ue_idx = up_edges.index(ue)
            if ue_idx < len(up_placed_edges):
                up_placed_edges[ue_idx]['matched'] = True

            up['placed_edges'] = up_placed_edges
            up['rot_relative'] = rot_angle
            up['final_cx'] = final_cx
            up['final_cy'] = final_cy
            up['is_assembled'] = True
            assembled.append(up)
        else:
            up = unplaced.pop(0)
            up_pts = enforce_ccw(up['contour_mm'])
            up['placed_pts'] = up_pts
            up['placed_edges'] = extract_edges_and_vertex_angles(up_pts, right_angle_thresh=5.0)
            up['rot_relative'] = 0.0
            up['final_cx'] = up['x_mm']
            up['final_cy'] = up['y_mm']
            up['is_assembled'] = False
            assembled.append(up)

    assembled = relocate_assembled_pieces_to_right_zone(assembled)

    for item in assembled:
        final_rot = wrap_angle_180(item.get('rot_relative', 0.0))
        targets.append({
            'id': item['id'],
            'p1': (item['x_mm'], item['y_mm']),
            'angle_diff': final_rot,
            'p2': (round(item.get('final_cx', target_base_cx), 1),
                   round(item.get('final_cy', target_base_cy), 1))
        })

    targets.sort(key=lambda x: x['id'])
    return targets, assembled


def solve_rectangle_stitching_puzzle(upper_pieces, M_inv=None):
    if not upper_pieces:
        return [], []

    num_pieces = len(upper_pieces)
    
    strategies = [
        [False] * 8,                       
        [False, True] + [False] * 6,       
        [False, True, True] + [False] * 5, 
        [False] + [True] * 7               
    ]

    best_plan, best_assembled = None, None

    for pass_idx, mask in enumerate(strategies):
        targets, assembled = run_single_stitching_pass(upper_pieces, mask)
        
        update_fourth_piece_solution(assembled, M_inv)
        
        if is_combined_shape_valid_rectangle(assembled, M_inv):
            best_plan, best_assembled = targets, assembled
            break
        else:
            best_plan, best_assembled = targets, assembled
 
    for item in best_assembled:
        item_id = item['id']
        edges = item.get('placed_edges', [])
        num_edges = len(edges)
        
        total_internal_angle = sum([e['ang1'] for e in edges])
        final_rot = wrap_angle_180(item.get('rot_relative', 0.0))
        p2_x = item.get('final_cx', 175.0)
        p2_y = item.get('final_cy', 150.0) 
        
        for e in edges:
            e_idx = e['edge_idx']
            p1_str = f"({e['p1'][0]:.1f}, {e['p1'][1]:.1f})"
            p2_str = f"({e['p2'][0]:.1f}, {e['p2'][1]:.1f})" 

    # 串口/控制协议对应的姿态标准格式输出
    print("------------------- [控制协议标准姿态数据] -------------------")
    sorted_assembled = sorted(best_assembled, key=lambda x: x['id'])
    for item in sorted_assembled:
        nid = item['id']
        p1_x = round(item['x_mm'] / 10.0, 1)
        p1_y = round(item['y_mm'] / 10.0, 1)
        p2_x = round(item.get('final_cx', 175.0) / 10.0, 1)
        p2_y = round(item.get('final_cy', 150.0) / 10.0, 1)
        angle = wrap_angle_180(-item.get('rot_relative', 0.0))
        
        print(f"N:{nid},P1:({p1_x:.1f},{p1_y:.1f}),A:{angle:+.1f},P2:({p2_x:.1f},{p2_y:.1f})")
    print("--------------------------------------------------------------\n")

    return best_plan, best_assembled


def analyze_piece_geometry_directed(cnt, M_trans):
    peri = cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, 0.015 * peri, True)
    pts_px = approx.reshape(-1, 2)

    if M_trans is None or len(pts_px) < 3:
        return {"contour_mm": []}

    pts_px_float = pts_px.astype("float32").reshape(-1, 1, 2)
    pts_mm = cv2.perspectiveTransform(pts_px_float, M_trans).reshape(-1, 2)

    return {"contour_mm": pts_mm.tolist()}


def process_frame_fast(maix_img):
    h = maix_img.height()
    w = maix_img.width()

    raw_bytes = maix_img.to_bytes()
    img_np = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((h, w, 3))

    a4_corners = find_a4_paper_smart(img_np)
    M_trans = None
    M_inv = None

    if a4_corners is not None:
        src_pts = a4_corners.astype("float32")
        dst_pts = np.array([
            [0, 0],
            [A4_WIDTH_MM, 0],
            [A4_WIDTH_MM, A4_HEIGHT_MM],
            [0, A4_HEIGHT_MM]
        ], dtype="float32")
        M_trans = cv2.getPerspectiveTransform(src_pts, dst_pts)
        M_inv = cv2.getPerspectiveTransform(dst_pts, src_pts)

    if a4_corners is not None:
        mask_a4 = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask_a4, [a4_corners.astype(int)], 255)
        kernel_erode = cv2.getStructuringElement(cv2.MORPH_RECT, (EDGE_MARGIN * 2, EDGE_MARGIN * 2))
        mask_a4_inner = cv2.erode(mask_a4, kernel_erode)

        line_mask_mm = np.array([[[144.5, -5.0]], [[152.5, -5.0]],
                                  [[152.5, 215.0]], [[144.5, 215.0]]], dtype="float32")
        line_mask_px = cv2.perspectiveTransform(line_mask_mm, M_inv).astype(int)
        cv2.fillPoly(mask_a4_inner, [line_mask_px], 0)

        hsv = cv2.cvtColor(img_np, cv2.COLOR_RGB2HSV)
        sat = hsv[:, :, 1]
        a4_sat_pixels = sat[mask_a4_inner == 255]
        bg_sat = np.median(a4_sat_pixels) if len(a4_sat_pixels) > 0 else 150
        diff_sat = cv2.absdiff(sat, int(bg_sat))

        blur_diff = cv2.GaussianBlur(diff_sat, GAUSS_K, 0)
        _, th = cv2.threshold(blur_diff, SAT_DIFF_THRESH, 255, cv2.THRESH_BINARY)
        th = cv2.bitwise_and(th, mask_a4_inner)
    else:
        gray = cv2.cvtColor(img_np, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, GAUSS_K, 0)
        _, th = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (MORPH_SIZE, MORPH_SIZE))
    closed = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    raw_results = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < AREA_MIN or area > AREA_MAX:
            continue
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.015 * peri, True)
        if len(approx) > 8 or len(approx) < 3:
            continue

        rect = cv2.minAreaRect(cnt)
        (rcx, rcy), (bw, bh), _ = rect
        if bw > 0 and bh > 0 and (max(bw, bh) / min(bw, bh) > 4.5):
            continue

        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue

        cx_px = int(M["m10"] / M["m00"])
        cy_px = int(M["m01"] / M["m00"])
        x_mm, y_mm = 0.0, 0.0

        if M_trans is not None:
            pt_px = np.array([[[cx_px, cy_px]]], dtype="float32")
            pt_mm = cv2.perspectiveTransform(pt_px, M_trans)
            x_mm = round(float(pt_mm[0][0][0]), 1)
            y_mm = round(float(pt_mm[0][0][1]), 1)

        geo_info = analyze_piece_geometry_directed(cnt, M_trans)

        pts = cnt.reshape(-1, 2)
        box = cv2.boxPoints(rect).astype(int)
        region = "UPPER" if x_mm < A4_HALF_X else "LOWER"

        raw_results.append({
            "cx": cx_px, "cy": cy_px, "x_mm": x_mm, "y_mm": y_mm,
            "area": int(area), "region": region, "geo": geo_info,
            "contour": pts.tolist(), "box": box.tolist(),
        })

    raw_results.sort(key=lambda item: item["area"], reverse=False)

    results = []
    upper_pieces = []
    for pid, item in enumerate(raw_results, start=1):
        item["id"] = pid
        results.append(item)
        if item["region"] == "UPPER":
            upper_pieces.append(item)

    stitch_plan, assembled_pieces = solve_rectangle_stitching_puzzle(upper_pieces, M_inv)
    return results, a4_corners, stitch_plan, assembled_pieces, M_inv


def update_fourth_piece_solution(assembled_pieces, M_inv):
    if M_inv is None or len(assembled_pieces) < 3:
        return

    assembled_true_pieces = [p for p in assembled_pieces if p.get('is_assembled', False)]
    
    if len(assembled_true_pieces) < 2:
        return

    all_right_px_pts = []
    for item in assembled_true_pieces:
        placed_pts_mm = item.get('placed_pts', [])
        if len(placed_pts_mm) >= 3:
            pts_mm_np = np.array(placed_pts_mm, dtype=np.float32).reshape(-1, 1, 2)
            pts_px_np = cv2.perspectiveTransform(pts_mm_np, M_inv).reshape(-1, 2).astype(int)
            all_right_px_pts.append(pts_px_np)

    if len(all_right_px_pts) >= 2:
        mask_canvas = np.zeros((CAM_H, CAM_W), dtype=np.uint8)
        for pts in all_right_px_pts:
            cv2.fillPoly(mask_canvas, [pts], 255)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask_canvas = cv2.morphologyEx(mask_canvas, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask_canvas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            main_cnt = max(contours, key=cv2.contourArea)
            peri = cv2.arcLength(main_cnt, True)
            approx = cv2.approxPolyDP(main_cnt, 0.012 * peri, True)

            pts_px_red = approx.reshape(-1, 2)
            M_trans = cv2.invert(M_inv)[1]
            pts_px_float = pts_px_red.astype("float32").reshape(-1, 1, 2)
            pts_mm_combined = cv2.perspectiveTransform(pts_px_float, M_trans).reshape(-1, 2).tolist()

            combined_pts_ccw = enforce_ccw(pts_mm_combined)
            combined_edges = extract_edges_and_vertex_angles(combined_pts_ccw, right_angle_thresh=5.0)
 
            unplaced_target = None
            for p in assembled_pieces:
                if not p.get('is_assembled', False):
                    unplaced_target = p
                    break

            if unplaced_target is not None:
                rot_4, fcx_4, fcy_4, new_pts_4 = solve_fourth_piece_with_outline(combined_edges, unplaced_target)

                unplaced_target['rot_relative'] = rot_4
                unplaced_target['final_cx'] = fcx_4
                unplaced_target['final_cy'] = fcy_4
                unplaced_target['placed_pts'] = new_pts_4
                unplaced_target['placed_edges'] = extract_edges_and_vertex_angles(new_pts_4, right_angle_thresh=5.0)
                unplaced_target['is_assembled'] = True   


def draw_results(maix_img, results, a4_corners, assembled_pieces, M_inv):
    if a4_corners is not None:
        pts = a4_corners.astype(int)
        for i in range(4):
            x1, y1 = pts[i]
            x2, y2 = pts[(i + 1) % 4]
            maix_img.draw_line(x1, y1, x2, y2, image.COLOR_YELLOW)
        origin_x, origin_y = pts[0]
        maix_img.draw_circle(origin_x, origin_y, 8, image.COLOR_RED, thickness=-1)
        maix_img.draw_string(origin_x + 10, origin_y + 10, "Origin (0,0)", image.COLOR_RED)

    for r in results:
        pts = r["contour"]
        num_pts = len(pts)
        for i in range(num_pts - 1):
            maix_img.draw_line(pts[i][0], pts[i][1], pts[i+1][0], pts[i+1][1], image.COLOR_GREEN)
        if num_pts > 2:
            maix_img.draw_line(pts[-1][0], pts[-1][1], pts[0][0], pts[0][1], image.COLOR_GREEN)

        maix_img.draw_circle(r["cx"], r["cy"], 5, image.COLOR_RED, thickness=-1)
        info_str = f"#{r['id']} ({r['x_mm']},{r['y_mm']})mm"
        maix_img.draw_string(r["cx"] + 10, r["cy"] - 10, info_str, image.COLOR_GREEN)

    all_right_px_pts = []
    if M_inv is not None and len(assembled_pieces) >= 2:
        prior_pieces = assembled_pieces[:-1]

        for item in prior_pieces:
            placed_pts_mm = item.get('placed_pts', [])
            if len(placed_pts_mm) >= 3 and item.get('is_assembled', False):
                pts_mm_np = np.array(placed_pts_mm, dtype=np.float32).reshape(-1, 1, 2)
                pts_px_np = cv2.perspectiveTransform(pts_mm_np, M_inv).reshape(-1, 2).astype(int)
                all_right_px_pts.append(pts_px_np)

                n = len(pts_px_np)
                for i in range(n):
                    x1, y1 = pts_px_np[i]
                    x2, y2 = pts_px_np[(i + 1) % n]
                    maix_img.draw_line(x1, y1, x2, y2, image.COLOR_BLUE)

        if DRAW_COMBINED_OUTLINE_TOGGLE and len(all_right_px_pts) >= 2:
            mask_canvas = np.zeros((CAM_H, CAM_W), dtype=np.uint8)
            for pts in all_right_px_pts:
                cv2.fillPoly(mask_canvas, [pts], 255)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask_canvas = cv2.morphologyEx(mask_canvas, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(mask_canvas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                main_cnt = max(contours, key=cv2.contourArea)
                peri = cv2.arcLength(main_cnt, True)
                approx = cv2.approxPolyDP(main_cnt, 0.012 * peri, True)
                pts_px_red = approx.reshape(-1, 2)
                n_red = len(pts_px_red)
                for i in range(n_red):
                    rx1, ry1 = pts_px_red[i]
                    rx2, ry2 = pts_px_red[(i + 1) % n_red]
                    maix_img.draw_line(rx1, ry1, rx2, ry2, image.COLOR_RED)
                    maix_img.draw_circle(rx1, ry1, 5, image.COLOR_RED, thickness=-1)

    if len(assembled_pieces) >= 1:
        last_item = assembled_pieces[-1]
        pid = last_item['id']
        toggle_key = f"#{pid}"
        if DRAW_PIECES_TOGGLE.get(toggle_key, True):
            pts_mm_last = last_item.get('placed_pts', [])
            if len(pts_mm_last) >= 3 and M_inv is not None:
                pts_mm_np_last = np.array(pts_mm_last, dtype=np.float32).reshape(-1, 1, 2)
                pts_px_np_last = cv2.perspectiveTransform(pts_mm_np_last, M_inv).reshape(-1, 2).astype(int)
                n_last = len(pts_px_np_last)
                for i in range(n_last):
                    x1, y1 = pts_px_np_last[i]
                    x2, y2 = pts_px_np_last[(i + 1) % n_last]
                    maix_img.draw_line(x1, y1, x2, y2, image.COLOR_BLUE)
                    maix_img.draw_circle(x1, y1, 4, image.COLOR_GREEN, thickness=-1)

    if M_inv is not None and assembled_pieces:
        for item in assembled_pieces:
            pid = item['id']
            toggle_key = f"#{pid}"
            if not DRAW_PIECES_TOGGLE.get(toggle_key, True):
                continue
            p2_x_mm = item.get('final_cx', 200.0)
            p2_y_mm = item.get('final_cy', 90.0)
            p2_mm_np = np.array([[[p2_x_mm, p2_y_mm]]], dtype=np.float32)
            p2_px_np = cv2.perspectiveTransform(p2_mm_np, M_inv).reshape(-1, 2).astype(int)
            p2_px_x, p2_px_y = p2_px_np[0]
            maix_img.draw_circle(p2_px_x, p2_px_y, 6, image.COLOR_RED, thickness=-1)
            maix_img.draw_string(p2_px_x + 8, p2_px_y - 8, f"P2-#{pid}", image.COLOR_BLUE)

    return maix_img


def main():
    cam = camera.Camera(CAM_W, CAM_H)
    disp = display.Display()
    serial_dev = uart.UART("/dev/ttyS0", 115200)
 
    frame_count = 0
    send_trigger = False

    while True:
        t0 = time.ticks_ms()
        img = cam.read()
        if img is None:
            time.sleep_ms(1)
            continue

        rx_data = serial_dev.read(100, 5)
        if rx_data and b'Y' in rx_data:
            send_trigger = True

        if frame_count % (SKIP_FRAME + 1) == 0:
            results, a4_corners, stitch_plan, assembled_pieces, M_inv = process_frame_fast(img)
            
            update_fourth_piece_solution(assembled_pieces, M_inv)

            if send_trigger:
                if is_combined_shape_valid_rectangle(assembled_pieces, M_inv):
                    valid_items = [p for p in assembled_pieces if p.get('is_assembled', False)]
                    sorted_items = sorted(valid_items, key=lambda x: x['id'])
                     
                    for item in sorted_items:
                        nid = item['id']
                        p1_x = round(item['x_mm'] / 10.0, 1)
                        p1_y = round(item['y_mm'] / 10.0, 1)
                        p2_x = round(item.get('final_cx', 175.0) / 10.0, 1)
                        p2_y = round(item.get('final_cy', 150.0) / 10.0, 1)
                        angle = wrap_angle_180(-item.get('rot_relative', 0.0))
                        
                        send_str = f"N{nid},P1,{p1_x:.1f},{p1_y:.1f},{angle:+.1f},P2,{p2_x:.1f},{p2_y:.1f},F\r\n"
                        serial_dev.write_str(send_str)
                        print(send_str, end='')
                        
                    send_trigger = False
                else:
                    print("")

            img = draw_results(img, results, a4_corners, assembled_pieces, M_inv)

        fps = 1000.0 / (time.ticks_ms() - t0 + 0.0001)
        img.draw_string(10, 10, f"FPS: {fps:.1f}", image.COLOR_WHITE)
        disp.show(img)
        frame_count += 1


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n用户退出")
    except Exception as e:
        import traceback
        traceback.print_exc()