import math
import numpy as np

# curve lookup (원본 그대로)
V_CURVE_LOOKUP_BP = [0., 1./800., 1./670., 1./560., 1./440., 1./360., 1./265., 1./190., 1./135., 1./85., 1./55., 1./30., 1./25.]
V_CRUVE_LOOKUP_VALS = [300, 150, 120, 110, 100, 90, 80, 70, 60, 50, 40, 15, 5]

def haversine(lon1, lat1, lon2, lat2):
  R = 6371000
  phi1, phi2 = math.radians(lat1), math.radians(lat2)
  dphi = math.radians(lat2 - lat1)
  dlambda = math.radians(lon2 - lon1)
  a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
  return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))

def closest_point_on_segment(p1, p2, current_position):
  x1, y1 = p1
  x2, y2 = p2
  px, py = current_position
  dx = x2 - x1
  dy = y2 - y1
  if dx == 0 and dy == 0:
    return p1
  t = ((px - x1) * dx + (py - y1) * dy) / (dx*dx + dy*dy)
  t = max(0, min(1, t))
  return (x1 + t*dx, y1 + t*dy)

def get_path_after_distance(start_index, coordinates, current_position, distance_m):
  total_distance = 0.0
  path_after_distance = []
  closest_index = -1
  closest_point = None
  min_distance = float('inf')

  start_index = max(0, start_index - 2)

  for i in range(start_index, len(coordinates) - 1):
    p1 = coordinates[i]
    p2 = coordinates[i + 1]
    candidate = closest_point_on_segment(p1, p2, current_position)
    d = haversine(current_position[0], current_position[1], candidate[0], candidate[1])
    if d < min_distance:
      min_distance = d
      closest_point = candidate
      closest_index = i
    elif d > min_distance and min_distance < 10:
      break

  start_index = closest_index

  if closest_index != -1:
    path_after_distance.append(closest_point)
    path_after_distance.append(coordinates[closest_index + 1])
    total_distance = haversine(closest_point[0], closest_point[1],
                               coordinates[closest_index+1][0], coordinates[closest_index+1][1])

    for i in range(closest_index + 1, len(coordinates) - 1):
      c1 = coordinates[i]
      c2 = coordinates[i + 1]
      seg = haversine(c1[0], c1[1], c2[0], c2[1])
      if total_distance + seg >= distance_m and seg > 0:
        rem = distance_m - total_distance
        ratio = rem / seg
        lon = c1[0] + ratio * (c2[0] - c1[0])
        lat = c1[1] + ratio * (c2[1] - c1[1])
        path_after_distance.append((lon, lat))
        break
      total_distance += seg
      path_after_distance.append(c2)

  return path_after_distance, start_index, closest_point

def gps_to_relative_xy(gps_path, reference_point, heading_deg):
  ref_lon, ref_lat = reference_point
  heading_rad = math.radians(heading_deg)
  rel = []
  for lon, lat in gps_path:
    x = (lon - ref_lon) * 40008000 * math.cos(math.radians(ref_lat)) / 360
    y = (lat - ref_lat) * 40008000 / 360
    x_rot = x * math.cos(heading_rad) - y * math.sin(heading_rad)
    y_rot = x * math.sin(heading_rad) + y * math.cos(heading_rad)
    rel.append((y_rot, x_rot))
  return rel

def calculate_curvature(p1, p2, p3):
  v1 = (p2[0]-p1[0], p2[1]-p1[1])
  v2 = (p3[0]-p2[0], p3[1]-p2[1])
  cross = v1[0]*v2[1] - v1[1]*v2[0]
  len1 = math.sqrt(v1[0]**2 + v1[1]**2)
  len2 = math.sqrt(v2[0]**2 + v2[1]**2)
  if len1 * len2 == 0:
    return 0.0
  return cross / (len1 * len2 * len1)

def curvature_to_speed(curv, nRoadLimitSpeed):
  speed = np.interp(abs(curv), V_CURVE_LOOKUP_BP, V_CRUVE_LOOKUP_VALS)
  if abs(curv) < 0.02:
    speed = max(speed, nRoadLimitSpeed)
  return speed
