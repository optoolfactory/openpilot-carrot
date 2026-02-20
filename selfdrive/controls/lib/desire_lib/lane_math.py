def calculate_lane_width(lane, lane_prob, current_lane, road_edge):
  t1 = t2 = None
  for i, v in enumerate(current_lane.t):
    if t1 is None and v >= 1.0:
      t1 = i
    if v >= 2.0:
      t2 = i
      break

  if t1 is None:
    t1 = 0
  if t2 is None:
    t2 = len(current_lane.t) - 1

  cur_y = current_lane.y[t1]
  dist_lane = abs(cur_y - lane.y[t1])

  dist_edge = abs(cur_y - road_edge.y[t1])
  dist_edge_far = abs(cur_y - road_edge.y[t2])

  lane_valid = lane_prob > 0.5
  return min(dist_lane, dist_edge), dist_edge, dist_edge_far, lane_valid
