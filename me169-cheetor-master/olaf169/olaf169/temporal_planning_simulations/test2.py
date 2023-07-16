sentry_points = ((3, 2), (6, 0), (6, 6))

def get_sentry_position(t):
    total_time = 10; num_seg = len(sentry_points); time_per_seg = (total_time / num_seg)
    
    last_pt_idx = int((t % total_time) / time_per_seg)
    
    percent_through_motion = ((t % total_time) - last_pt_idx * time_per_seg) / time_per_seg
    past_point = sentry_points[last_pt_idx]
    next_point = sentry_points[(last_pt_idx + 1) % num_seg]

    x = past_point[0] + percent_through_motion * (next_point[0] - past_point[0])
    y = past_point[1] + percent_through_motion * (next_point[1] - past_point[1])
    return x, y

print(get_sentry_position(6.6))