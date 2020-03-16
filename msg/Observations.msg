# Message of the observations for decision making

# Possible_speeds: -100 Free;  0 Static; 1 Slow; 2 Fast; 100 Blocked

int8 front_left
int8 front
int8 front_right
int8 center_left
int8 center_right
int8 rear_left
int8 rear_right
int8 back_left
int8 back_right
# lane: -1 right of goal;  0 goal lane; 1 Left of goal
int8 lane
# dist_goal: Distance (m) to end of road / exit / end lane
# (0: d_goal <= 1.5;
# 1: 1.5 < d_goal <= 5.5;
# 2: 5.5 < diff_goal <= 9.5;
# 3: 9.5 < diff_goal

int8 dist_goal