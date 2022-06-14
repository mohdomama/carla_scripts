import numpy as np

def speed_control(v, v_desired, dt, previous):
    kp = 0.7
    ki = 0.2
    kd = 0.2

    throttle_previous, int_val = previous
    last_error = 0 # Ideally we should save error every time and use here. Insted of setting it to zero
    throttle_output = 0
    brake_output    = 0

    # pid control
    st = dt

    # error term
    delta_v = v_desired - v

    # I
    integral = int_val + delta_v * st # int_val = integral so far

    # D
    derivate = (delta_v - last_error) / st

    rst = kp * delta_v + ki * integral + kd * derivate

    if rst > 0:
        throttle_output = np.tanh(rst)
        throttle_output = max(0.0, min(1.0, throttle_output))
        if throttle_output - throttle_previous > 0.1:
            throttle_output = throttle_previous + 0.1
    else:
        throttle_output = 0

    previous = throttle_output, integral
    return throttle_output, previous


def steer_control(x, y, yaw, v, waypoints):
    steer_output = 0

    # Use stanley controller for lateral control
    # 0. spectify stanley params
    k_e = 0.3
    k_v = 10

    # 1. calculate heading error
    yaw_path = np.arctan2(waypoints[-1][1]-waypoints[0][1], waypoints[-1][0]-waypoints[0][0])
    yaw_diff = yaw_path - yaw 
    if yaw_diff > np.pi:
        yaw_diff -= 2 * np.pi
    if yaw_diff < - np.pi:
        yaw_diff += 2 * np.pi

    # 2. calculate crosstrack error
    current_xy = np.array([x, y])
    crosstrack_error = np.min(np.sum((current_xy - np.array(waypoints)[:, :2])**2, axis=1))

    yaw_cross_track = np.arctan2(y-waypoints[0][1], x-waypoints[0][0])
    yaw_path2ct = yaw_path - yaw_cross_track
    if yaw_path2ct > np.pi:
        yaw_path2ct -= 2 * np.pi
    if yaw_path2ct < - np.pi:
        yaw_path2ct += 2 * np.pi
    if yaw_path2ct > 0:
        crosstrack_error = abs(crosstrack_error)
    else:
        crosstrack_error = - abs(crosstrack_error)

    yaw_diff_crosstrack = np.arctan(k_e * crosstrack_error / (k_v + v))
    
    print(crosstrack_error, yaw_diff, yaw_diff_crosstrack)

    # 3. control low
    steer_expect = yaw_diff + yaw_diff_crosstrack
    if steer_expect > np.pi:
        steer_expect -= 2 * np.pi
    if steer_expect < - np.pi:
        steer_expect += 2 * np.pi
    steer_expect = min(1.22, steer_expect)
    steer_expect = max(-1.22, steer_expect)

    # 4. update
    steer_output = steer_expect

    return steer_output

###############

def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def calc_target_index(x, y, yaw, cx, cy):
    """
    Compute index in the trajectory list of the target.

    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # TODO: Check is this is correct
    L = 1.5
    # Calc front axle position
    fx = x + L * np.cos(yaw)
    fy = y + L * np.sin(yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(yaw + np.pi / 2),
                      -np.sin(yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle

def stanley_control(cx, cy, cyaw, x, y, yaw, v, last_target_idx):
    k = 0.01  # control gain

    current_target_idx, error_front_axle = calc_target_index(x, y, yaw, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, v)
    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx



def main():
    pass

if __name__=='__main__':
    main()