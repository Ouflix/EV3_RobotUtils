def Forward_final(distance, speed, stopping_margin=5):

    initial_distance = robot.distance()
    target_distance = distance - stopping_margin  # Stop slightly earlier

    while abs(robot.distance() - initial_distance) < target_distance:
        robot.drive(speed, 0)

    # Gradual slowing before stopping
    while abs(robot.distance() - initial_distance) < distance:
        robot.drive(speed * 0.2, 0)
        time.sleep(0.01)  # Allow time for motor update

    robot.stop()
    left_wheel.brake()
    right_wheel.brake()

    # Return the final traveled distance
    final_distance = robot.distance() - initial_distance
    return final_distance


def TurnNew_final(degrees, Kp=4.0, Ki=0.025, Kd=1.5):
    resetAngles()

    # Initialize PID variables
    integral = 0.0
    last_error = 0.0
    integral_limit = 200.0  # Lower limit to avoid integral windup

    while True:
        # Get the current gyro angle
        current_angle = gyro.angle()

        # Calculate error
        error = degrees - current_angle

        # Check if we are close enough to the target
        if abs(error) < 0.3:  # Tighter stopping threshold for better accuracy
            robot.stop()
            break

        # Calculate PID terms
        integral += error
        integral = max(min(integral, integral_limit), -integral_limit)

        derivative = error - last_error
        turn_speed = (Kp * error) + (Ki * integral) + (Kd * derivative)

        # Clamp turn speed to ensure smooth and accurate turns
        turn_speed = max(min(turn_speed, 70), -70)  # Lowered max speed for fine adjustments

        # Apply the turn speed to the robot
        robot.drive(0, turn_speed)

        last_error = error
        time.sleep(0.002)  # Faster updates for finer control

    robot.stop()
    return gyro.angle()
