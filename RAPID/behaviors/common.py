from ..utils import euclidian_distance


def return_home_or_finish(robot, set_finishing_status=False):
    """Send the robot back to its initial position, or finish if it is already there."""
    init_pos = (int(robot.init_transform.x), int(robot.init_transform.y))
    if euclidian_distance((int(robot.transform.x), int(robot.transform.y)), init_pos) > robot.treshold_for_target:
        if set_finishing_status:
            robot.status = "finishing"
        robot.target = init_pos
        robot.last_plan_time = robot.env.step
    else:
        robot.finish()
