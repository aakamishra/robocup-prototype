# pylint: disable=maybe-no-member
import numpy as np
import time
from typing import Tuple


class Actions:
    """
    Definitions + supporting logic for simple robot actions
    (have a single step/end condition, return True when done)
    """
    def pivot_with_ball_speeds(self,
                               robot_id,
                               face_pos: Tuple[float, float]) -> bool:
        """Move robot around ball without losing possession"""
        ball_pos = self.gs.get_ball_position()
        kick_pos = self.best_kick_pos(ball_pos, face_pos)
        robot_pos = self.gs.get_robot_position(self._team, robot_id)
        angle = self.wrap_pi(kick_pos[2] - robot_pos[2])

        step = 100
        astep = 1

        vel = np.cross(ball_pos - robot_pos[0:2], [0, 0, 1])
        vel = step * vel * np.sign(angle) / np.linalg.norm(vel)
        self.logger.debug(vel)
        self.set_speeds(robot_id, vel[0], vel[1], np.sign(angle) * astep)

    def pivot_with_ball(self, robot_id, face_pos: Tuple[float, float]) -> bool:
        """Move robot around ball without losing possession"""
        ball_pos = self.gs.get_ball_position()
        kick_pos = self.best_kick_pos(ball_pos, face_pos)
        robot_pos = self.gs.get_robot_position(self._team, robot_id)
        # pivot gradually towards kicking position
        dw = self.wrap_pi(kick_pos[2] - robot_pos[2])
        turn_increment = dw / 3
        min_turn_increment = .05
        turn_increment += min_turn_increment / (dw / abs(dw))
        angle = robot_pos[2] + turn_increment
        waypoint = self.gs.dribbler_to_robot_pos(ball_pos, angle)
        self.set_waypoints(robot_id, [waypoint], is_urgent=False)
        remaining_error = abs(self.wrap_pi(robot_pos[2] - kick_pos[2]))
        if remaining_error < min_turn_increment and \
           self.gs.ball_in_dribbler(self._team, robot_id):
            return True
        else:
            return False

    def charge_up_to(self, robot_id, kick_speed: float) -> bool:
        """Charge kicker up to a power level that attains a specific kick_speed.
        Definition of kick speed is specified in the robot commands API/lib."""
        commands = self.gs.get_robot_commands(self._team, robot_id)
        status = self.gs.get_robot_status(self._team, robot_id)
        if status.kick_velocity() < kick_speed:
            commands.is_charging = True
        else:
            commands.is_charging = False
        return not commands.is_charging  # return whether we are done charging

    def kick_ball(self, robot_id: int):
        commands = self.gs.get_robot_commands(self._team, robot_id)
        commands.is_kicking = True

    def set_dribbler(self, robot_id: int, is_dribbling: bool) -> bool:
        commands = self.gs.get_robot_commands(self._team, robot_id)
        commands.is_dribbling = is_dribbling

    def stop(self, robot_id: int):
        commands = self.gs.get_robot_commands(self._team, robot_id)
        commands.set_speeds(0, 0, 0)
        commands.clear_waypoints()

    # tell specific robot to move straight towards given location
    def move_straight(self, robot_id: int,
                      goal_pos: Tuple[float, float, float],
                      is_urgent: bool = False):
        self.set_waypoints(robot_id, [goal_pos], is_urgent)
        return self.is_done_moving(robot_id)

    # find a legal path for robot to go to position, returns whether arrived
    # TODO: still goes through defense area - need to fix see is_path_blocked
    def full_path_find(self, robot_id: int,
                       goal_pos: Tuple[float, float, float],
                       allow_illegal: bool = False) -> bool:
        """ Tries to find a legal non-colliding path to goal position.
        If goal position is not legal or is blocked, goes somewhere nearby.
        """
        # If the goal is illegal or occupied, find somewhere nearby
        is_legal = self.gs.is_pos_legal(goal_pos, self._team, robot_id)
        is_open = self.gs.is_position_open(goal_pos, self._team, robot_id)
        if not is_legal or not is_open:
            goal_pos = self.find_legal_pos(robot_id, goal_pos)

        start_pos = self.gs.get_robot_position(self._team, robot_id)

        # always check if we can just go straight
        if not self.is_path_blocked(start_pos, goal_pos, robot_id,
                                    buffer_dist=0,
                                    allow_illegal=allow_illegal):
            self.move_straight(robot_id, np.array(goal_pos))
            self.logger.debug("Robot %s going straight from %s to %s",
                              robot_id, start_pos, goal_pos)
            return self.is_done_moving(robot_id)

        # now check if current waypoints are already going where we want
        current_goal = self.get_goal_pos(robot_id)
        SAME_GOAL_THRESHOLD = 100  # TODO
        is_same_goal = current_goal is not None and \
            np.linalg.norm(goal_pos[:2] - current_goal[:2]) < SAME_GOAL_THRESHOLD  # noqa
        commands = self.gs.get_robot_commands(self._team, robot_id)
        current_waypoints = [start_pos] + commands.waypoints
        current_path_collides = False
        for i in range(len(current_waypoints) - 1):
            wp, next_wp = current_waypoints[i], current_waypoints[i+1]
            if self.is_path_blocked(wp, next_wp, robot_id,
                                    allow_illegal=allow_illegal):
                current_path_collides = True

        # avoid rerunning too often so we don't crash the system
        # RRT_MIN_INTERVAL = .1
        # recently_called = robot_id in self._last_pathfind_times and \
        #  time.time() - self._last_pathfind_times[robot_id] < RRT_MIN_INTERVAL
        # only rerun for same goal if long time has elapsed or path collides
        # mainly in case something very strange has happened
        MIN_REFRESH_INTERVAL = 3
        need_refresh = robot_id not in self._last_pathfind_times or \
            time.time() - self._last_pathfind_times[robot_id] > MIN_REFRESH_INTERVAL  # noqa
        self.logger.debug("Robot: %s Start: %s Goal: %s Waypoints: %s",
                          robot_id, start_pos, goal_pos, current_waypoints)
        if (current_path_collides or not is_same_goal or need_refresh):
            self._last_pathfind_times[robot_id] = time.time()
            is_success = self.RRT_path_find(
                start_pos, goal_pos, robot_id, allow_illegal=allow_illegal)
            if not is_success:
                self.logger.debug(f"Robot {robot_id} RRT path find failed")
                return False
        return self.is_done_moving(robot_id)

    def path_find(self, robot_id: int,
                  goal_pos: Tuple[float, float, float],
                  allow_illegal: bool = False) -> bool:
        """ Makes the robot to start moving to a destination using
        a greedy approach"""
        # If the goal is illegal or occupied, find somewhere nearby
        is_legal = self.gs.is_pos_legal(goal_pos, self._team, robot_id)
        is_open = self.gs.is_position_open(goal_pos, self._team, robot_id)
        if not is_legal or not is_open:
            goal_pos = self.find_legal_pos(robot_id, goal_pos)

        start_pos = self.gs.get_robot_position(self._team, robot_id)
        # always check if we can just go straight
        if not self.is_path_blocked(start_pos, goal_pos, robot_id,
                                    buffer_dist=0,
                                    allow_illegal=allow_illegal):
            self.move_straight(robot_id, np.array(goal_pos))
            return self.is_done_moving(robot_id)

        # now check if current waypoints are already going where we want
        current_goal = self.get_goal_pos(robot_id)
        SAME_GOAL_THRESHOLD = 100  # TODO
        is_same_goal = current_goal is not None and \
            np.linalg.norm(goal_pos[:2] - current_goal[:2]) < SAME_GOAL_THRESHOLD  # noqa
        commands = self.gs.get_robot_commands(self._team, robot_id)
        current_waypoints = [start_pos] + commands.waypoints
        # greedy approach only cares about first segment
        fst_segmt = None if len(current_waypoints) < 2 else current_waypoints[:2]  # noqa
        fst_segmt_collides = False if fst_segmt is None \
            else self.is_path_blocked(fst_segmt[0], fst_segmt[1], robot_id,
                                      allow_illegal=allow_illegal)

        if fst_segmt is None:
            fst_segmt_len = 0
        else:
            fst_segmt_len = np.linalg.norm(fst_segmt[1][:2] - fst_segmt[0][:2])

        # distance where we should just let the robot go
        TRIVIAL_DISTANCE = 3 * self.gs.ROBOT_RADIUS
        # need frequent refreshes since we do not have full path planning
        MIN_REFRESH_INTERVAL = .1
        need_refresh = robot_id not in self._last_pathfind_times or \
            time.time() - self._last_pathfind_times[robot_id] > MIN_REFRESH_INTERVAL  # noqa

        if (fst_segmt_collides or not is_same_goal or \
            (need_refresh and not SAME_GOAL_THRESHOLD < fst_segmt_len < TRIVIAL_DISTANCE)):  # noqa
            self._last_pathfind_times[robot_id] = time.time()
            is_success = self.greedy_path_find(start_pos, goal_pos, robot_id,
                                               allow_illegal=allow_illegal)
            if not is_success:
                return False

        return self.is_done_moving(robot_id)
