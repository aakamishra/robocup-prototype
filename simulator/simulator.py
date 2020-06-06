# pylint: disable=line-too-long
import time
import numpy as np
from typing import Tuple
import logging
from coordinator import Provider  # pylint: disable=import-error

logger = logging.getLogger(__name__)


class Simulator(Provider):
    """Simulator class spins to update gamestate instead of vision and comms.
       Applies rudimentary physics and commands, to allow offline prototyping.
    """
    # TODO: when we get multiple comms, connect to all available robots

    def __init__(self, initial_setup):
        super().__init__()
        self.logger = None
        self._initial_setup = initial_setup
        self._viz_events_handled = 0
        self._owned_fields = [
            # act as vision provider
            '_ball_position',
            '_blue_robot_positions',
            '_yellow_robot_positions',
            # also act as robot feedback
            '_blue_robot_status',
            '_yellow_robot_status',
        ]

    def put_fake_robot(self, team: str,
                       robot_id: int,
                       position: Tuple[float, float, float]) -> None:
        """initialize a robot with given id + team at (x, y, w) position"""
        if position[2] is None:
            position[2] = 0
        self.gs.update_robot_position(team, robot_id, position)
        commands = self.gs.get_robot_commands(team, robot_id)
        commands.clear_waypoints()

    def put_fake_ball(self, position, velocity=None):
        "initialize ball position data to reflect desired position + velocity"
        if velocity is None:
            velocity = np.array([0, 0])
        self.gs.clear_ball_position()
        # use small dt to minimize deceleration correction
        dt = .05
        prev_pos = position - velocity * dt
        self.gs.update_ball_position(prev_pos, time.time() - dt)
        self.gs.update_ball_position(position, time.time())

    def pre_run(self):
        if self.logger is None:
            self.create_logger()
        self.logger.debug("Calling pre_run in visualization")
        # logger.info("\nSimulator running with initial setup: {}".format(
        #     self._initial_setup
        # ))
        # initialize the chosen scenario
        if self._initial_setup == 'full_teams':
            for i in range(1, 7):
                left_pos = np.array([-3000, 200 * (i - 3.5), 0])
                right_pos = np.array([3000, 200 * (i - 3.5), 3.14])
                if self.gs.is_blue_defense_side_left():
                    blue_pos = left_pos
                    yellow_pos = right_pos
                else:
                    blue_pos = right_pos
                    yellow_pos = left_pos
                self.put_fake_robot('blue', i, blue_pos)
                self.put_fake_robot('yellow', i, yellow_pos)
            self.put_fake_ball(np.array([0, 0]))
        elif self._initial_setup == "moving_ball":
            self.put_fake_robot('blue', 1, np.array([-3000, 0, 0]))
            self.put_fake_ball(np.array([-2000, 1200]), np.array([0, -1200]))
        elif self._initial_setup == "entry_video":
            SCALE = 1  # if mini field
            pfr = self.put_fake_robot
            self.put_fake_ball(np.array([2000, 900]) * SCALE, np.array([0, 0]))
            pfr('blue', 0, np.array([1000, 900, 0]) * SCALE)
            pfr('blue', 8, np.array([2000, -1100, 0]) * SCALE)
            pfr('yellow', 0, np.array([1800, -500, 0]) * SCALE)
            pfr('yellow', 1, np.array([3000, 1200, 0]) * SCALE)
            pfr('yellow', 2, np.array([3000, -1500, 0]) * SCALE)
            pfr('yellow', 3, np.array([3500, 500, 0]) * SCALE)
            pfr('yellow', 4, np.array([3500, -500, 0]) * SCALE)
        elif self._initial_setup == "clear_field_test":
            self.put_fake_robot('blue', 1, np.array([-3000, 0, 0]))
        elif self._initial_setup == "clear_field_kickoff_test":
            self.put_fake_robot('blue', 1, np.array(
                [-self.gs.ROBOT_RADIUS * 1.1, 0, 0]))
            self.put_fake_ball(np.array([0, 0]), np.array([0, 0]))
        elif self._initial_setup == "surrounded_by_opponents_test":
            self.put_fake_robot('blue', 1, np.array([-3000, 0, 0]))
            self.put_fake_robot('yellow', 1, np.array([-3000, 200, 0]))
            self.put_fake_robot('yellow', 2, np.array([-3000, -200, 0]))
            self.put_fake_robot('yellow', 3, np.array([-3180, 100, 0]))
            self.put_fake_robot('yellow', 4, np.array([-3180, -100, 0]))
            self.put_fake_robot('yellow', 5, np.array([-2820, 100, 0]))
            self.put_fake_robot('yellow', 6, np.array([-2820, -100, 0]))
        else:
            logger.error("(initial_setup not recognized, empty field). "
                         "initial_setup: %s", self._initial_setup)

    def run(self):
        # allow user to move the ball via UI
        if self._viz_events_handled < self.gs.viz_inputs['simulator_events_count']:  # noqa
            self._viz_events_handled += 1
            if self.gs.viz_inputs['user_selected_ball']:
                new_pos = self.gs.viz_inputs['user_click_position']
                if new_pos is not None:
                    v = self.gs.viz_inputs['user_drag_vector']
                    v = np.array([0, 0]) if v is None else v
                    self.put_fake_ball(new_pos[:2], v)
        # teleport selected robot if desired
        if self.gs.viz_inputs['teleport_selected_robot'] and \
           self.gs.viz_inputs['user_selected_robot'] is not None:
            team, robot_id = self.gs.viz_inputs['user_selected_robot']
            commands = self.gs.get_robot_commands(team, robot_id)
            if len(commands.waypoints) > 0:
                destination = commands.waypoints[-1]
                self.gs.update_robot_position(team, robot_id, destination)

        # move ball according to prediction
        ball_pos = self.gs.get_ball_position()
        if ball_pos is not None:
            new_ball_pos = self.gs.predict_ball_pos(self.delta_time)
            self.gs.update_ball_position(new_ball_pos)

        for (team, robot_id), pos in \
                self.gs.get_all_robot_positions():
            # refresh positions of all robots
            pos = self.gs.get_robot_position(team, robot_id)
            self.gs.update_robot_position(team, robot_id, pos)

            # handle collisions with other robots
            for (team2, robot_id2), pos2 in \
                    self.gs.get_all_robot_positions():
                if ((team2, robot_id2) != (team, robot_id) and
                        self.gs.robot_overlap(pos, pos2).any()):
                    overlap = self.gs.robot_overlap(pos, pos2)
                    overlap = np.append(overlap, 0)
                    self.gs.update_robot_position(
                        team, robot_id, pos - overlap / 2)
                    self.gs.update_robot_position(
                        team2, robot_id2, pos2 + overlap / 2)
            # collision with ball
            ball_pos = self.gs.get_ball_position()
            ball_overlap = self.gs.robot_ball_overlap(pos)
            if ball_overlap.any():
                self.logger.info("Ball overlap with robot: %s", ball_overlap)
                # find where ball collided with robot
                collision_pos = ball_pos + ball_overlap
                ball_v = self.gs.get_ball_velocity()
                if ball_v.any():
                    collision_pos = ball_pos
                    ball_direction = ball_v / np.linalg.norm(ball_v)
                    step = 1
                    # trace back one step at a time to collision point
                    while self.gs.robot_ball_overlap(pos, collision_pos).any():
                        collision_pos -= ball_direction * step
                # keep velocity in direction tangent to bot at collision
                radius_vector = collision_pos - pos[:2]
                if self.gs.is_robot_front_sector(pos, collision_pos):
                    # we are in the front sector, use flat angle
                    radius_vector = self.gs.dribbler_pos(
                        team, robot_id) - pos[:2]
                tangent_vector = np.array(
                    [radius_vector[1], -radius_vector[0]])
                assert(tangent_vector.any())
                tangent_vector /= np.linalg.norm(tangent_vector)
                new_v = np.dot(ball_v, tangent_vector) * tangent_vector
                self.put_fake_ball(collision_pos, new_v)

        for (team, robot_id), robot_commands in \
                self.gs.get_all_robot_commands():
            robot_status = self.gs.get_robot_status(team, robot_id)
            # move robots according to commands
            pos = self.gs.get_robot_position(team, robot_id)
            new_pos = robot_commands.predict_pos(pos, self.delta_time)
            self.gs.update_robot_position(
                team, robot_id, new_pos
            )
            # simulate dribbling as gravity zone
            if robot_commands.is_dribbling:
                ball_pos = self.gs.get_ball_position()
                dribbler_center = self.gs.dribbler_pos(team, robot_id)
                robot_pos = self.gs.get_robot_position(team, robot_id)
                # simplistic model of capturing ball only if slow enough
                ball_v = self.gs.get_ball_velocity()
                DRIBBLE_CAPTURE_VELOCITY = 20
                if self.gs.ball_in_dribbler(team, robot_id) and \
                        np.linalg.norm(ball_v) < DRIBBLE_CAPTURE_VELOCITY:
                    pullback_velocity = (robot_pos[:2] - ball_pos) * 2
                    centering_velocity = (dribbler_center - ball_pos) * 1
                    total_velocity = pullback_velocity + centering_velocity
                    new_pos = ball_pos + total_velocity * self.delta_time
                    new_pos -= self.gs.robot_ball_overlap(robot_pos, new_pos)
                    self.put_fake_ball(new_pos)
            # simulate charging
            if robot_commands.is_charging:
                robot_status.simulate_charge(self.delta_time)
            # kick according to commands
            if robot_commands.is_kicking:
                if self.gs.ball_in_dribbler(team, robot_id):
                    ball_pos = self.gs.get_ball_position()
                    # (hacky) offset it outside the robot radius
                    kick_direction = self.gs.get_robot_direction(
                        team, robot_id)
                    ball_pos += kick_direction * 40
                    new_velocity = robot_status.kick_velocity() * \
                        self.gs.get_robot_direction(team, robot_id)
                    new_pos = ball_pos + new_velocity * self.delta_time
                    self.put_fake_ball(new_pos, new_velocity)
                robot_status.simulate_kick()
