import numpy as np


class Roles:
    """High level strategic roles and analysis"""
    # get behind ball without touching it, to avoid pushing it in
    def get_behind_ball(self):
        ball_pos = self._gs.get_ball_position()
        # TODO, and move to routines!

    def goalie(self, robot_id, is_opposite_goal=False):
        """Commands a given robot id to play as goalie"""
        team = self._team
        GOALIE_OFFSET = 600  # goalie stays this far from goal center
        # for demo purposes, allow playing as opposite goalie
        if is_opposite_goal:
            team = 'yellow' if team == 'blue' else 'blue'
        shot_location = self._gs.is_shot_coming(team)
        if shot_location is not None:
            # robot goes to ball using to nearest interception point
            # Note that that if the robot CAN intercept the ball, this function
            # returns the same thing as intercept_range
            safest_intercept_point = self.safest_intercept_point(robot_id)
            self.move_straight(robot_id, safest_intercept_point, is_urgent=True)
        elif self._gs.is_ball_behind_goalie() and (shot_location is None):
            goal_posts_pos = self._gs.get_defense_goal(team)
            goalie_x, goalie_y, goalie_w = self._gs.get_robot_position(team, robot_id)
            self.move_straight(robot_id, np.array([goal_posts_pos[0][0], goalie_y, goalie_w]))
            # x, y = self._gs.get_ball_pos()
            # goalie_x, goalie_y, goalie_w = self._gs.get_robot_position(team, robot_id)
            # self.move_straight(robot_id, self.block_goal_center_pos(2*GOALIE_OFFSET, ball_pos=None, team=team))
            # goalie_x, goalie_y, goalie_w = self._gs.get_robot_position(team, robot_id)
            # goal_posts_pos = self._gs.get_defense_goal(self, team)
            # if y > 0:
            #     bottom_post_x, bottom_post_y = goal_posts_pos[1]
            #     self.move_straight(robot_id, np.array([goalie_x, bottom_post_y + self._gs.ROBOT_RADIUS, goalie_w]))
            #     self.move_straight(robot_id, np.array([bottom_post_x, bottom_post_y + self._gs.ROBOT_RADIUS, goalie_w]))
            #     self.move_straight(robot_id, np.array([bottom_post_x, y, goalie_w]))
            # else:
            #     top_post_x, top_post_y = goal_posts_pos[0]
            #     self.move_straight(robot_id, np.array([goalie_x, bottom_post_y - self._gs.ROBOT_RADIUS, goalie_w]))
            #     self.move_straight(robot_id, np.array([bottom_post_x, bottom_post_y - self._gs.ROBOT_RADIUS, goalie_w]))
            #     self.move_straight(robot_id, np.array([bottom_post_x, y, goalie_w]))
        else:
            goalie_pos = self.block_goal_center_pos(GOALIE_OFFSET, ball_pos=None, team=team)
            if goalie_pos.any():
                self.move_straight(robot_id, goalie_pos)

    def attacker(self, robot_id):
        """Commands a given robot id to play as attacker"""
        team = self._team
        # Shooting velocity
        shoot_velocity = 1200
        # TODO: Movement and receive ball
        # Shoots if has the ball
        if self._gs.ball_in_dribbler(team, robot_id):
            if self.within_shooting_range(team, robot_id):
                goal = self._gs.get_attack_goal(team)
                center_of_goal = (goal[0] + goal[1]) / 2
                self.prepare_and_kick(robot_id, center_of_goal, shoot_velocity)
            else:
                pass
        else:
            if self._gs.is_pos_legal(self._gs.get_ball_position(), team, robot_id):
                self.get_ball(robot_id, charge_during=shoot_velocity)

    def defender(self, robot_id):
        currPos = self._gs.get_robot_position(self._team, robot_id)[0:2]
        goal_top, goal_bottom = self._gs.get_defense_goal(self._team)
        goal_center = (goal_top + goal_bottom) / 2
        maxDistance = np.linalg.norm(currPos - goal_center)
        interceptPos = self.block_goal_center_pos(maxDistance, ball_pos=None, team=self._team)
        self.move_straight(robot_id, interceptPos)

    def defender(self, robot_id):
        currPos = self._gs.get_robot_position(self._team, robot_id)[0:2]
        goal_top, goal_bottom = self._gs.get_defense_goal(self._team)
        goal_center = (goal_top + goal_bottom) / 2
        maxDistance = np.linalg.norm(currPos - goal_center)
        interceptPos = self.block_goal_center_pos(maxDistance, ball_pos=None, team=self._team)
        self.move_straight(robot_id, interceptPos)
        self.move_straight(robot_id, ball_pos)
