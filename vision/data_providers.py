
'''A class to provide robot position data from the cameras'''
import sslclient
import threading
import numpy as np
from collections import Counter
from typing import Tuple
from coordinator import Provider


class SSLVisionDataProvider(Provider):
    def __init__(self, HOST='224.5.23.2', PORT=10006):
        super().__init__()
        self.HOST = HOST
        self.PORT = PORT

        self._ssl_vision_client = None
        self._ssl_vision_thread = None
        # cache data from different cameras so we can merge them
        # camera_id : latest raw data
        sslclient_detection = sslclient.messages_robocup_ssl_detection_pb2.SSL_DetectionFrame  # noqa
        self._raw_camera_data = {
            0: sslclient_detection(),
            1: sslclient_detection(),
            2: sslclient_detection(),
            3: sslclient_detection(),
        }
        self._owned_fields = [
            '_ball_position',
            '_blue_robot_positions',
            '_yellow_robot_positions'
        ]

    def pre_run(self):
        """Starts listen to SSL-vision and updating gamestate with new data"""
        self._ssl_vision_client = sslclient.client()
        self._ssl_vision_client.connect()
        self._ssl_vision_thread = threading.Thread(
            target=self.receive_data_loop
        )
        # set to daemon mode so it will be easily killed
        self._ssl_vision_thread.daemon = True
        self._ssl_vision_thread.start()

    def post_run(self):
        if self._ssl_vision_client:
            self._ssl_vision_thread.join()
            self._ssl_vision_thread = None
            self._ssl_vision_client = None

    # loop for reading messages from ssl vision, otherwise they pile up
    def receive_data_loop(self):
        while self._ssl_vision_client:
            data = self._ssl_vision_client.receive()
            # print(data)
            # get a detection packet from any camera, and store it
            if data.HasField('detection'):
                cid = data.detection.camera_id
                self._raw_camera_data[cid] = data.detection

    def run(self):
        # update positions of all robots seen by data feed
        for team in ['blue', 'yellow']:
            robot_positions = self.get_robot_positions(team)
            # print(robot_positions)
            for robot_id, pos in robot_positions.items():
                self.gs.update_robot_position(team, robot_id, pos)
        # update position of the ball
        ball_data = self._get_ball_position()
        if ball_data is not None:
            self.gs.update_ball_position(ball_data)

    def get_robot_positions(self, team='blue'):
        robot_positions = {}
        # track how many cameras see each robot, for averaging
        num_cameras_seen = Counter()
        for camera_id, raw_data in self._raw_camera_data.items():
            if team == 'blue':
                team_data = raw_data.robots_blue
            else:
                assert(team == 'yellow')
                team_data = raw_data.robots_yellow
            for robot_data in team_data:
                robot_id = robot_data.robot_id
                num_cameras_seen[robot_id] += 1
                # only update data if it has higher confidence
                CONFIDENCE_THRESHOLD = .5
                if robot_data.confidence >= CONFIDENCE_THRESHOLD:
                    # average in the new data
                    pos = np.array([robot_data.x,
                                    robot_data.y,
                                    robot_data.orientation])
                    if robot_id not in robot_positions:
                        robot_positions[robot_id] = pos
                    else:
                        times_seen = num_cameras_seen[robot_id]
                        current_pos = robot_positions[robot_id]
                        average_pos = np.array([
                            (current_pos[0] * (times_seen - 1) + pos[0]) / times_seen,  # noqa
                            (current_pos[1] * (times_seen - 1) + pos[1]) / times_seen,  # noqa
                            # TODO: safely average orientation?
                            self._circular_mean(
                                (times_seen - 1, 1),
                                (robot_data.orientation, pos[2])
                            )
                        ])
                        robot_positions[robot_id] = average_pos
        # if (team == 'blue'):
        #    print(robot_positions[0])
        return robot_positions

    def _circular_mean(self, weights, angles):
        "helper function for averaging angles by converting to points"
        x = y = 0.
        for angle, weight in zip(angles, weights):
            x += np.cos(angle) * weight
            y += np.sin(angle) * weight
        if x == 0 and y == 0:
            print('freak coincidence?')
            return 0
        mean = np.arctan2(y, x)
        return mean

    def _get_ball_position(self) -> Tuple[float, float]:
        "Returns average ball readings of the cameras."
        average_ball = None
        times_seen = 0
        # TODO: Do some adv. processing based on which camera has seen the ball
        for camera_id, raw_data in self._raw_camera_data.items():
            balls = raw_data.balls
            CONFIDENCE_THRESHOLD = .5
            if len(balls) > 0 and balls[0].confidence >= CONFIDENCE_THRESHOLD:
                ball = balls[0]
                times_seen += 1
                if average_ball is None:
                    average_ball = np.array([ball.x, ball.y])
                else:
                    pos = np.array([ball.x, ball.y])
                    average_ball = (average_ball * (times_seen - 1) + pos) / times_seen  # noqa
        return average_ball
