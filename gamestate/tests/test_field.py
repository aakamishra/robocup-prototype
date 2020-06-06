# pylint: disable=import-error
from ..gamestate import GameState


def test_defense_area():
    gs = GameState()
    team = 'blue'
    # middle of goal should be in defense area
    goalposts = gs.get_defense_goal(team)
    assert gs.is_in_defense_area(goalposts[0], team)
