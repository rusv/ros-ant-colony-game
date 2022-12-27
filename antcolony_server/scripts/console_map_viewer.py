#!/usr/bin/env python

import rospy

import antcolony_msgs.srv
from antcolony_msgs.msg import Action, Ant, Hive, Sugar, Player, State

MAP_SIZE = 40
PLAYER_ONE_ID = 1
PLAYER_TWO_ID = 2


def clear_terminal():
    """
    Clears terminal entirely
    """
    print("\033[2J")


"""
Color	Foreground Code	Background Code
Black
	30	40
Red
	31	41
Green
	32	42
Yellow
	33	43
Blue
	34	44
Magenta
	35	45
Cyan
	36	46
White
	37	47
"""

def red_text(txt):
    """
    Renders text red
    :param txt:  text to pring
    :return: rendered text
    """
    return "\033[47m\033[31m {}\033[00m" .format(txt)

#\033[;34mBlue text\033[0m
def blue_text(txt):
    """
    Renders text blue
    :param txt:  text to pring
    :return: rendered text
    """
    return "\033[47m\033[34m {}\033[00m" .format(txt)


def light_grey_text(txt):
    """
    Renders text blue
    :param txt:  text to pring
    :return: rendered text
    """
    return "\033[1;47m\033[30m {}\033[00m" .format(txt)


def coordinate_hash(x, y):
    """
    Hashes two dimension coordinates into unique integer
    :param x: coordinate x
    :param y: coordinate y
    :return: computed unique hash
    """
    return MAP_SIZE * y + x


def visualize_game_callback(game_state):
    clear_terminal()
    ants = {}
    hives = {}
    sugars = {}
    occupancy_map = {}
    for ant in game_state.ants:
        ants[ant.id] = ant
        occupancy_map[coordinate_hash(ant.x, ant.y)] = ant.id
    for hive in game_state.hives:
        hives[hive.id] = hive
        occupancy_map[coordinate_hash(hive.x, hive.y)] = hive.id
    for sugar in game_state.sugars:
        sugars[sugar.id] = sugar
        occupancy_map[coordinate_hash(sugar.x, sugar.y)] = sugar.id
    print("Game map: {}x{}".format(MAP_SIZE, MAP_SIZE))
    for y in range(MAP_SIZE):
        line = ''
        for x in range(MAP_SIZE):
            key = coordinate_hash(x, y)
            id = occupancy_map.get(key, None)
            if id is None:
                line = line + light_grey_text(" ")
            elif id in sugars:
                line = line + light_grey_text('#')
            elif id in ants:
                if ants[id].player_id == PLAYER_ONE_ID:
                    line = line + red_text('*')
                elif ants[id].player_id == PLAYER_TWO_ID:
                    line = line + blue_text('*')
                else:
                    line = line + " "
            elif id in hives:
                if hives[id].player_id == PLAYER_ONE_ID:
                    line = line + red_text('%')
                elif hives[id].player_id == PLAYER_TWO_ID:
                    line = line + blue_text('%')
                else:
                    line = line + " "
            else:
                line = line + " "
        print(line)
    player_one_score = None
    player_two_score = None
    game_tick = game_state.tick
    for player in game_state.players:
        if player.id == PLAYER_ONE_ID:
            player_one_score = player.score
        if player.id == PLAYER_TWO_ID:
            player_two_score = player.score
    print("Red score: {:3d} Blue score: {:3d} tick:  {:5d}".format(player_one_score, player_two_score, game_tick))


if __name__ == '__main__':
    rospy.init_node('console_map_viewer')
    rospy.Subscriber("/antcolony_server_node/game_state", State, visualize_game_callback)
    print("INFO: Ready to visualize..")
    rospy.spin()