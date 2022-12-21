#!/usr/bin/env python
import os
import random
import uuid

import rospy

import antcolony_msgs.srv
from antcolony_msgs.msg import Action, Ant, Hive, Sugar, Order, Player, State

CONF_USE_CLOSEST_NEIGHBOR = True


class GameStateFunctions:
    """
    By design behavior tree tasks get state through an object of this class
    """

    def __init__(self, game_state, player_id, map_size):
        self.game_state = game_state
        self.player_id = player_id
        self.map_size = map_size
        self.max_coord = map_size - 1

        self.my_ants = {}
        self.enemy_ants = {}
        self.ants = {}
        self.sugars = {}
        self.my_hive = None
        self.enemy_hive = None
        self.occupancy_map = {}
        self.object_map = {}

    def coord_hash(self, x, y):
        return self.map_size * y + x

    def aggregate_information(self):
        # Hashing ant data
        for ant in self.game_state.ants:
            self.occupancy_map[self.coord_hash(ant.x, ant.y)] = ant.id
            self.object_map[ant.id] = ant
            self.ants[ant.id] = ant
            if ant.player_id == self.player_id:
                self.my_ants[ant.id] = ant
            else:
                self.enemy_ants[ant.id] = ant
        # Hashing hive data
        for hive in self.game_state.hives:
            self.occupancy_map[self.coord_hash(hive.x, hive.y)] = hive.id
            self.object_map[hive.id] = hive
            if hive.player_id == self.player_id:
                self.my_hive = hive
            else:
                self.enemy_hive = hive

        # Hashing sugar data
        for sugar in self.game_state.sugars:
            self.occupancy_map[self.coord_hash(sugar.x, sugar.y)] = sugar.id
            self.object_map[sugar.id] = sugar
            self.sugars[sugar.id] = sugar

    @staticmethod
    def get_manhattan_distance(x1, y1, x2, y2):
        return abs(x1 - x2) + abs(y1 - y2)

    def is_adjacent(self, x1, y1, x2, y2):
        return self.get_manhattan_distance(x1, y1, x2, y2) == 1

    def generate_neighbors(self, x, y, ant_id=None):
        neighbors = []
        if CONF_USE_CLOSEST_NEIGHBOR and ant_id is not None:
            ant = self.ants[ant_id]
            closest_neighbor = None
            closest_distance = 1000
            for neighbor in self.generate_neighbors(x, y):
                distance = self.get_manhattan_distance(
                    ant.x, ant.y, neighbor[0], neighbor[1]
                )
                if distance < closest_distance:
                    closest_distance = distance
                    closest_neighbor = neighbor
            if closest_neighbor is not None:
                neighbors.append(closest_neighbor)
        # TODO: do not append the closest neighbor twice.
        if x > 0:
            neighbors.append((x - 1, y))
        if y > 0:
            neighbors.append((x, y - 1))
        if x < self.max_coord:
            neighbors.append((x + 1, y))
        if y < self.max_coord:
            neighbors.append((x, y + 1))
        for n in neighbors:
            yield n

    def is_occupied(self, x, y):
        return self.coord_hash(x, y) in self.occupancy_map

    def get_id_at_coords(self, x, y):
        key = self.coord_hash(x, y)
        if key in self.occupancy_map:
            return self.occupancy_map[key]
        else:
            return None

    def object_exists(self, id):
        return id in self.object_map

    def my_ant_exists(self, id):
        return id in self.my_ants

    def sugar_exists(self, id):
        return id in self.sugars

    def get_ant_coords(self, id):
        ant = self.ants[id]
        return ant.x, ant.y

    def get_sugar_coords(self, id):
        sugar = self.sugars[id]
        return sugar.x, sugar.y

    def all_my_ants(self):
        for ant in self.my_ants.values():
            yield ant

    def all_sugars(self):
        for sugar in self.sugars.values():
            yield sugar


class GamePlayControl:
    """
    By design behavior tree tasks act on game state through this class
    """

    def __init__(self):
        self.actions = {}

    def add_action(self, ant_id, x, y, action_type):
        action = Action()
        action.id = ant_id
        action.x = x
        action.y = y
        action.action_type = action_type
        self.actions[ant_id] = action

    def cancel_action(self, ant_id):
        if ant_id in self.actions:
            del self.actions[ant_id]

    def get_actions(self):
        return self.actions.values()


class TaskState:
    """
    Enum for states of behavior tree task
    """

    RUNNING = "RUNNING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"


class BaseTask:
    """
    A base class for behaviour tree task
    """

    def __init__(self):
        self.state = TaskState.RUNNING

    def has_terminal_state(self):
        return self.state != TaskState.RUNNING

    def is_valid(self):
        return True

    def is_succeeded(self):
        return self.state == TaskState.SUCCEEDED

    def set_succeeded(self):
        self.state = TaskState.SUCCEEDED

    def is_failed(self):
        return self.state == TaskState.FAILED

    def set_failed(self):
        self.state = TaskState.FAILED

    # Child class overrides this method
    def run_impl(self):
        pass

    def run(self):
        if self.has_terminal_state():
            return
        if self.is_succeeded():
            self.state = TaskState.SUCCEEDED
            return
        if not self.is_valid():
            self.state = TaskState.FAILED
            return

        self.run_impl()


class GameControlTask(BaseTask):
    """
    A base class for behavior tree task with access to reading current situation and acting on it
    """

    def __init__(self):
        BaseTask.__init__(self)
        self.game_functions = None
        self.game_play_control = None

    # Some classes may need to override this method
    def update_child_states(self, game_state_functions, game_play_control):
        pass

    def update_state(self, game_state_functions, game_play_control):
        self.game_functions = game_state_functions
        self.game_play_control = game_play_control
        self.update_child_states(self.game_functions, self.game_play_control)


class MoveAntToTargetPointTask(GameControlTask):
    """
    Until ant exists move to target point
    """

    def __init__(self, ant_id, target_x, target_y):
        GameControlTask.__init__(self)
        self.ant_id = ant_id
        self.target_x = target_x
        self.target_y = target_y

    def is_valid(self):
        return self.game_functions.my_ant_exists(
            self.ant_id
        ) and not self.game_functions.is_occupied(self.target_x, self.target_y)

    def is_succeeded(self):
        ant_x, ant_y = self.game_functions.get_ant_coords(self.ant_id)
        return (ant_x == self.target_x) and (ant_y == self.target_y)

    def run_impl(self):
        self.game_play_control.add_action(
            self.ant_id, self.target_x, self.target_y, Action.MOVE_ACTION
        )


class AttackTargetAtPointTask(GameControlTask):
    """
    Until ant exists and adjacent target exists at target coordinates attack it
    """

    def __init__(self, ant_id, target_x, target_y, target_id):
        GameControlTask.__init__(self)
        self.ant_id = ant_id
        self.target_x = target_x
        self.target_y = target_y
        self.target_id = target_id

    def is_valid(self):
        ant_x, ant_y = self.game_functions.get_ant_coords(self.ant_id)
        id_at_target_coords = self.game_functions.get_id_at_coords(
            self.target_x, self.target_y
        )
        return (
            self.game_functions.my_ant_exists(self.ant_id)
            and self.game_functions.is_adjacent(
                ant_x, ant_y, self.target_x, self.target_y
            )
            and id_at_target_coords is not None
            and id_at_target_coords == self.target_id
        )

    def is_succeeded(self):
        return not self.game_functions.object_exists(self.target_id)

    def run_impl(self):
        self.game_play_control.add_action(
            self.ant_id, self.target_x, self.target_y, Action.ATTACK_ACTION
        )


class SucceedAllTask(GameControlTask):
    """
    Runs a list of tasks sequentially until first failure
    Succeeds if last tasks succeeded
    """

    def __init__(self, task_list):
        GameControlTask.__init__(self)
        self.task_list = task_list
        self.position_index = 0

    def update_child_states(self, game_state_functions, game_play_control):
        for task in self.task_list:
            task.update_state(game_state_functions, game_play_control)

    def is_valid(self):
        return (
            self.position_index < len(self.task_list)
            and not self.task_list[self.position_index].has_terminal_state()
        )

    def is_succeeded(self):
        return self.task_list[-1].is_succeeded()

    def run_impl(self):
        self.task_list[self.position_index].run()
        if self.task_list[self.position_index].is_succeeded():
            self.position_index += 1


class SucceedAnyTask(GameControlTask):
    """
    Runs a list of tasks sequentially until first success
    Succeeds if any task succeeds
    """

    def __init__(self, task_list):
        GameControlTask.__init__(self)
        self.task_list = task_list
        self.position_index = 0

    def update_child_states(self, game_state_functions, game_play_control):
        for task in self.task_list:
            task.update_state(game_state_functions, game_play_control)

    def is_valid(self):
        return self.position_index < len(self.task_list)

    def is_succeeded(self):
        for task in self.task_list:
            if task.is_succeeded():
                return True
        return False

    def run_impl(self):
        self.task_list[self.position_index].run()
        if self.task_list[self.position_index].has_terminal_state():
            self.position_index += 1


class MineSugarTask(GameControlTask):
    """
    Try to mine sugar by reaching it from one of sides.
    Has references to a given sugar (by its coordinates) and a given ant (by its id).
    Also has children tasks to move to target point and mine sugar at point.
    """

    def __init__(self, ant_id, sugar_id):
        GameControlTask.__init__(self)
        self.ant_id = ant_id
        self.sugar_id = sugar_id
        self.child_any_task = None

    def setup(self):
        any_task_list = []
        sugar_x, sugar_y = self.game_functions.get_sugar_coords(self.sugar_id)
        for target_x, target_y in self.game_functions.generate_neighbors(
            sugar_x, sugar_y, self.ant_id
        ):
            move_task = MoveAntToTargetPointTask(self.ant_id, target_x, target_y)
            attack_task = AttackTargetAtPointTask(
                self.ant_id, sugar_x, sugar_y, self.sugar_id
            )
            all_task = SucceedAllTask([move_task, attack_task])
            any_task_list.append(all_task)
        self.child_any_task = SucceedAnyTask(any_task_list)

    def update_child_states(self, game_state_functions, game_play_control):
        if self.child_any_task is not None:
            self.child_any_task.update_state(game_state_functions, game_play_control)

    def is_valid(self):
        return not self.child_any_task.has_terminal_state()

    def is_succeeded(self):
        return self.child_any_task.is_succeeded()

    def run_impl(self):
        self.child_any_task.run()


class QueenTask(GameControlTask):
    """
    Orders ants to mine sugar. Runs once per tick.
    """

    def __init__(self):
        GameControlTask.__init__(self)
        self.mine_sugar_tasks = {}

    def update_child_states(self, game_state_functions, game_play_control):
        for child_task in self.mine_sugar_tasks.values():
            child_task.update_state(game_state_functions, game_play_control)

    def create_mine_sugar_task(self, ant):
        sugars = []
        for sugar in self.game_functions.all_sugars():
            sugars.append(sugar)
        random.shuffle(sugars)
        if len(sugars) > 0:
            mine_sugar_task = MineSugarTask(ant.id, sugars[0].id)
            mine_sugar_task.update_state(self.game_functions, self.game_play_control)
            mine_sugar_task.setup()
            return mine_sugar_task
        return None

    def run_impl(self):
        for ant in self.game_functions.all_my_ants():
            if (
                ant.id in self.mine_sugar_tasks
                and not self.mine_sugar_tasks[ant.id].has_terminal_state()
            ):
                # This and already has a sugar mining task, run it.
                self.mine_sugar_tasks[ant.id].run()
            else:
                mine_sugar_task = self.create_mine_sugar_task(ant)
                if mine_sugar_task is not None:
                    self.mine_sugar_tasks[ant.id] = mine_sugar_task


class RegisterTask(BaseTask):
    """
    Registers client to play a game
    """

    def __init__(self, player_token):
        BaseTask.__init__(self)
        self.player_token = player_token

    def run_impl(self):
        try:
            rospy.wait_for_service("/antcolony_server_node/register_cmd", timeout=3.0)
        except rospy.ROSException:
            print("ERROR: Wait for service join_cmd failed by timeout")
            self.state = TaskState.FAILED
            rospy.signal_shutdown("Wait for service join_cmd failed")
            return
        except rospy.ROSInterruptException:
            rospy.signal_shutdown("Interrupt")
            exit(1)
        try:
            register_cmd = rospy.ServiceProxy(
                "/antcolony_server_node/register_cmd", antcolony_msgs.srv.Register
            )
            response = register_cmd(self.player_token)
            if response.status == antcolony_msgs.srv.RegisterResponse.STATUS_OK:
                self.state = TaskState.SUCCEEDED
            else:
                print(
                    "ERROR: Command to join game failed: {}".format(
                        response.error_message
                    )
                )
                self.state = TaskState.FAILED

        except rospy.ServiceException as e:
            print("ERROR: join_cmd service call failed: %s" % e)
            self.state = TaskState.FAILED


class PlayGameTask(BaseTask):
    def __init__(self, player_token, player_id):
        BaseTask.__init__(self)
        self.game_state = None
        self.player_token = player_token
        self.player_id = player_id
        self.queen_task = QueenTask()
        self.play_service_available = False

    def set_game_state(self, game_state):
        self.game_state = game_state

    def run_impl(self):
        functions = GameStateFunctions(self.game_state, self.player_id, 40)
        functions.aggregate_information()
        control = GamePlayControl()
        self.queen_task.update_state(functions, control)
        self.queen_task.run()
        order = Order()
        order.token = self.player_token
        order.actions = control.get_actions()
        if not self.play_service_available:
            try:
                rospy.wait_for_service("/antcolony_server_node/play_cmd", timeout=3.0)
            except rospy.ROSException:
                print("ERROR: Wait for service play_cmd failed by timeout")
                self.state = TaskState.FAILED
                rospy.signal_shutdown("Wait for service play_cmd failed")
                return
            except rospy.ROSInterruptException:
                print("ERROR: Interrupted")
                rospy.signal_shutdown("Interrupt")
                exit(1)
            self.play_service_available = True
        try:
            play_cmd = rospy.ServiceProxy(
                "/antcolony_server_node/play_cmd", antcolony_msgs.srv.Play
            )
            response = play_cmd(order)
            if response.status != antcolony_msgs.srv.PlayResponse.STATUS_OK:
                print(
                    "ERROR: Command to play game failed: {}".format(
                        response.error_message
                    )
                )

        except rospy.ServiceException as e:
            print("ERROR: play_cmd service call failed: %s" % e)
            self.state = TaskState.FAILED

        except rospy.ROSInterruptException:
            print("ERROR: Interrupted")
            rospy.signal_shutdown("Interrupt")
            exit(1)


def game_state_callback(game_state):
    """Rus once per tick."""
    global play_game_task
    play_game_task.set_game_state(game_state)
    play_game_task.run()
    if game_state.tick == 499:
        rospy.signal_shutdown("INFO: Game is over")


if __name__ == "__main__":
    rospy.init_node("antcolony_client_{}".format(str(uuid.uuid4())[0:8]))
    player_token = os.environ.get("ANTCOLONY_PLAYER_TOKEN")
    player_id = int(os.environ.get("ANTCOLONY_PLAYER_ID"))  # type: ignore
    print("Env setting: %s" % os.environ.get("ANTCOLONY_USE_CLOSEST_NEIGHBOR"))
    CONF_USE_CLOSEST_NEIGHBOR = (
        False
        if not os.environ.get("ANTCOLONY_USE_CLOSEST_NEIGHBOR")
        and (
            str(os.environ.get("ANTCOLONY_USE_CLOSEST_NEIGHBOR")).lower()
            in ["0", "false", "no", "none"]
        )
        else True
    )

    print("Ants going for closest neighbor: %s" % CONF_USE_CLOSEST_NEIGHBOR)

    # Registering for the game
    register_task = RegisterTask(player_token)
    while True:
        register_task.run()
        if register_task.state == TaskState.SUCCEEDED:
            break

    print("INFO: Registration completed..")

    # Subscribing to game state and playing
    global play_game_task
    play_game_task = PlayGameTask(player_token, player_id)
    rospy.Subscriber("/antcolony_server_node/game_state", State, game_state_callback)

    print("INFO: Ready to play..")

    rospy.spin()
