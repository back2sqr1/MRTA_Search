import copy
import json, uuid
from .robot_class import Robot, RobotMap
from .robot_manager import RobotManager, euclidean_distance, DISTANCE_TOLERANCE   
from .time_step_node_class import TimeStepNode
import os

COST_TOLERANCE = 0.001

class RobotAssignments:
    def __init__(self, node : TimeStepNode, location_to_pin: dict[str, tuple[int, int]]):
        self.list = []
        self.node = node
        self.location_to_pin = location_to_pin
        for robot_id, robot in node.robot_map.items():
            if robot.assigned_loc != '':
                self.list.append((robot_id, location_to_pin[robot.assigned_loc]))

    def __str__(self):
        if not self.list:
            return "No robot assignments"
        
        assignments = []

        for robot_id, position in self.list:
            robot = self.node.robot_map[robot_id]
            if robot.assigned_loc != '':
                assignments.append(f"{robot_id} -> {position}")

        return "Robot Assignments: " + ", ".join(assignments)

    def __repr__(self):
        return self.__str__()

class SearchTree:
    def __init__(self):
        self.location_to_pin : dict[str, tuple[int, int]] = {}
        self.pin_to_location : dict[tuple[int, int], str] = {}
        self.location_to_prop : dict[str, list[str]] = {}
        self.prop_to_location: dict[str, str] = {}
        self.props : set[str] = set()
        self.starting_prop: str = ''
        self.next_query: dict[str, list[str]] = {}
        self.cost_map: dict[str, float] = {}
        self.bdd_config = self.import_bdd_config()
        

    def import_bdd_config(self):
        with open ('src/my_robot_bringup/config/bdd.json', 'r') as file:
            bdd_config = json.load(file)

        self.props = set(bdd_config['props'])
        for prop in self.props:
            self.prop_to_location[prop] = []

        
        for loc, pin in bdd_config['locations'].items():
            tuple_pin = tuple(pin)
            self.location_to_pin[loc] = tuple_pin
            self.location_to_prop[loc] = []
            self.pin_to_location[tuple_pin] = loc

        self.prop_to_location = bdd_config['prop_to_location']
        for prop, locs in bdd_config['prop_to_location'].items():
            for loc in locs:
                if loc not in self.location_to_prop:
                    self.location_to_prop[loc] = []
                self.location_to_prop[loc].append(prop)

        self.starting_prop = bdd_config['starting_prop']
        self.next_query = bdd_config['edges'] 

        return bdd_config
    
    def known_properties(self, visited_locations : set[str]) -> set[str]:
        """Returns the set of properties that are known to be true in the visited locations."""
        known_props = set()
        for loc in visited_locations:
            for prop in self.location_to_prop[loc]:
                if prop not in known_props:
                    known_props.add(prop)
        return known_props

    def check_robot_destinations(self, robot_map: dict[str, Robot]):
        visited_locations = set()

        for robot in robot_map.values():
            if robot.assigned_loc == '':
                continue
            target_location = self.location_to_pin[robot.assigned_loc]
            if euclidean_distance(robot.position, target_location) < DISTANCE_TOLERANCE:
                visited_locations.add(robot.assigned_loc)
        
        return visited_locations

    def process_robot_movement(self, robot_manager: RobotManager, robot_map: RobotMap, current_node: TimeStepNode):
        while robot_manager.count_traveling_robots(robot_map=robot_map) > 0:
            arrived_robots = robot_manager.update_robot_positions(robot_map=robot_map)
            
 
            robot_moving_node = TimeStepNode(
                robot_map=robot_map,
                id = str(uuid.uuid1()),
                query = current_node.query,
                type = 'robot_moving',
                resolved_questions = current_node.resolved_questions,
                next = [],
            )
            robot_moving_node.visited_locations = copy.deepcopy(current_node.visited_locations)
            current_node.next.append(robot_moving_node)
            current_node = robot_moving_node

            

            visited_locations = copy.deepcopy(current_node.visited_locations)

            query_node = TimeStepNode(
                robot_map=robot_map,
                id = str(uuid.uuid1()),
                query = current_node.query,
                type = 'query',
                resolved_questions = current_node.resolved_questions,
                next = []
            )
            query_node.visited_locations = visited_locations
            query_node.visited_locations.update(self.check_robot_destinations(robot_map))
            for robot in arrived_robots:
                query_node.robot_map[robot.id].assigned_loc = ''


            current_node.next.append(query_node)
            current_node = query_node
            robot_manager.update_time_step(current_node, visited_locations)
        return current_node

    def process_combinations(self, combination: dict[str, str], robot_manager: RobotManager, current_time_step: TimeStepNode, robot_map_original: RobotMap):
        robot_map = copy.deepcopy(robot_map_original)
        for robot_id, location in combination.items():
            robot_manager.assign_robot_to_location(robot_id=robot_id, location=location, robot_map=robot_map)
        
        self.process_robot_movement(robot_manager, robot_map, current_time_step)

    def search(self, initial_robot_map: RobotMap, initial_resolution: dict[str, str]) -> TimeStepNode:
        robot_manager = RobotManager(
            robot_map=copy.deepcopy(initial_robot_map),
            next_question_map=self.next_query,
            initial_question=self.starting_prop,
            props=self.props,
            location_to_pin=self.location_to_pin,
            pin_to_location=self.pin_to_location,
            location_to_prop=self.location_to_prop,
            initial_resolution=copy.deepcopy(initial_resolution)
        )

        while robot_manager.time_step_queue: 
            current_time_step = robot_manager.time_step_queue.pop(0)
        
            if not current_time_step.robot_map:
                continue
            
            original_robot_map = copy.deepcopy(current_time_step.robot_map)
            combinations = robot_manager.generate_combinations(
                property=current_time_step.query, 
                robot_map=original_robot_map,
                visited_locations=current_time_step.visited_locations
            )

            for combination in combinations:
                self.process_combinations(combination=combination,
                                          robot_manager=robot_manager,
                                          current_time_step=current_time_step,
                                          robot_map_original=original_robot_map)
        return robot_manager.head_time_step_node
    

    def determine_cost(self, node: TimeStepNode, recursive_count:int = 0) -> float:
        if len(node.next) == 0:
            cost = node.get_cost()
            self.cost_map[node.id] = cost
            return node.get_cost()

        if node.id in self.cost_map:
            return self.cost_map[node.id]

        if node.type == 'robot_moving':
            return self.determine_cost(node.next[0], recursive_count + 1)

        elif node.type == 'query':
            cost = 0
            for next_node in node.next:
                cost = max(cost, self.determine_cost(next_node, recursive_count + 1))
            return cost

        elif node.type == 'robot_assignment':
            cost = float('inf')
            for next_node in node.next:
                cost = min(cost, self.determine_cost(next_node, recursive_count + 1))
            return cost
        else:
            raise ValueError(f"Unknown node type: {node.type}")
        
    

    # By cost = cumulative distance traveled by all robots
    def get_best_plan(self, initial_robot_map: RobotMap, initial_resolution: dict[str, str]) -> tuple[list[(str, tuple[int, int])], list[str]]:
        cur_node = self.search(initial_robot_map, initial_resolution)
        best_cost = self.determine_cost(cur_node)
        best_plan_text = []
        best_plan : list[(str, tuple[int, int])] = []
        while cur_node is not None:
            with open ('current_node.txt', 'a') as f:
                f.write(f"{cur_node}\n")
            for next_node in cur_node.next:
                if (abs(self.determine_cost(next_node) - best_cost)) < COST_TOLERANCE:
                    if next_node.type == 'robot_moving':
                        best_plan_text.append(str(RobotAssignments(next_node, self.location_to_pin)))
                    elif next_node.type == 'query':
                        best_plan_text.append(next_node.resolved_questions)
                    elif next_node.type == 'robot_assignment':
                        for robot_id, robot in next_node.robot_map.items():
                            if robot.assigned_loc != '':
                                best_plan.append((robot_id, self.location_to_pin[robot.assigned_loc]))

                                best_plan_text.append(f"{robot_id} -> {robot.assigned_loc}")

                    cur_node = next_node
                    break

            else:
                cur_node = None

        return (best_plan, best_plan_text)

# search_tree = SearchTree()
# initial_robot_map = RobotMap({
#     'robot_1': Robot(id='robot_1', position=(1, 1)),
#     'robot_2': Robot(id='robot_2', position=(2, 2))
# })
# initial_resolution = {
#     'p': 'T'
# }
# if os.path.exists('current_node.txt'):
#     os.remove('current_node.txt')
# if os.path.exists('best_path.txt'):
#     os.remove('best_path.txt')

# best_plan, best_plan_text = search_tree.get_best_plan(initial_robot_map, initial_resolution)



# with open('best_path.txt', 'a') as f:
#     for assignment in best_plan_text:
#         f.write(f"{assignment}\n")

# print(best_plan)
    



