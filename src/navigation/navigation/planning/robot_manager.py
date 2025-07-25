from .robot_class import Robot, RobotMap
from .time_step_node_class import TimeStepNode
import copy
import uuid

def euclidean_distance(pos1, pos2):
    """Calculate the Euclidean distance between two positions."""
    return ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) ** 0.5

def _known_properties(visited_locations, location_to_prop : dict[str, list[str]]) -> set[str]:
    """Returns the set of properties that are known to be true in the visited locations."""
    known_props = set()
    for loc in visited_locations:
        for prop in location_to_prop[loc]:
            known_props.add(prop)
    return known_props

DISTANCE_TOLERANCE = 0.01

class RobotManager:
    next_question_map : dict[str, list[str]] = {}
    head_time_step_node : TimeStepNode = None
    initial_question : str = ''
    time_step_queue : list[TimeStepNode] = []
    props : set[str] = set()
    location_to_pin : dict[str, tuple[int, int]] = {}
    pin_to_location: dict[tuple[int, int], str] = {}
    location_to_prop : dict[str, list[str]] = {}
    initial_resolution : dict[str, str] = {}

    def __init__(self, robot_map, next_question_map, initial_question, props, location_to_pin=None, pin_to_location=None, location_to_prop=None, initial_resolution=None):
        self.next_question_map = next_question_map
        self.initial_question = initial_question
        self.props = props
        self.location_to_pin = location_to_pin
        self.pin_to_location = pin_to_location
        self.location_to_prop = location_to_prop

        robot_map_copy = copy.deepcopy(robot_map)

        start_node = TimeStepNode(
            id = str(uuid.uuid1()),
            robot_map = robot_map_copy,
            query = initial_question,
            next = [],
            type = 'robot_assignment',
            resolved_questions= copy.deepcopy(initial_resolution) if initial_resolution else {},
        )
        start_node.visited_locations = set()
        self.head_time_step_node = TimeStepNode(
            id = str(uuid.uuid1()),
            robot_map = robot_map_copy,
            query = initial_question,
            next = [start_node],
            type = 'query',
            resolved_questions= copy.deepcopy(initial_resolution) if initial_resolution else {},
        )
        self.head_time_step_node.visited_locations = set()
        self.time_step_queue = []
        self.time_step_queue.append(start_node)

    def count_traveling_robots(self, robot_map: RobotMap) -> int:
        """Counts the number of robots that are currently traveling."""
        count = 0
        for robot in robot_map.values():
            if robot.assigned_loc == '' or robot.assigned_loc is None:
                continue
            
            target_location = self.location_to_pin[robot.assigned_loc]

            if euclidean_distance(robot.position, target_location) > DISTANCE_TOLERANCE:
                count += 1
        return count


    def possible_resolutions(self, index : int, known_properties : list[str], resolved_questions : dict[str, str]) -> list[dict[str, str]]:
        resolutions : list[dict[str, str]] = []

        if index < 0 or index >= len(known_properties):
            resolutions.append(copy.deepcopy(resolved_questions))
            return resolutions
        
        property = known_properties[index]
        if property in resolved_questions:
            return self.possible_resolutions(index + 1, known_properties, resolved_questions)

        true_resolution = copy.deepcopy(resolved_questions)
        true_resolution[property] = 'T'
        resolutions.extend(self.possible_resolutions(index + 1, known_properties, true_resolution))

        false_resolution = copy.deepcopy(resolved_questions)
        false_resolution[property] = 'F'
        resolutions.extend(self.possible_resolutions(index + 1, known_properties, false_resolution))

        return resolutions
    
    def update_time_step(self, current_time_step: TimeStepNode, visited_locations_this_step: set[str]):
        resolved_questions = copy.deepcopy(current_time_step.resolved_questions)
        robot_map = copy.deepcopy(current_time_step.robot_map)

        new_visited_locations = copy.deepcopy(current_time_step.visited_locations)
        new_visited_locations.update(visited_locations_this_step)

        known_properties = _known_properties(new_visited_locations, self.location_to_prop)
        query = copy.deepcopy(current_time_step.query)
        possible_resolutions = self.possible_resolutions(0, copy.deepcopy(list(known_properties)), resolved_questions)
        for resolution in possible_resolutions:
            next_question = copy.deepcopy(query)

            while next_question in resolution:
                truth = resolution[next_question] == 'T'

                if truth:
                    next_question = self.next_question_map[next_question][1]
                else:
                    next_question = self.next_question_map[next_question][0]

            if next_question == current_time_step.query:
                continue

            next_time_step = TimeStepNode(
                id = str(uuid.uuid1()),
                robot_map = copy.deepcopy(robot_map),
                query = next_question,
                next = [],
                type = 'robot_assignment',
                resolved_questions= copy.deepcopy(resolution),
            )
            next_time_step.visited_locations = copy.deepcopy(new_visited_locations)
            current_time_step.next.append(next_time_step)

            if next_question in self.props:
                self.time_step_queue.append(next_time_step)

    def update_robot_positions(self, robot_map: RobotMap):
        minimal_arrival_time = float('inf')
        robots_that_arrived : list[Robot] = []
        
        
        for robot in robot_map.values():
            arrival_time = robot.time
            location_pin = robot.position
            if robot.assigned_loc in self.location_to_pin:
                location_pin = self.location_to_pin[robot.assigned_loc]

            dist = euclidean_distance(location_pin, robot.position)

            if dist < DISTANCE_TOLERANCE:
                robots_that_arrived.append(robot)
                continue
            elif arrival_time + dist / robot.velocity < minimal_arrival_time:
                arrival_time = robot.time + dist / robot.velocity
                minimal_arrival_time = arrival_time
        
        def move_robot_towards_location(robot: Robot, target_location: tuple[int, int], time_diff: float):
            distance = euclidean_distance(robot.position, target_location)
            distance_traveled = robot.velocity * time_diff
            if (distance < distance_traveled) or (abs(distance - distance_traveled) < DISTANCE_TOLERANCE):
                # Robot is already at the target location or can reach it
                robot.position = (target_location[0], target_location[1])
                robot.cost += distance
            else :             

                direction = [
                    target_location[0] - robot.position[0],
                    target_location[1] - robot.position[1]
                ]
                
                direction_magnitude = (direction[0] ** 2 + direction[1] ** 2) ** 0.5
                normalized_direction = [
                    direction[0] / direction_magnitude,
                    direction[1] / direction_magnitude
                ]
                new_position = (
                    robot.position[0] + normalized_direction[0] * time_diff * robot.velocity,
                    robot.position[1] + normalized_direction[1] * time_diff * robot.velocity
                )

                robot.position = new_position
                robot.cost += distance_traveled
        

        for robot in robot_map.values():
            time_diff = minimal_arrival_time - robot.time
            robot.time = minimal_arrival_time

            if robot.assigned_loc != '':
                target_location = self.location_to_pin[robot.assigned_loc]
                if target_location == None:
                    target_location = robot.position
                move_robot_towards_location(robot, target_location, time_diff)

        return robots_that_arrived

    def assign_robot_to_location(self, robot_id: str, location: str, robot_map: RobotMap):
        robot = robot_map[robot_id]
        robot.assigned_loc = location

    def generate_combinations(self, property: str, robot_map: RobotMap, visited_locations: set[str]) -> list[dict[str, str]]:
        locations = list(self.location_to_pin.keys())
        robot_ids = list(robot_map.keys())
        combinations : list[dict [str, str]] = []
        
        property_locations = []
        for loc in locations:
            if property in self.location_to_prop[loc]:
                property_locations.append(loc)

        if len(property_locations) == 0:
            return combinations
            
        
        def generate_assignments(robot_index: int, current_assignment: dict[str, str], used_locations: set[str] = set()):
            if robot_index == len(robot_ids):
                # Check if at least one robot is assigned to a the property location
                values = current_assignment.values()
                flag = False
                for loc in values:
                    if loc in property_locations:
                        flag = True
                        break
                    
                if flag:
                    combinations.append(copy.deepcopy(current_assignment))

                return
                        

            robot_id = robot_ids[robot_index]
            known_props = _known_properties(used_locations, self.location_to_prop)
            generate_assignments(robot_index + 1, current_assignment, used_locations)  # Skip this robot

            for location in locations:
                props = self.location_to_prop[location]
                skip = True
                for prop in props:
                    if prop not in known_props:
                        skip = False
                        break

               
                
                if (location not in used_locations) and (not skip):
                    new_assignment = copy.deepcopy(current_assignment)
                    new_used_locations = copy.deepcopy(used_locations)
                    new_assignment[robot_id] = location
                    new_used_locations.add(location)
                    generate_assignments(robot_index + 1, new_assignment, new_used_locations)

        generate_assignments(0, {}, set(visited_locations))
        return combinations


