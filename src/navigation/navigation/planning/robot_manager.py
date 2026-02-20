from .robot_class import Robot, RobotMap, clone_robot_map
from .time_step_node_class import TimeStepNode
from collections import deque
import uuid
import random

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
HEDGE_LOC_PREFIX = '~hedge_'


def _trivial_circle(boundary):
    """Minimum enclosing circle of 0, 1, 2, or 3 boundary points."""
    if len(boundary) == 0:
        return ((0.0, 0.0), 0.0)
    if len(boundary) == 1:
        return (boundary[0], 0.0)
    if len(boundary) == 2:
        cx = (boundary[0][0] + boundary[1][0]) / 2.0
        cy = (boundary[0][1] + boundary[1][1]) / 2.0
        r = euclidean_distance(boundary[0], (cx, cy))
        return ((cx, cy), r)
    # 3 points: compute circumcenter
    ax, ay = boundary[0]
    bx, by = boundary[1]
    cx, cy = boundary[2]
    D = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(D) < 1e-10:
        # Collinear: MEC of the two most distant points
        d01 = euclidean_distance(boundary[0], boundary[1])
        d12 = euclidean_distance(boundary[1], boundary[2])
        d02 = euclidean_distance(boundary[0], boundary[2])
        if d01 >= d12 and d01 >= d02:
            return _trivial_circle([boundary[0], boundary[1]])
        elif d12 >= d01 and d12 >= d02:
            return _trivial_circle([boundary[1], boundary[2]])
        else:
            return _trivial_circle([boundary[0], boundary[2]])
    sa = ax * ax + ay * ay
    sb = bx * bx + by * by
    sc = cx * cx + cy * cy
    ux = (sa * (by - cy) + sb * (cy - ay) + sc * (ay - by)) / D
    uy = (sa * (cx - bx) + sb * (ax - cx) + sc * (bx - ax)) / D
    center = (ux, uy)
    r = euclidean_distance(center, boundary[0])
    return (center, r)


def _in_circle(circle, p):
    """Return True if p is inside or on the boundary of circle (with epsilon tolerance)."""
    center, r = circle
    return euclidean_distance(center, p) <= r + 1e-10


def _welzl(pts, boundary, n):
    """Recursive Welzl algorithm for minimum enclosing circle of pts[0:n]."""
    if n == 0 or len(boundary) == 3:
        return _trivial_circle(boundary)
    p = pts[n - 1]
    D = _welzl(pts, boundary, n - 1)
    if _in_circle(D, p):
        return D
    return _welzl(pts, boundary + [p], n - 1)


def min_enclosing_circle(points):
    """Compute the minimum enclosing circle of a list of (x, y) points.

    Uses Welzl's algorithm (O(n) expected time after shuffling).
    Returns (center, radius) where center is (x, y).
    """
    pts = list(points)
    random.shuffle(pts)
    return _welzl(pts, [], len(pts))


class RobotManager:
    next_question_map : dict[str, list[str]] = {}
    head_time_step_node : TimeStepNode = None
    initial_question : str = ''
    time_step_queue: deque = deque()
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
        self.hedge_locations: set[str] = set()

        robot_map_copy = clone_robot_map(robot_map)

        # Compute initial visited locations from robot starting positions
        initial_visited = set()
        if pin_to_location:
            for robot in robot_map_copy.values():
                pos_tuple = tuple(robot.position) if not isinstance(robot.position, tuple) else robot.position
                if pos_tuple in pin_to_location:
                    initial_visited.add(pin_to_location[pos_tuple])

        start_node = TimeStepNode(
            id = str(uuid.uuid1()),
            robot_map = robot_map_copy,
            query = initial_question,
            next = [],
            type = 'robot_assignment',
            resolved_questions= initial_resolution.copy() if initial_resolution else {},
        )
        start_node.visited_locations = set(initial_visited)
        self.head_time_step_node = TimeStepNode(
            id = str(uuid.uuid1()),
            robot_map = robot_map_copy,
            query = initial_question,
            next = [start_node],
            type = 'query',
            resolved_questions= initial_resolution.copy() if initial_resolution else {},
        )
        self.head_time_step_node.visited_locations = set(initial_visited)
        self.time_step_queue = deque([start_node])

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
            resolutions.append(resolved_questions.copy())
            return resolutions

        property = known_properties[index]
        if property in resolved_questions:
            return self.possible_resolutions(index + 1, known_properties, resolved_questions)

        true_resolution = resolved_questions.copy()
        true_resolution[property] = 'T'
        resolutions.extend(self.possible_resolutions(index + 1, known_properties, true_resolution))

        false_resolution = resolved_questions.copy()
        false_resolution[property] = 'F'
        resolutions.extend(self.possible_resolutions(index + 1, known_properties, false_resolution))

        return resolutions
    
    def _advance_query(self, query, known_properties, resolved_questions):
        """Walk the BDD from query, branching only on known properties on the path.

        Instead of enumerating all 2^n resolution combinations and then
        walking the BDD for each, this walks the BDD directly and only
        branches when it encounters a known-but-unresolved property.
        Properties not on the BDD path don't affect which node we reach,
        so enumerating their combinations is unnecessary.

        Deduplicates on next_question: if multiple BDD paths reach the
        same node, only one child is created (the sub-trees are identical
        since in a BDD, future behavior depends only on the current node,
        not how we reached it).

        Returns a list of (next_question, resolution_dict) pairs.
        """
        results = []
        seen_next = set()
        stack = [(query, resolved_questions.copy())]

        while stack:
            node, resolutions = stack.pop()

            # Terminal node: no children, query fully resolved on this path
            children = self.next_question_map.get(node)
            if not children:
                if node not in seen_next:
                    seen_next.add(node)
                    results.append((node, resolutions))
                continue

            low_child = children[0]   # False branch
            high_child = children[1]  # True branch

            if node in resolutions:
                # Already resolved: follow the appropriate branch
                if resolutions[node] == 'T':
                    stack.append((high_child, resolutions))
                else:
                    stack.append((low_child, resolutions))
            elif node in known_properties:
                # Known but not yet resolved: branch into T and F
                true_res = resolutions.copy()
                true_res[node] = 'T'
                stack.append((high_child, true_res))

                false_res = resolutions.copy()
                false_res[node] = 'F'
                stack.append((low_child, false_res))
            else:
                # Unknown property: can't advance, this is the next question
                if node not in seen_next:
                    seen_next.add(node)
                    results.append((node, resolutions))

        return results

    def update_time_step(self, current_time_step: TimeStepNode, visited_locations_this_step: set[str]):
        resolved_questions = current_time_step.resolved_questions.copy()
        robot_map = clone_robot_map(current_time_step.robot_map)

        new_visited_locations = set(current_time_step.visited_locations)
        new_visited_locations.update(visited_locations_this_step)

        known_properties = _known_properties(new_visited_locations, self.location_to_prop)
        query = current_time_step.query

        advanced = self._advance_query(query, known_properties, resolved_questions)

        for next_question, resolution in advanced:
            if next_question == current_time_step.query:
                continue

            next_time_step = TimeStepNode(
                id = str(uuid.uuid1()),
                robot_map = clone_robot_map(robot_map),
                query = next_question,
                next = [],
                type = 'robot_assignment',
                resolved_questions= resolution.copy(),
            )
            next_time_step.visited_locations = set(new_visited_locations)
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

    def _ensure_hedge_registered(self, query) -> set[str]:
        """Idempotently compute and register a Chebyshev-center hedge for each BDD branch of `query`.

        For each immediate child of `query` in the BDD that is a real proposition
        (in self.props), computes the MEC center of all locations that satisfy that
        child and registers it as a virtual hedge location with no props.
        children[0] is the false branch (_F), children[1] is the true branch (_T).
        Returns the set of registered hedge keys (0, 1, or 2 entries).
        """
        children = self.next_question_map.get(query, [])
        branch_labels = ['_F', '_T']
        hedge_keys = set()

        for i, child in enumerate(children):
            if child not in self.props:
                continue

            hedge_key = HEDGE_LOC_PREFIX + query + branch_labels[i]

            if hedge_key in self.location_to_pin:
                hedge_keys.add(hedge_key)
                continue

            positions = [
                self.location_to_pin[loc]
                for loc, props in self.location_to_prop.items()
                if not loc.startswith(HEDGE_LOC_PREFIX) and child in props
            ]

            if len(positions) < 2:
                continue

            center, _ = min_enclosing_circle(positions)
            center = (round(center[0], 4), round(center[1], 4))

            if center in self.pin_to_location:
                continue

            self.location_to_pin[hedge_key] = center
            self.pin_to_location[center] = hedge_key
            self.location_to_prop[hedge_key] = []
            self.hedge_locations.add(hedge_key)
            hedge_keys.add(hedge_key)

        return hedge_keys

    def generate_combinations(self, property: str, robot_map: RobotMap, visited_locations: set[str]) -> list[dict[str, str]]:
        hedge_keys = self._ensure_hedge_registered(property)
        locations = list(self.location_to_pin.keys())
        robot_ids = list(robot_map.keys())
        combinations : list[dict [str, str]] = []

        property_locations = []
        for loc in locations:
            if property in self.location_to_prop[loc]:
                property_locations.append(loc)


        if len(property_locations) == 0:
            return combinations

        # Locations where a robot is already en route are effectively
        # committed — sending a second robot there is always suboptimal.
        en_route_locations = set()
        for robot in robot_map.values():
            if robot.assigned_loc and robot.assigned_loc != '':
                en_route_locations.add(robot.assigned_loc)

        # Symmetry breaking: identify groups of interchangeable robots
        # (same position and same accumulated cost). Within a group, we
        # enforce that assigned locations are in sorted order to avoid
        # generating mirror-image assignments with identical cost.
        robot_state = {}
        for rid in robot_ids:
            r = robot_map[rid]
            pos = (round(r.position[0], 2), round(r.position[1], 2))
            robot_state[rid] = (pos, round(r.cost, 2), round(r.time, 2))

        # prev_in_group[i] = index of the previous robot in the same
        # equivalence group, or -1 if this robot is the first in its group.
        prev_in_group = [-1] * len(robot_ids)
        for i in range(1, len(robot_ids)):
            for j in range(i - 1, -1, -1):
                if robot_state[robot_ids[i]] == robot_state[robot_ids[j]]:
                    prev_in_group[i] = j
                    break

        def generate_assignments(robot_index: int, current_assignment: dict[str, str], used_locations: set[str]):
            if robot_index == len(robot_ids):
                # Check if at least one robot is assigned to a the property location or is at the location
                values = current_assignment.values()
                flag = False
                for loc in values:
                    if loc in property_locations:
                        flag = True
                        break

                for robot in robot_ids:
                    if flag:
                        break
                    for loc in property_locations:
                        if self.location_to_pin[loc] == robot_map[robot].position:
                            flag = True
                            break

                if flag:
                    combinations.append(current_assignment.copy())

                return


            robot_id = robot_ids[robot_index]
            known_props = _known_properties(used_locations, self.location_to_prop)

            # Symmetry breaking: if a previous robot in the same equivalence
            # group was skipped, this one must also be skipped (otherwise we
            # generate mirror-image assignments that have identical cost).
            prev_idx = prev_in_group[robot_index]
            prev_skipped = prev_idx >= 0 and robot_ids[prev_idx] not in current_assignment

            generate_assignments(robot_index + 1, current_assignment, used_locations)  # Skip this robot

            if prev_skipped:
                # Symmetric partner was skipped — skip this robot too
                return

            # Symmetry breaking: if the previous robot in the same group was
            # assigned to location L, only consider locations >= L (sorted order).
            min_loc = None
            if prev_idx >= 0 and robot_ids[prev_idx] in current_assignment:
                min_loc = current_assignment[robot_ids[prev_idx]]

            for location in locations:
                # Symmetry: enforce sorted-location order within equivalent robots
                if min_loc is not None and location < min_loc:
                    continue

                props = self.location_to_prop[location]
                skip = True
                for prop in props:
                    if prop not in known_props:
                        skip = False
                        break

                if location in hedge_keys:
                    skip = False

                if (location not in used_locations) and (location not in en_route_locations) and (not skip):
                    new_assignment = current_assignment.copy()
                    new_used_locations = set(used_locations)
                    new_assignment[robot_id] = location
                    new_used_locations.add(location)
                    generate_assignments(robot_index + 1, new_assignment, new_used_locations)

        generate_assignments(0, {}, set(visited_locations))
        return combinations


