import copy
import json, uuid
from .robot_class import Robot, RobotMap
from .robot_manager import RobotManager, euclidean_distance, DISTANCE_TOLERANCE, _known_properties
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
        self.robot_manager = None

    def import_srql_config(self, adjacency_list: dict[str, list[str]], locs: dict[str, tuple[int, int]], code_to_locations_map: dict[str, list[str]], root: str):
        bdd_config = {}
        bdd_config['edges'] = adjacency_list
        bdd_config['locations'] = locs
        self.prop_to_location = code_to_locations_map
        bdd_config['prop_to_location'] = self.prop_to_location
        for loc, pin in locs.items():
            tuple_pin = tuple(pin)
            self.location_to_pin[loc] = tuple_pin
            self.pin_to_location[tuple_pin] = loc
            self.location_to_prop[loc] = []

        self.props = set(code_to_locations_map.keys())
        for prop, locs in bdd_config['prop_to_location'].items():
            for loc in locs:
                if loc not in self.location_to_prop:
                    self.location_to_prop[loc] = []
                self.location_to_prop[loc].append(prop)

        self.next_query = bdd_config['edges']
        self.starting_prop = root

        return bdd_config

    def import_bdd_config(self):
        # Try to load bdd.json, but don't fail if it doesn't exist
        # (we may be using import_srql_config instead)
        try:
            with open('src/my_robot_bringup/config/bdd.json', 'r') as file:
                bdd_config = json.load(file)
        except FileNotFoundError:
            # Return empty config - will be populated via import_srql_config
            return {}

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

    def process_robot_movement(self, robot_map: RobotMap, current_node: TimeStepNode):
        while self.robot_manager.count_traveling_robots(robot_map=robot_map) > 0:
            arrived_robots = self.robot_manager.update_robot_positions(robot_map=robot_map)

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
            self.robot_manager.update_time_step(current_node, visited_locations)
        return current_node

    def process_combinations(self, combination: dict[str, str], current_time_step: TimeStepNode, robot_map_original: RobotMap):
        robot_map = copy.deepcopy(robot_map_original)
        for robot_id, location in combination.items():
            self.robot_manager.assign_robot_to_location(robot_id=robot_id, location=location, robot_map=robot_map)

        self.process_robot_movement(robot_map, current_time_step)

    def search(self, initial_robot_map: RobotMap, initial_resolution: dict[str, str]) -> TimeStepNode:


        self.robot_manager = RobotManager(
            robot_map=copy.deepcopy(initial_robot_map),
            next_question_map=self.next_query,
            initial_question=self.starting_prop,
            props=self.props,
            location_to_pin=self.location_to_pin,
            pin_to_location=self.pin_to_location,
            location_to_prop=self.location_to_prop,
            initial_resolution=copy.deepcopy(initial_resolution)
        )



        while self.robot_manager.time_step_queue: 
            current_time_step = self.robot_manager.time_step_queue.pop(0)

            if not current_time_step.robot_map:
                continue
            
            original_robot_map = copy.deepcopy(current_time_step.robot_map)
            combinations = self.robot_manager.generate_combinations(
                property=current_time_step.query,  
                robot_map=original_robot_map,
                visited_locations=current_time_step.visited_locations
            )


            for combination in combinations:
                self.process_combinations(combination=combination,
                                          current_time_step=current_time_step,
                                          robot_map_original=original_robot_map)
        return self.robot_manager.head_time_step_node


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
    def get_best_plan(self, initial_robot_map: RobotMap, initial_resolution: dict[str, str], get_only_cost: bool = False) -> tuple[list[(str, tuple[int, int])], list[str], list[dict]]:
        cur_node = self.search(initial_robot_map, initial_resolution)
        best_cost = self.determine_cost(cur_node)
        best_plan_text = []
        best_plan: list[(str, tuple[int, int])] = []
        detailed_steps: list[dict] = []  # Detailed step-by-step instructions
        step_number = 0

        if get_only_cost:
            return best_cost

        while cur_node is not None:
            for next_node in cur_node.next:
                if (abs(self.determine_cost(next_node) - best_cost)) < COST_TOLERANCE:
                    step_info = {
                        'step': step_number,
                        'type': next_node.type,
                        'query': next_node.query,
                        'resolved_questions': dict(next_node.resolved_questions),
                        'visited_locations': list(next_node.visited_locations),
                        'cumulative_cost': next_node.get_cost(),
                        'robots': {}
                    }

                    # Capture robot state at this step
                    for robot_id, robot in next_node.robot_map.items():
                        robot_info = {
                            'position': robot.position,
                            'assigned_location': robot.assigned_loc if robot.assigned_loc else None,
                            'target_position': self.location_to_pin.get(robot.assigned_loc) if robot.assigned_loc else None,
                            'cost_so_far': robot.cost,
                            'time': robot.time
                        }
                        step_info['robots'][robot_id] = robot_info

                    # Generate human-readable instruction
                    if next_node.type == 'robot_moving':
                        instructions = []
                        for robot_id, robot in next_node.robot_map.items():
                            if robot.assigned_loc:
                                target = self.location_to_pin.get(robot.assigned_loc)
                                instructions.append(f"{robot_id}: Move to {robot.assigned_loc} at {target}")
                        step_info['instruction'] = "; ".join(instructions) if instructions else "Robots in transit"
                        best_plan_text.append(str(RobotAssignments(next_node, self.location_to_pin)))

                    elif next_node.type == 'query':
                        step_info['instruction'] = f"Evaluate query: {next_node.query}"
                        if next_node.resolved_questions:
                            step_info['instruction'] += f" | Resolved: {next_node.resolved_questions}"
                        best_plan_text.append(next_node.resolved_questions)

                    elif next_node.type == 'robot_assignment':
                        instructions = []
                        for robot_id, robot in next_node.robot_map.items():
                            if robot.assigned_loc != '':
                                target = self.location_to_pin[robot.assigned_loc]
                                best_plan.append((robot_id, target))
                                instructions.append(f"{robot_id}: Assigned to {robot.assigned_loc} at {target}")
                                best_plan_text.append(f"{robot_id} -> {robot.assigned_loc}")
                        step_info['instruction'] = "; ".join(instructions) if instructions else "No assignments"

                    detailed_steps.append(step_info)
                    step_number += 1
                    cur_node = next_node
                    break

            else:
                cur_node = None

        return (best_plan, best_plan_text, detailed_steps)

    # ── Fast cost-only search (for sifting) ──────────────────────────────

    def get_cost_only(self, initial_robot_map: RobotMap,
                      initial_resolutions: dict[str, str]) -> float:
        """Compute minimax planning cost via DFS with alpha-beta pruning.

        This is an approximation that models simultaneous robot arrivals
        (all robots arrive at once, then branch) rather than sequential
        arrivals.  Valid for sifting where only relative cost comparisons
        between variable orderings matter.
        """
        robot_positions = {}
        robot_costs = {}
        for rid, robot in initial_robot_map.items():
            robot_positions[rid] = robot.position
            robot_costs[rid] = robot.cost

        return self._minimax_cost(
            query=self.starting_prop,
            robot_positions=robot_positions,
            robot_costs=robot_costs,
            visited_locations=set(),
            resolved_questions=dict(initial_resolutions),
            alpha=0.0,
            beta=float('inf'),
        )

    def _minimax_cost(self, query: str, robot_positions: dict[str, tuple],
                      robot_costs: dict[str, float],
                      visited_locations: set[str],
                      resolved_questions: dict[str, str],
                      alpha: float, beta: float) -> float:
        """Recursive DFS minimax with alpha-beta pruning.

        Levels alternate:
          MIN - choose best robot assignment   (we pick)
          MAX - adversary picks worst resolution (environment picks)
        """
        # Terminal: query is not a known prop (leaf of the BDD)
        if query not in self.props:
            return sum(robot_costs.values())

        # ── MIN level: try every robot-assignment combination ────────
        combinations = self._generate_combinations_fast(
            query, robot_positions, visited_locations)

        if not combinations:
            return sum(robot_costs.values())

        # Sort by move_cost (cheapest first) so alpha-beta prunes more
        combinations.sort(key=lambda c: c[1])

        best = float('inf')

        for combo, move_cost in combinations:
            # Prune: since combos are sorted ascending and future_cost >= 0,
            # if move_cost alone already >= best, all remaining are worse.
            # NO PRUNING check
            # if move_cost >= best:
            #     break

            # # Alpha-beta prune at MIN level
            # if best <= alpha:
            #     break

            # Compute new robot positions and costs after this assignment
            new_positions = dict(robot_positions)
            new_costs = dict(robot_costs)
            new_visited = set(visited_locations)
            for rid, loc in combo.items():
                target = self.location_to_pin[loc]
                dist = euclidean_distance(new_positions[rid], target)
                new_costs[rid] = new_costs[rid] + dist
                new_positions[rid] = target
                new_visited.add(loc)

            # Also mark locations where robots already are
            for rid, pos in new_positions.items():
                if pos in self.pin_to_location:
                    new_visited.add(self.pin_to_location[pos])

            # ── MAX level: adversary picks worst resolution ──────
            child_beta = min(beta, best)
            worst = self._max_over_resolutions(
                query, new_positions, new_costs, new_visited,
                resolved_questions, alpha, child_beta)

            if worst < best:
                best = worst

        return best

    def _max_over_resolutions(self, query: str,
                              robot_positions: dict[str, tuple],
                              robot_costs: dict[str, float],
                              visited_locations: set[str],
                              resolved_questions: dict[str, str],
                              alpha: float, beta: float) -> float:
        """MAX level: adversary picks the worst-case resolution."""
        known_props = _known_properties(visited_locations, self.location_to_prop)
        # Find unresolved known properties
        unresolved = [p for p in known_props if p not in resolved_questions]

        worst = 0.0

        # Iterate over all resolutions via backtracking
        for resolution in self._generate_resolutions_fast(unresolved, resolved_questions):
            # Follow BDD edges to find next query
            next_query = self._resolve_query(query, resolution)

            if next_query == query:
                # No progress – skip this resolution
                continue

            child_cost = self._minimax_cost(
                next_query, robot_positions, robot_costs,
                visited_locations, resolution, worst, beta)

            if child_cost > worst:
                worst = child_cost

            # Alpha-beta prune at MAX level
            if worst >= beta:
                break

        return worst

    def _resolve_query(self, query: str,
                       resolution: dict[str, str]) -> str:
        """Follow BDD edges given a resolution to find the next query."""
        current = query
        while current in resolution:
            truth = resolution[current] == 'T'
            if truth:
                current = self.next_query[current][1]
            else:
                current = self.next_query[current][0]
        return current

    def _generate_combinations_fast(self, query: str,
                                    robot_positions: dict[str, tuple],
                                    visited_locations: set[str]
                                    ) -> list[tuple[dict[str, str], float]]:
        """Generate (assignment_dict, move_cost) pairs via backtracking.

        Returns a list of (combo, move_cost) where move_cost is the
        analytical euclidean cost for that assignment.
        """
        locations = list(self.location_to_pin.keys())
        robot_ids = list(robot_positions.keys())

        property_locations = [
            loc for loc in locations
            if query in self.location_to_prop.get(loc, [])
        ]
        if not property_locations:
            return []

        # Check if any robot is already at a property location
        robot_at_prop = False
        for rid in robot_ids:
            pos = robot_positions[rid]
            if pos in self.pin_to_location:
                loc = self.pin_to_location[pos]
                if loc in property_locations:
                    robot_at_prop = True
                    break

        results: list[tuple[dict[str, str], float]] = []
        assignment: dict[str, str] = {}
        used_locations: set[str] = set(visited_locations)
        assignment_cost: float = 0.0

        def backtrack(robot_index: int):
            nonlocal assignment_cost
            if robot_index == len(robot_ids):
                # Validate: at least one robot targets a property location
                if not robot_at_prop:
                    covers = any(loc in property_locations
                                 for loc in assignment.values())
                    if not covers:
                        return
                results.append((dict(assignment), assignment_cost))
                return

            rid = robot_ids[robot_index]

            # Option: skip this robot (no assignment)
            backtrack(robot_index + 1)

            known_props = _known_properties(used_locations, self.location_to_prop)
            for loc in locations:
                if loc in used_locations:
                    continue
                # Only assign to locations with unknown properties
                props = self.location_to_prop.get(loc, [])
                if all(p in known_props for p in props):
                    continue

                target = self.location_to_pin[loc]
                dist = euclidean_distance(robot_positions[rid], target)

                assignment[rid] = loc
                used_locations.add(loc)
                assignment_cost += dist

                backtrack(robot_index + 1)

                del assignment[rid]
                used_locations.discard(loc)
                assignment_cost -= dist

        backtrack(0)
        return results

    @staticmethod
    def _generate_resolutions_fast(unresolved: list[str],
                                   resolved_questions: dict[str, str]
                                   ):
        """Yield all T/F resolutions for unresolved properties via backtracking.

        Mutates and restores *resolved_questions* in place for speed;
        callers must not hold references across yields.
        """
        if not unresolved:
            yield dict(resolved_questions)
            return

        def _backtrack(idx):
            if idx == len(unresolved):
                yield dict(resolved_questions)
                return

            prop = unresolved[idx]
            for val in ('T', 'F'):
                resolved_questions[prop] = val
                yield from _backtrack(idx + 1)
            del resolved_questions[prop]

        yield from _backtrack(0)



