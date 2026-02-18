import json, uuid
from .robot_class import Robot, RobotMap, clone_robot_map
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
        self.time_map: dict[str, float] = {}
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

            # Snapshot robot_map so all assignments are preserved on this node
            robot_moving_node = TimeStepNode(
                robot_map=clone_robot_map(robot_map),
                id = str(uuid.uuid1()),
                query = current_node.query,
                type = 'robot_moving',
                resolved_questions = current_node.resolved_questions,
                next = [],
            )
            robot_moving_node.visited_locations = set(current_node.visited_locations)
            current_node.next.append(robot_moving_node)
            current_node = robot_moving_node

            visited_locations = set(current_node.visited_locations)
            visited_locations.update(self.check_robot_destinations(robot_map))

            # Clear arrived robots on the working robot_map AFTER checking destinations
            for robot in arrived_robots:
                robot_map[robot.id].assigned_loc = ''

            # Snapshot the cleared state for the query node
            query_node = TimeStepNode(
                robot_map=clone_robot_map(robot_map),
                id = str(uuid.uuid1()),
                query = current_node.query,
                type = 'query',
                resolved_questions = current_node.resolved_questions,
                next = []
            )
            query_node.visited_locations = visited_locations

            current_node.next.append(query_node)
            current_node = query_node
            self.robot_manager.update_time_step(current_node, visited_locations)
        return current_node

    def process_combinations(self, combination: dict[str, str], current_time_step: TimeStepNode, robot_map_original: RobotMap):
        robot_map = clone_robot_map(robot_map_original)
        for robot_id, location in combination.items():
            self.robot_manager.assign_robot_to_location(robot_id=robot_id, location=location, robot_map=robot_map)

        self.process_robot_movement(robot_map, current_time_step)

    def search(self, initial_robot_map: RobotMap, initial_resolution: dict[str, str]) -> TimeStepNode:
        """BFS search: builds the full search tree in memory."""

        self.robot_manager = RobotManager(
            robot_map=clone_robot_map(initial_robot_map),
            next_question_map=self.next_query,
            initial_question=self.starting_prop,
            props=self.props,
            location_to_pin=self.location_to_pin,
            pin_to_location=self.pin_to_location,
            location_to_prop=self.location_to_prop,
            initial_resolution=initial_resolution.copy() if initial_resolution else {}
        )

        while self.robot_manager.time_step_queue:
            current_time_step = self.robot_manager.time_step_queue.pop(0)

            if not current_time_step.robot_map:
                continue

            original_robot_map = clone_robot_map(current_time_step.robot_map)
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

    # ---- DFS: O(D) memory, returns cost only, no tree stored ----

    def search_dfs(self, initial_robot_map: RobotMap, initial_resolution: dict[str, str], cost_metric: str = 'distance', collect_assignments: bool = False):
        """DFS search: computes minimax cost without storing the tree.
        Only the current path is in memory — O(D) where D is max depth.

        When *collect_assignments* is True the return value is
        ``(cost, assignments)`` where *assignments* maps each BDD node
        code to the robot_id that covers it (same format as
        ``get_optimal_node_assignments``).  Otherwise returns *cost* only.
        """

        self.robot_manager = RobotManager(
            robot_map=clone_robot_map(initial_robot_map),
            next_question_map=self.next_query,
            initial_question=self.starting_prop,
            props=self.props,
            location_to_pin=self.location_to_pin,
            pin_to_location=self.pin_to_location,
            location_to_prop=self.location_to_prop,
            initial_resolution=initial_resolution.copy() if initial_resolution else {}
        )

        # Compute initial visited locations from robot starting positions
        visited = set()
        for robot in initial_robot_map.values():
            pos = tuple(robot.position) if not isinstance(robot.position, tuple) else robot.position
            if pos in self.pin_to_location:
                visited.add(self.pin_to_location[pos])

        assignments = {} if collect_assignments else None

        cost = self._dfs_assignment(
            clone_robot_map(initial_robot_map),
            self.starting_prop,
            initial_resolution.copy() if initial_resolution else {},
            visited,
            cost_metric,
            assignments,
        )

        if collect_assignments:
            # Mirror polarities: ensure both @X and @-X are covered
            mirrored: dict[str, str] = {}
            for code, robot_id in assignments.items():
                if code.startswith('@-'):
                    opposite = '@' + code[2:]
                elif code.startswith('@'):
                    opposite = '@-' + code[1:]
                else:
                    continue
                if opposite not in assignments and opposite not in mirrored:
                    mirrored[opposite] = robot_id
            assignments.update(mirrored)
            return cost, assignments

        return cost

    # ---- DFS with plan: same as search_dfs but also records step-by-step actions ----

    def _make_step_dict(self, step_num: int, step_type: str, query: str,
                        resolved_questions: dict, visited: set,
                        robot_map: RobotMap, cost_metric: str) -> dict:
        """Build a step info dict matching the format produced by get_best_plan."""
        total_cost = sum(r.cost for r in robot_map.values())
        max_time = max((r.time for r in robot_map.values()), default=0.0)
        step_info = {
            'step': step_num,
            'type': step_type,
            'query': query,
            'resolved_questions': dict(resolved_questions),
            'visited_locations': list(visited),
            'cumulative_cost': total_cost,
            'parallel_time': max_time,
            'robots': {},
            'instruction': '',
        }
        for robot_id, robot in robot_map.items():
            step_info['robots'][robot_id] = {
                'position': robot.position,
                'assigned_location': robot.assigned_loc if robot.assigned_loc else None,
                'target_position': self.location_to_pin.get(robot.assigned_loc) if robot.assigned_loc else None,
                'cost_so_far': robot.cost,
                'time': robot.time,
            }
        if step_type == 'robot_moving':
            instructions = []
            for robot_id, robot in robot_map.items():
                if robot.assigned_loc:
                    target = self.location_to_pin.get(robot.assigned_loc)
                    instructions.append(f"{robot_id}: Move to {robot.assigned_loc} at {target}")
            step_info['instruction'] = "; ".join(instructions) if instructions else "Robots in transit"
        elif step_type == 'query':
            step_info['instruction'] = f"Evaluate query: {query}"
            if resolved_questions:
                step_info['instruction'] += f" | Resolved: {resolved_questions}"
        elif step_type == 'robot_assignment':
            instructions = []
            for robot_id, robot in robot_map.items():
                if robot.assigned_loc:
                    target = self.location_to_pin.get(robot.assigned_loc)
                    instructions.append(f"{robot_id}: Assigned to {robot.assigned_loc} at {target}")
            step_info['instruction'] = "; ".join(instructions) if instructions else "No assignments"
        return step_info

    def _dfs_assignment_plan(self, robot_map: RobotMap, query: str,
                             resolved_questions: dict, visited: set,
                             cost_metric: str, assignments: dict,
                             steps: list, step_counter: list) -> float:
        """MIN node with step collection: tries all combos, follows the cheapest."""
        combinations = self.robot_manager.generate_combinations(
            property=query, robot_map=robot_map, visited_locations=visited)
        if not combinations:
            return self._dfs_leaf_cost(robot_map, cost_metric)

        min_cost = float('inf')
        best_sub_steps = []
        best_sub = None
        best_combo = None
        best_counter_val = step_counter[0]

        for combination in combinations:
            rm = clone_robot_map(robot_map)
            for robot_id, location in combination.items():
                self.robot_manager.assign_robot_to_location(robot_id, location, rm)

            # Record the robot_assignment step for this combination
            assign_step = self._make_step_dict(
                step_counter[0], 'robot_assignment', query,
                resolved_questions, visited, rm, cost_metric)

            sub = {} if assignments is not None else None
            sub_steps = []
            sub_counter = [step_counter[0] + 1]

            cost = self._dfs_movement_plan(
                rm, query, resolved_questions, visited,
                cost_metric, sub, sub_steps, sub_counter)

            if cost < min_cost or best_combo is None:
                min_cost = cost
                best_sub_steps = [assign_step] + sub_steps
                best_sub = sub
                best_combo = combination
                best_counter_val = sub_counter[0]

        step_counter[0] = best_counter_val

        if assignments is not None:
            if query in self.prop_to_location:
                loc_to_robot = {loc: rid for rid, loc in best_combo.items()}
                for rid, robot in robot_map.items():
                    pos = tuple(robot.position) if not isinstance(robot.position, tuple) else robot.position
                    if pos in self.pin_to_location:
                        loc = self.pin_to_location[pos]
                        if loc not in loc_to_robot:
                            loc_to_robot[loc] = rid
                for loc in self.prop_to_location[query]:
                    if loc in loc_to_robot and query not in assignments:
                        assignments[query] = loc_to_robot[loc]
                        break
            if best_sub:
                for k, v in best_sub.items():
                    if k not in assignments:
                        assignments[k] = v

        steps.extend(best_sub_steps)
        return min_cost

    def _dfs_movement_plan(self, robot_map: RobotMap, query: str,
                           resolved_questions: dict, visited: set,
                           cost_metric: str, assignments: dict,
                           steps: list, step_counter: list) -> float:
        """Simulate movement with step collection.

        At MAX (observation) nodes follows the worst-case branch, consistent
        with the minimax cost.
        """
        while self.robot_manager.count_traveling_robots(robot_map) > 0:
            arrived = self.robot_manager.update_robot_positions(robot_map)

            # robot_moving step: robots still have assigned_loc set
            move_step = self._make_step_dict(
                step_counter[0], 'robot_moving', query,
                resolved_questions, visited, robot_map, cost_metric)
            step_counter[0] += 1

            new_visited = set(visited)
            new_visited.update(self.check_robot_destinations(robot_map))

            for robot in arrived:
                robot_map[robot.id].assigned_loc = ''

            # query step: arrived robots now have cleared assigned_loc
            query_step = self._make_step_dict(
                step_counter[0], 'query', query,
                resolved_questions, new_visited, robot_map, cost_metric)
            step_counter[0] += 1

            known = _known_properties(new_visited, self.location_to_prop)
            advanced = self.robot_manager._advance_query(query, known, resolved_questions)
            branches = [(nq, res) for nq, res in advanced if nq != query]

            if branches:
                # Record consumed codes: BDD nodes skipped by _advance_query
                # because their location was already visited.  Mirrors the
                # identical block in _dfs_movement.
                if assignments is not None:
                    loc_to_robot = {}
                    for robot_id, robot in robot_map.items():
                        if robot.assigned_loc:
                            loc_to_robot[robot.assigned_loc] = robot_id
                        pos = tuple(robot.position) if not isinstance(robot.position, tuple) else robot.position
                        if pos in self.pin_to_location:
                            loc = self.pin_to_location[pos]
                            if loc not in loc_to_robot:
                                loc_to_robot[loc] = robot_id
                    for nq, res in branches:
                        consumed = self._trace_consumed_codes(query, res)
                        for code in consumed:
                            if code not in assignments and code in self.prop_to_location:
                                for loc in self.prop_to_location[code]:
                                    if loc in loc_to_robot:
                                        assignments[code] = loc_to_robot[loc]
                                        break

                # MAX node: world picks worst-case outcome.
                # Collect assignments from ALL branches (the world can take any
                # path, so every reachable BDD node needs a color); only record
                # steps from the worst-case branch.
                max_cost = 0.0
                worst_sub_steps = []
                best_counter_val = step_counter[0]

                for nq, res in branches:
                    sub = {} if assignments is not None else None
                    sub_steps = []
                    sub_counter = [step_counter[0]]
                    if nq in self.robot_manager.props:
                        cost = self._dfs_assignment_plan(
                            clone_robot_map(robot_map), nq, res, set(new_visited),
                            cost_metric, sub, sub_steps, sub_counter)
                    else:
                        cost = self._dfs_leaf_cost(robot_map, cost_metric)
                    # Merge assignments from every branch, not just worst-case
                    if assignments is not None and sub:
                        for k, v in sub.items():
                            if k not in assignments:
                                assignments[k] = v
                    if cost >= max_cost:
                        max_cost = cost
                        worst_sub_steps = sub_steps
                        best_counter_val = sub_counter[0]

                # Continued movement is also a possible branch
                if self.robot_manager.count_traveling_robots(robot_map) > 0:
                    cont_sub = {} if assignments is not None else None
                    cont_steps = []
                    cont_counter = [step_counter[0]]
                    cont_cost = self._dfs_movement_plan(
                        clone_robot_map(robot_map), query, resolved_questions,
                        new_visited, cost_metric, cont_sub, cont_steps, cont_counter)
                    if assignments is not None and cont_sub:
                        for k, v in cont_sub.items():
                            if k not in assignments:
                                assignments[k] = v
                    if cont_cost >= max_cost:
                        max_cost = cont_cost
                        worst_sub_steps = cont_steps
                        best_counter_val = cont_counter[0]

                step_counter[0] = best_counter_val
                steps.extend([move_step, query_step] + worst_sub_steps)
                return max_cost

            steps.append(move_step)
            steps.append(query_step)
            visited = new_visited

        return self._dfs_leaf_cost(robot_map, cost_metric)

    def search_dfs_with_plan(self, initial_robot_map: RobotMap,
                             initial_resolution: dict,
                             cost_metric: str = 'distance',
                             collect_assignments: bool = False):
        """DFS search that also produces detailed plan steps for the optimal path.

        Unlike search_dfs (cost only), this records each robot_assignment,
        robot_moving, and query step along the minimax-optimal path so the
        result can be printed and visualised like get_best_plan.

        At MAX (observation) nodes the worst-case branch is followed, which
        is consistent with the minimax cost.  At MIN (assignment) nodes the
        cheapest combination is followed.

        Returns:
            collect_assignments=True:  (cost, assignments, detailed_steps)
            collect_assignments=False: (cost, detailed_steps)
        """
        self.robot_manager = RobotManager(
            robot_map=clone_robot_map(initial_robot_map),
            next_question_map=self.next_query,
            initial_question=self.starting_prop,
            props=self.props,
            location_to_pin=self.location_to_pin,
            pin_to_location=self.pin_to_location,
            location_to_prop=self.location_to_prop,
            initial_resolution=initial_resolution.copy() if initial_resolution else {}
        )

        visited = set()
        for robot in initial_robot_map.values():
            pos = tuple(robot.position) if not isinstance(robot.position, tuple) else robot.position
            if pos in self.pin_to_location:
                visited.add(self.pin_to_location[pos])

        assignments = {} if collect_assignments else None
        detailed_steps: list = []
        step_counter = [0]

        cost = self._dfs_assignment_plan(
            clone_robot_map(initial_robot_map),
            self.starting_prop,
            initial_resolution.copy() if initial_resolution else {},
            visited,
            cost_metric,
            assignments,
            detailed_steps,
            step_counter,
        )

        # Renumber steps sequentially (branch exploration may create gaps)
        for i, step in enumerate(detailed_steps):
            step['step'] = i

        if collect_assignments:
            mirrored: dict[str, str] = {}
            for code, robot_id in assignments.items():
                if code.startswith('@-'):
                    opposite = '@' + code[2:]
                elif code.startswith('@'):
                    opposite = '@-' + code[1:]
                else:
                    continue
                if opposite not in assignments and opposite not in mirrored:
                    mirrored[opposite] = robot_id
            assignments.update(mirrored)
            return cost, assignments, detailed_steps

        return cost, detailed_steps

    def _dfs_leaf_cost(self, robot_map: RobotMap, cost_metric: str) -> float:
        if cost_metric == 'time':
            return max(r.time for r in robot_map.values())
        return sum(r.cost for r in robot_map.values())

    def _dfs_assignment(self, robot_map: RobotMap, query: str, resolved_questions: dict, visited: set, cost_metric: str, assignments: dict = None) -> float:
        """MIN node: try every robot-to-location combination, return minimum cost.

        When *assignments* is not None, only the best combination's
        sub-assignments are merged into it, and the robot covering the
        current *query* is recorded.
        """
        combinations = self.robot_manager.generate_combinations(
            property=query, robot_map=robot_map, visited_locations=visited
        )
        if not combinations:
            return self._dfs_leaf_cost(robot_map, cost_metric)

        min_cost = float('inf')
        best_sub = None
        best_combo = None
        for combination in combinations:
            rm = clone_robot_map(robot_map)
            for robot_id, location in combination.items():
                self.robot_manager.assign_robot_to_location(robot_id, location, rm)
            sub = {} if assignments is not None else None
            cost = self._dfs_movement(rm, query, resolved_questions, visited, cost_metric, sub)
            if cost < min_cost or best_combo is None:
                min_cost = cost
                best_sub = sub
                best_combo = combination

        if assignments is not None:
            # Record which robot covers the current query code
            if query in self.prop_to_location:
                # Build location -> robot_id from the best combination + existing positions
                loc_to_robot = {loc: rid for rid, loc in best_combo.items()}
                for rid, robot in robot_map.items():
                    pos = tuple(robot.position) if not isinstance(robot.position, tuple) else robot.position
                    if pos in self.pin_to_location:
                        loc = self.pin_to_location[pos]
                        if loc not in loc_to_robot:
                            loc_to_robot[loc] = rid
                for loc in self.prop_to_location[query]:
                    if loc in loc_to_robot and query not in assignments:
                        assignments[query] = loc_to_robot[loc]
                        break
            # Merge sub-assignments from the best combination only
            if best_sub:
                for k, v in best_sub.items():
                    if k not in assignments:
                        assignments[k] = v

        return min_cost

    def _dfs_movement(self, robot_map: RobotMap, query: str, resolved_questions: dict, visited: set, cost_metric: str, assignments: dict = None) -> float:
        """Simulate robot movement. At each arrival, MAX over observation branches.

        At MAX nodes all branches are explored, so every branch's
        assignments are collected into *assignments*.
        """
        while self.robot_manager.count_traveling_robots(robot_map) > 0:
            arrived = self.robot_manager.update_robot_positions(robot_map)

            new_visited = set(visited)
            new_visited.update(self.check_robot_destinations(robot_map))

            for robot in arrived:
                robot_map[robot.id].assigned_loc = ''

            known = _known_properties(new_visited, self.location_to_prop)
            advanced = self.robot_manager._advance_query(query, known, resolved_questions)
            branches = [(nq, res) for nq, res in advanced if nq != query]

            if branches:
                # Record consumed codes (BDD nodes skipped by _advance_query)
                if assignments is not None:
                    loc_to_robot = {}
                    for robot_id, robot in robot_map.items():
                        if robot.assigned_loc:
                            loc_to_robot[robot.assigned_loc] = robot_id
                        pos = tuple(robot.position) if not isinstance(robot.position, tuple) else robot.position
                        if pos in self.pin_to_location:
                            loc = self.pin_to_location[pos]
                            if loc not in loc_to_robot:
                                loc_to_robot[loc] = robot_id
                    for nq, res in branches:
                        consumed = self._trace_consumed_codes(query, res)
                        for code in consumed:
                            if code not in assignments and code in self.prop_to_location:
                                for loc in self.prop_to_location[code]:
                                    if loc in loc_to_robot:
                                        assignments[code] = loc_to_robot[loc]
                                        break

                # MAX node: worst-case over observation outcomes
                max_cost = 0
                for nq, res in branches:
                    if nq in self.robot_manager.props:
                        cost = self._dfs_assignment(clone_robot_map(robot_map), nq, res, set(new_visited), cost_metric, assignments)
                    else:
                        cost = self._dfs_leaf_cost(robot_map, cost_metric)
                    max_cost = max(max_cost, cost)

                # Continued movement is also a branch (more robots still traveling)
                if self.robot_manager.count_traveling_robots(robot_map) > 0:
                    continued = self._dfs_movement(clone_robot_map(robot_map), query, resolved_questions, new_visited, cost_metric, assignments)
                    max_cost = max(max_cost, continued)

                return max_cost

            visited = new_visited

        return self._dfs_leaf_cost(robot_map, cost_metric)

    # COST IS BY DISTAMCE
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

    def determine_cost_with_pruning(self, node: TimeStepNode, alpha: float = float('-inf'), beta: float = float('inf')) -> float:
        """Alpha-beta pruned minimax cost. Only used during sifting for speed.

        Alpha tracks the best (highest) value the MAX player (query nodes)
        can guarantee. Beta tracks the best (lowest) value the MIN player
        (robot_assignment nodes) can guarantee. When alpha >= beta, the
        remaining children cannot influence the result and are pruned.
        """
        if len(node.next) == 0:
            return node.get_cost()

        if node.type == 'robot_moving':
            return self.determine_cost_with_pruning(node.next[0], alpha, beta)

        elif node.type == 'query':
            # MAX node: adversarial world picks worst outcome
            cost = 0
            for next_node in node.next:
                cost = max(cost, self.determine_cost_with_pruning(next_node, alpha, beta))
                alpha = max(alpha, cost)
                if alpha >= beta:
                    break  # Beta cutoff: MIN ancestor already has a cheaper option
            return cost

        elif node.type == 'robot_assignment':
            # MIN node: planner picks best (cheapest) assignment
            cost = float('inf')
            for next_node in node.next:
                cost = min(cost, self.determine_cost_with_pruning(next_node, alpha, beta))
                beta = min(beta, cost)
                if beta <= alpha:
                    break  # Alpha cutoff: MAX ancestor already has a costlier option
            return cost
        else:
            raise ValueError(f"Unknown node type: {node.type}")

    def determine_time_with_pruning(self, node: TimeStepNode, alpha: float = float('-inf'), beta: float = float('inf')) -> float:
        """Alpha-beta pruned minimax time. Only used during sifting for speed.

        Same semantics as determine_cost_with_pruning but evaluates
        wall-clock parallel completion time (max over robots) instead of
        cumulative distance.
        """
        if len(node.next) == 0:
            return node.get_time()

        if node.type == 'robot_moving':
            return self.determine_time_with_pruning(node.next[0], alpha, beta)

        elif node.type == 'query':
            # MAX node: adversarial world picks worst outcome
            time = 0
            for next_node in node.next:
                time = max(time, self.determine_time_with_pruning(next_node, alpha, beta))
                alpha = max(alpha, time)
                if alpha >= beta:
                    break  # Beta cutoff: MIN ancestor already has a faster option
            return time

        elif node.type == 'robot_assignment':
            # MIN node: planner picks fastest assignment
            time = float('inf')
            for next_node in node.next:
                time = min(time, self.determine_time_with_pruning(next_node, alpha, beta))
                beta = min(beta, time)
                if beta <= alpha:
                    break  # Alpha cutoff: MAX ancestor already has a slower option
            return time
        else:
            raise ValueError(f"Unknown node type: {node.type}")

    # TIME IS BY PARALLEL COMPLETION: max over robots (wall-clock time)
    def determine_time(self, node: TimeStepNode, recursive_count: int = 0) -> float:
        if len(node.next) == 0:
            time = node.get_time()
            self.time_map[node.id] = time
            return time

        if node.id in self.time_map:
            return self.time_map[node.id]

        if node.type == 'robot_moving':
            return self.determine_time(node.next[0], recursive_count + 1)

        elif node.type == 'query':
            # Worst case over all possible observations
            time = 0
            for next_node in node.next:
                time = max(time, self.determine_time(next_node, recursive_count + 1))
            return time

        elif node.type == 'robot_assignment':
            # Best assignment: pick the one that finishes earliest
            time = float('inf')
            for next_node in node.next:
                time = min(time, self.determine_time(next_node, recursive_count + 1))
            return time
        else:
            raise ValueError(f"Unknown node type: {node.type}")
        
    

    def get_optimal_node_assignments(self, root_node: TimeStepNode, use_time: bool = False) -> dict[str, str]:
        """Walk the search tree to find the optimal robot for every BDD node.

        At robot_assignment (MIN) nodes, follows the best combination.
        At query (MAX) nodes, explores ALL branches so that every
        reachable BDD node gets an assignment.

        Args:
            root_node: Root of the search tree (from search()).
            use_time: If True, pick the best assignment by time instead of cost.

        Returns:
            dict mapping BDD query code (e.g. "@-87") to robot_id.
        """
        # Populate cost/time maps so determine_cost/time can be used for comparisons.
        if use_time:
            self.determine_time(root_node)
        else:
            self.determine_cost(root_node)

        assignments: dict[str, str] = {}
        self._walk_for_assignments(root_node, assignments, use_time)

        # Mirror polarities: ensure both @X and @-X are covered
        mirrored: dict[str, str] = {}
        for code, robot_id in assignments.items():
            if code.startswith('@-'):
                opposite = '@' + code[2:]   # @-87 -> @87
            elif code.startswith('@'):
                opposite = '@-' + code[1:]  # @87 -> @-87
            else:
                continue
            if opposite not in assignments and opposite not in mirrored:
                mirrored[opposite] = robot_id
        assignments.update(mirrored)

        return assignments

    def _find_next_ras(self, node: TimeStepNode) -> list[TimeStepNode]:
        """Traverse from node through robot_moving/query chains to find nearest robot_assignment descendants."""
        if not node.next:
            return []
        result = []
        for child in node.next:
            if child.type == 'robot_assignment':
                result.append(child)
            else:
                result.extend(self._find_next_ras(child))
        return result

    def _trace_consumed_codes(self, from_code: str, resolved_questions: dict[str, str]) -> list[str]:
        """Follow BDD edges from from_code using resolved_questions to collect consumed codes.

        Starting from from_code, follows BDD edges (self.next_query) using
        resolved_questions to determine direction (T->high edge, F->low edge).
        Collects all codes that are already resolved. Stops at unresolved codes
        or terminals.
        """
        consumed = []
        current = from_code
        while current in resolved_questions and current in self.next_query:
            edges = self.next_query[current]
            if len(edges) < 2:
                break  # terminal
            consumed.append(current)
            truth = resolved_questions[current] == 'T'
            current = edges[1] if truth else edges[0]
        return consumed

    def _walk_for_assignments(self, node: TimeStepNode, assignments: dict[str, str], use_time: bool) -> None:
        if not node.next:
            return

        if node.type == 'robot_moving':
            self._walk_for_assignments(node.next[0], assignments, use_time)

        elif node.type == 'query':
            # Explore every possible world outcome
            for child in node.next:
                self._walk_for_assignments(child, assignments, use_time)

        elif node.type == 'robot_assignment':
            # Pick the best (MIN) combination
            if use_time:
                best_child = min(node.next, key=lambda n: self.determine_time(n))
            else:
                best_child = min(node.next, key=lambda n: self.determine_cost(n))

            # Build loc_to_robot: location -> robot_id
            loc_to_robot: dict[str, str] = {}
            for robot_id, robot in best_child.robot_map.items():
                # Robots assigned to a location
                if robot.assigned_loc:
                    loc_to_robot[robot.assigned_loc] = robot_id
                # Robots already at a location (by position)
                pos_tuple = tuple(robot.position)
                if pos_tuple in self.pin_to_location:
                    loc = self.pin_to_location[pos_tuple]
                    if loc not in loc_to_robot:
                        loc_to_robot[loc] = robot_id

            # Map the current query code to the robot visiting its location
            if node.query and node.query in self.prop_to_location:
                for loc in self.prop_to_location[node.query]:
                    if loc in loc_to_robot:
                        if node.query not in assignments:
                            assignments[node.query] = loc_to_robot[loc]
                        break

            # Find descendant robot_assignment nodes and trace consumed codes
            desc_ras = self._find_next_ras(best_child)
            for desc_ra in desc_ras:
                consumed = self._trace_consumed_codes(node.query, desc_ra.resolved_questions)
                for code in consumed:
                    if code not in assignments and code in self.prop_to_location:
                        for loc in self.prop_to_location[code]:
                            if loc in loc_to_robot:
                                assignments[code] = loc_to_robot[loc]
                                break

            self._walk_for_assignments(best_child, assignments, use_time)

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
        
        # RELIES ON THE FACT THE ROBOTS MOVE ONE AT A TIME

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

    # By time = wall-clock parallel completion time (max over robots)
    def get_best_plan_by_time(self, initial_robot_map: RobotMap, initial_resolution: dict[str, str], get_only_time: bool = False) -> tuple[list[(str, tuple[int, int])], list[str], list[dict]]:
        cur_node = self.search(initial_robot_map, initial_resolution)
        best_time = self.determine_time(cur_node)
        best_plan_text = []
        best_plan: list[(str, tuple[int, int])] = []
        detailed_steps: list[dict] = []
        step_number = 0

        if get_only_time:
            return best_time

        while cur_node is not None:
            for next_node in cur_node.next:
                if (abs(self.determine_time(next_node) - best_time)) < COST_TOLERANCE:
                    step_info = {
                        'step': step_number,
                        'type': next_node.type,
                        'query': next_node.query,
                        'resolved_questions': dict(next_node.resolved_questions),
                        'visited_locations': list(next_node.visited_locations),
                        'parallel_time': next_node.get_time(),
                        'cumulative_cost': next_node.get_cost(),
                        'robots': {}
                    }

                    for robot_id, robot in next_node.robot_map.items():
                        robot_info = {
                            'position': robot.position,
                            'assigned_location': robot.assigned_loc if robot.assigned_loc else None,
                            'target_position': self.location_to_pin.get(robot.assigned_loc) if robot.assigned_loc else None,
                            'cost_so_far': robot.cost,
                            'time': robot.time
                        }
                        step_info['robots'][robot_id] = robot_info

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



