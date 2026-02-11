from copy import deepcopy
import itertools
import json
import sys
import os
from get_bdd_from_srql import get_bdd_from_srql
from customdd.autoref import BDD

# Add src directory to path so we can import navigation package
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from navigation.navigation.planning.create_plan import SearchTree
from navigation.navigation.planning.robot_class import Robot, RobotMap
import random


class PlanningCostEvaluator:
    """
    Encapsulates all data needed to compute multi-robot planning costs for BDD sifting.

    This class is designed to be used as a cost function during Rudell's sifting algorithm.
    Instead of using simple edge costs, it computes the full multi-robot planning cost
    by building a SearchTree and running determine_cost().

    Usage:
        evaluator = PlanningCostEvaluator(elegant_var_dict, robots, locs_with_coords)
        cost = evaluator(bdd, wrld, qry)
    """

    def __init__(self, elegant_var_dict, robots, locs_with_coords, cost_metric='distance'):
        """
        Initialize the planning cost evaluator.

        Args:
            elegant_var_dict: Dict mapping elegant names to BDD variable names
            robots: Dict with 'num_robots' and 'robot_starts' list
            locs_with_coords: Dict mapping location names to {'x': int, 'y': int}
            cost_metric: 'distance' for sum of robot distances, 'time' for parallel wall-clock time
        """
        self.elegant_var_dict = elegant_var_dict
        self.robots = robots
        self.locs_with_coords = locs_with_coords
        self.cost_metric = cost_metric

        # Precompute location tuples
        self.locs = {}
        for loc_name, coords in locs_with_coords.items():
            self.locs[loc_name] = (coords['x'], coords['y'])

        # Saved state from the most recent evaluation.
        # on_var_sifted can read these to colour the PDF without
        # running a second search.
        self.last_temp_bdd = None
        self.last_product_root = None
        self.last_search_tree = None
        self.last_root_node = None

    def __call__(self, bdd, wrld, qry):
        """
        Compute the planning cost for the current BDD variable ordering.

        This method:
        1. Creates a SEPARATE BDD for the product graph (to avoid modifying the original during sifting)
        2. Forms the product graph from world and query BDDs
        3. Builds an adjacency list from the product graph
        4. Creates a SearchTree and imports the BDD configuration
        5. Runs search() to build the planning tree
        6. Returns the cost via determine_cost()

        Args:
            bdd: The BDD manager (autoref.BDD wrapper)
            wrld: The world constraints BDD node
            qry: The query BDD node

        Returns:
            float: The multi-robot planning cost (uses MIN over assignments, MAX over queries)
        """
        # Create a SEPARATE BDD for the product graph to avoid modifying the original
        # This is critical during sifting - we can't add nodes to a BDD while reordering it
        product_bdd = BDD()
        for var in sorted(bdd._bdd.vars, key=bdd._bdd.vars.get):
            product_bdd.add_var(var, None)

        # Form product graph in the separate BDD
        product_root = bdd.form_product_graph(product_bdd, wrld, qry)

        # Handle trivial cases
        if abs(int(product_root)) == 1:
            return 0  # Tautology or contradiction - no planning needed

        # Build adjacency list from product graph (using product_bdd)
        adjacency_list = build_adjacency_list(product_bdd, product_root)

        # Get code-to-locations mapping (using product_bdd)
        code_to_locations_map = get_code_to_locations_map(product_bdd, product_root, self.elegant_var_dict)

        # Create and configure SearchTree
        search_tree = SearchTree()
        search_tree.bdd_config = search_tree.import_srql_config(
            adjacency_list,
            self.locs,
            code_to_locations_map,
            str(product_root)
        )

        # Build initial robot map
        r_map = {}
        for robot in self.robots['robot_starts']:
            robot_id = robot['robot_id']
            start_loc = robot['start_location']
            r_map[robot_id] = Robot(id=robot_id, position=self.locs[start_loc])

        initial_robot_map = RobotMap(r_map)
        initial_resolutions = {}

        # During sifting we only need the cost, not the plan.
        # Use alpha-beta pruning to skip branches that cannot affect
        # the minimax result (only safe here because we discard the tree
        # immediately after; plan extraction still uses the un-pruned path).
        root_node = search_tree.search(initial_robot_map, initial_resolutions)
        if self.cost_metric == 'time':
            cost = search_tree.determine_time_with_pruning(root_node)
        else:
            cost = search_tree.determine_cost_with_pruning(root_node)

        # Save state so on_var_sifted can colour the PDF without a
        # redundant search.  The last call before on_var_sifted is
        # always at the best position for that variable.
        self.last_temp_bdd = product_bdd
        self.last_product_root = product_root
        self.last_search_tree = search_tree
        self.last_root_node = root_node

        return cost

def print_descendants(bdd, u, elegant_var_dict, visited=None):
    """Print BDD descendants with code-to-elegant name conversion."""
    if visited is None:
        visited = set()
    
    # Get the node ID from the Function object
    p = abs(u.node)
    
    # visited ?
    if p in visited:
        return
    # remember
    visited.add(p)
    
    # Convert code name to elegant name
    code_name = str(u)  # This gives us "@87", "@-1", etc.
    node_id = u.node
    abs_node_id = abs(node_id)
    
    # Check if terminal node
    if abs_node_id == 1:
        # Since the root is complemented, the semantics are flipped
        # @1 means "no plan" and @-1 means "plan exists"
        if node_id > 0:
            terminal_value = "No Plan Available"
        else:
            terminal_value = "Plan Exists"
        print(f"{code_name} -> Terminal: {terminal_value}")
    else:
        # Get variable info
        try:
            level, _, _ = bdd._bdd._succ[abs_node_id]
            # Find variable name from level
            var_name = None
            for var, var_level in bdd.vars.items():
                if var_level == level:
                    var_name = var
                    break
            
            # Find elegant name from var name
            elegant_name = None
            if var_name:
                for elegant, var in elegant_var_dict.items():
                    if var == var_name:
                        elegant_name = elegant
                        break
            
            if elegant_name:
                print(f"{code_name} -> {var_name} -> {repr(elegant_name)}")
            else:
                print(f"{code_name} -> {var_name} -> Not found")
                
        except KeyError:
            print(f"{code_name} -> Unknown node")
    
    # Check if this is a terminal node by accessing the underlying BDD directly
    i, v, w = bdd._bdd.succ(u.node)
    
    # u is terminal ?
    if v is None:
        return
    
    # Wrap the successors as Function objects
    v_func = bdd._wrap(v)
    w_func = bdd._wrap(w)
    
    print_descendants(bdd, v_func, elegant_var_dict, visited)
    print_descendants(bdd, w_func, elegant_var_dict, visited)


def build_adjacency_list(bdd, u, visited=None, adjacency_list=None):
    """
    Build an adjacency list representation of the BDD.
    
    Args:
        bdd: The BDD instance
        u: Starting node (Function object)
        visited: Set to track visited nodes
        adjacency_list: Dictionary to store the adjacency list
        
    Returns:
        dict: Adjacency list where keys are code names (@-87) and values are lists of connected nodes
    """
    if visited is None:
        visited = set()
    if adjacency_list is None:
        adjacency_list = {}
    
    # Get the node ID from the Function object
    p = abs(u.node)
    
    # Already visited?
    if p in visited:
        return adjacency_list
    
    # Mark as visited
    visited.add(p)
    
    # Get code name for current node
    code_name = str(u)  # This gives us "@87", "@-1", etc.
    
    # Initialize adjacency list for this node
    if code_name not in adjacency_list:
        adjacency_list[code_name] = []
    
    # Get successors
    i, v, w = bdd._bdd.succ(u.node)
    
    # If terminal node, no successors - just add it to the list and return
    if v is None:
        return adjacency_list
    
    # Wrap the successors as Function objects
    v_func = bdd._wrap(v)
    w_func = bdd._wrap(w)
    
    # Add edges to adjacency list
    v_code = str(v_func)
    w_code = str(w_func)
    
    adjacency_list[code_name].append(v_code)  # Low/False edge
    adjacency_list[code_name].append(w_code)  # High/True edge
    
    # Ensure terminal nodes are also added to the adjacency list
    if v_code not in adjacency_list:
        adjacency_list[v_code] = []
    if w_code not in adjacency_list:
        adjacency_list[w_code] = []
    
    # Recursively process successors
    build_adjacency_list(bdd, v_func, visited, adjacency_list)
    build_adjacency_list(bdd, w_func, visited, adjacency_list)
    
    return adjacency_list


def print_adjacency_list(adjacency_list):
    """Print the adjacency list in a readable format."""
    print("\nBDD Adjacency List:")
    print("=" * 40)
    for node, edges in adjacency_list.items():
        if edges:  # Non-terminal nodes
            print(f"{node} -> {edges}")
        else:  # Terminal nodes
            print(f"{node} -> [] (terminal)")


def get_code_to_locations_map(bdd, u, elegant_var_dict, visited=None):
    """
    Get mapping from BDD node codes to visiting locations.
    
    Args:
        bdd: The BDD instance
        u: Starting node (Function object)
        elegant_var_dict: Dictionary mapping elegant names to variable names
        visited: Set to track visited nodes
        
    Returns:
        dict: Map where keys are BDD node codes and values are lists of visiting locations
    """
    if visited is None:
        visited = set()
    
    code_to_locations = {}
    p = abs(u.node)
    
    if p in visited:
        return code_to_locations
    visited.add(p)
    
    code_name = str(u)
    abs_node_id = abs(u.node)
    
    if abs_node_id != 1:  # Not terminal
        try:
            level, _, _ = bdd._bdd._succ[abs_node_id]
            var_name = None
            for var, var_level in bdd.vars.items():
                if var_level == level:
                    var_name = var
                    break
            
            elegant_name = None
            if var_name:
                for elegant, var in elegant_var_dict.items():
                    if var == var_name:
                        elegant_name = elegant
                        break
            
            if elegant_name:
                # Extract location
                lines = elegant_name.split('\n')
                if len(lines) >= 1 and lines[0].startswith('Visit: '):
                    location = lines[0].replace('Visit: ', '').strip()
                    code_to_locations[code_name] = [location]
        except KeyError:
            pass
    
    # Process successors
    i, v, w = bdd._bdd.succ(u.node)
    if v is not None:
        v_func = bdd._wrap(v)
        w_func = bdd._wrap(w)
        code_to_locations.update(get_code_to_locations_map(bdd, v_func, elegant_var_dict, visited))
        code_to_locations.update(get_code_to_locations_map(bdd, w_func, elegant_var_dict, visited))
    
    return code_to_locations


def determine_plan_cost(bdd, product_root, elegant_var_dict, visited=None):
    # Build and display adjacency list
    adjacency_list = build_adjacency_list(bdd, product_root)

    # TODO: Integrate PLANNING
    # props -> adjacency_list
    # locations -> locs
    # edges -> adjacency_list
    # prop to location -> code_to_locations_map
    locs = {}
    for loc_name, coords in locs_with_coords.items():
        locs[loc_name] = (coords['x'], coords['y'])
        
    # Build visit location map
    # Get the code to locations mapping
    code_to_locations_map = get_code_to_locations_map(bdd, product_root, elegant_var_dict)
    # Create SearchTree instance and configure it with SRQL data
    search_tree = SearchTree()
    search_tree.bdd_config = search_tree.import_srql_config(adjacency_list, locs, code_to_locations_map, str(product_root))

    r_map = {}
    for robot in robots['robot_starts']:
        r_map[robot['robot_id']] = Robot(id=robot['robot_id'], position=locs[robot['start_location']])

    initial_robot_map = RobotMap(r_map)
    


    initial_resolutions = {}
    best_plan, *_ = search_tree.get_best_plan(initial_robot_map, initial_resolutions)
    return best_plan, search_tree.determine_cost(node=search_tree.robot_manager.head_time_step_node)


if __name__ == "__main__":
    bdd, product_root, elegant_var_dict, dumpformat, robots, locs_with_coords, q, w = get_bdd_from_srql()
    bdd.dump('product_graph', roots=[q, w], method="new", v_dict=elegant_var_dict, root_names_map={int(product_root):"Plan"}, leaf_names_map={True:"Yes ", False:" No "}, simplify_names=True, clean_print=True, fileformat=dumpformat)
    bdd.dump('plan', roots=[product_root], method="new", v_dict=elegant_var_dict, root_names_map={int(product_root):"Plan"}, leaf_names_map={True:"Yes ", False:" No "}, simplify_names=True, clean_print=True, fileformat=dumpformat)

    print("Finished the after PDFs")

    order = deepcopy(bdd.vars)

    # Clean up all references to prevent memory leak
    del product_root, q, w

