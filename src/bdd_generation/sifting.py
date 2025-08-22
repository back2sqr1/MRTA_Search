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


bdd, product_root, elegant_var_dict, dumpformat, robots, locs_with_coords, q, w = get_bdd_from_srql()
bdd.dump('product_graph', roots=[q, w], method="new", v_dict=elegant_var_dict, root_names_map={int(product_root):"Plan"}, leaf_names_map={True:"Yes ", False:" No "}, simplify_names=True, clean_print=True, fileformat=dumpformat)
bdd.dump('plan', roots=[product_root], method="new", v_dict=elegant_var_dict, root_names_map={int(product_root):"Plan"}, leaf_names_map={True:"Yes ", False:" No "}, simplify_names=True, clean_print=True, fileformat=dumpformat)

print("Finished the after PDFs")

order = deepcopy(bdd.vars)





# REMEMBER: PROPERTIES can involve different results from different locations
# ex: diptypch(loc01), diptypch(loc02)
# however, I'm trying these as different properties in my code (DAVID)

# TODO: DO Complete search
it = 0
# Get all permutations and shuffle them randomly
diff_adj_list = set()
diff_cost = set()
possible_best_plan, cost_2 = determine_plan_cost(bdd, q, elegant_var_dict)
if len(possible_best_plan) == 0:
    cost_2 = 0
print("Possible Plan", possible_best_plan, cost_2)
print("FINISHED PLANNING")

for perm in itertools.permutations(order):


    perm_dict = dict(zip(perm, range(len(perm))))
    # convert perm to dict
    # bdd.dump(f'perm_{it}', roots=[q, w], method="new", v_dict=elegant_var_dict, root_names_map={int(product_root):"Plan"}, leaf_names_map={True:"Yes ", False:" No "}, simplify_names=True, clean_print=True, fileformat=dumpformat)
    # bdd.dump(f'plan_{it}', roots=[product_root], method="new", v_dict=elegant_var_dict, root_names_map={int(product_root):"Plan"}, leaf_names_map={True:"Yes ", False:" No "}, simplify_names=True, clean_print=True, fileformat=dumpformat)

    bdd.reorder(perm_dict)
    print("FINISHED REORDERING")
    best_plan, cost = determine_plan_cost(bdd, product_root, elegant_var_dict, visited=None)
    if len(best_plan) == 0:
        cost = 0

    print(f"Plan {it} has cost {cost}")
    print(best_plan)

    it+=1

# Clean up all references to prevent memory leak
del product_root, q, w

