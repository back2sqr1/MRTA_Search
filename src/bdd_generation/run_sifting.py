#!/usr/bin/env python3
"""Run Rudell's Sifting Algorithm with Planning-Based Cost.

1. Loads an SRQL file and generates a BDD
2. Computes the BEFORE cost (initial variable ordering)
3. Runs Rudell's sifting algorithm using planning-based cost
4. Computes the AFTER cost (optimized variable ordering)
5. Generates PDF visualizations at each sifting step

Usage:
    python run_sifting.py [--input INPUT_FILE]
"""

import sys
import os
import argparse

# Add source directories to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from get_bdd_from_srql import get_bdd_from_srql
from customdd.autoref import BDD
from customdd.bdd import custom_reorder
from sifting import PlanningCostEvaluator, build_adjacency_list, get_code_to_locations_map

from navigation.navigation.planning.create_plan import SearchTree
from navigation.navigation.planning.robot_class import Robot, RobotMap

OUTPUT_FORMATS = ["pdf"]
DUMP_DEFAULTS = dict(
    method="new",
    leaf_names_map={True: "Yes", False: "No"},
    simplify_names=True,
    clean_print=True,
    fileformat=OUTPUT_FORMATS,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def build_readable_var_map(elegant_var_dict):
    """Reverse mapping from internal var names (e.g. 'vvv0') to readable names.

    Readable names are collapsed to a single line for console output.
    """
    return {internal: readable.replace("\n ", ",")
            for readable, internal in elegant_var_dict.items()}


def dump_bdd_pdf(bdd, filename, roots, label, elegant_var_dict):
    """Dump a BDD to PDF with standard formatting.

    ``roots`` should be a list of Function objects (autoref nodes).
    """
    root_names = {int(r): label for r in roots}
    bdd.dump(filename, roots=roots, v_dict=elegant_var_dict,
             root_names_map=root_names, **DUMP_DEFAULTS)


def dump_product_graph_pdf(bdd, wrld, qry, filename, label, elegant_var_dict):
    """Form a product graph in a *temporary* BDD and dump it as PDF.

    A temporary BDD is used so that no new nodes are added to the main
    BDD — this is critical when called mid-sifting.
    """
    temp_bdd = BDD(bdd.vars)
    prod = bdd.form_product_graph(temp_bdd, wrld, qry)
    if abs(int(prod)) == 1:
        return
    dump_bdd_pdf(temp_bdd, filename, [prod], label, elegant_var_dict)


def print_section(title):
    """Print a section header."""
    print("\n" + "=" * 60)
    print(title)
    print("=" * 60)


def sorted_var_names(bdd):
    """Return variable names sorted by their current level."""
    return sorted(bdd.vars.keys(), key=lambda v: bdd.vars[v])


# ---------------------------------------------------------------------------
# Planning
# ---------------------------------------------------------------------------

def compute_plan_and_cost(bdd, wrld, qry, elegant_var_dict, robots, locs_with_coords, cost_metric='distance'):
    """Compute the planning cost and detailed plan for the current variable ordering.

    Args:
        cost_metric: 'distance' for sum of robot distances, 'time' for parallel wall-clock time.

    Returns (cost, best_plan, detailed_steps, product_root).
    """
    product_root = bdd.form_product_graph(bdd, wrld, qry)
    if abs(int(product_root)) == 1:
        return 0, [], [], product_root

    locs = {name: (coords['x'], coords['y'])
            for name, coords in locs_with_coords.items()}

    adjacency_list = build_adjacency_list(bdd, product_root)
    code_to_locations_map = get_code_to_locations_map(bdd, product_root, elegant_var_dict)

    search_tree = SearchTree()
    search_tree.bdd_config = search_tree.import_srql_config(
        adjacency_list, locs, code_to_locations_map, str(product_root))

    r_map = {
        r['robot_id']: Robot(id=r['robot_id'], position=locs[r['start_location']])
        for r in robots['robot_starts']
    }

    if cost_metric == 'time':
        best_plan, _, detailed_steps = search_tree.get_best_plan_by_time(RobotMap(r_map), {})
        if not detailed_steps:
            return 0, [], [], product_root
        cost = search_tree.determine_time(node=search_tree.robot_manager.head_time_step_node)
    else:
        best_plan, _, detailed_steps = search_tree.get_best_plan(RobotMap(r_map), {})
        if not detailed_steps:
            return 0, [], [], product_root
        cost = search_tree.determine_cost(node=search_tree.robot_manager.head_time_step_node)

    return cost, best_plan, detailed_steps, product_root


def print_detailed_plan(detailed_steps, label):
    """Print a step-by-step plan to the console."""
    print(f"\n  {label} Step-by-Step Plan:")
    print("  " + "-" * 56)

    if not detailed_steps:
        print("    No steps (trivial query)")
        return

    for step in detailed_steps:
        print(f"\n  Step {step['step']} [{step['type'].upper()}]")
        print(f"    {step.get('instruction', 'N/A')}")

        if step['visited_locations']:
            print(f"    Visited: {step['visited_locations']}")
        if step['resolved_questions']:
            print(f"    Resolved: {step['resolved_questions']}")

        for robot_id, info in step['robots'].items():
            status = f"at {info['position']}"
            if info['assigned_location']:
                status += f" -> {info['assigned_location']}"
            print(f"    {robot_id}: {status} (cost: {info['cost_so_far']:.2f})")

    print("\n  " + "-" * 56)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='Run Rudell Sifting with Planning Cost')
    parser.add_argument('--input', type=str, default='examples_2/tate-ex1.srql',
                        help='Input SRQL file')
    parser.add_argument('--cost-metric', type=str, choices=['distance', 'time'],
                        default=None,
                        help='Cost metric: "distance" (sum of all robot distances) '
                             'or "time" (wall-clock parallel completion time)')
    args = parser.parse_args()

    cost_metric = args.cost_metric
    if cost_metric is None:
        print("\nSelect cost metric:")
        print("  [1] distance  — sum of all robot distances (sequential)")
        print("  [2] time      — wall-clock parallel completion time")
        choice = input("Enter 1 or 2 (default: 1): ").strip()
        cost_metric = 'time' if choice == '2' else 'distance'

    print(f"\nCost metric: {cost_metric}")

    # --- Load ---
    print_section("RUDELL'S SIFTING WITH PLANNING-BASED COST")

    input_path = os.path.join(os.path.dirname(__file__), args.input)
    if not os.path.exists(input_path):
        input_path = args.input
    print(f"\nLoading: {input_path}")

    bdd, _, elegant_var_dict, _, robots, locs_with_coords, q, w = get_bdd_from_srql(input_path)
    var_to_readable = build_readable_var_map(elegant_var_dict)

    print(f"Variables: {len(bdd.vars)}")
    print(f"Robots: {robots['num_robots']}")
    print(f"Locations: {list(locs_with_coords.keys())}")

    # --- Before ---
    print_section("BEFORE SIFTING (Initial Variable Ordering)")
    print(f"\nVariable order: {sorted_var_names(bdd)}")

    cost_before, _, steps_before, prod_before = compute_plan_and_cost(
        bdd, w, q, elegant_var_dict, robots, locs_with_coords, cost_metric)
    print(f"\nCost: {cost_before:.4f}")
    print_detailed_plan(steps_before, "BEFORE")

    if abs(int(prod_before)) != 1:
        dump_bdd_pdf(bdd, 'before_sifting', [prod_before],
                     f"Before Sifting (cost={cost_before:.2f})", elegant_var_dict)

    # --- Sift ---
    print_section("RUNNING RUDELL'S SIFTING ALGORITHM")

    evaluator = PlanningCostEvaluator(
        elegant_var_dict=elegant_var_dict,
        robots=robots,
        locs_with_coords=locs_with_coords,
        cost_metric=cost_metric)

    print("\nSifting in progress...")
    print("(Each variable is swept across all positions to find optimal placement)\n")

    sifting_step = [0]

    def on_var_sifted(var, best_level, cost):
        step = sifting_step[0]
        readable = var_to_readable.get(var, var)
        print(f"  Sifted '{readable}' -> level {best_level}, cost = {cost:.4f}")
        dump_product_graph_pdf(
            bdd, w, q,
            filename=f'sifting_step_{step}',
            label=f"Step {step}: sifted '{readable}' (cost={cost:.2f})",
            elegant_var_dict=elegant_var_dict)
        sifting_step[0] += 1

    custom_reorder(bdd, w, q, plan_cost_fn=evaluator, on_var_sifted=on_var_sifted)
    print("\nSifting complete!")

    # --- After ---
    print_section("AFTER SIFTING (Optimized Variable Ordering)")
    print(f"\nVariable order: {sorted_var_names(bdd)}")

    cost_after, _, steps_after, prod_after = compute_plan_and_cost(
        bdd, w, q, elegant_var_dict, robots, locs_with_coords, cost_metric)
    
    print(f"\nCost: {cost_after:.4f}")
    print_detailed_plan(steps_after, "AFTER")

    if abs(int(prod_after)) != 1:
        dump_bdd_pdf(bdd, 'after_sifting', [prod_after],
                     f"After Sifting (cost={cost_after:.2f})", elegant_var_dict)

    # --- Summary ---
    print_section("SUMMARY")
    metric_label = "Time" if cost_metric == 'time' else "Cost"
    print(f"\nMetric: {cost_metric}")
    print(f"{metric_label} BEFORE sifting: {cost_before:.4f}")
    print(f"{metric_label} AFTER sifting:  {cost_after:.4f}")

    if cost_before > 0:
        improvement = cost_before - cost_after
        pct = (improvement / cost_before) * 100
        print(f"Improvement: {improvement:.4f} ({pct:.1f}% reduction)")
    else:
        print("Trivial query - no improvement possible")

    print("\nGenerated files:")
    print("  - before_sifting.pdf  (Initial variable ordering)")
    for i in range(sifting_step[0]):
        print(f"  - sifting_step_{i}.pdf  (After sifting variable {i + 1})")
    print("  - after_sifting.pdf   (After Rudell's sifting)")
    print("\nDone!")


if __name__ == "__main__":
    main()
