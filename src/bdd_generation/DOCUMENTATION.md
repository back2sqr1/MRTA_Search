# BDD Generation and Multi-Robot Planning System Documentation

This documentation covers the complete pipeline for creating Binary Decision Diagrams (BDDs) from SRQL specifications and optimizing them using Rudell's sifting algorithm with a custom cost function based on multi-robot path planning.

## Table of Contents

1. [Overview](#overview)
2. [SRQL Language Reference](#srql-language-reference)
3. [BDD Generation Pipeline](#bdd-generation-pipeline)
4. [Sifting Algorithm](#sifting-algorithm)
5. [Cost Function](#cost-function)
6. [Multi-Robot Planning Integration](#multi-robot-planning-integration)
7. [File Reference](#file-reference)

---

## Overview

This system implements:

- **BDD Creation**: Based on Dr. Randal E. Bryant's seminal paper "Graph-based algorithms for Boolean function manipulation" (1986)
- **Variable Reordering**: Uses Richard Rudell's dynamic variable ordering algorithm (1993)
- **Custom Cost Function**: Integrates multi-robot path planning costs instead of traditional BDD size metrics
- **Multi-Robot Planning**: Uses a search tree with robot assignments to compute optimal observation plans

### Architecture Diagram

```
SRQL File (.srql)
       |
       v
  [sparser.py] ─────> Parsed World Dictionary
       |                    |
       v                    v
[get_bdd_from_srql.py] ─> BDD + Product Graph
       |                    |
       v                    v
[customdd/bdd.py] ────> compute_query_cost()
       |                    |
       v                    v
[sifting.py] ─────────> Sifted BDD with optimal variable ordering
       |                    |
       v                    v
[navigation/planning/] ─> Multi-robot plan with minimal travel cost
```

---

## SRQL Language Reference

SRQL (Scenario Robot Query Language) is a domain-specific language for specifying:
- World topology (locations and their coordinates)
- Robot configurations
- Observable properties
- World constraints (logical formulas)
- Queries to evaluate

### File Formats

| Extension | Purpose |
|-----------|---------|
| `.srql`   | Full world specification with locations, robots, properties, constraints, and queries |
| `.qqrl`   | Query-only files (constraints only, no world definition) |

### Syntax Reference

#### 1. Location Definitions

Locations define the navigable points in the world.

**New Format (with coordinates):**
```srql
location <name> at <x>, <y>
```

**Old Format (without coordinates):**
```srql
location <name>
```

**Examples:**
```srql
location locA at 0, 0
location locB at 0, 1
location loc15 at 2, 4

# Old format (requires transition costs)
location a11
location a22
```

**Rules:**
- Names must start with a lowercase letter or underscore
- Names can contain letters, numbers, hyphens, underscores, and apostrophes
- Coordinates are integers

---

#### 2. Robot Configuration

Specifies the number of robots and their starting positions.

**Syntax:**
```srql
robots <count>
robot <robot_id> starts-at <location_name>
```

**Example:**
```srql
robots 2
robot robot1 starts-at locB
robot robot2 starts-at locA
```

**Rules:**
- `robots` must appear before individual `robot` declarations
- Each robot must have a unique ID
- Starting location must be a defined location
- If omitted, defaults to 1 robot with no specified start

---

#### 3. Transitions (Old Format Only)

Defines movement costs between locations. Used when coordinates are not specified.

**Syntax:**
```srql
transition <source> <destination> <cost>
```

**Example:**
```srql
transition a11 a21 2.0
transition a12 a11 2.0
transition loc01 loc02 42.57
```

**Rules:**
- Transitions are directed (a->b is different from b->a)
- Cost can be an integer or float
- Required when using the old format without coordinates

---

#### 4. Observation Costs (Old Format Only)

Defines the cost to make an observation at a location.

**Syntax:**
```srql
observation-cost <location> <cost>
```

**Example:**
```srql
observation-cost a22 1.5
observation-cost loc15 20.0
```

**Rules:**
- Cost is added to travel cost when visiting a location
- If not specified, defaults to 0

---

#### 5. Recognizable Properties

Defines observable properties that can be true or false at locations.

**Syntax:**
```srql
recognizable-property <property_name>
```

**Example:**
```srql
recognizable-property marilyn-diptych
recognizable-property painting
recognizable-property sculpture
recognizable-property english
```

**Rules:**
- Property names follow the same rules as location names
- Properties create Boolean variables of the form `property(location)` for each location

---

#### 6. Starting Location (Old Format Only)

Specifies where the single robot begins.

**Syntax:**
```srql
starting-location <location>
```

**Example:**
```srql
starting-location a11
```

---

#### 7. World Constraints

Constraints define logical rules about the world. There are three types:

##### 7.1 Facts (Ground Formulas)

Simple true/false assertions about specific property-location pairs.

**Syntax:**
```srql
<property>(<location>)
!<property>(<location>)
~<property>(<location>)
```

**Example:**
```srql
corner(loc11)
!corner(locA)
~ell-shaped(loc01)
```

##### 7.2 Simple Formulas (No Quantifiers)

Formulas wrapped in braces using propositional logic.

**Syntax:**
```srql
{ <formula> }
```

**Operators (usable inside `{}`):**

| Operator | Meaning | Aliases | Notes |
|----------|---------|---------|-------|
| `&`      | AND     | `&&`    | Both operands must be true |
| `\|`     | OR      | `\|\|` | At least one operand must be true |
| `^`      | XOR     |         | Exactly one operand must be true. **Caution:** the product graph may infer variables from world constraints, which can prevent the planner from investigating both sides — see Known Limitations below |
| `!`      | NOT     |         | Prefix negation. (`~` works only outside braces in flat facts) |
| `->`     | IMPLIES |         | `a -> b` means "if a then b" (equivalent to `!a \| b`) |
| `<->`    | EQUIV   |         | `a <-> b` means "a if and only if b" (true when both are equal) |
| `()`     | Grouping |        | Parentheses for controlling evaluation order |
| `True`   | Boolean true |    | Constant |
| `False`  | Boolean false |   | Constant |

**Precedence (highest to lowest binding):**

| Priority | Operator | Associativity |
|----------|----------|---------------|
| 1 (highest) | `!` (NOT) | Right |
| 2        | `&` (AND) | Left |
| 3        | `\|` (OR) | Left |
| 4        | `^` (XOR) | Left |
| 5        | `->` (IMPLIES) | Left |
| 6 (lowest) | `<->` (EQUIV) | Left |

**Atoms:**

Atoms inside formulas take the form `property(location)`, where both the property
and location must be declared earlier in the file. Quantifier variables (`?x`) can
be used in place of locations when the formula is wrapped in a quantifier block.

**Symbols blocked inside `{}`:**

The SRQL lexer only passes a limited character set through to the Boolean formula
parser. The following operators are supported by the underlying BDD engine but
**cannot** be used inside `{}` braces in SRQL files:

| Symbol | Meaning | Why blocked |
|--------|---------|-------------|
| `~`    | NOT (alternative) | `~` not in SRQL brace charset. Use `!` instead |
| `=>`   | IMPLIES (alternative) | `=` not in SRQL brace charset. Use `->` instead |
| `<=>`  | EQUIV (alternative) | `=` not in SRQL brace charset. Use `<->` instead |
| `/\`   | AND (alternative) | `/` not in SRQL brace charset. Use `&` instead |
| `\/`   | OR (alternative) | `/` not in SRQL brace charset. Use `\|` instead |
| `ite(a, b, c)` | If-then-else | `,` not in SRQL brace charset |

> **Note:** `~` and `!` both work for negation in **flat facts** outside braces
> (e.g., `!art(hub)` or `~art(hub)`), but only `!` works inside `{}` formulas.

**Example:**
```srql
{ english(a12) }

{
  marilyn-diptych(loc15) | marilyn-diptych(loc06) | marilyn-diptych(loc07) | marilyn-diptych(loc08)
}

# EQUIV example: art at b if and only if sign at a
{ sign(a) <-> art(b) }

# IMPLIES example: if art at b, then no art at c, d, or e
{ art(b) -> !art(c) & !art(d) & !art(e) }
```

##### 7.3 Quantified Formulas

Formulas with universal or existential quantifiers.

**Universal Quantifier Syntax:**
```srql
[?x]{ <formula using ?x> }
[?x, ?y]{ <formula using ?x and ?y> }
[?x, ?y | ?x != ?y]{ <formula> }
```

**Existential Quantifier Syntax:**
```srql
[E?x]{ <formula using ?x> }
```

**Examples:**

```srql
# Universal: Every painting is exactly one of three nationalities
[?x]{
    ( english(?x)  & !(italian(?x) | dutch(?x)  )) |
    ( italian(?x)  & !(english(?x) | dutch(?x)  )) |
    (   dutch(?x)  & !(english(?x) | italian(?x)))
}

# Universal with constraint: At most one Marilyn exists
[?u, ?v | ?u != ?v]{
  marilyn-diptych(?u) -> !marilyn-diptych(?v)
}

# Existential: There exists at least one sculpture
[E?x]{
    sculpture(?x)
}
```

**Quantifier Rules:**
- `?x` is a universal quantifier variable (expands to AND over all locations)
- `E?x` is an existential quantifier variable (expands to OR over all locations)
- `| ?x != ?y` adds a constraint that the variables must be different
- Only one existential quantifier is supported per formula
- Variables range over all defined locations

---

#### 8. Query

The query defines what property or condition we want to determine.

**Syntax:**
```srql
Query: { <formula> }
Query: [?x]{ <formula> }
```

**Examples:**
```srql
# Simple query: Is the Marilyn at loc15?
Query: { (marilyn-diptych(loc15)) }

# Compound query with XOR
Query: { ((english(a21)) ^ (italian(a22))) | (english(a11) | dutch(a11)) }

# Quantified query: Is the Marilyn NOT in an L-shaped room?
Query: [?x]{ (marilyn-diptych(?x)) -> (!ell-shaped(?x)) }
```

---

### Complete SRQL Examples

#### Example 1: Museum Navigation (New Format)

```srql
# Define the map with pin coordinates
location locA at 0, 0
location locB at 0, 1
location loc06 at 1, 2
location loc07 at 2, 2
location loc08 at 3, 2
location loc15 at 2, 4

# Robot configuration
robots 2
robot robot1 starts-at locB
robot robot2 starts-at locA

# Observable property
recognizable-property marilyn-diptych

# Constraint: Marilyn can only be in one of the big rooms
{
  marilyn-diptych(loc15) | marilyn-diptych(loc06) | marilyn-diptych(loc07) | marilyn-diptych(loc08)
}

# Constraint: There is exactly one Marilyn
[?u, ?v | ?u != ?v]{
  marilyn-diptych(?u) -> !marilyn-diptych(?v)
}

# Query: Is the Marilyn in the corner room (loc15)?
Query: { (marilyn-diptych(loc15)) }
```

#### Example 2: Art Gallery (Old Format with Transitions)

```srql
# Define locations
location a22
location a11
location a12
location a21

# Define transitions with costs
transition a11 a21 2.0
transition a12 a11 2.0
transition a22 a12 2.0
transition a21 a22 2.0

# Observation costs
observation-cost a22 1.5
observation-cost a11 1.25
observation-cost a21 0.5
observation-cost a12 0.5

# Observable properties
recognizable-property english
recognizable-property italian
recognizable-property dutch

# Starting location
starting-location a11

# Fact: English painting at a12
{ english(a12) }

# Constraint: Each painting is exactly one nationality
[?x]{
    ( english(?x)  & !(italian(?x) | dutch(?x)  )) |
    ( italian(?x)  & !(english(?x) | dutch(?x)  )) |
    (   dutch(?x)  & !(english(?x) | italian(?x)))
}

# Query: Is there an English at a21 XOR Italian at a22, or is there English/Dutch at a11?
Query: { ((english(a21)) ^ (italian(a22))) | (english(a11) | dutch(a11)) }
```

---

## BDD Generation Pipeline

### Step 1: Parsing (`sparser.py`)

The parser uses PLY (Python Lex-Yacc) to tokenize and parse SRQL files.

**Output Structure:**
```python
{
    'locations': [{'name': 'locA', 'x': 0, 'y': 0}, ...],
    'robots': {'num_robots': 2, 'robot_starts': [{'robot_id': 'robot1', 'start_location': 'locB'}, ...]},
    'transitions': [('a11', 'a21', 2.0), ...],
    'obscosts': [('a22', 1.5), ...],
    'properties': ['marilyn-diptych', 'painting', ...],
    'constraints': [(quantifier_dict, formula_string), ...],
    'query': (quantifier_dict, formula_string)
}
```

### Step 2: Variable Creation (`get_bdd_from_srql.py`)

For each `property` and `location`, a BDD variable is created:

```python
# Variable naming: vvv0, vvv1, vvv2, ...
# Mapping: property(location) -> internal variable name

var_dict = {
    'marilyn-diptych(locA)': 'vvv0',
    'marilyn-diptych(locB)': 'vvv1',
    'marilyn-diptych(loc06)': 'vvv2',
    ...
}

# Block structure: Variables at same location form a block
block_dict = {
    'vvv0': 1,  # locA
    'vvv1': 2,  # locB
    'vvv2': 3,  # loc06
    ...
}
```

### Step 3: Formula Transformation

Quantified formulas are expanded (unrolled) over all locations:

```python
# Input: [?x]{ english(?x) }
# Output: (english(a11) & english(a12) & english(a21) & english(a22))

# Input: [E?x]{ sculpture(?x) }
# Output: (sculpture(loc01) | sculpture(loc02) | ... | sculpture(locN))
```

### Step 4: BDD Construction

The BDD is constructed using the formula operators:
- World constraints are ANDed together
- Constants are applied using `bdd.let()`
- Product graph is formed from world and query BDDs

---

## Sifting Algorithm

Based on Rudell's "Dynamic variable ordering for ordered binary decision diagrams" (1993).

### Standard Sifting

Minimizes BDD size by finding optimal variable ordering:

```
for each variable v in BDD:
    1. Move v to one end of the ordering
    2. Sweep v to the other end, recording size at each position
    3. Place v at the position that minimizes size
```

### Planning-Based Cost Sifting

Instead of minimizing BDD size, minimizes **multi-robot planning cost** using `SearchTree.determine_cost()`.

#### Architecture

```
custom_reorder(bdd, wrld, qry, plan_cost_fn=evaluator)
    ↓
_apply_custom_sifting() / _apply_custom_block_sifting()
    ↓
[For each variable position]:
    1. Swap variables
    2. Call plan_cost_fn(bdd, wrld, qry)
         ↓
         PlanningCostEvaluator.__call__()
             ├─ Form product graph
             ├─ Build adjacency list
             ├─ Create SearchTree
             ├─ Run search()
             └─ Return determine_cost()
    3. Record cost at this position
    ↓
Select position with minimum cost
```

#### Usage Example

```python
from sifting import PlanningCostEvaluator
from get_bdd_from_srql import get_bdd_from_srql
from customdd.bdd import custom_reorder

# Get BDD and world data
bdd, product_root, elegant_var_dict, dumpformat, robots, locs_with_coords, q, w = get_bdd_from_srql()

# Create the planning cost evaluator
evaluator = PlanningCostEvaluator(
    elegant_var_dict=elegant_var_dict,
    robots=robots,
    locs_with_coords=locs_with_coords
)

# Run sifting with planning-based cost function
custom_reorder(
    bdd,
    wrld=w,
    qry=q,
    plan_cost_fn=evaluator  # Uses SearchTree.determine_cost()
)

# The BDD is now reordered to minimize multi-robot planning cost
```

#### PlanningCostEvaluator Class

Located in `sifting.py`, this class encapsulates all data needed for planning-based cost evaluation:

```python
class PlanningCostEvaluator:
    def __init__(self, elegant_var_dict, robots, locs_with_coords):
        """
        Args:
            elegant_var_dict: Dict mapping elegant names to BDD variable names
            robots: Dict with 'num_robots' and 'robot_starts' list
            locs_with_coords: Dict mapping location names to {'x': int, 'y': int}
        """

    def __call__(self, bdd, wrld, qry) -> float:
        """
        Compute multi-robot planning cost for current BDD ordering.

        Returns:
            Cost using MIN over robot assignments, MAX over query branches
        """
```

### Block Sifting

Variables in the same block (same location) are kept contiguous:

```python
# Block structure ensures variables for same location stay together
# e.g., all properties at loc15 form one block
custom_reorder(bdd, wrld, qry, blocksdefn=block_dict, plan_cost_fn=evaluator)
```

---

## Cost Function

The cost function (`calc_cost` in `get_bdd_from_srql.py`) computes the cost of transitioning between observations.

### Cost Computation Logic

```python
def calc_cost(src, dest):
    """
    Computes cost of moving from src observation to dest observation.

    src: Previous variable (or "<start>" for initial state)
    dest: Next variable (or "<term>" for terminal state)

    Returns: Travel distance + observation cost
    """
    if dest == "<term>":
        return 0  # No cost at terminal

    if src == "<start>" and dest == "<start>":
        return 0  # Tautology, empty plan suffices

    obcost = observation_cost[location_of(dest)]

    if src == "<start>":
        return floyd_table[(start_location, location_of(dest))] + obcost

    if location_of(dest) == location_of(src):
        return 0  # Same location, no travel cost

    return floyd_table[(location_of(src), location_of(dest))] + obcost
```

### Key Properties

1. **Floyd-Warshall Table**: Precomputed all-pairs shortest paths
2. **Observation Cost**: Added when visiting a new location
3. **Same-Location Optimization**: Zero cost for observing multiple properties at same location
4. **Block Structure**: Variables at same location grouped together

---

## Multi-Robot Planning Integration

Located in `src/navigation/navigation/planning/`.

### SearchTree (`create_plan.py`)

The search tree explores all possible robot assignment configurations:

```python
class SearchTree:
    def search(self, initial_robot_map, initial_resolution):
        """
        Breadth-first search through configuration space.

        Node Types:
        - robot_assignment: Assigns robots to locations
        - robot_moving: Robots in transit
        - query: Observation/decision point
        """
        while time_step_queue:
            current = time_step_queue.pop(0)
            combinations = generate_combinations(current.query, robot_map)
            for combo in combinations:
                process_combinations(combo, current, robot_map)
```

### Cost Aggregation (`determine_cost`)

```python
def determine_cost(node):
    """
    Recursively compute cost through search tree.

    - robot_moving nodes: Pass through (no aggregation)
    - query nodes: MAX of all child costs (worst-case over uncertainty)
    - robot_assignment nodes: MIN of all child costs (best assignment choice)
    """
    if node.type == 'robot_moving':
        return determine_cost(node.next[0])
    elif node.type == 'query':
        return max(determine_cost(child) for child in node.next)
    elif node.type == 'robot_assignment':
        return min(determine_cost(child) for child in node.next)
```

### RobotManager (`robot_manager.py`)

Manages robot state and movement simulation:

```python
class RobotManager:
    def update_robot_positions(self, robot_map):
        """Move robots toward assigned locations using Euclidean distance."""
        minimal_arrival_time = min(robot.time + distance/velocity for robot in robots)
        for robot in robots:
            move_robot_towards_location(robot, target, time_diff)
        return arrived_robots

    def generate_combinations(self, property, robot_map, visited):
        """Generate all valid robot-to-location assignments."""
        # At least one robot must be assigned to a property location
        # Locations with all known properties are skipped
```

---

## File Reference

### Core Files

| File | Purpose |
|------|---------|
| `sparser.py` | PLY-based parser for SRQL/QQRL files |
| `get_bdd_from_srql.py` | Converts parsed SRQL to BDD, defines `calc_cost()` |
| `sifting.py` | Integration between BDD optimization and planning |
| `customdd/bdd.py` | Core BDD implementation with sifting algorithms |

### Navigation Planning

| File | Purpose |
|------|---------|
| `navigation/planning/create_plan.py` | SearchTree for multi-robot planning |
| `navigation/planning/robot_manager.py` | Robot state and movement simulation |
| `navigation/planning/robot_class.py` | Robot and RobotMap classes |
| `navigation/planning/time_step_node_class.py` | TimeStepNode for search tree |

### Example Files

| File | Description |
|------|-------------|
| `examples_2/tate-ex1.srql` | Simple 2-robot museum navigation |
| `examples_2/decision-tree.sqrl` | Branching out example A -> B, C OR -> D, E |
| `examples_2/three-wings.srql` | Three different ways to go, fastest is when two robots go to 2 wings that are closer |

### Key Functions

| Function | Location | Purpose |
|----------|----------|---------|
| `parse_world(s)` | sparser.py:332 | Parse SRQL string to dictionary |
| `get_bdd_from_srql()` | get_bdd_from_srql.py:195 | Main BDD generation entry point |
| `calc_cost(src, dest)` | get_bdd_from_srql.py:150 | Edge cost function (travel + observation) |
| `compute_query_cost()` | customdd/bdd.py:2530 | Compute BDD traversal cost (simple Dijkstra) |
| `compute_max_cost()` | customdd/bdd.py:2461 | Dijkstra-style cost on product graph |
| `custom_reorder()` | customdd/bdd.py:2546 | Rudell's sifting with optional `plan_cost_fn` |
| `PlanningCostEvaluator` | sifting.py:17 | Callable class for multi-robot planning cost |
| `determine_plan_cost()` | sifting.py:317 | Legacy function for planning-based cost |
| `SearchTree.determine_cost()` | create_plan.py:204 | Core cost aggregation (MIN/MAX) |
| `SearchTree.determine_time()` | create plan.py:238 | Cost in time, only with single pins, no midpoint or centroid |

---

## References

1. Bryant, R.E. (1986). "Graph-based algorithms for Boolean function manipulation". IEEE Transactions on Computers, C-35(8), 677-690.

2. Rudell, R. (1993). "Dynamic variable ordering for ordered binary decision diagrams". IEEE/ACM ICCAD, 42-47.

3. Brace, K.S., Rudell, R.L., & Bryant, R.E. (1990). "Efficient implementation of a BDD package". 27th ACM/IEEE DAC, 40-45.
