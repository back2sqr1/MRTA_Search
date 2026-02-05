#!/usr/bin/python
import sys
import os
import re
import sparser
import argparse
import random
from time import process_time

locs = None
props = None
verbose = False
idreg = r'[a-zA-Z_][a-zA-Z_0-9\-\'\_]*'

from customdd.autoref import BDD
from customdd.bdd import compute_query_cost
#, _check_ordering_respects_blocks _check_ordering_respects_blocks(bdd, block_dict, None)



def transform_grounded_expression(varsmap, theexpr):
    global locs, props
    # slight optimization, theexpr is short, so we figure out which are the substitutions to make:
    hotkeys = set(re.findall(idreg+r'\('+idreg+r'\)',theexpr))
    #print("hotkeys = %s" % repr(hotkeys))
    try:
        for k in hotkeys:
            theexpr =  theexpr.replace(k,"("+varsmap[k]+")")
    except KeyError as e:
        spl = re.match('('+idreg+r')\(('+idreg+r')\)',k)
        if not (spl[1] in props):
            print("Property '%s' is unknown" % spl[1])
        if not (spl[2] in locs):
            print("Location '%s' is unknown" % spl[2])
        print("Unable to make sense of '%s'." % k)
        sys.exit(1)

    return theexpr

def unroll_quantifiers(quants, constrs, theexpr):
    global locs, props, verbose

    nq =  len(quants)
    if nq == 0:
        #print("Processing empty quantifier for '%s'" % theexpr)
        if theexpr.find('?') >= 0: 
            print("Unbound variable '?%s' in formula '%s'." % (theexpr[theexpr.find('?')+1], theexpr))
            sys.exit(1)
        return (True, [theexpr])

        
    result = None

    domlist = locs
    ndom =  len(domlist)
    try:
        pre_proc_constrs = [(quants.index(x),quants.index(y)) for (x,y) in constrs]
    except ValueError as e:
        print("Quantifier constraint '%s' refers to something that is not variable." % constrs)
        sys.exit(1)


    exists = [x[1:] for x in quants if x[0] == 'E']
    if (len(exists) > 0):
        assert (len(quants) == 1), "We only support one existential quantifier, on its own"

    # A x  P(x) -> P(x1) & P(x2) & P(x3)
    # E x  P(x) -> (P(x1) | P(x2) | P(x3)) 
    if exists:
        quants = exists

    counter = [0] * nq
    done = False 
    cursor = 0
    while not done:
        skip = False
        for (r,s) in pre_proc_constrs:
            if counter[r] == counter[s]:
                skip = True
                break

        if not skip: 
            # Makes this substitution
            if verbose:
                print("%s"  % repr(counter), end='\t')
                for i in range(0,len(counter)):
                    print("%d %s -> %s " % (i, quants[i], domlist[counter[i]]), end='')
                print(" ")

            res =  theexpr
            for i in range(0,len(counter)):
                res =  res.replace(quants[i],domlist[counter[i]])

            if not result: # first iteration, we check for unbound var
                if res.find('?') >= 0: 
                    print("Unbound variable '?%s' in formula '%s'." % (res[res.find('?')+1], theexpr))
                    sys.exit(1)
                result = set([ res ])
            else:
                result.add(res)

        while counter[cursor] == ndom-1:
            cursor = cursor + 1
            if cursor == nq:  
                done = True
                break
        if not done:
            counter[0:cursor] = [0] * cursor 
            counter[cursor] = counter[cursor] + 1
            cursor = 0

    if exists:
        return (False, result)

    return (True, result)

def formula_to_sat(varsmap, form):
    if not form[0]: # fact
        #print("FACT: %s" % form[1])
        trs = transform_grounded_expression(varsmap, form[1])
        return ['('+trs+')']
    else: # has quantifiers
        #print("Q %s : %s" % (form[0]['quants'], form[1]))
        (univ, expandedf) = unroll_quantifiers(form[0]['quants'], form[0]['constraints'],form[1])
        if univ:
            expandedf_grounded = ['('+transform_grounded_expression(varsmap, x)+')' for x in expandedf]
        else: # exististential quantifer
            terms = ['('+transform_grounded_expression(varsmap, x)+')' for x in expandedf]
            expandedf_grounded = ["("+ ("|".join(terms)) + ")"]
        return expandedf_grounded

def allidentical(l):
    if l:
        first = l[0]
        return all([first == x for x in l])
    else:
        return True

def print_floyd_table():
    _f_table = calc_cost.floydtable
    for s in ["   "] + locs:
            print("   %s" % s, end="\t")
    print("")
    for s in locs:
        print("%s" % s, end="\t")
        for d in locs:
            print("% 6.2f" % _f_table[(s,d)], end="\t")
        print("")


def calc_cost(src, dest): 
    global verbose
    # This computes the cost of being at src, and having already observed, and arriving at dest and observing, with no observations made in the route in between.
    _f_table = calc_cost.floydtable
    _v_map =  calc_cost.var_to_loc_map
    _obs_cost = calc_cost.obscosts 
    _s_loc = calc_cost.start_loc
    assert(_f_table[(_s_loc,_s_loc)] == 0)

    if dest == "<term>": # Special signifier of ending point (for a terminal cost)
        return 0

    try:
        obcost = _obs_cost[_v_map[dest]]        
    except:
        obcost = 0

    if src == "<start>" and dest == "<start>":
        if verbose:
            print("Tautology, empty plan suffices")
        return 0

    if src == "<start>": # Special signifier of initialization point
        return _f_table[(_s_loc,_v_map[dest])] + obcost
            # If already at starting point, only have to observe, otherwise we move and the observe

    if _v_map[dest] == _v_map[src]:  # We do not have to pay the observation cost as we'll have arrived and paid it
        return 0                     # NB: If you change this value, then the "block" treatment has vastly distinct semantics!

    return _f_table[(_v_map[src],_v_map[dest])] + obcost



def rev_query_v_dict(v_dict, n):
    for l in v_dict:
        if v_dict[l] == n:
            return l
    return ""

def reorder_blocks_for_size(bdd, block_dict):
    block_size = sum([1 for x in block_dict.values() if x == 1]) # Hack: Now optimize each block of size, for this part we do assume the blocks are the same size:
    for r in range(0, len(bdd.vars), block_size):
        BDD.reorder(bdd, window=(r, r+block_size-1))


def get_bdd_from_srql(input_file=None):
    global locs, props, verbose

    parser = argparse.ArgumentParser()

    # Use provided input_file or default
    if input_file is None:
        # Try to find the file relative to this script's location
        script_dir = os.path.dirname(os.path.abspath(__file__))
        input_file = os.path.join(script_dir, "examples_2/tate-ex1.srql")

    dumpformat = ["pdf", "dot"]

    pz = sparser.parse_world(open(input_file,'r').read())

    if not pz:
        print("Unable to parse the input file '%s'" % input_file)
        sys.exit(1)

    locs = [loc['name'] for loc in pz['locations']]
    # Also create a location dictionary with coordinates
    locs_with_coords = {loc['name']: {'x': loc['x'], 'y': loc['y']} for loc in pz['locations']}
    props = pz['properties']
    qry = pz['query']


    if not qry: 
        print("Please provide some query!")
        sys.exit(1)




    #print("Generating variables:")
    block_dict = {}
    block_counter = 0
    internal_var_prefix = 'vvv'
    so_far = 0
    var_dict = {}
    elegant_var_dict = {}
    var_to_loc_dict = {}
    for l in pz['locations']:
        block_counter = block_counter + 1 # Zero cost to sense two things in the same location, so form blocks at locations
        for p in pz['properties']:
            v_name = ("%s%d" % (internal_var_prefix, so_far))
            var_dict[p+"("+l['name']+")"] = v_name
            elegant_var_dict["Visit: "+l['name']+"\n Observe: "+p] = v_name
            var_to_loc_dict[v_name] = l['name']
            block_dict[v_name] = block_counter
            so_far = so_far + 1

    #print(repr(var_dict))

    calc_cost.var_to_loc_map = var_to_loc_dict

    var_to_loc_dict

    all_constrs = []

    const_dict = {}
    
    for f in pz['constraints']:
        if f[0]: # has quantifiers
            all_constrs = all_constrs + (formula_to_sat(var_dict,f))
        else:
            # add facts to constant dictionary
            for k in f[1]:
                const_dict[var_dict[k]] = f[1][k]

    query_sat = "("+ ("&".join(formula_to_sat(var_dict, qry))) + ")"

    world_expr = "("+ ("&".join(all_constrs)) + ")"
    # print(query_sat)
    # print(world_expr)

    bdd = BDD()
    vs = list(var_dict.values())
    bdd.declare(*vs)
    w = bdd.let(const_dict, bdd.add_expr(world_expr))

    #print(query_sat)
    q = bdd.let(const_dict, bdd.add_expr(query_sat))

    bdd.collect_garbage()  # optional

    product_root = bdd.form_product_graph(bdd, w, q, v_dict=var_dict)
    bdd.collect_garbage()  # optional
    
    if 'robots' in pz:
        robots = pz['robots']


    return bdd, product_root, elegant_var_dict, dumpformat, robots, locs_with_coords, q, w


