#!/usr/bin/python
import sys 
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


def main():
    global locs, props, verbose

    parser = argparse.ArgumentParser()

    parser.add_argument('input_file') # Positional .srql file
    parser.add_argument('--reps', type=int, default=0, required=False, help="Run repetitions to account for noise in process time.")
    parser.add_argument('--seed', type=int, default=0, required=False, help="Use this random seed to shuffle original variable order")
    parser.add_argument('--method',required=False, default='c', choices=['n', 's','c','[c]','[c][s]','[s][c]','[s][c][s]'], help="Method to use: {n: none, s:size-reorder, c:cost-reorder, [c]:cost-reorder-with-blocks, [c][s]:cost-then-size-reorder-with-blocks, [s][c]:size-then-cost-reorder-with-blocks, [s][c][s]:size-then-cost-then-size-reorder-with-blocks}")
    parser.add_argument('--verbose', dest='verbose', action='store_true', help="Provide extra information")
    parser.add_argument('--dump-pdfs', dest='pdfs', action='store_true', help="Provide PDFs of the BDDs")
    parser.add_argument('--dump-dots', dest='dots', action='store_true', help="Provide dot files for the BDDs")

    args = parser.parse_args()
    if (args.seed != 0) and (args.reps == 0):
        print(f'Warning: you specified a seed, but it is being ignored. That value is only used then experimental conditions are established, i.e., when --reps N, with 1 <= N is provided.')
    elif (args.seed != 0):
        random.seed(args.seed) 

    dumpformat = []
    if args.pdfs:
        dumpformat.append("pdf")

    if args.dots:
        dumpformat.append("dot")


    pz = sparser.parse_world(open(args.input_file,'r').read())
    if (args.reps == 0):
        verbose = args.verbose # When not in experimental instrumentation mode, just be verbose to std out, otherwise it is used to generate experimental output

    if pz:
        locs = pz['locations']
        props = pz['properties']
        qry = pz['query']

        if verbose:
            sep = "-"*40+"\n"
            limit = 15
            print("Total of %d locations:" % (len(pz['locations'])))
            if len(pz['locations']) > limit:
                print(pz['locations'][0:limit],"...")
            else:
                print(pz['locations'])
            print(sep)

            print("Total of %d transitions:" % (len(pz['transitions'])))
            for t in pz['transitions'][0:limit]:
                print("[%s]\t-----(% 6.2f)---->\t[%s]" %(t[0], t[2], t[1]))
            if len(pz['transitions']) > limit:
                print(".\n.\n.")
            print(sep)

            print("Starting location: %s\n" % (pz['startloc']))

            print("Total of %d observation costs:" % (len(pz['obscosts'])))
            for c in pz['obscosts'][0:limit]:
                print("[%s]  =% 6.2f" %(c[0], c[1]))
            if len(pz['obscosts']) > limit:
                print(".\n.\n.")
            print(sep)

            print("Total of %d properties:" % (len(pz['properties'])))
            if len(pz['properties']) > limit:
                print(pz['properties'][0:limit],"...")
            else:
                print(pz['properties'])

            print(sep)
            print("These constraints were obtained:")
            for f in pz['constraints'][0:limit]:
                if f[0]: # has quantifiers
                    #print("Q %s : %s" % (f[0]['quants'], f[1]))
                    if (f[0]['constraints']) : 
                        print("Q[%s|%s]: %s" % ( (r' '.join(f[0]['quants'])), (f[0]['constraints']), f[1]) )
                    else:
                        print("Q[%s]: %s" % ( (r' '.join(f[0]['quants'])), f[1]) )
                else:
                    print("FACT: %s" % f[1])
            print(sep)

            if qry: 
                print("The following query:")
                print(qry)
            else:
                print("No query provided")
            print(sep)

        if not qry: 
            print("Please provide some query!")
            sys.exit(1)

        floydtable = {}
        for s in locs:
            for d in locs:
                floydtable[(s,d)] = float('inf')
            floydtable[(s,s)] = 0

        for (s,d,c) in pz['transitions']:
            floydtable[(s,d)] = c

        for m in locs:
            for s in locs:
                for d in locs:
                    floydtable[(s,d)] = min(floydtable[(s,d)], floydtable[(s,m)]+floydtable[(m,d)])

        calc_cost.floydtable = floydtable
        calc_cost.obscosts = {} 
        for (l,c) in pz['obscosts']:
            calc_cost.obscosts[l] = c
        calc_cost.start_loc = pz['startloc']

        if verbose:
            print_floyd_table()


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
                var_dict[p+"("+l+")"] = v_name
                elegant_var_dict["Visit: "+l+"\n Observe: "+p] = v_name
                var_to_loc_dict[v_name] = l
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

        #print("Building BDD with %d variables" % (len(var_dict)))
        #print(world_expr)

        bdd = BDD()
        vs = list(var_dict.values())
        bdd.declare(*vs)
        w = bdd.let(const_dict, bdd.add_expr(world_expr))

        #print(query_sat)
        q = bdd.let(const_dict, bdd.add_expr(query_sat))

        bdd.collect_garbage()  # optional

        #if verbose:
        #    models = list(bdd.pick_iter(w,care_vars=var_dict.values()))
        #    if not models:
        #        print("Zero models")
        #    else:
        #        kz = list(models[0].keys())
        #        kz.sort()
        #        mlist = []
        #        for m in models:
        #            s = ""
        #            for k in kz:
        #                #s = s + ("%s:%s\t" % (k, m[k]))
        #                if m[k]:
        #                    s = s + (" %s\t" % (rev_query_v_dict(var_dict, k)))
        #                else:
        #                    s = s + ("!%s\t" % (rev_query_v_dict(var_dict, k)))
        #            s = s + "\n"
        #            mlist.append(s)
        #        mlist.sort()
        #        c = 0
        #        for m in mlist:
        #            c = c + 1
        #            print("%d) %s" % (c,m), end='')
        #        print("")

        if args.reps == 0:

            
            if dumpformat:
                #print("Writing the before PDFs")
                bdd.dump('before',roots=[q, w], method="new", v_dict=var_dict, root_names_map={int(w):"World", int(q):"Query"}, simplify_names=False, show_pass_thru_nodes=False, pass_thru_strict=True, condition_upon = {int(w):True}, fileformat=dumpformat)    
                #print("Finished the before PDFs")
            # No instrumentation for experimental is used, just run the requested method.
            if args.method == 'n':
                pass
            elif args.method == 's':
                BDD.reorder(bdd)
            elif args.method == 'c':
                BDD.custom_reorder(bdd, w, q, costfxn = calc_cost) 
            elif args.method == '[c]' or args.method == '[c][s]' or args.method == '[s][c]' or args.method == '[s][c][s]':
                if args.method[1] == 's':
                    reorder_blocks_for_size(bdd, block_dict)
                BDD.custom_reorder(bdd, w, q, costfxn = calc_cost, blocksdefn=block_dict) 
                if args.method[-2] == 's':
                    reorder_blocks_for_size(bdd, block_dict)
            else: 
                assert False, "I'm confused: how did you get here?"

            if dumpformat:
                #print("Writing the after PDFs")
                bdd.dump('after',roots=[q, w], method="new", v_dict=var_dict, root_names_map={int(w):"World", int(q):"Query"}, simplify_names=False, show_pass_thru_nodes=False, pass_thru_strict=True, condition_upon = {int(w):True}, fileformat=dumpformat)    
                product_root = bdd.form_product_graph(bdd, w, q, v_dict=var_dict)
                bdd.collect_garbage()  # optional
                bdd.dump('plan', roots=[product_root], method="new", v_dict=elegant_var_dict, root_names_map={int(product_root):"Plan"}, leaf_names_map={True:"Yes ", False:" No "}, simplify_names=True, clean_print=True, fileformat=dumpformat)
                #print("Finished the after PDFs")
                # print(bdd)
                print("Solved: construction saved in 'before', then after optimization appears in 'after'; the final bdd for the query is in 'plan'") 
            else:
                print("Solved")
        else:
            # We are asked to instrument and run an experiment

            # Force into a consistent order so we have a well-defined initial ordering
            the_keys = [(len(k), k) for k in list(bdd.vars.keys())] # the length bit here because otherwise it will do it lexicographically, not numerically
            the_keys.sort()

            block_size = sum([1 for x in block_dict.values() if x == 1]) # Hack: Assuming the blocks are the same size here
            # Collect into blocks
            blocked_keys = []
            for k in range(0, len(the_keys), block_size):
                blocked_keys.append(the_keys[k:(k+block_size)])
            if (args.seed != 0): # When 0 we don't shuffle the blocks
                random.shuffle(blocked_keys)

            the_keys = []
            for l in blocked_keys:
                for v in l:
                    the_keys.append(v)

            num_order = {}
            c = 0 
            for k in the_keys:
                num_order[k[1]] = c
                c = c + 1

            std_times = []
            size_before = []
            size_after = []
            cost_before = []
            cost_after = []
            for i in range(args.reps):
                BDD.reorder(bdd, num_order) # Setup identical initial conditions
                bdd.collect_garbage() 
                size_before.append(len(bdd))

                if dumpformat:
                    #print("Writing the before PDFs")
                    print("\tsize = %d" % ((len(bdd))))
                    #bdd.dump('world-classic.pdf',roots=[q, w], method="classic")
                    bdd.dump('before',roots=[q, w], method="new", v_dict=var_dict, root_names_map={int(w):"World", int(q):"Query"}, simplify_names=False, show_pass_thru_nodes=False, pass_thru_strict=True, condition_upon = {int(w):True}, fileformat=dumpformat)    
                    product_root = bdd.form_product_graph(bdd, w, q, v_dict=var_dict)
                    bdd.collect_garbage()  # optional
                    bdd.dump('before-plan', roots=[product_root], method="new", v_dict=var_dict, root_names_map={int(product_root):"Plan"}, leaf_names_map={True:"Yes ", False:" No "}, simplify_names=True, fileformat=dumpformat)
                    #print("Finished the before PDFs")


                #bdd.dump('world-classic.pdf',roots=[q, w], method="classic")
                #bdd.dump('world-pre-reordered.pdf',roots=[q, w], method="new", v_dict=var_dict, root_names_map={int(w):"World", int(q):"Query"}, simplify_names=False, show_pass_thru_nodes=False, pass_thru_strict=False, condition_upon = {int(w):True})    
                #p0_bdd = BDD()
                #p0_bdd.declare(*sorted(bdd._bdd.vars, key=bdd._bdd.vars.get))
                #product_root = bdd.form_product_graph(p0_bdd, w, q, v_dict=var_dict)
                #p0_bdd.dump('product-pre-reordered.pdf',method="new",roots=[product_root],v_dict=var_dict, simplify_names = True)


                cost_before.append(compute_query_cost(bdd, w, q, calc_cost))

                #print("starting")
                std_start_time = process_time()
                if args.method == 'n':
                    pass
                elif args.method == 's':
                    BDD.reorder(bdd)
                elif args.method == 'c':
                    BDD.custom_reorder(bdd, w, q, costfxn = calc_cost) 
                elif args.method == '[c]' or args.method == '[c][s]' or args.method == '[s][c]' or args.method == '[s][c][s]':
                    if args.method[1] == 's':
                        reorder_blocks_for_size(bdd, block_dict)
                    BDD.custom_reorder(bdd, w, q, costfxn = calc_cost, blocksdefn=block_dict) 
                    if args.method[-2] == 's':
                        reorder_blocks_for_size(bdd, block_dict)
                else: 
                    assert False, "I'm utterly confused: how did you get here?"
                std_finish_time = process_time()

                if dumpformat:
                    #print("Writing the after PDFs")
                    bdd.dump('after',roots=[q, w], method="new", v_dict=var_dict, root_names_map={int(w):"World", int(q):"Query"}, simplify_names=False, show_pass_thru_nodes=False, pass_thru_strict=True, condition_upon = {int(w):True}, fileformat=dumpformat)    
                    product_root = bdd.form_product_graph(bdd, w, q, v_dict=var_dict)
                    bdd.collect_garbage()  # optional
                    bdd.dump('after-plan', roots=[product_root], method="new", v_dict=var_dict, root_names_map={int(product_root):"Plan"}, leaf_names_map={True:"Yes ", False:" No "}, simplify_names=True, fileformat=dumpformat)
                    #print("Finished the after PDFs")

                size_after.append(len(bdd))
                cost_after.append(compute_query_cost(bdd, w, q, calc_cost))
                std_times.append(std_finish_time - std_start_time)

                # DAVID : Printing BDD - consisting of edges and nodes
                

            assert allidentical(size_before)
            assert allidentical(size_after)
            assert allidentical(cost_before)
            assert allidentical(cost_after)
            result = {}
            result['times'] = std_times
            result['before_size'] = size_before[0]
            result['before_cost'] = cost_before[0]
            result['after_size'] = size_after[0]
            result['after_cost'] = cost_after[0]
            result['seed'] = args.seed 
            if args.verbose:
                result['init_order'] = num_order

            print(result)

        if args.verbose:
            print("VERBOSE OUTPUT")
            print("LOCATIONS", locs)
            print("PROPERTIES", props)
            print("QUERY", qry)

    else:
        if not pz:
            print("Unable to parse world file")
        if not q:
            print("Unable to parse query:\n%s" % query)


if __name__ == "__main__":
    main()
