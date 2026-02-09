"""Wraps `dd.bdd` to automate reference counting.

For function docstrings, refer to `dd.bdd`.
"""
# Copyright 2015 by California Institute of Technology
# All rights reserved. Licensed under BSD-3.
#
import logging
import warnings

from customdd import _abc
from customdd import _copy
from customdd import bdd as _bdd
# inline:
# import pydot


log = logging.getLogger(__name__)


class BDD(_abc.BDD):
    """Shared ordered binary decision diagram.

    It takes and returns `Function` instances,
    which automate reference counting.

    Attributes:

      - `vars`: `dict` mapping `variables` to `int` levels
          Do not assign the `dict` itself.

    For docstrings, refer to methods of `dd.bdd.BDD`,
    with the difference that `Function`s replace nodes
    as arguments and returned types.
    """
    # omitted docstrings are inheritted from `super()`

    def __init__(self, levels=None):
        manager = _bdd.BDD(levels)
        self._bdd = manager
        self.vars = manager.vars

    def __eq__(self, other):
        return (self._bdd is other._bdd)

    def __len__(self):
        return len(self._bdd)

    def __contains__(self, u):
        if self is not u.bdd:
            raise ValueError('`self is not u.bdd`')
        return u.node in self._bdd

    def __str__(self):
        return (
            'Binary decision diagram (`dd.bdd.BDD` wrapper):\n'
            '------------------------\n'
            '\t {n_vars} BDD variables\n'
            '\t {n} nodes\n').format(
                n_vars=len(self.vars), n=len(self))

    def _wrap(self, u):
        """Return `Function` for node `u`.

        @param u: node in `self._bdd`
        @type u: `int`
        """
        assert u in self._bdd
        return Function(u, self)

    def configure(self, **kw):
        return self._bdd.configure(**kw)

    def succ(self, u):
        i, v, w = self._bdd.succ(u.node)
        v = self._wrap(v)
        w = self._wrap(w)
        return i, v, w

    def incref(self, u):
        self._bdd.incref(u.node)

    def decref(self, u, **kw):
        self._bdd.decref(u.node)

    def add_var(self, var, level=None):
        return self._bdd.add_var(var, level=level)

    def var(self, var):
        r = self._bdd.var(var)
        return self._wrap(r)

    def var_at_level(self, level):
        return self._bdd.var_at_level(level)

    def level_of_var(self, var):
        return self._bdd.level_of_var(var)

    @property
    def var_levels(self):
        return self._bdd.var_levels

    def reorder(self, var_order=None, window=None):
        reorder(self, var_order, window)

    def custom_reorder(bdd, world, query, costfxn=None, blocksdefn=None):
        _bdd.custom_reorder(bdd, world, query, costfxn, blocksdefn)

    def compute_max_cost(bdd, product_root, **kwargs):
        return _bdd.compute_max_cost(bdd, product_root, **kwargs)

    def copy(self, u, other):
        assert u in self, u
        if self is other:
            log.warning('copying node to same manager')
            return u
        r = self._bdd.copy(u.node, other._bdd)
        return other._wrap(r)

    def support(self, u, as_levels=False):
        assert u in self, u
        return self._bdd.support(u.node, as_levels)

    def let(self, definitions, u):
        assert u in self, u
        if not definitions:
            return u
        d = definitions
        var = next(iter(d))
        value = d[var]
        if isinstance(value, Function):
            d = {
                var: value.node
                for var, value in d.items()}
        r = self._bdd.let(d, u.node)
        return self._wrap(r)

    def quantify(self, u, qvars, forall=False):
        assert u in self, u
        r = self._bdd.quantify(u.node, qvars, forall)
        return self._wrap(r)

    def forall(self, qvars, u):
        return self.quantify(u, qvars, forall=True)

    def exist(self, qvars, u):
        return self.quantify(u, qvars, forall=False)

    def ite(self, g, u, v):
        assert g in self, g
        assert u in self, u
        assert v in self, v
        r = self._bdd.ite(g.node, u.node, v.node)
        return self._wrap(r)

    def find_or_add(self, var, low, high):
        """Return node `IF var THEN high ELSE low`."""
        level = self.level_of_var(var)
        r = self._bdd.find_or_add(level, low.node, high.node)
        return self._wrap(r)

    def count(self, u, nvars=None):
        assert u in self, u
        return self._bdd.count(u.node, nvars)

    def pick_iter(self, u, care_vars=None):
        assert u in self, u
        return self._bdd.pick_iter(u.node, care_vars)

    def add_expr(self, e):
        r = self._bdd.add_expr(e)
        return self._wrap(r)

    def to_expr(self, u):
        assert u in self, u
        return self._bdd.to_expr(u.node)

    def apply(self, op, u, v=None, w=None):
        assert u in self, u
        if v is None:
            assert w is None, w
            r = self._bdd.apply(op, u.node)
        elif w is None:
            assert v in self, v
            assert w is None, w
            r = self._bdd.apply(op, u.node, v.node)
        else:
            assert v in self, v
            assert w in self, w
            r = self._bdd.apply(op, u.node, v.node, w.node)
        return self._wrap(r)

    def _add_int(self, i):
        r = self._bdd._add_int(i)
        return self._wrap(r)

    def cube(self, dvars):
        r = self._bdd.cube(dvars)
        return self._wrap(r)

    def collect_garbage(self):
        """Recursively remove nodes with zero reference count."""
        self._bdd.collect_garbage()

    def dump(self, filename, roots=None,
             filetype=None, **kw):
        """Write BDDs to `filename`.

        The file type is inferred from the
        extension (case insensitive),
        unless a `filetype` is explicitly given.

        `filetype` can have the values:

          - `'p'` for Pickle
          - `'pdf'` for PDF
          - `'png'` for PNG
          - `'svg'` for SVG
          - `'json'` for JSON

        If `filetype is None`, then `filename`
        must have an extension that matches
        one of the file types listed above.

        Dump nodes reachable from `roots`.
        If `roots is None`,
        then all nodes in the manager are dumped.

        Dumping a JSON file requires that `roots`
        be nonempty.

        @type filename: `str`
        @type filetype: `str`, e.g., `"pdf"`
        @type roots: container of nodes
        """
        # The method's docstring is a slight modification
        # of the docstring of the method `dd._abc.BDD.dump`.
        if filetype is None:
            name = filename.lower()
            if name.endswith('.pdf'):
                filetype = 'pdf'
            elif name.endswith('.png'):
                filetype = 'png'
            elif name.endswith('.svg'):
                filetype = 'svg'
            elif name.endswith('.p'):
                filetype = 'pickle'
            elif name.endswith('.json'):
                filetype = 'json'
            elif kw.get("fileformat", False):
                filetype = '<list>' # This is going to be skipped and the fileformat list used
            else:
                raise Exception((
                    'cannot infer file type '
                    'from extension of file '
                    'name "{f}"').format(
                        f=filename))
        if filetype == 'json':
            _copy.dump_json(roots, filename)
        else:
            if roots is not None:
                roots = [u.node for u in roots]
            self._bdd.dump(filename, roots=roots,
                           filetype=filetype,**kw)

    def load(self, filename, levels=True):
        """Load nodes from Pickle or JSON file `filename`.

        If `levels is True`,
        then load variables at the same levels.
        Otherwise, add missing variables.

        @type filename: `str`
        @return: roots of the loaded BDDs
        @rtype: depends on the contents of the file,
            either:
            - `dict` that maps names (as `str`)
              to nodes, or
            - `list` of nodes
        """
        # This method's docstring is a slight
        # modification of the docstring of
        # the method `dd._abc.BDD.dump`.
        name = filename.lower()
        if name.endswith('.p'):
            return self._load_pickle(
                filename, levels=levels)
        elif name.endswith('.json'):
            return _copy.load_json(filename, self)
        else:
            raise ValueError(
                'Unknown file type of "{f}"'.format(
                    f=filename))

    def _load_pickle(self, filename, levels=True):
        umap = self._bdd.load(filename, levels=levels)
        umap = {u: self._wrap(umap[u]) for u in umap}
        return umap

    def assert_consistent(self):
        self._bdd.assert_consistent()

    @property
    def false(self):
        u = self._bdd.false
        return self._wrap(u)

    @property
    def true(self):
        u = self._bdd.true
        return self._wrap(u)



    def who_steps(self,cnode): # returns ((True / False if world steps), (True / False if query steps), level)
        ((w, w_par), (q, q_par)) = cnode
        step_w = step_q = True
        if (abs(int(w)) == 1): 
            step_w = False
            wi = float('inf')
        else:
            wi, _, _ = self.succ(w)
        if (abs(int(q)) == 1): 
            step_q = False
            qi = float('inf')
        else:
            qi, _, _ = self.succ(q)
        assert (step_w or step_q)
        if (step_w and step_q):
            if wi > qi: step_w = False
            if wi < qi: step_q = False
        return (step_w, step_q, min(wi,qi))

    def safe_succ(self, n):
        if (abs(int(n)) != 1):
            return self.succ(n)
        else:
            return (float('inf'), n, n)

    def progress_along_right(self, cnode):
        ((wprior, w_par), (qprior, q_par)) = cnode
        assert (w_par != 0) and (q_par != 0), 'This function requires a real pair of vertices, not a virtual root.'
        wi, _, ww = self.safe_succ(wprior)
        qi, _, qw = self.safe_succ(qprior)
        wn = wprior if wi > qi else ww 
        qn = qprior if qi > wi else qw 
        return ((wn, w_par), (qn, q_par))


    def progress_along_left(self, cnode):
        ((wprior, w_par), (qprior, q_par)) = cnode
        assert (w_par != 0) and (q_par != 0), 'This function requires a real pair of vertices, not a virtual root.'
        wi, wv, _ = self.safe_succ(wprior)
        qi, qv, _ = self.safe_succ(qprior)
        wn = wprior if wi > qi else wv 
        qn = qprior if qi > wi else qv 
        who_steps_now = self.who_steps(cnode)
        if (int(wn) < 0) and (who_steps_now[0]): w_par = -w_par
        if (int(qn) < 0) and (who_steps_now[1]): q_par = -q_par
        return ((wn, w_par), (qn, q_par))


    def dump_product_graph(self, fname, world, query, **kw):
        import pydot

        def construct_dot_canvas():

            g = pydot.Dot('bdd', graph_type='digraph')
            #
            # This block (and function) builds the set of levels that appear in the BDD.
            def walklevels(n, set_of_levels):
                if (abs(int(n)) == 1):
                    return
                i, v, w = self.succ(n)
                set_of_levels.add(i)
                walklevels(v, set_of_levels)
                walklevels(w, set_of_levels)
            #
            set_of_levels = set()
            walklevels(world, set_of_levels)
            walklevels(query, set_of_levels)
            set_of_levels.add(1+(max(set_of_levels))) # We add a level for the leaves. 
            # We use this to construct the layering in the pydot setup
            #
            # Make layers 
            skeleton = list()
            subgraphs = dict()
            # layer for external BDD references
            layers = [-1] + sorted(set_of_levels)
            # add nodes for BDD levels
            for i in layers:
                h = pydot.Subgraph('', rank='same')
                g.add_subgraph(h)
                subgraphs[i] = h
                # add phantom node
                u = 'L{i}'.format(i=i)
                skeleton.append(u)
                if i == -1:
                    # layer for external BDD references
                    label = 'ref'
                else:
                    # BDD level
                    label = str(i)
                nd = pydot.Node(name=u, label=label, shape='none')
                h.add_node(nd)
            # auxiliary edges for ranking
            for i, u in enumerate(skeleton[:-1]):
                v = skeleton[i + 1]
                e = pydot.Edge(str(u), str(v), style='invis')
                g.add_edge(e)
            # make leaves:
            nd = pydot.Node(name="1", label="  True   ", shape="box", color="darkseagreen4", fillcolor="lightgreen", style="filled")
            subgraphs[max(layers)].add_node(nd)
            nd = pydot.Node(name="-1", label="[False]", shape="box", color="firebrick4", fillcolor="lightcoral", style="filled")
            subgraphs[max(layers)].add_node(nd)
            return {'g':g, 'subgraphs':subgraphs}

        simplabels = kw.get('simplify_names',{})

        v_dict = kw.get('v_dict',{})
        def rev_query_v_dict(n):
            for l in v_dict:
                if v_dict[l] == n:
                    return l
            return ""


        idx2vars = {}
        for i in self.vars.keys():
            idx2vars[self.vars[i]] = i
        leaf_level = max(idx2vars.keys())
        
        def idx2var(ix):
            if (ix == -1): 
                return "<leaf>"
            return idx2vars[abs(ix)]

 
        def dot_name_pair(thepr, fst_sfx, snd_sfx): # turn a pair into a unique string that encodes the pair of vertices, their parity, and a suffix.
                                                    # parity keeps the sign values as the are traced; the suffix tells you if you are tracing a "pass_thru" on an edge 
                                                    # the string is chosen to use characters the won't confuse dot.
            def dot_name_single(pr,sfx): 
                s = '' if sfx == '' else "*"+sfx
                try:
                    num = abs(int(pr[0]))
                    if (pr[1]) < 0: return '['+str(num)+s+']'
                    else: return str(num)+s
                except ValueError:
                    return str(pr[0])+s # Ignore the second element
            return dot_name_single(thepr[0], fst_sfx)+"/"+dot_name_single(thepr[1], snd_sfx)


        def dotwalkpair(prenm, p_node, n_node, edge_type, dotdict, depth=0, existing_nodes = {}):
            # The idea is that p_node appears in the DAG, and we are seeking another to add, n_node is a candidate to add, but we need to determine whether it needs to be evaluated or not.
            # For instance, n_node's output may be inferrable from the world, and so we push forward to the next node. When we do find a node that must appear, an edge of type "edge_type" will
            # be added from p_node.

            #print(" "*depth, end='')
            #print("dotwalkpair(<(%s, %d)|(%s, %d)>,\t <(%s, %d)|(%s, %d)>,\t%s, %d)" % (str((p_node[0][0])), p_node[0][1], str((p_node[1][0])), p_node[1][1], str((n_node[0][0])), n_node[0][1], str((n_node[1][0])), n_node[1][1], edge_type, depth))
            edge_style_label = {"both":"dashed", "left":"dashed", "right":"solid"}

            # The task is to determine whether n_node should be processed (evaluated/dot node generated) or whether to 
            # pass onto the next level (recursively). We pass because this is a node whose value can be inferred.

            #if (depth > 2): return # useful for debugging, we cut off at a fixed level

            # If we have arrived at a leaf, we know that this branch evaluates to.
            ((w, w_par), (q, q_par)) = n_node
            if (abs(int(q)) == 1): # The query is at an end, we should connect to leaf, that is all
                #if ((q_par) == 1):
                #    print("returning 'True' (along %s edge)" % edge_type)
                #else:
                #    print("returning 'False' (along %s edge)" % edge_type)

                return ((q_par) == 1)

            inferred_any = None # We use this variable as a ternary: None, True, or False

            if (self.who_steps(n_node)[0]): # Check if moving down the next layer happens in the world  (it can be both query AND world, or query XOR world)
                wi, wv, ww = self.succ(w) # To decide if w can be skipped over, we have to peek ahead.
                if (abs(int(wv)) == 1): # The world vertex reaches the leaf on the left, and we condition on the world, so we can infer what the value must be
                    if (int(wv) != w_par): # Note that determine whether it is a free positive or a free negative value, we need to know the parity
                        inferred_any = True
                if (abs(int(ww)) == 1): # The world vertex reaches the leaf on the right, ... etc.
                    if (int(ww) != w_par): # Ditto
                        inferred_any = False

            if (inferred_any == True) or (inferred_any == False): # Skip this node

                if inferred_any:
                    #print("pushing right")
                    nxt = self.progress_along_right(n_node)
                else:
                    #print("pushing left")
                    nxt = self.progress_along_left(n_node)
                
                return dotwalkpair(prenm, p_node, nxt, edge_type, dotdict, depth+1, existing_nodes)

            elif inferred_any == None:

                # We know that between p_node and n_node there is some query that must happen.
                # First, determine what form that query takes:
                w_steps, q_steps, nd_level = self.who_steps(n_node)
                nxt_w = (n_node[0][0], n_node[0][1]) if w_steps else (p_node[0][0], p_node[0][1]) 
                nxt_q = (n_node[1][0], n_node[1][1]) if q_steps else (p_node[1][0], p_node[1][1]) 
                w_suffix = '' if w_steps else edge_type[0]
                q_suffix = '' if q_steps else edge_type[0]
                nxt = (nxt_w, nxt_q)
                nxt = (nxt_w, nxt_q)
                nm = dot_name_pair(nxt, w_suffix, q_suffix)

                #print(" "*depth, end='')
                #print("\t The plan is to add dot node called '%s' " % (nm))

 

                connect_left = None
                connect_right = None
                already_exists = False

                if not(nm in existing_nodes):
                    #print(" "*depth, end='')
                    #print("Just before %s or %s, need to progress both ways" % (n_node, nxt))
                    r = dotwalkpair(nm, nxt, self.progress_along_right(n_node), 'right', dotdict, depth+1, existing_nodes)
                    l = dotwalkpair(nm, nxt, self.progress_along_left(n_node), 'left', dotdict, depth+1, existing_nodes)

                    #print(" "*depth, end='')
                    #print("got l = %s and r = %s" % (l, r))

                    if (l == r) and ((l == False) or (l == True)):
                        existing_nodes[nm] = l
                        # We can supress this one, as it leads to the same value ineviablely
                    else:
                        if l != "Both":
                            connect_left = l
                        if r != "Both":
                            connect_right = r

                        existing_nodes[nm] = "Both"
                else:
                    already_exists = True

                if existing_nodes[nm] != "Both": 
                    return existing_nodes[nm]

                if not already_exists:
                    #print("\t Adding dot node called '%s'  (which as value=%s)" % (nm, existing_nodes[nm]))
                    wname = nm.split('/')[0]
                    qname = nm.split('/')[1]
                    ndname = wname + "/" + qname
                    #print("nm = %s, ndname = %s \t\tnd_level=%d, and varname=%s" % (nm, ndname, nd_level, idx2var(nd_level)))
                    prefix = "" if simplabels else idx2var(nd_level)+"   "+ ndname+"\n"
                    nd = pydot.Node(name=nm, label=prefix+rev_query_v_dict(idx2var(nd_level)), shape='oval', color='black', fillcolor="white", style="filled")
                    dotdict['subgraphs'][nd_level].add_node(nd)

                    if (connect_left != None):
                        dotdict['g'].add_edge(pydot.Edge(nm, "1" if connect_left else "-1", style=edge_style_label["left"]))

                    if (connect_right!= None):
                        dotdict['g'].add_edge(pydot.Edge(nm, "1" if connect_right else "-1", style=edge_style_label["right"]))

                if edge_type == "both":
                    dotdict['g'].add_edge(pydot.Edge(prenm, nm, style=edge_style_label["right"]))
                dotdict['g'].add_edge(pydot.Edge(prenm, nm, style=edge_style_label[edge_type]))

                return "Both"

            else:
                assert False, "Confused about how you arrived here."


            qi, qv, qw = self.succ(q) 

        def dotwalkpairinit(world, query, pydotdict):
            root_w_name = "rt_w"
            root_q_name = "rt_q"
            root = ((root_w_name, 0),(root_q_name, 0)) # We use parity = 0 to signify the extra vertex before the start of the bdd
            dot_name_root = dot_name_pair(root, '', '')
            dot_nd = pydot.Node(name=dot_name_root , label=dot_name_root, shape='none')
            pydotdict['subgraphs'][-1].add_node(dot_nd)
            dotwalkpair(dot_name_root, root, ((world, sgn(world)), (query, sgn(query))), "both", pydotdict)

 
        pydotdict = construct_dot_canvas()

        dotwalkpairinit(world, query, pydotdict)

        #pydotdict['g'].write_raw("product.dot")
        pydotdict['g'].write_pdf(fname)

 
    def form_product_graph(self, bdd, world, query, **kw):

        def str_name_pair(thepr, fst_sfx, snd_sfx): 
            def _name_single(pr,sfx): 
                s = '' if sfx == '' else "-"+sfx
                try:
                    num = abs(int(pr[0]))
                    if (pr[1]) < 0: return '['+str(num)+s+']'
                    else: return str(num)+s
                except ValueError:
                    return str(pr[0])+s # Ignore the second element
            return _name_single(thepr[0], fst_sfx)+"/"+_name_single(thepr[1], snd_sfx)

        def walkpair(prenm, p_node, n_node, edge_type, depth, index2bdd_var, nodes_already_visited):
            ((w, w_par), (q, q_par)) = n_node
            if (abs(int(q)) == 1): # The query is at an end, we should connect to leaf, that is all
                if (q_par == 1):
                    return bdd.true
                else:
                    return bdd.false

            inferred_any = None # We use this variable as a ternary: None, True, or False

            if (self.who_steps(n_node)[0]): # Check if moving down the next layer happens in the world  (it can be both query AND world, or query XOR world)
                wi, wv, ww = self.succ(w) # To decide if w can be skipped over, we have to peek ahead.
                if (abs(int(wv)) == 1): # The world vertex reaches the leaf on the left, and we condition on the world, so we can infer what the value must be
                    if (int(wv) != w_par): # Note that determine whether it is a free positive or a free negative value, we need to know the parity
                        inferred_any = True
                if (abs(int(ww)) == 1): # The world vertex reaches the leaf on the right, ... etc.
                    if (int(ww) != w_par): # Ditto
                        inferred_any = False

            if (inferred_any == True) or (inferred_any == False): # Skip this node
                if inferred_any:
                    nxt = self.progress_along_right(n_node)
                else:
                    nxt = self.progress_along_left(n_node)
                
                return walkpair(prenm, p_node, nxt, edge_type, depth+1, index2bdd_var,  nodes_already_visited)
            elif inferred_any == None:
                w_steps, q_steps, nd_level = self.who_steps(n_node)
                nxt_w = (n_node[0][0], n_node[0][1]) if w_steps else (p_node[0][0], p_node[0][1]) 
                nxt_q = (n_node[1][0], n_node[1][1]) if q_steps else (p_node[1][0], p_node[1][1]) 
                w_suffix = '' if w_steps else edge_type[0]
                q_suffix = '' if q_steps else edge_type[0]
                nxt = (nxt_w, nxt_q)
                nm = str_name_pair(nxt, w_suffix, q_suffix)

                already_exists = False
                if not(nm in nodes_already_visited):
                    r = walkpair(nm, nxt, self.progress_along_right(n_node), 'right', depth+1, index2bdd_var, nodes_already_visited)
                    l = walkpair(nm, nxt, self.progress_along_left(n_node), 'left', depth+1, index2bdd_var, nodes_already_visited)
                    #print("We are adding %s" % (index2bdd_var[abs(nd_level)]))
                    nodes_already_visited[nm] = bdd.ite(index2bdd_var[abs(nd_level)], r, l)
                return nodes_already_visited[nm]


        idx2bdd_vars = {}
        for i in self.vars.keys():
            idx2bdd_vars[self.vars[i]] = bdd.var(i)

        root_w_name = "rt_w"
        root_q_name = "rt_q"
        root = ((root_w_name, 0),(root_q_name, 0)) # Use parity = 0 to signify the extra vertex before the start of the bdd
        return walkpair(str_name_pair(root, '', ''), root, ((world, sgn(world)), (query, sgn(query))), "both", 0, idx2bdd_vars, {})



def image(trans, source, rename, qvars, forall=False):
    assert trans.bdd is source.bdd
    u = _bdd.image(trans.node, source.node, rename,
                   qvars, trans.manager, forall)
    return trans.bdd._wrap(u)


def preimage(trans, target, rename, qvars, forall=False):
    assert trans.bdd is target.bdd
    u = _bdd.preimage(trans.node, target.node, rename,
                      qvars, trans.manager, forall)
    return trans.bdd._wrap(u)


def reorder(bdd, order=None, window=None):
    """Apply Rudell's sifting algorithm to `bdd`."""
    _bdd.reorder(bdd._bdd, order=order, window=window)


def copy_vars(source, target):
    _copy.copy_vars(source._bdd, target._bdd)


def copy_bdd(u, target):
    r = _bdd.copy_bdd(u.node, u.manager, target._bdd)
    return target._wrap(r)


class Function(_abc.Operator):
    r"""Convenience wrapper for edges returned by `BDD`.

    ```python
    import dd.autoref

    bdd = dd.autoref.BDD()
    bdd.declare('x', 'y')
    nd = bdd._bdd.add_expr(r'x /\ y')
        # `nd` is an integer
        # `bdd._bdd` is an instance of the
        # class `dd.bdd.BDD`
    u = _bdd.Function(nd, bdd)
    ```

    Attributes:

    - `node`: `int` that describes edge (signed node)
    - `bdd`: `dd.autoref.BDD` instance that node belongs to
    - `manager`: `dd.bdd.BDD` instance that node belongs to

    Operations are valid only between functions with
    the same `BDD` in `Function.bdd`.

    After all references to a `Function` have been deleted,
    the reference count of its associated node is decremented.
    To explicitly release a `Function` instance, invoke `del f`.

    The design here is inspired by the PyEDA package.
    """

    def __init__(self, node, bdd):
        assert node in bdd._bdd, node
        self.bdd = bdd
        self.manager = bdd._bdd
        self.node = node
        self.manager.incref(node)

    def __hash__(self):
        return self.node

    def to_expr(self):
        """Return Boolean expression of function as `str`."""
        return self.manager.to_expr(self.node)

    def __int__(self):
        return self.node

    def __len__(self):
        return len(self.manager.descendants([self.node]))

    @property
    def dag_size(self):
        return len(self)

    def __del__(self):
        """Decrement reference count of `self.node` in `self.bdd`."""
        self.manager.decref(self.node)

    def __eq__(self, other):
        if other is None:
            return False
        assert self.bdd is other.bdd, (self.bdd, other.bdd)
        return self.node == other.node

    def __ne__(self, other):
        if other is None:
            return True
        assert self.bdd is other.bdd, (self.bdd, other.bdd)
        return not (self == other)

    def __le__(self, other):
        return (other | ~ self) == self.bdd.true

    def __lt__(self, other):
        return self <= other and self != other

    def __invert__(self):
        return self._apply('not', other=None)

    def __and__(self, other):
        return self._apply('and', other)

    def __or__(self, other):
        return self._apply('or', other)

    def implies(self, other):
        return self._apply('implies', other)

    def equiv(self, other):
        return self._apply('equiv', other)

    def _apply(self, op, other):
        """Return result of operation `op` with `other`."""
        # unary op ?
        if other is None:
            u = self.manager.apply(op, self.node)
        else:
            assert self.bdd is other.bdd, (self.bdd, other.bdd)
            u = self.manager.apply(op, self.node, other.node)
        return Function(u, self.bdd)

    @property
    def level(self):
        i, _, _ = self.manager._succ[abs(self.node)]
        return i

    @property
    def var(self):
        i, low, _ = self.manager._succ[abs(self.node)]
        if low is None:
            return None
        return self.manager.var_at_level(i)

    @property
    def low(self):
        _, v, _ = self.manager._succ[abs(self.node)]
        if v is None:
            return None
        return Function(v, self.bdd)

    @property
    def high(self):
        _, _, w = self.manager._succ[abs(self.node)]
        if w is None:
            return None
        return Function(w, self.bdd)

    @property
    def ref(self):
        return self.manager._ref[abs(self.node)]

    @property
    def negated(self):
        return self.node < 0

    @property
    def support(self):
        return self.manager.support(self.node)

def sgn(x):
    return (-1 if (int(x) < 0) else (1 if (int(x) > 0) else 0))
