import re, sys
# -----------------------------------------------------------------------------


reserved = {
   'location'               : 'LOC',
   'at'                     : 'AT',
   'transition'             : 'TRX',
   'recognizable-property'  : 'PRP',
   'observation-cost'       : 'OCS',
   'robots'                 : 'ROB',
   'robot'                  : 'ROBOT',
   'starts-at'              : 'STARTS',
   r'Query'                 : 'QRY',
   }

tokens = ['ID', 'VAR', 'EVAR', 'NEWLINE', 
    'FLOAT', 'INT',
    'LBRAC','RBRAC',
    'LPAREN','RPAREN',
    'OPAQUEFORMULA','COMMA',  'COLON', 'BAR', 'NEQ', 'NOT', 'NEG',
    ]  + list(reserved.values()) 

# Tokens

t_LPAREN = r'\('
t_RPAREN = r'\)'
t_LBRAC = r'\['
t_RBRAC = r'\]'
t_COMMA = r'\,'
t_COLON = r'\:'
t_BAR = r'\|'
t_NEQ = r'\!\='
t_NOT = r'\!'
t_NEG = r'\~'
t_VAR   = r'\?[a-zA-Z_]'
t_EVAR   = r'E\?[a-zA-Z_]'

def t_ID(t):
   r'[a-zQ_][a-zA-Z_0-9\-\'\_]*' # usual choices, and -, _, and '  
                                 # FYI: The capital Q starting is for Query
   t.type = reserved.get(t.value,'ID')    # Check for reserved words
   return t

def t_OPAQUEFORMULA(t):
   r'\{[a-zA-Z_0-9\-\>\<\'\_\ \(\)\?\!\&\|\^\n]*\}' # 
   t.value = re.match(r'\{(.*)\}',re.sub(r'\s+','',t.value))[1]
   return t

def t_NEWLINE(t):
    r'\n+'
    t.lexer.lineno += t.value.count("\n")
    return t
    
# Read in a float. This rule has to be done before the int rule.
def t_FLOAT(t):
    r'-?\d+\.\d*(e-?\d+)?'
    t.value = float(t.value)
    return t

def t_INT(t):
    r'\d+'
    t.value = int(t.value)
    return t

def t_COMMENT(t):
    r'\#.*\n'
    t.lexer.lineno += t.value.count("\n")
    pass

# Ignored characters
t_ignore = " \t"

def t_error(t):
    print("Illegal character '%s'" % t.value[0])
    p_error(t)
    
# Build the lexer
import ply.lex as lex
lex.lex()


# Define a Node class in order to permit explicit construction of a parse tree
class Node:
    def left(self):
        if self.children:
            return self.children[0]
        
    def right(self):
        if self.children:
            return self.children[1]
            
    def __init__(self,type,children=None,leaf=None):
        self.type = type
        if children:
            self.children = children
        else:
            self.children = [ ]
        self.leaf = leaf
    
    def __str__(self):
        return "___str___ todo"
        
    def __repr__(self):
        return "___repr___ todo"

# Precedence rules for the arithmetic operators
precedence = (
    ('left','LOC','AT','TRX','OCS', 'PRP', 'ROB', 'ROBOT', 'STARTS'),
    #('right','NOT'),
    #('left','AND','OR'),
    )

def p_world_spec(p):
    '''spec : locations robots properties worldconstrs query
            | locations robots properties worldconstrs
            | locations transitions obscosts properties worldconstrs query
            | locations transitions obscosts properties worldconstrs'''
    if len(p) == 6:  # new format with robots
        p[0] = {'locations':p[1], 'robots':p[2], 'transitions':[], 'obscosts':[], 'properties':p[3], 'constraints':p[4], 'query':p[5]}
    elif len(p) == 5:  # new format with robots, no query
        p[0] = {'locations':p[1], 'robots':p[2], 'transitions':[], 'obscosts':[], 'properties':p[3], 'constraints':p[4], 'query':None}
    elif len(p) == 7:  # old format with transitions/obscosts
        p[0] = {'locations':p[1], 'robots':{'num_robots': 1, 'robot_starts': []}, 'transitions':p[2], 'obscosts':p[3], 'properties':p[4], 'constraints':p[5], 'query':p[6]}
    else:  # old format with transitions/obscosts, no query
        p[0] = {'locations':p[1], 'robots':{'num_robots': 1, 'robot_starts': []}, 'transitions':p[2], 'obscosts':p[3], 'properties':p[4], 'constraints':p[5], 'query':None}

def p_blanks(p): # This is a bit obnoxious, but we need the tokenizer to give us blank lines because we used it as a delimiter
    '''blanks : NEWLINE blanks
              | '''

def p_locations(p):
    '''locations : blanks loc locations
                 | blanks loc
                 | blanks'''
    if len(p) == 2:
        p[0] = []  # empty case
    elif len(p) == 3:
        p[0] = [p[2]] #singleton case
    else: # |p| = [. b loc []]
        p[0] = [p[2]] + p[3] # general list

def p_loc(p):
    '''loc : LOC ID AT INT COMMA INT NEWLINE
           | LOC ID NEWLINE'''
    if len(p) == 4:  # Old format: location name
        p[0] = {'name': p[2], 'x': None, 'y': None}
    else:  # New format: location name at x, y
        p[0] = {'name': p[2], 'x': p[4], 'y': p[6]}


def p_robots(p):
    '''robots : blanks robotcount robotstarts
              | blanks robotcount
              | blanks'''
    if len(p) == 2:  # No robots section
        p[0] = {'num_robots': 1, 'robot_starts': []}
    elif len(p) == 3:  # Only robot count
        p[0] = {'num_robots': p[2], 'robot_starts': []}
    else:  # Robot count and starts
        p[0] = {'num_robots': p[2], 'robot_starts': p[3]}

def p_robotcount(p):
    '''robotcount : ROB INT NEWLINE'''
    p[0] = p[2]

def p_robotstarts(p):
    '''robotstarts : blanks robotstart robotstarts
                   | blanks robotstart
                   | blanks'''
    if len(p) == 2:
        p[0] = []  # empty case
    elif len(p) == 3:
        p[0] = [p[2]]  # singleton case
    else:
        p[0] = [p[2]] + p[3]  # general list

def p_robotstart(p):
    '''robotstart : ROBOT ID STARTS ID NEWLINE'''
    p[0] = {'robot_id': p[2], 'start_location': p[4]}


def p_transitions(p):
    '''transitions  : blanks trx transitions
                    | blanks trx
                    | blanks'''
    if len(p) == 2:
        p[0] = []  # empty case
    elif len(p) == 3:
        p[0] = [p[2]] #singleton case
    else: # |p| = [. b loc []]
        p[0] = [p[2]] + p[3] # general list


def p_trx(p):
    '''trx : TRX ID ID num NEWLINE'''
    p[0] = (p[2], p[3], p[4])


def p_num(p):
    '''num : FLOAT
           | INT'''
    p[0] = p[1]

def p_obscosts(p):
    '''obscosts : blanks ocost obscosts
                | blanks ocost
                | blanks'''
    if len(p) == 2:
        p[0] = []  # empty case
    elif len(p) == 3:
        p[0] = [p[2]] #singleton case
    else: # |p| = [. b loc []]
        p[0] = [p[2]] + p[3] # general list


def p_ocost(p):
    '''ocost : OCS ID num NEWLINE'''
    p[0] = (p[2], p[3])


def p_properties(p):
    '''properties : blanks prop properties
                  | blanks prop
                  | blanks'''
    if len(p) == 2:
        p[0] = []  # empty case
    elif len(p) == 3:
        p[0] = [p[2]] #singleton case
    else: # |p| = [. b loc []]
        p[0] = [p[2]] + p[3] # general list

def p_prop(p):
    '''prop : PRP ID NEWLINE'''
    p[0] = p[2]

def p_worldconstrs(p): 
    '''worldconstrs : blanks formula worldconstrs 
                    | blanks formula
                    | blanks'''
    if len(p) == 2:
        p[0] = []  # empty case
    elif len(p) == 3:
        p[0] = [p[2]] #singleton case
    else: # |p| = [. b loc []]
        p[0] = [p[2]] + p[3] # general list


def p_formula(p):
    '''formula : passthruformula
               | flatfactformula'''
    p[0] = p[1]

def p_flatfactformula(p):
    '''flatfactformula : ID LPAREN ID RPAREN
                       | neg ID LPAREN ID RPAREN'''
    #print("|p| = %d" % (len(p)))
    #print("p[1] = %s" % (p[1]))
    #print("p[2] = %s" % (p[2]))
    if len(p) == 5:
        d = dict()
        d["".join(p[1:5])] = True
        p[0] = ([], d)
    else:
        d = dict()
        d["".join(p[2:6])] = False
        p[0] = ([], d)

def p_passthruformula(p):
    '''passthruformula : quantifier promelaformula
                       | promelaformula'''
    if len(p) == 2:
        p[0] = ({'quants':[], 'constraints':[]}, p[1])
    else:
        p[0] = (p[1], p[2])


def p_quantifier(p):
    '''quantifier : LBRAC RBRAC
                  | LBRAC qlist RBRAC
                  | LBRAC qlist BAR qdifflist RBRAC'''
    if len(p) == 3:
        p[0] = {'quants':[],'constraints':[]}
    elif len(p) == 4:
        p[0] = {'quants':p[2],'constraints':[]}
    else:
        p[0] = {'quants':p[2],'constraints':p[4]}

def p_qlist(p):
    '''qlist : VAR COMMA qlist
             | VAR
             | EVAR'''
    if len(p) == 2:
        p[0] = [p[1]]
    else:
        p[0] = [p[1]] + p[3]

def p_qdifflist(p):
    '''qdifflist : diff COMMA qdifflist
                 | diff '''
    if len(p) == 2:
        p[0] = [p[1]]
    else:
        p[0] = [p[1]] + p[3]

def p_diff(p):
    '''diff : VAR NEQ VAR'''
    p[0] = (p[1], p[3])


def p_promelaformula(p):
    '''promelaformula : OPAQUEFORMULA'''
    p[0] = p[1]

def p_neg(p):
    '''neg : NOT 
           | NEG'''
    p[0] = p[1]

def p_query(p):
    '''query : QRY COLON passthruformula blanks''' 
    p[0] = p[3]


def p_error(p):
    print("Line %d: Syntax error at '%s'" % (p.lexer.lineno, p.value[0:30]+"..."))
    sys.exit(1)

import ply.yacc as yacc
yacc.yacc()

def parse_world(s):
    #print("s = '",s,"'")
    #lex.input(s)
    #while True:
    #    tok = lex.token()
    #    if not tok:
    #        break      # No more input
    #    print(tok)
    ##return -1
    #print("   ")
    lex.lexer.lineno = 0
    return yacc.parse(s)


def parse_query(s):
    #print("s = '",s,"'")
    #lex.input(s)
    #while True:
    #    tok = lex.token()
    #    if not tok:
    #        break      # No more input
    #    print(tok)
    ##return -1
    #print("   ")
    lex.lexer.lineno = 0
    p = yacc.parse(s)
    if p['locations'] or p['transitions'] or p['properties']:
        print("It looks like you're trying to parse world as a query?")

    return p['constraints']

