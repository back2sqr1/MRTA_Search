"""
BDD Variable Name Conversion Utilities

This module provides functions to convert between different representations
of BDD variables:
1. Code names: e.g., "@-87", "@86", "@48", "@-1" (Function.__str__() output)
2. Var names: e.g., "vvv84" (variable names in the BDD)  
3. Elegant names: e.g., "Visit: loc15\n Observe: ell-shaped" (human-readable descriptions)
"""

def create_var_name_converter(bdd, elegant_var_dict):
    """
    Create conversion functions for BDD variable names.
    
    Args:
        bdd: The BDD instance (dd.autoref.BDD)
        elegant_var_dict: Dictionary mapping elegant names to var names
        
    Returns:
        tuple: (code_to_elegant, code_to_var, var_to_elegant, elegant_to_var, 
                get_node_info, code_to_node_id)
    """
    
    # Create reverse mapping from var names to elegant names
    var_to_elegant_dict = {v: k for k, v in elegant_var_dict.items()}
    
    # Create mapping from variable names to levels
    var_to_level = bdd.vars
    
    # Create mapping from levels to variable names  
    level_to_var = {level: var for var, level in var_to_level.items()}
    
    def code_to_node_id(code_name):
        """
        Convert code name (e.g., "@-87") to node ID (int).
        
        Args:
            code_name (str): Code name like "@-87", "@86"
            
        Returns:
            int: Node ID (can be negative for complemented edges)
        """
        if not code_name.startswith('@'):
            raise ValueError(f"Code name must start with '@', got: {code_name}")
        
        try:
            return int(code_name[1:])
        except ValueError:
            raise ValueError(f"Invalid code name format: {code_name}")
    
    def code_to_var(code_name):
        """
        Convert code name (e.g., "@-87") to variable name (e.g., "vvv84").
        
        Args:
            code_name (str): Code name like "@-87", "@86"
            
        Returns:
            str: Variable name like "vvv84", or None if terminal node
        """
        node_id = code_to_node_id(code_name)
        abs_node_id = abs(node_id)
        
        # Check if it's a terminal node
        if abs_node_id == 1:
            return None  # Terminal nodes don't have variables
            
        # Get the level and variable for this node
        try:
            level, _, _ = bdd._bdd._succ[abs_node_id]
            if level in level_to_var:
                return level_to_var[level]
            else:
                return None
        except KeyError:
            return None
    
    def code_to_elegant(code_name):
        """
        Convert code name (e.g., "@-87") to elegant name.
        
        Args:
            code_name (str): Code name like "@-87", "@86"
            
        Returns:
            str: Elegant name like "Visit: loc15\n Observe: ell-shaped", 
                 or None if terminal/not found
        """
        var_name = code_to_var(code_name)
        if var_name is None:
            return None
        return var_to_elegant_dict.get(var_name)
    
    def var_to_elegant(var_name):
        """
        Convert variable name (e.g., "vvv84") to elegant name.
        
        Args:
            var_name (str): Variable name like "vvv84"
            
        Returns:
            str: Elegant name like "Visit: loc15\n Observe: ell-shaped"
        """
        return var_to_elegant_dict.get(var_name)
    
    def elegant_to_var(elegant_name):
        """
        Convert elegant name to variable name.
        
        Args:
            elegant_name (str): Elegant name like "Visit: loc15\n Observe: ell-shaped"
            
        Returns:
            str: Variable name like "vvv84"
        """
        return elegant_var_dict.get(elegant_name)
    
    def get_node_info(code_name):
        """
        Get comprehensive information about a BDD node from its code name.
        
        Args:
            code_name (str): Code name like "@-87", "@86"
            
        Returns:
            dict: Information about the node including:
                - node_id: The integer node ID
                - is_complemented: Whether this is a complemented edge  
                - var_name: Variable name (if not terminal)
                - elegant_name: Human-readable name (if available)
                - is_terminal: Whether this is a terminal node
                - terminal_value: If terminal, the boolean value
        """
        node_id = code_to_node_id(code_name)
        abs_node_id = abs(node_id)
        is_complemented = node_id < 0
        is_terminal = abs_node_id == 1
        
        info = {
            'node_id': node_id,
            'is_complemented': is_complemented,
            'is_terminal': is_terminal,
            'terminal_value': None,
            'var_name': None,
            'elegant_name': None
        }
        
        if is_terminal:
            # Terminal node: True if node_id > 0, False if node_id < 0  
            info['terminal_value'] = node_id > 0
        else:
            var_name = code_to_var(code_name)
            elegant_name = code_to_elegant(code_name)
            info['var_name'] = var_name
            info['elegant_name'] = elegant_name
            
        return info
    
    return (code_to_elegant, code_to_var, var_to_elegant, elegant_to_var, 
            get_node_info, code_to_node_id)


def demo_conversion(bdd, elegant_var_dict, sample_codes):
    """
    Demonstrate the conversion functions with sample code names.
    
    Args:
        bdd: The BDD instance
        elegant_var_dict: Dictionary mapping elegant names to var names
        sample_codes: List of code names to demonstrate with
    """
    print("BDD Variable Name Conversion Demo")
    print("=" * 50)
    
    # Create conversion functions
    (code_to_elegant, code_to_var, var_to_elegant, elegant_to_var, 
     get_node_info, code_to_node_id) = create_var_name_converter(bdd, elegant_var_dict)
    
    for code in sample_codes:
        print(f"\nCode: {code}")
        print("-" * 20)
        
        try:
            info = get_node_info(code)
            print(f"Node ID: {info['node_id']}")
            print(f"Complemented: {info['is_complemented']}")
            print(f"Terminal: {info['is_terminal']}")
            
            if info['is_terminal']:
                print(f"Terminal Value: {info['terminal_value']}")
            else:
                print(f"Variable Name: {info['var_name']}")
                if info['elegant_name']:
                    print(f"Elegant Name: {repr(info['elegant_name'])}")
                else:
                    print("Elegant Name: Not found")
                    
        except Exception as e:
            print(f"Error: {e}")


if __name__ == "__main__":
    # Example usage would go here
    pass
