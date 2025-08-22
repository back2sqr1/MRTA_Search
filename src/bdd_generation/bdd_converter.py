"""
Simple BDD Variable Name Converter

Usage:
    from bdd_converter import BDDConverter
    
    converter = BDDConverter(bdd, elegant_var_dict)
    
    # Convert code names to elegant names
    elegant = converter.code_to_elegant("@-87")
    # Returns: 'Visit: loc06\n Observe: marilyn-diptych'
    
    # Convert code names to var names  
    var = converter.code_to_var("@86")
    # Returns: 'vvv40'
    
    # Get complete info about a node
    info = converter.get_info("@48")
    # Returns full dictionary with all information
"""

class BDDConverter:
    """Simple converter between BDD variable name formats."""
    
    def __init__(self, bdd, elegant_var_dict):
        """
        Initialize converter.
        
        Args:
            bdd: The BDD instance (dd.autoref.BDD)
            elegant_var_dict: Dict mapping elegant names to var names
        """
        self.bdd = bdd
        self.elegant_var_dict = elegant_var_dict
        self.var_to_elegant = {v: k for k, v in elegant_var_dict.items()}
        self.level_to_var = {level: var for var, level in bdd.vars.items()}
    
    def code_to_node_id(self, code_name):
        """Convert '@-87' to -87"""
        if not code_name.startswith('@'):
            raise ValueError(f"Code name must start with '@', got: {code_name}")
        return int(code_name[1:])
    
    def code_to_var(self, code_name):
        """Convert '@-87' to 'vvv35' (or None for terminal)"""
        node_id = self.code_to_node_id(code_name)
        abs_node_id = abs(node_id)
        
        if abs_node_id == 1:  # Terminal node
            return None
            
        try:
            level, _, _ = self.bdd._bdd._succ[abs_node_id]
            return self.level_to_var.get(level)
        except KeyError:
            return None
    
    def code_to_elegant(self, code_name):
        """Convert '@-87' to 'Visit: loc06\\n Observe: marilyn-diptych'"""
        var_name = self.code_to_var(code_name)
        if var_name is None:
            return None
        return self.var_to_elegant.get(var_name)
    
    def var_to_elegant(self, var_name):
        """Convert 'vvv35' to 'Visit: loc06\\n Observe: marilyn-diptych'"""
        return self.var_to_elegant.get(var_name)
    
    def elegant_to_var(self, elegant_name):
        """Convert 'Visit: loc06\\n Observe: marilyn-diptych' to 'vvv35'"""
        return self.elegant_var_dict.get(elegant_name)
    
    def get_info(self, code_name):
        """Get complete information about a BDD node."""
        node_id = self.code_to_node_id(code_name)
        abs_node_id = abs(node_id)
        is_terminal = abs_node_id == 1
        
        info = {
            'code': code_name,
            'node_id': node_id,
            'is_complemented': node_id < 0,
            'is_terminal': is_terminal,
        }
        
        if is_terminal:
            info['terminal_value'] = node_id > 0
            info['var_name'] = None
            info['elegant_name'] = None
        else:
            var_name = self.code_to_var(code_name)
            elegant_name = self.code_to_elegant(code_name)
            info['var_name'] = var_name
            info['elegant_name'] = elegant_name
            
        return info
    
    def print_conversion(self, code_name):
        """Print all conversions for a code name."""
        info = self.get_info(code_name)
        print(f"Code: {code_name}")
        if info['is_terminal']:
            print(f"  -> Terminal node (value: {info['terminal_value']})")
        else:
            print(f"  -> Var: {info['var_name']}")
            print(f"  -> Elegant: {repr(info['elegant_name'])}")
        print()


# Example usage
if __name__ == "__main__":
    # This would be used like:
    # converter = BDDConverter(bdd, elegant_var_dict)
    # converter.print_conversion("@-87")
    pass
