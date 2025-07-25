class Robot:
    def __init__(self, id, position = None, assigned_loc = '', cost = 0.0, time = 0.0):
        if position is None:
            position = [0, 0]
        self.id : str = id
        self.position: tuple[int, int] = position
        self.assigned_loc: str = assigned_loc
        self.cost: float = cost
        self.time: float = time
        self.velocity: float = 1.0
        
    
    def __str__(self):
        return f"Robot(id={self.id}, position=({round(self.position[0], 2)}, {round(self.position[1], 2)}), assigned_loc={self.assigned_loc}, cost={round(self.cost, 2)}, time={round(self.time, 2)})"
    
    def __repr__(self):
        return self.__str__()
        
    def __eq__(self, value):
        return isinstance(value, Robot) and self.id == value.id


RobotMap = dict[str, Robot]