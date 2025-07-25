from .robot_class import Robot, RobotMap

class TimeStepNode: 
    @property
    def id(self) -> str:
        return self._id
    @property
    def robot_map(self) -> RobotMap:
        return self._robot_map
    
    @property
    def query(self) -> str:
        return self._query
    
    @property 
    def type(self) -> str:
        return self._type



    

    def __init__(self, id: str, robot_map: RobotMap, query: str, type: str, resolved_questions: dict[str, str], next: list['TimeStepNode'], visited_locations: set[str] | None = None):
        self._id = id
        self._robot_map = robot_map
        self._query = query
        self._type = type
        self.resolved_questions = resolved_questions
        self.next = next if next is not None else []
        self.visited_locations = visited_locations if visited_locations is not None else set()

    def __eq__(self, other: 'TimeStepNode') -> bool:
        if not isinstance(other, TimeStepNode):
            return False
        return (self.id == other.id and 
                self.robot_map == other.robot_map and 
                self.query == other.query and 
                self.type == other.type and 
                self.resolved_questions == other.resolved_questions and 
                self.next == other.next and
                self.visitedLocations == other.visitedLocations)
    
    def get_cost(self) -> float:
        total_cost = 0.0
        for robot in self.robot_map.values():
            total_cost += robot.cost
        return total_cost

    def get_time(self) -> float:
        max_time = 0.0
        for robot in self.robot_map.values():
            max_time = max(max_time, robot.time)
        return max_time
    
    def __str__(self):
        s = f"TimeStepNode(query={self.query}, type={self.type}, resolved_questions={self.resolved_questions}, robot_map={self.robot_map}, visited_locations={self.visited_locations})"
        s += '\nNext:\n'
        for next_node in self.next:
            s += f"  - {next_node.robot_map}, COST: {next_node.get_cost()}), RESOLVED: {next_node.resolved_questions}\n"
        return s
    
    def __repr__(self):
        return self.__str__()

