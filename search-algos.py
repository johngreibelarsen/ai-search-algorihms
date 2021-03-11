""" This modul only contains snippets of search algorithms"""

def extract_path(location_steps, start, goal):
    """
    Functionality to reconstruct paths. A path is a sequence of direction (actions) from an initial location to a goal
    location.

    :param location_steps: a dict containing a location as key and the previous location as a tuple value
    (previous loction, direction). A location is in the form of a 2-dim tuple (X,Y)
    :param start: the start location in form of a 2-dim tuple (X,Y)
    :param goal: the goal location in form of a 2-dim tuple (X,Y)
    :return: a list of actions in form of direction: Directions.NORTH, Directions.SOUTH, Directions.WEST, Directions.EAST
    """
    path = []
    current = goal
    while current != start:
       path.append(location_steps[current][1])
       current = location_steps[current][0]
    path.reverse() # optional
    return path

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.
    """
    from util import Stack

    frontier = Stack()
    frontier.push(problem.getStartState()) # tuple (X,Y)
    visited = set()
    came_from = dict()
    came_from[problem.getStartState()] = None
    goal_location = None

    while not frontier.isEmpty():
        current = frontier.pop()
        visited.add(current)

        if problem.isGoalState(current):
            goal_location = current
            break

        for location, direction, _ in problem.getSuccessors(current):
            if (location not in visited):
                frontier.push(location)
                came_from[location] = (current, direction)

    return extract_path(came_from, problem.getStartState(), goal_location)

