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
    frontier = util.Stack()
    visited = set()
    actions = []
    frontier.push( (problem.getStartState(), actions) )
    visited.add(problem.getStartState())

    while not frontier.isEmpty():
        state, actions = frontier.pop()
        visited.add(state)

        if problem.isGoalState(state):
            break

        for location, direction, _ in problem.getSuccessors(state):
            if (location not in visited):
                frontier.push( (location, actions + [direction]) )

    return actions


def depthFirstSearch_old(problem):
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


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    actions = []
    frontier = util.Queue()
    frontier.push( (problem.getStartState(), actions) )
    visited = [problem.getStartState()]

    while not frontier.isEmpty():
        state, actions = frontier.pop()

        if problem.isGoalState(state):
            break

        for location, direction, _ in problem.getSuccessors(state):
            if location not in visited:
                frontier.push( (location, actions + [direction]) )
                visited.append( location )
    return actions


def breadthFirstSearch_old(problem):
    """
    Search the shallowest nodes in the search tree first.
    """
    from util import Queue

    frontier = Queue()
    frontier.push(problem.getStartState()) # tuple (X,Y)
    came_from = dict()
    came_from[problem.getStartState()] = None
    goal_location = None

    while not frontier.isEmpty():
        current = frontier.pop()

        if problem.isGoalState(current):
            goal_location = current
            break

        for location, direction, _ in problem.getSuccessors(current):
            if (location not in came_from):
                frontier.push(location)
                came_from[location] = (current, direction)

    return extract_path(came_from, problem.getStartState(), goal_location)




class PositionSearchProblem(search.SearchProblem):
    """
    A search problem defines the state space, start state, goal test, successor
    function and cost function.  This search problem can be used to find paths
    to a particular point on the pacman board.

    The state space consists of (x,y) positions in a pacman game.

    Note: this search problem is fully specified; you should NOT change it.
    """

    def __init__(self, gameState, costFn = lambda x: 1, goal=(1,1), start=None, warn=True, visualize=True):
        """
        Stores the start and goal.

        gameState: A GameState object (pacman.py)
        costFn: A function from a search state (tuple) to a non-negative number
        goal: A position in the gameState
        """
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition()
        if start != None: self.startState = start
        self.goal = goal
        self.costFn = costFn
        self.visualize = visualize
        if warn and (gameState.getNumFood() != 1 or not gameState.hasFood(*goal)):
            print('Warning: this does not look like a regular search maze')

        # For display purposes
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE

    def getStartState(self):
        return self.startState

    def isGoalState(self, state):
        isGoal = state == self.goal

        # For display purposes only
        if isGoal and self.visualize:
            self._visitedlist.append(state)
            import __main__
            if '_display' in dir(__main__):
                if 'drawExpandedCells' in dir(__main__._display): #@UndefinedVariable
                    __main__._display.drawExpandedCells(self._visitedlist) #@UndefinedVariable

        return isGoal

    def getSuccessors(self, state):
        """
        Returns successor states, the actions they require, and a cost of 1.

         As noted in search.py:
             For a given state, this should return a list of triples,
         (successor, action, stepCost), where 'successor' is a
         successor to the current state, 'action' is the action
         required to get there, and 'stepCost' is the incremental
         cost of expanding to that successor
        """

        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x,y = state
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextState = (nextx, nexty)
                cost = self.costFn(nextState)
                successors.append( ( nextState, action, cost) )

        # Bookkeeping for display purposes
        self._expanded += 1 # DO NOT CHANGE
        if state not in self._visited:
            self._visited[state] = True
            self._visitedlist.append(state)

        return successors

    def getCostOfActions(self, actions):
        """
        Returns the cost of a particular sequence of actions. If those actions
        include an illegal move, return 999999.
        """
        if actions == None: return 999999
        x,y= self.getStartState()
        cost = 0
        for action in actions:
            # Check figure out the next state and see whether its' legal
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]: return 999999
            cost += self.costFn((x,y))
        return cost
