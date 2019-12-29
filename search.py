# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    return my_generic_search(problem, 'dfs')
    # util.raiseNotDefined()


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    return my_generic_search(problem, 'bfs')
    # util.raiseNotDefined()


# virtually, only bfs and dfs are allowed
def my_generic_search(problem, search_type):
    # init
    root = problem.getStartState()
    visited = set()
    if search_type == 'dfs':
        stru = util.Stack()
    elif search_type == 'bfs':
        stru = util.Queue()
    else:
        util.raiseNotDefined()
        return []
    # parent state, child, direction
    stru.push(StateInfo(None, root, None))

    # process
    while not stru.isEmpty():
        info = stru.pop()
        if problem.isGoalState(info.state):
            return info.get_path()

        if info.state not in visited:
            visited.add(info.state)
            for succ, direction, cost in problem.getSuccessors(info.state):
                stru.push(StateInfo(info, succ, direction))
    print('Unexpected position! search fails\n')
    return []


def retrieve_path_from_dict(goal_stat):
    res = []
    stat = goal_stat
    while stat[0] is not None:
        res.append(stat[2])
        stat = stat[0]
    res.reverse()
    return res


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    # Dijkstra
    root = problem.getStartState()
    visited = set()
    pq = util.PriorityQueue()       # node : least cost from root
    root_info = StateInfo(None, root, None, 0) # info.other is the least cost from root
    pq.push(root_info, 0)

    # process
    while not pq.isEmpty():
        info = pq.pop()
        # visit the node
        if problem.isGoalState(info.state):
            return info.get_path()

        if info.state in visited:
            continue
        visited.add(info.state)
        # add new nodes to pq
        least_cost = info.other
        for state, direction, cost in problem.getSuccessors(info.state):
            tp_cost = least_cost + cost
            chld = StateInfo(info, state, direction, tp_cost)
            pq.push(chld, tp_cost)

    print('Oops! Sth went wrong. Maybe no goal?')
    return []


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    root = problem.getStartState()
    visited = set()
    pq = util.PriorityQueue()  # node : least cost from root
    root_info = StateInfo(None, root, None, (0, 0))  # info.other is (evaluated_cost, least_cost)
    pq.push(root_info, 0)

    # process
    while not pq.isEmpty():
        info = pq.pop()
        # visit the node
        if problem.isGoalState(info.state):
            return info.get_path()

        if info.state in visited:
            continue
        visited.add(info.state)
        # add new nodes to pq
        least_cost = info.other[1]
        for state, direction, cost in problem.getSuccessors(info.state):
            tp_cost = least_cost + cost + heuristic(state, problem)
            chld = StateInfo(info, state, direction, (tp_cost, least_cost + cost))
            pq.push(chld, tp_cost)

    print('Oops! Sth went wrong. Maybe no goal?')
    return []


class StateInfo:
    def __init__(self, parent_info, state, direction, other=None):
        self.parent_info = parent_info
        self.state = state
        self.direction = direction
        self.other = other

    # def __init__(self, other_info):
    #     self.parent_info = other_info.parent_info
    #     self.state = other_info.state
    #     self.direction = other_info.direction

    def get_path(self):
        res = []
        info = self
        while info.parent_info is not None:
            res.append(info.direction)
            info = info.parent_info
        res.reverse()
        return res

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
