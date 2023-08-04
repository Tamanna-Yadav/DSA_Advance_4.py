#Q1.Breadth First Traversal for a Graph

from collections import defaultdict

# This class represents a directed graph
# using adjacency list representation
class Graph:
    # Constructor
    def __init__(self):
        # default dictionary to store graph
        self.graph = defaultdict(list)

    # function to add an edge to graph
    def addEdge(self,u,v):
        self.graph[u].append(v)

    # Function to print a BFS of graph
    def BFS(self, s):
        # Create a queue for BFS
        queue = []
        # Add the source node in visited and enqueue it
        queue.append(s)
        visited = [s]

        while queue:
            # Dequeue a vertex from queue and print it
            s = queue.pop(0)
            print (s, end = " ")

            # Get all adjacent vertices of the dequeued vertex s.
            # If an adjacent has not been visited, then add it in visited and enqueue it
            for i in self.graph[s]:
                if i not in visited:
                    queue.append(i)
                    visited.append(i)

# Driver code
# Create a graph given in the above diagram
g = Graph()
g.addEdge(0, 1)
g.addEdge(0, 2)
g.addEdge(1, 2)
g.addEdge(2, 0)
g.addEdge(2, 3)
g.addEdge(3, 3)

print ("BFS starting from vertex 2:")
g.BFS(2)

#Q2.Depth First Traversal for a Graph

from collections import defaultdict

# This class represents a directed graph using
# adjacency list representation
class Graph:

    # Constructor
    def __init__(self):
        # default dictionary to store graph
        self.graph = defaultdict(list)

    # function to add an edge to graph
    def addEdge(self,u,v):
        self.graph[u].append(v)

    # A function used by DFS
    def DFSUtil(self, v, visited):
        # Mark the current node as visited and print it
        visited.add(v)
        print(v, end=' ')

        # Recur for all the vertices adjacent to this vertex
        for neighbour in self.graph[v]:
            if neighbour not in visited:
                self.DFSUtil(neighbour, visited)

    # The function to do DFS traversal. It uses recursive DFSUtil()
    def DFS(self, v):
        # Create a set to store visited vertices
        visited = set()

        # Call the recursive helper function to print DFS traversal
        self.DFSUtil(v, visited)

# Driver code
if __name__ == "_main_":
    g = Graph()
    g.addEdge(0, 1)
    g.addEdge(0, 2)
    g.addEdge(1, 2)
    g.addEdge(2, 0)
    g.addEdge(2, 3)
    g.addEdge(3, 3)

    print("Following is Depth First Traversal (starting from vertex 2):")
    g.DFS(2)

#Q3.Count the number of nodes at given level in a tree using BFS


from collections import deque

adj = [[] for i in range(1001)]

def addEdge(v, w):
    # Add w to v’s list.
    adj[v].append(w)
    # Add v to w's list.
    adj[w].append(v)

def BFS(s, l):
    V = 100
    # Mark all the vertices as not visited
    visited = [False] * V
    level = [0] * V

    for i in range(V):
        visited[i] = False
        level[i] = 0

    # Create a queue for BFS
    queue = deque()

    # Mark the current node as visited and enqueue it
    visited[s] = True
    queue.append(s)
    level[s] = 0

    while (len(queue) > 0):
        # Dequeue a vertex from queue and print
        s = queue.popleft()
        #queue.pop_front()

        # Get all adjacent vertices of the dequeued vertex s.
        # If a adjacent has not been visited, then mark it visited and enqueue it
        for i in adj[s]:
            if (not visited[i]):
                # Setting the level of each node with an increment in the level of parent node
                level[i] = level[s] + 1
                visited[i] = True
                queue.append(i)

    count = 0
    for i in range(V):
        if (level[i] == l):
            count += 1
    return count

# Driver code
if __name__ == "_main_":
    addEdge(0, 1)
    addEdge(0, 2)
    addEdge(1, 3)
    addEdge(2, 4)
    addEdge(2, 5)

    level = 2

    print("Number of nodes at level", level, "is", BFS(0, level))

#Q4.Count number of trees in a forest

# A utility function to add an
# edge in an undirected graph.
def addEdge(adj, u, v):
	adj[u].append(v)
	adj[v].append(u)

# A utility function to do DFS of graph
# recursively from a given vertex u.
def DFSUtil(u, adj, visited):
	visited[u] = True
	for i in range(len(adj[u])):
		if (visited[adj[u][i]] == False):
			DFSUtil(adj[u][i], adj, visited)

# Returns count of tree is the
# forest given as adjacency list.
def countTrees(adj, V):
	visited = [False] * V
	res = 0
	for u in range(V):
		if (visited[u] == False):
			DFSUtil(u, adj, visited)
			res += 1
	return res

# Driver code
if __name__ == '_main_':
	V = int(input("Enter the number of vertices: "))
	E = int(input("Enter the number of edges: "))
	adj = [[] for i in range(V)]
	for i in range(E):
		u, v = map(int, input("Enter an edge: ").split())
		addEdge(adj, u, v)
	print(countTrees(adj, V))

#Q5.Detect Cycle in a Directed Graph

# Python program to detect cycle
# in a graph

from collections import defaultdict

class Graph():
	def __init__(self, vertices):
		self.graph = defaultdict(list)
		self.V = vertices

	def addEdge(self, u, v):
		self.graph[u].append(v)

	def isCyclicUtil(self, v, visited, recStack):

		# Mark current node as visited and
		# adds to recursion stack
		visited[v] = True
		recStack[v] = True

		# Recur for all neighbours
		# if any neighbour is visited and in
		# recStack then graph is cyclic
		for neighbour in self.graph[v]:
			if visited[neighbour] == False:
				if self.isCyclicUtil(neighbour, visited, recStack) == True:
					return True
			elif recStack[neighbour] == True:
				return True

		# The node needs to be popped from
		# recursion stack before function ends
		recStack[v] = False
		return False

	# Returns true if graph is cyclic else false
	def isCyclic(self):
		visited = [False] * self.V
		recStack = [False] * self.V
		for node in range(self.V):
			if visited[node] == False:
				if self.isCyclicUtil(node, visited, recStack) == True:
					return True
		return False

g = Graph(4)
g.addEdge(0, 1)
g.addEdge(0, 2)
g.addEdge(1, 2)
g.addEdge(2, 0)
g.addEdge(2, 3)
g.addEdge(3, 3)
if g.isCyclic() == 1:
	print ("Graph has a cycle")
else:
	print ("Graph has no cycle")


# Miscellaneous Question
#Q1.**Implement n-Queen’s Problem

global N
N = 4


def printSolution(board):
	for i in range(N):
		for j in range(N):
			if board[i][j] == 1:
				print("Q",end=" ")
			else:
				print(".",end=" ")
		print()


# A utility function to check if a queen can
# be placed on board[row][col]. Note that this
# function is called when "col" queens are
# already placed in columns from 0 to col -1.
# So we need to check only left side for
# attacking queens
def isSafe(board, row, col):

	# Check this row on left side
	for i in range(col):
		if board[row][i] == 1:
			return False

	# Check upper diagonal on left side
	for i, j in zip(range(row, -1, -1),
					range(col, -1, -1)):
		if board[i][j] == 1:
			return False

	# Check lower diagonal on left side
	for i, j in zip(range(row, N, 1),
					range(col, -1, -1)):
		if board[i][j] == 1:
			return False

	return True


def solveNQUtil(board, col):

	# Base case: If all queens are placed
	# then return true
	if col >= N:
		return True

	# Consider this column and try placing
	# this queen in all rows one by one
	for i in range(N):

		if isSafe(board, i, col):

			# Place this queen in board[i][col]
			board[i][col] = 1

			# Recur to place rest of the queens
			if solveNQUtil(board, col + 1) == True:
				return True

			# If placing queen in board[i][col
			# doesn't lead to a solution, then
			# queen from board[i][col]
			board[i][col] = 0

	# If the queen can not be placed in any row in
	# this column col then return false
	return False


# This function solves the N Queen problem using
# Backtracking. It mainly uses solveNQUtil() to
# solve the problem. It returns false if queens
# cannot be placed, otherwise return true and
# placement of queens in the form of 1s.
# note that there may be more than one
# solutions, this function prints one of the
# feasible solutions.
def solveNQ():
	board = [[0, 0, 0, 0],
			[0, 0, 0, 0],
			[0, 0, 0, 0],
			[0, 0, 0, 0]]

	if solveNQUtil(board, 0) == False:
		print("Solution does not exist")
		return False

	printSolution(board)
	return True


# Driver Code
if _name_ == '_main_':
	solveNQ()