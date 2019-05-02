import numpy as np
import tf.transformations as tft


def getRobotGridPosition(transMsg, gridInfo):
    pos = np.array([transMsg.transform.translation.x - gridInfo.origin.position.x, transMsg.transform.translation.y - gridInfo.origin.position.y, 0, 1])
    quat = gridInfo.origin.orientation
    mat = tft.quaternion_matrix(tft.quaternion_inverse([quat.x, quat.y, quat.z, quat.w]))
    gridPos = (mat.dot(pos[np.newaxis].T).flatten()[:2]) / gridInfo.resolution
    roundedPos = np.round(gridPos)
    pos = roundedPos if np.allclose(gridPos, roundedPos) else np.floor(gridPos)
    return pos

def gridToMapCoordinates(position, gridInfo):
    position = position * gridInfo.resolution
    originPos = np.array([gridInfo.origin.position.x, gridInfo.origin.position.y])
    pos = np.array([position[0], position[1], 0, 1])
    quat = gridInfo.origin.orientation
    mat = tft.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    
    pos = mat.dot(pos[np.newaxis].T).flatten()[:2] + originPos
    return pos

def mapToGridCoodrinates(position, gridInfo):
    pos = np.array([position[0] - gridInfo.origin.position.x, position[1] - gridInfo.origin.position.y, 0, 1])
    quat = gridInfo.origin.orientation
    mat = tft.quaternion_matrix(tft.quaternion_inverse([quat.x, quat.y, quat.z, quat.w]))
    gridPos = (mat.dot(pos[np.newaxis].T).flatten()[:2]) / gridInfo.resolution
    roundedPos = np.round(gridPos)
    pos = roundedPos if np.allclose(gridPos, roundedPos) else np.floor(gridPos)
    return pos

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None, distance = None):
        self.parent = parent
        self.position = position
        self.distance = distance

    def __eq__(self, other):
        return self.position == other.position

def getDist(x,y):
    return (x[0]-y[0])**2  + (x[1]-y[1])**2

def check(test,array):
    return any(np.array_equal(x, test) for x in array)

def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start, getDist(start,end))
    end_node = Node(None, end, None)
    open_list = []
    closed_list = []
    #print(start)
    #print(start_node)
    # Add the start node
    open_list.append(start_node)
    #print(len(open_list))
    # Loop until you find the end
    #print(type(open_list))
    while len(open_list) > 0:
        #print("-----")
        current_node = Node(None, None, 10000000)
        for op in open_list:
            #print(op.position)
            if op.distance < current_node.distance:
                current_node = op
        #print("help")
        open_list.remove(current_node)
        closed_list.append(current_node)
        current_index = 0
        #print("chosen:")
        #print(current_node.position)
        if np.all(current_node.position == end_node.position):
            path = []
            current = current_node
            while current.parent is not None:
                path.append(current.position)
                current = current.parent
            print("returning:")
            print(path)
            return path[::-1] # Return reversed path
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: # Adjacent squares
            node_position = [current_node.position[0] + new_position[0], current_node.position[1] + new_position[1]]
            #   #print(len(maze))
          # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            ###print(node_position)
            ##print(maze)
            #print(maze)
            if maze[int(node_position[0])][int(node_position[1])] > 0:
                #print(maze[int(node_position[0])][int(node_position[1])])
                continue

            # Create new node
            new_node = Node(current_node, node_position,getDist(node_position,end))
            if maze[int(node_position[0])][int(node_position[1])] < 0:
                new_node = Node(current_node, node_position,10000)
                print(new_node.position)
            ##print(new_node)
            # Append
            children.append(new_node)
        #print("help2")
        # Loop through children
        for child in children:
            allowed = True
            #print("childS   ")
            #print(child.position)
            #print("we add")
            # Child is on the closed list
            for closed_child in closed_list:
                if child.position[0] == closed_child.position[0] and child.position[1] == closed_child.position[1]:
                    #print("out1")
                    allowed = False


            # Child is already in the open list
            for open_child in open_list:
                if child.position[0] == open_child.position[0] and child.position[1] == open_child.position[1]:
                    #print("out2")
                    allowed = False   
 

            # Add the child to the open list
            if (allowed):
                #print("added")
                open_list.append(child)

    #print("shit happend ")
    return None