import numpy as np
import tf.transformations as tft

from operator import itemgetter
import math
import sys
import matplotlib.pyplot as plt

def getRobotGridPosition(transMsg, gridInfo):
    pos = np.array([transMsg.transform.translation.x - gridInfo.origin.position.x, transMsg.transform.translation.y - gridInfo.origin.position.y, 0, 1])
    quat = gridInfo.origin.orientation
    mat = tft.quaternion_matrix(tft.quaternion_inverse([quat.x, quat.y, quat.z, quat.w]))
    gridPos = (mat.dot(pos[np.newaxis].T).flatten()[:2]) / gridInfo.resolution
    roundedPos = np.round(gridPos)
    pos = roundedPos if np.allclose(gridPos, roundedPos) else np.floor(gridPos)
    return pos

def getGridPosition(position, gridInfo):
    pos = np.array([position[0] - gridInfo.origin.position.x, position[1] - gridInfo.origin.position.y, 0, 1])
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

def findClosestFrontier(start,frontiers):
    distances = []
    shortDist = 10000
    ind = -1
    for a in range(len(frontiers)):
        i = frontiers[a]
        dist = math.sqrt((start[0]-i[0])**2 + (start[1]-i[1])**2) # euclid
        if dist < shortDist:
            shortDist = dist
            ind = a

    return ind

class NodeB:
    def __init__(self):
        self.gridIndex = 0
        self.value = 0

    def __str__(self):
        return "["+str(self.gridIndex)+"]:"+str(self.value)

    def __repr__(self):
        return "["+str(self.gridIndex)+"]:"+str(self.value)

    def __eq__(self, other):
        sameInd = self.gridIndex == other.gridIndex
        sameVal = self.value == other.value
        return (sameInd and sameVal)

def detectFrontiers(start,grid,rows,cols,threshold):

    startInd = (start[1]*cols) + start[0]
    start = NodeB()
    start.gridIndex = startInd
    start.value = grid[startInd]

    mapOpen = []
    mapClosed = []
    frontierOpen = []
    frontierClosed = []

    mapOpen.append(start)
    print("MAP:")
    print(mapOpen)
    #np.set_printoptions(threshold=sys.maxsize)
    #print(grid.reshape(50,50))
    #plt.scatter(range(0,50),range(0,50),c=grid.reshape(50,50))
    im=np.array([50 if x==-1 else x for x in grid])
    im = im.reshape(rows,cols)
    plt.imshow(im)
    #plt.show()

    frontiers = []
    newFrontiers = []

    while len(mapOpen)>0:
        p = mapOpen.pop(0)
        #print("actualP="+str(p))

        #print("mapClosed="+str(mapClosed))
        if p in mapClosed:
            #print("CLOSED!")
            continue

        if isFrontier(p,grid,cols,threshold):
            frontierOpen.append(p)

            while len(frontierOpen)>0:
                q = frontierOpen.pop(0)
                #print("actualQ="+str(q))
                if q in mapClosed:
                    continue
                if q in frontierClosed:
                    continue
                if isFrontier(q,grid,cols,threshold):
                    newFrontiers.append(q)
                    neighbours = getNeighbours(q,grid,rows,cols,threshold)
                    for w in neighbours:
                        isFrontierOpen = w in frontierOpen
                        isFrontierClosed = w in frontierClosed
                        isMapClosed = w in mapClosed
                        isOK = (not isFrontierOpen) and (not isFrontierClosed) and (not isMapClosed)
                        if isOK:
                            frontierOpen.append(w)
                            #print("add to examine:"+str(w))
                frontierClosed.append(q)
            for i in newFrontiers:
                mapClosed.append(i)
                frontiers.append(i)
            print("found frontiers="+str(newFrontiers))
            newFrontiers = []

        neighboursP = getNeighbours(p,grid,rows,cols,threshold)
        for v in neighboursP:
            isMapOpen = v in mapOpen
            isMapClosed = v in mapClosed
            isWorth = (not isMapOpen) and (not isMapClosed)
            if isWorth:
                neighboursV = getNeighbours(v,grid,rows,cols,threshold)
                isOpenSpace = any(x.value > -1 for x in neighboursV)
                #isOpenSpace = True in openSpace
                isOK = isWorth and isOpenSpace
                if isOK:
                    mapOpen.append(v)
                    #print("not here yet:"+str(v))

        mapClosed.append(p)

    print("DONE")
    print(frontiers)

    return returnFrontiers(frontiers,cols)

def returnFrontiers(frontiers,cols):
    ret = []
    for i in frontiers:
        parentInd = i.gridIndex
        parentR = int(parentInd / cols)
        parentC = int(parentInd % cols)
        val = np.array([[parentC],[parentR]])
        ret.append(val)

    print("Frontiers:")
    print(ret)
    return ret

def isFrontier(node,grid,cols,threshold):
    if node.value > -1:
        return False

    neighbours = getNeighbours(node,grid,rows,cols,threshold)
    print("Neighbours="+str(neighbours))
    #tmpPri = neighbours
    #tmpPri.insert(4,'ahoj')
    #print(tmpPri.reshape(3,3))
    #okoli = [x.value > -1 for x in neighbours]
    #okoli.insert(4,'ahoj')
    #print("Okoli="+str(okoli))
    openSpace = any(x.value > -1 for x in neighbours)
    print("["+str(node.gridIndex)+"]"+":"+str(node.value)+" is FRONTIER: "+str(openSpace))

    #if True in openSpace:
    #    return True
    return openSpace

def getNeighbours(node,grid,rows,cols,threshold):
    ret = []

    parentInd = node.gridIndex
    parentR = int(parentInd / rows)
    parentC = int(parentInd % cols)
    #print("parent="+str(parentR)+","+str(parentC))

    if ((parentR-1)>0) and ((parentR-1)<rows) and ((parentC-1)>0) and ((parentC-1)<cols):
        nodeInd = (parentR-1)*cols + parentC-1
        ret=addNode(nodeInd,grid,ret,threshold)

    if ((parentR-1)>0) and ((parentR-1)<rows) and ((parentC)>0) and ((parentC)<cols):
        nodeInd = (parentR-1)*cols + parentC
        ret=addNode(nodeInd,grid,ret,threshold)

    if ((parentR-1)>0) and ((parentR-1)<rows) and ((parentC+1)>0) and ((parentC+1)<cols):
        nodeInd = (parentR-1)*cols + parentC+1
        ret=addNode(nodeInd,grid,ret,threshold)

    if ((parentR)>0) and ((parentR)<rows) and ((parentC-1)>0) and ((parentC-1)<cols):
        nodeInd = (parentR)*cols + parentC-1
        ret=addNode(nodeInd,grid,ret,threshold)

    if ((parentR)>0) and ((parentR)<rows) and ((parentC+1)>0) and ((parentC+1)<cols):
        nodeInd = (parentR)*cols + parentC+1
        ret=addNode(nodeInd,grid,ret,threshold)

    if ((parentR+1)>0) and ((parentR+1)<rows) and ((parentC-1)>0) and ((parentC-1)<cols):
        nodeInd = (parentR+1)*cols + parentC-1
        ret=addNode(nodeInd,grid,ret,threshold)

    if ((parentR+1)>0) and ((parentR+1)<rows) and ((parentC)>0) and ((parentC)<cols):
        nodeInd = (parentR+1)*cols + parentC
        ret=addNode(nodeInd,grid,ret,threshold)

    if ((parentR+1)>0) and ((parentR+1)<rows) and ((parentC+1)>0) and ((parentC+1)<cols):
        nodeInd = (parentR+1)*cols + parentC+1
        ret=addNode(nodeInd,grid,ret,threshold)

    return ret

def addNode(nodeInd,grid,ret,threshold):
    if nodeInd >= 0 and nodeInd < len(grid):
        if grid[nodeInd]>threshold:
            #parentR = int(nodeInd / rows)
            #parentC = int(nodeInd % cols)
            #print("Prekazka: "+str(parentR)+","+str(parentC))
            return ret
        node = NodeB()
        node.gridIndex = nodeInd
        node.value = grid[nodeInd]
        ret.append(node)

    return ret

def AstarSearch(start, goal, grid, rows, cols):
    #open = np.array(dtype=dtype)
    open = []
    closed = []
    #path = []

    goalInd = int((goal[1]*cols) + goal[0])
    startInd = int((start[1]*cols) + start[0])
    print("startInd="+str(startInd))
    print("start in grid="+str(grid[startInd]))
    print("goalInd="+str(goalInd))

    #startNode = np.array([[startInd],[f],[g],[h],[startInd]]) # f,g,h,parent
    startNode = Node()
    startNode.gridIndex = startInd
    startNode.g = 0
    startNode.h = euclidianDistance(startInd,goalInd,cols)
    startNode.f = startNode.h
    startNode.parent = None
    #print("startNOde="+str(startNode))

    goal=Node()
    goal.gridIndex = goalInd

    if startInd==goalInd:
        return getPath(startNode,cols)

    #open=startNode
    open.append(startNode)
    closed.append(startNode)
    #open.append(startNode)
    print("open="+str(open))
    isOpened = any(x.gridIndex == startNode.gridIndex for x in open)
    print("isopen:"+str(isOpened))
    ind = open.index(startNode)
    print("INDEX="+str(ind))
    #path=startNode

    #open.append([start])
    #print("start="+str(start))
    #print("grid="+str(grid))
    """open=np.hstack((open,startNode))
    print("open="+str(open))
    print("openinds="+str(open[0,:]))
    print(np.isin(startNode[0], open[0,:]))"""
    #if

    while (len(open) > 0):
        #sorted(open, order='f')
        open.sort(key=lambda x: x.f)
        #open=open[:, open[1].argsort()]
#        print("sorted="+str(open))
        #q= open[0]
        q = open.pop(0)
#        print("actual="+str(q))
        """if q.gridIndex==goalInd:
            goal.parent = q
            return goal"""
        #open=np.delete(open,0)
        #if len(closed)==0:
        #    closed = q
        #else:
        #closed.append(q)
#        print("open="+str(open))
#        print("closed="+str(closed))
        #print("openinds="+str(open[0,:]))
        add = getSuccessors(q,grid,rows,cols)

        for succ in add:
            #print("now:"+str(succ.gridIndex))
            if succ.gridIndex==goalInd:
                goal.parent = q
                return getPath(goal,cols)

            succ.g=q.g + euclidianDistance(q.gridIndex,succ.gridIndex,cols)
            succ.h=euclidianDistance(succ.gridIndex,goalInd,cols)
            succ.f = succ.g + succ.h

            isOpened = any(x.gridIndex == succ.gridIndex for x in open)
#            print("isOpened?"+str(isOpened))
            isClosed = any(x.gridIndex == succ.gridIndex for x in closed)
#            print("isClosed?"+str(isClosed))

            if ((not isOpened) and  (not isClosed)):
                open.append(succ)
                closed.append(succ)
            elif isOpened:
                old = next((x for x in open if x.gridIndex == succ.gridIndex), None)
                #print("IND="+str(ind))
                #ind = open.index(succ)
                if succ.f < old.f:
                    open.remove(old)
                    closed.remove(old)
                    open.append(succ)
                    closed.append(succ)
            elif isClosed:
                old = next((x for x in closed if x.gridIndex == succ.gridIndex), None)
                if succ.f < old.f:
                    open.append(succ)
                    closed.remove(old)
                    closed.append(succ)

    print("END")
    #print(goal)
    if grid[goalInd]==False:
        print("CANNOT PLAN TO OBSTACLE!")
    else:
        print("NOT CONNECTED")
    return getPath(goal,cols)


def getPath(goal,cols):
    print("DONE - getting path")
    path=[]

    parent = goal
    while True:
        parentR = parent.gridIndex / cols
        parentC = parent.gridIndex % cols
        val = np.array([[parentC],[parentR]]) #[parentR,parentC]
        path.insert(0,val)
        parent = parent.parent
        if parent is None:
            return path

def euclidianDistance(start,goal,cols):
    startR = start / cols
    startC = start % cols

    goalR = goal / cols
    goalC = goal % cols

    h = math.sqrt((startC-goalC)**2 + (startR-goalR)**2)

    return h


def getSuccessors(node,grid,rows,cols):
    ret = []

    parentInd = node.gridIndex
    parentR = int(parentInd / cols)
    parentC = int(parentInd % cols)
    #print("parent="+str(parentR)+","+str(parentC))

    if ((parentR-1)>0) and ((parentR-1)<rows) and ((parentC-1)>0) and ((parentC-1)<cols):
        succInd = (parentR-1)*cols + parentC-1
        #print("succInd="+str(succInd))
        ret=addSucc(node,succInd,grid,ret)

    if ((parentR-1)>0) and ((parentR-1)<rows) and ((parentC)>0) and ((parentC)<cols):
        succInd = (parentR-1)*cols + parentC
        ret=addSucc(node,succInd,grid,ret)

    if ((parentR-1)>0) and ((parentR-1)<rows) and ((parentC+1)>0) and ((parentC+1)<cols):
        succInd = (parentR-1)*cols + parentC+1
        ret=addSucc(node,succInd,grid,ret)

    if ((parentR)>0) and ((parentR)<rows) and ((parentC-1)>0) and ((parentC-1)<cols):
        succInd = (parentR)*cols + parentC-1
        ret=addSucc(node,succInd,grid,ret)

    if ((parentR)>0) and ((parentR)<rows) and ((parentC+1)>0) and ((parentC+1)<cols):
        succInd = (parentR)*cols + parentC+1
        ret=addSucc(node,succInd,grid,ret)

    if ((parentR+1)>0) and ((parentR+1)<rows) and ((parentC-1)>0) and ((parentC-1)<cols):
        succInd = (parentR+1)*cols + parentC-1
        ret=addSucc(node,succInd,grid,ret)

    if ((parentR+1)>0) and ((parentR+1)<rows) and ((parentC)>0) and ((parentC)<cols):
        succInd = (parentR+1)*cols + parentC
        ret=addSucc(node,succInd,grid,ret)

    if ((parentR+1)>0) and ((parentR+1)<rows) and ((parentC+1)>0) and ((parentC+1)<cols):
        succInd = (parentR+1)*cols + parentC+1
        ret=addSucc(node,succInd,grid,ret)

    #print("successors="+str(ret))

    return ret

def addSucc(parent,succInd,grid,ret):
    if succInd >= 0 and succInd < len(grid): #and grid[succInd]==True:
        if grid[succInd]==False:
            #parentR = int(succInd / 50)
            #parentC = int(succInd % 50)
            #print("Prekazka: "+str(parentR)+","+str(parentC))
            return ret
        #succ = np.array([[succInd],[0],[0],[0],[parentInd]])
        succ = Node()
        succ.gridIndex = succInd
        succ.parent = parent
        #if len(ret) == 0:
        #    ret = succ
        #else:
        #    ret=np.hstack((ret,succ))
        ret.append(succ)

    return ret

class Node:
    def __init__(self):
        self.gridIndex = 0
        self.f = 0
        self.g = 0
        self.h = 0
        self.parent = None

    def __str__(self):
        #node = np.array([[self.gridIndex],[self.f],[self.g],[self.h],[self.parent]]) # f,g,h,parent
        return "["+str(self.gridIndex)+"]:"+str(self.f)+"="+str(self.g)+"+"+str(self.h)

    def __repr__(self):
        #node = np.array([[self.gridIndex],[self.f],[self.g],[self.h],[self.parent]]) # f,g,h,parent
        return "\n"+"["+str(self.gridIndex)+"]:"+str(self.f)+"="+str(self.g)+"+"+str(self.h)


    """def getGridIndex(self):
        return self.gridIndex

    def setF(self,f):
        self.f = f

    def setG(self,g):
        self.g = g

    def setH(self,h):
        self.h = h

    def getF(self):
        return self.f

    def getG(self):
        return self.g

    def getH(self):
        return self.h

    def setParent(self,parent):
        self.parent = parent

    def getParent(self):
        return self.parent"""
