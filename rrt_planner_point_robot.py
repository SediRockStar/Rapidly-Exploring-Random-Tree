'''
Hello and welcome to the first assignment :)
Please try to enjoy implementing algorithms you learned from the course; in this regard, we also have tried
to provide a good starting point for you to develop your own code. In this script, you may find different functions
that should be defined by you to perform specific tasks. A good suggestion would be to begin with the primary
function for the RRT algo named as "rrt_search". Following this function line by line you may realize how to define
each function to make it work!
Note, you may use the following commands to run this script with the required arguments:
python3 rrt_planner_point_robot.py --arg1_name your_input1 --arg2_name your_input2 e.g.
python3 rrt_planner_point_robot.py --world="shot.png"
To see the list of arguments, take a look at utils.py
Also note:
G is a list containing two groups: the first group includes the nodes IDs (0,1,2,...), while the second holds all pairs of nodes creating an edge
Vertices is a list of nodes locations in the map; it corresponds to the nodes' IDs mentioned earlier
GOOD LUCK!
'''

import random
import drawSample
import sys
import imageToRects
import utils


def redraw(canvas):
    canvas.clear()
    canvas.markit(tx, ty, r=SMALLSTEP)
    drawGraph(G, canvas)
    for o in obstacles: canvas.showRect(o, outline='blue', fill='blue')
    canvas.delete("debug")


def drawGraph(G, canvas):
    global vertices, nodes, edges
    if not visualize: return
    for i in G[edges]:
        # e.g. vertices: [[10, 270], [10, 280]]
        canvas.polyline([vertices[i[0]], vertices[i[1]]])


# Use this function to generate points randomly for the RRT algo
def genPoint():
    # if args.rrt_sampling_policy == "uniform":
    #     # Uniform distribution
    #     x = random.random()*XMAX
    #     y = random.random()*YMAX
    # elif args.rrt_sampling_policy == "gaussian":
    #     # Gaussian with mean at the goal
    #     x = random.gauss(tx, sigmax_for_randgen)
    #     y = random.gauss(ty, sigmay_for_randgen)
    # else:
    #     print ("Not yet implemented")
    #     quit(1)

    bad = 1
    while bad:
        bad = 0
        if args.rrt_sampling_policy == "uniform":
            # Uniform distribution
            x = random.random() * XMAX
            y = random.random() * YMAX
        elif args.rrt_sampling_policy == "gaussian":
            # Gaussian with mean at the goal
            x = random.gauss(tx, sigmax_for_randgen)
            y = random.gauss(ty, sigmay_for_randgen)
        else:
            print("Not yet implemented")
            quit(1)
        # range check for gaussian
        if x < 0: bad = 1
        if y < 0: bad = 1
        if x > XMAX: bad = 1
        if y > YMAX: bad = 1
    return [x, y]


def returnParent(k, canvas):
    """ Return parent note for input node k. """
    for e in G[edges]:
        if e[1] == k:
            canvas.polyline([vertices[e[0]], vertices[e[1]]], style=3)
            return e[0]


def genvertex():
    vertices.append(genPoint())
    return len(vertices) - 1


def pointToVertex(p):
    vertices.append(p)
    return len(vertices) - 1


def pickvertex():
    return random.choice(range(len(vertices)))


def lineFromPoints(p1, p2):
    # Calculate the slope (m)
    if p2[0] - p1[0] != 0:
        slope = (p2[1] - p1[1]) / (p2[0] - p1[0])
    else:
        # Handle the case of a vertical line
        slope = float('inf')

    # Calculate the y-intercept (b)
    y_intercept = p1[1] - slope * p1[0]

    return slope, y_intercept


def pointPointDistance(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** (1 / 2)


def closestPointToPoint(G, p2):
    min_distance = float('inf')
    closest_vertex = 0

    for id in G[nodes]:
        vertex= vertices[id]
        distance = pointPointDistance(vertex, p2)
        if distance < min_distance:
            min_distance = distance
            closest_vertex = id

    return closest_vertex
    # return vertex index


def lineHitsRect(p1, p2, r):
    # Unpack rectangle coordinates
    left_top_x, left_top_y, right_bottom_x, right_bottom_y = r

    # Calculate the line equation
    slope, y_intercept = lineFromPoints(p1, p2)

    # Check if any point of intersection lies within the rectangle
    if left_top_x <= p1[0] <= right_bottom_x and left_top_y <= p1[1] <= right_bottom_y:
        return True

    if left_top_x <= p2[0] <= right_bottom_x and left_top_y <= p2[1] <= right_bottom_y:
        return True

    # Check intersection with left side of the rectangle
    if slope != float('inf'):
        y_left = slope * left_top_x + y_intercept
        if left_top_y <= y_left <= right_bottom_y and left_top_x >= min(p1[0], p2[0]) and left_top_x <= max(p1[0], p2[0]):
            return True

    # Check intersection with right side of the rectangle
    if slope != float('inf'):
        y_right = slope * right_bottom_x + y_intercept
        if left_top_y <= y_right <= right_bottom_y and min(p1[0], p2[0]) <= right_bottom_x <= max(p1[0], p2[0]):
            return True

    # Check intersection with top side of the rectangle
    if slope != 0:
        x_top = (left_top_y - y_intercept) / slope
        if left_top_x <= x_top <= right_bottom_x and min(p1[1], p2[1]) <= left_top_y <= max(p1[1], p2[1]):
            return True

    # Check intersection with bottom side of the rectangle
    if slope != 0:
        x_bottom = (right_bottom_y - y_intercept) / slope
        if left_top_x <= x_bottom <= right_bottom_x and min(p1[1], p2[1]) <= right_bottom_y <= max(p1[1], p2[1]):
            return True

    return False


def inRect(p, rect, dilation):
    """ Return 1 in p is inside rect, dilated by dilation (for edge cases). """
    if p[0] <= rect[0] - dilation: return 0
    if p[1] <= rect[1] - dilation: return 0
    if p[0] >= rect[2] + dilation: return 0
    if p[1] >= rect[3] + dilation: return 0
    return 1


def addNewPoint(p1, p2, stepsize):
    pt= vertices[p1]
    distance_p1_p2 = pointPointDistance(pt, p2)

    if distance_p1_p2 == 0:
        raise ValueError("Points p1 and p2 are the same.")

    unit_vector = [(p2[0] - pt[0]) / distance_p1_p2, (p2[1] - pt[1]) / distance_p1_p2]
    dis = stepsize if distance_p1_p2 > stepsize else distance_p1_p2
    new_point = [pt[0] + dis * unit_vector[0], pt[1] + dis * unit_vector[1]]
    return new_point



def rrt_search(G, tx, ty, canvas):
    # Please carefully read the comments to get clues on where to start
    # Fill this function as needed to work ...
    global sigmax_for_randgen, sigmay_for_randgen
    n = 0
    nsteps = 0
    num_iterations= 0
    while 1:  # Main loop
        # This generates a point in form of [x,y] from either the normal dist or the Gaussian dist
        p = genPoint()

        # This function must be defined by you to find the closest point in the existing graph to the guiding point
        cp = closestPointToPoint(G, p)
        v = addNewPoint(cp, p, SMALLSTEP)
        num_iterations+= 1
        if visualize:
            # if nsteps%500 == 0: redraw()  # erase generated points now and then or it gets too cluttered
            n = n + 1
            if n > 10:
                canvas.events()
                n = 0
        flag= 0
        for o in obstacles:
            # The following function defined by you must handle the occlusion cases
            if lineHitsRect(vertices[cp], v, o) or inRect(v, o, 1):
                flag= 1
        if flag== 1:
            continue
        k = pointToVertex(v)  # is the new vertex ID
        G[nodes].append(k)
        G[edges].append((cp, k))
        if visualize:
            canvas.polyline([vertices[cp], vertices[k]])

        if pointPointDistance(v, [tx, ty]) < SMALLSTEP:
            #print("Target achieved.", nsteps, "nodes in entire tree")
            if visualize:
                t = pointToVertex([tx, ty])  # is the new vertex ID
                G[edges].append((k, t))
                if visualize:
                    canvas.polyline([p, vertices[t]], 1)
                # while 1:
                #     # backtrace and show the solution ...
                #     canvas.events()
                nsteps = 0
                totaldist = 0
                while 1:
                    oldp = vertices[k]  # remember point to compute distance
                    k = returnParent(k, canvas)  # follow links back to root.
                    canvas.events()
                    if k <= 1: break  # have we arrived?
                    nsteps = nsteps + 1  # count steps
                    totaldist = totaldist + pointPointDistance(vertices[k], oldp)  # sum lengths
                print(str(totaldist)+","+str(nsteps)+","+str(num_iterations))

                global prompt_before_next
                if prompt_before_next:
                    d = sys.stdin.readline().strip().lstrip()
                    """canvas.events()
                    print("More [c,q,g,quit]>")
                    d = sys.stdin.readline().strip().lstrip()
                    print("[" + d + "]")
                    if d == "c": canvas.delete()
                    if d == "q": return
                    if d == "g": prompt_before_next = 0
                    if d == "quit": exit(0)"""
                    exit(0)
                break


def main():
    # seed
    random.seed(args.seed)
    if visualize:
        canvas = drawSample.SelectRect(xmin=0, ymin=0, xmax=XMAX, ymax=YMAX, nrects=0,
                                       keepcontrol=0)  # , rescale=800/1800.)
        for o in obstacles: canvas.showRect(o, outline='red', fill='blue')
    while 1:
        # graph G
        redraw(canvas)
        G[edges].append((0, 1))
        G[nodes].append(1)
        if visualize: canvas.markit(tx, ty, r=SMALLSTEP)
        drawGraph(G, canvas)
        rrt_search(G, tx, ty, canvas)

    if visualize:
        canvas.mainloop()


if __name__ == '__main__':
    args = utils.get_args()
    visualize = utils.get_args()
    drawInterval = 10  # 10 is good for normal real-time drawing

    prompt_before_next = 1  # ask before re-running sonce solved
    SMALLSTEP = args.step_size  # what our "local planner" can handle.
    map_size, obstacles = imageToRects.imageToRects(args.world)
    # Note the obstacles are the two corner points of a rectangle (left-top, right-bottom)
    # Each obstacle is (x1,y1), (x2,y2), making for 4 points

    XMAX = map_size[0]
    YMAX = map_size[1]
    # The boundaries of the world are (0,0) and (XMAX,YMAX)

    G = [[0], []]  # nodes, edges
    # vertices = [[args.start_pos_x, args.start_pos_y], [args.start_pos_x, args.start_pos_y + 10]]
    vertices = [(args.start_pos_x, args.start_pos_y), (args.start_pos_x, args.start_pos_y + 10)]

    # goal/target
    tx = args.target_pos_x
    ty = args.target_pos_y

    # start
    sigmax_for_randgen = XMAX / 2.0
    sigmay_for_randgen = YMAX / 2.0
    nodes = 0
    edges = 1

    main()
