import time
import random
import drawSample
import math
import sys
import imageToRects
import utils


def redraw(canvas):
    canvas.clear()
    canvas.markit( tx, ty, r=SMALLSTEP )
    drawGraph(G, canvas)
    for o in obstacles: canvas.showRect(o, outline='blue', fill='blue')
    canvas.delete("debug")


def drawGraph(G, canvas):
    global vertices,nodes,edges
    if not visualize: return
    for i in G[edges]:
       canvas.polyline(  [vertices[i[0]], vertices[i[1]] ]  )


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
            x = random.random()*XMAX
            y = random.random()*YMAX
            theta= random.uniform(-1, 1)*SMALLTHETA
        elif args.rrt_sampling_policy == "gaussian":
            # Gaussian with mean at the goal
            x = random.gauss(tx, sigmax_for_randgen)
            y = random.gauss(ty, sigmay_for_randgen)
            theta= 0
        else:
            print ("Not yet implemented")
            quit(1)
        # range check for gaussian
        if x<0: bad = 1
        if y<0: bad = 1
        if x>XMAX: bad = 1
        if y>YMAX: bad = 1
        if theta< 0: bad = 1
        if theta> 180: bad = 1
    return [x,y], theta

def returnParent(k, canvas):
    """ Return parent note for input node k. """
    for e in G[edges]:
        if e[1]==k:
            canvas.polyline(  [vertices[e[0]], vertices[e[1]] ], style=3  )
            return e[0]

def genvertex():
    vertices.append( genPoint() )
    return len(vertices)-1

def pointToVertex(p):
    vertices.append( p )
    return len(vertices)-1

def pickvertex():
    return random.choice( range(len(vertices) ))


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

    for id in G[0]:
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


def calculateEndpoints(p, length, small_theta):

    if small_theta< 90:
        right_end_x = p[0] + length / 2.0 * math.cos(math.radians(small_theta))
        right_end_y = p[1] + length / 2.0 * math.sin(math.radians(small_theta))

        left_end_x = p[0] - length / 2.0 * math.cos(math.radians(small_theta))
        left_end_y = p[1] - length / 2.0 * math.sin(math.radians(small_theta))
    else:
        right_end_x = p[0] - length / 2.0 * math.cos(math.radians(small_theta))
        right_end_y = p[1] - length / 2.0 * math.sin(math.radians(small_theta))

        left_end_x = p[0] + length / 2.0 * math.cos(math.radians(small_theta))
        left_end_y = p[1] + length / 2.0 * math.sin(math.radians(small_theta))

    left_end = [left_end_x, left_end_y]
    right_end = [right_end_x, right_end_y]

    return left_end, right_end

def detectPointInParallelogram(p, p1, p2, p3, p4):

    line1 = lineFromPoints(p1, p2)
    line2 = lineFromPoints(p2, p3)
    line3 = lineFromPoints(p3, p4)
    line4 = lineFromPoints(p1, p4)

    # If the point lies in the parallelogram, the product of the distances from the point to the parallel lines should be both negative or zero
    return (p[1] - line1[0] * p[0] - line1[1]) * (p[1] - line3[0] * p[0] - line3[1]) <=0 and (p[1] - line2[0] * p[0] - line2[1])* (p[1] - line4[0] * p[0] - line4[1]) <= 0

def robotHitsRect(p1, p2, r, theta):

    # Unpack rectangle coordinates
    left_top_x, left_top_y, right_bottom_x, right_bottom_y = r

    p1_left_end, p1_right_end= calculateEndpoints(p1, LENGTH, theta)
    p2_left_end, p2_right_end = calculateEndpoints(p2, LENGTH, theta)

    peripheral_intersection= (lineHitsRect(p1_left_end, p2_left_end, r) or lineHitsRect(p1_right_end, p2_right_end, r) or
                              lineHitsRect(p1_left_end, p1_right_end, r) or lineHitsRect(p2_left_end, p2_right_end, r))

    # Detect if any point of the rectangle lies within the parallelogram
    rectangle_intersection1= detectPointInParallelogram([left_top_x, left_top_y], p1_left_end, p2_left_end, p2_right_end, p1_right_end)
    rectangle_intersection2= detectPointInParallelogram([right_bottom_x, right_bottom_y], p1_left_end, p2_left_end, p2_right_end, p1_right_end)

    return peripheral_intersection or rectangle_intersection1 or rectangle_intersection2


def inRect(p,rect,dilation, theta):
    """ Return 1 in p is inside rect, dilated by dilation (for edge cases). """
    dilated_rec= rect[0] - dilation, rect[1] - dilation, rect[2] + dilation, rect[3] + dilation

    p_left_end, p_right_end = calculateEndpoints(p, LENGTH, theta)


    return lineHitsRect(p_left_end, p_right_end, dilated_rec)


def rotationCollisionDetection(vertex, o, cur_theta, new_theta):
    # This function detects if the robot will hit the obstacle if it rotates from cur_theta to new_theta
    # The robot is assumed to be at cp
    # Upack the obstacle coordinates
    left_top_x, left_top_y, right_bottom_x, right_bottom_y = o
    center_x, center_y= vertex
    if abs(center_x- left_top_x)< LENGTH/2.0:
        # Calculate the intersection of the obstacle with the sector
        dis= math.sqrt((LENGTH/2.0)**2- (center_x- left_top_x)**2)
        y_intersection1, y_intersection_2= center_y- dis, center_y+ dis
        # Check if the intersection lies within the obstacle
        if left_top_y<= y_intersection1<= right_bottom_y:
            intersection_angle= math.degrees(math.atan2(y_intersection1- center_y, center_x- left_top_x))
            if intersection_angle< 0:
                intersection_angle= 180- abs(intersection_angle)
            if min(cur_theta, new_theta)<= intersection_angle<= max(cur_theta, new_theta):
                return True
        if left_top_y<= y_intersection_2<= right_bottom_y:
            intersection_angle= math.degrees(math.atan2(y_intersection_2- center_y, center_x- left_top_x))
            if intersection_angle< 0:
                intersection_angle= 180- abs(intersection_angle)
            if min(cur_theta, new_theta)<= intersection_angle<= max(cur_theta, new_theta):
                return True

    if abs(center_x- right_bottom_x)< LENGTH/2.0:
        # Calculate the intersection of the obstacle with the sector
        dis= math.sqrt((LENGTH/2.0)**2- (center_x- right_bottom_x)**2)
        y_intersection1, y_intersection_2= center_y- dis, center_y+ dis
        # Check if the intersection lies within the obstacle
        if left_top_y<= y_intersection1<= right_bottom_y:
            intersection_angle= math.degrees(math.atan2(y_intersection1- center_y, center_x- right_bottom_x))
            if intersection_angle< 0:
                intersection_angle= 180- abs(intersection_angle)
            if min(cur_theta, new_theta)<= intersection_angle<= max(cur_theta, new_theta):
                return True
        if left_top_y<= y_intersection_2<= right_bottom_y:
            intersection_angle= math.degrees(math.atan2(y_intersection_2- center_y, center_x- right_bottom_x))
            if intersection_angle< 0:
                intersection_angle= 180- abs(intersection_angle)
            if min(cur_theta, new_theta)<= intersection_angle<= max(cur_theta, new_theta):
                return True

    if abs(center_y- left_top_y)< LENGTH/2.0:
        # Calculate the intersection of the obstacle with the sector
        dis= math.sqrt((LENGTH/2.0)**2- (center_y- left_top_y)**2)
        x_intersection1, x_intersection_2= center_x- dis, center_x+ dis
        # Check if the intersection lies within the obstacle
        if left_top_x<= x_intersection1<= right_bottom_x:
            intersection_angle= math.degrees(math.atan2(center_y- left_top_y, x_intersection1- center_x))
            if intersection_angle< 0:
                intersection_angle= 180- abs(intersection_angle)
            if min(cur_theta, new_theta)<= intersection_angle<= max(cur_theta, new_theta):
                return True
        if left_top_x<= x_intersection_2<= right_bottom_x:
            intersection_angle= math.degrees(math.atan2(center_y- left_top_y, x_intersection_2- center_x))
            if intersection_angle< 0:
                intersection_angle= 180- abs(intersection_angle)
            if min(cur_theta, new_theta)<= intersection_angle<= max(cur_theta, new_theta):
                return True

    if abs(center_y- right_bottom_y)< LENGTH/2.0:
        # Calculate the intersection of the obstacle with the sector
        dis= math.sqrt((LENGTH/2.0)**2- (center_y- right_bottom_y)**2)
        x_intersection1, x_intersection_2= center_x- dis, center_x+ dis
        # Check if the intersection lies within the obstacle
        if left_top_x<= x_intersection1<= right_bottom_x:
            intersection_angle= math.degrees(math.atan2(center_y- right_bottom_y, x_intersection1- center_x))
            if intersection_angle< 0:
                intersection_angle= 180- abs(intersection_angle)
            if min(cur_theta, new_theta)<= intersection_angle<= max(cur_theta, new_theta):
                return True
        if left_top_x<= x_intersection_2<= right_bottom_x:
            intersection_angle= math.degrees(math.atan2(center_y- right_bottom_y, x_intersection_2- center_x))
            if intersection_angle< 0:
                intersection_angle= 180- abs(intersection_angle)
            if min(cur_theta, new_theta)<= intersection_angle<= max(cur_theta, new_theta):
                return True

    return False





def addNewPoint(p1, p2, stepsize):
    pt = vertices[p1]
    distance_p1_p2 = pointPointDistance(pt, p2)

    if distance_p1_p2 == 0:
        raise ValueError("Points p1 and p2 are the same.")

    unit_vector = [(p2[0] - pt[0]) / distance_p1_p2, (p2[1] - pt[1]) / distance_p1_p2]
    dis= stepsize if distance_p1_p2> stepsize else distance_p1_p2
    new_point = [pt[0] + dis * unit_vector[0], pt[1] + dis * unit_vector[1]]
    return new_point

def rrt_search(G, tx, ty, canvas, user_theta=0):
    #Fill this function as needed to work ...
    global sigmax_for_randgen, sigmay_for_randgen
    n=0
    nsteps=0
    cur_theta= user_theta
    thetas= [cur_theta, cur_theta]
    num_iterations = 0
    while 1:
        p, theta = genPoint()
        cp = closestPointToPoint(G,p)
        v = addNewPoint(cp, p, SMALLSTEP)
        num_iterations += 1
        if visualize:
            # if nsteps%500 == 0: redraw()  # erase generated points now and then or it gets too cluttered
            n=n+1
            if n>10:
                canvas.events()
                n=0
        #print(len(G[nodes]))

        # Adjust the angle of the robot,
        # We can then make sure one end of the robot is always above the horizontal axis
        new_theta = thetas[cp] + theta
        if new_theta > 180:
            new_theta = new_theta - 180
        if new_theta< 0:
            new_theta= 180+ new_theta
        flag = 0
        i= 0
        for o in obstacles:

            if robotHitsRect(vertices[cp],v,o, new_theta) or inRect(v,o,1, new_theta) or rotationCollisionDetection(v, o, thetas[cp], new_theta):
                #print(robotHitsRect(vertices[cp],v,o), inRect(v,o,1), i)
                flag = 1
            i+= 1
        if flag == 1:
            continue

        k = pointToVertex(v)   # is the new vertex ID
        G[nodes].append(k)
        G[edges].append( (cp,k) )
        thetas.append(new_theta)
        if visualize:
            canvas.polyline(  [vertices[cp], vertices[k] ]  )
            left_point, right_point= calculateEndpoints(vertices[k], LENGTH, new_theta)
            canvas.polyline([left_point, right_point], style=2)


        if pointPointDistance( v, [tx,ty] ) < SMALLSTEP:
            #print ("Target achieved.", nsteps, "nodes in entire tree")
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
                print(str(num_iterations))

                global prompt_before_next
                if prompt_before_next:
                    #d = sys.stdin.readline().strip().lstrip()
                    """canvas.events()
                    print("More [c,q,g,quit]>")
                    
                    print("[" + d + "]")
                    if d == "c": canvas.delete()
                    if d == "q": return
                    if d == "g": prompt_before_next = 0
                    if d == "quit": exit(0)"""
                    #exit(0)
                break

def main():
    #seed
    random.seed(args.seed)
    if visualize:
        canvas = drawSample.SelectRect(xmin=0,ymin=0,xmax=XMAX ,ymax=YMAX, nrects=0, keepcontrol=0)#, rescale=800/1800.)
        for o in obstacles: canvas.showRect(o, outline='red', fill='blue')
    while 1:
        if visualize:
            redraw(canvas)
        G[edges].append( (0,1) )
        G[nodes].append(1)
        if visualize:
            canvas.markit( tx, ty, r=SMALLSTEP )
            drawGraph(G, canvas)
        rrt_search(G, tx, ty, canvas)

    if visualize:
        canvas.mainloop()

if __name__ == '__main__':
    # display = drawSample.SelectRect(imfile=im2Small,keepcontrol=0,quitLabel="")
    args = utils.get_args()
    visualize = utils.get_args()
    drawInterval = 100  # 10 is good for normal real-time drawing

    prompt_before_next = 1  # ask before re-running sonce solved
    SMALLSTEP = args.step_size  # what our "local planner" can handle.

    SMALLTHETA = 5  # degrees
    LENGTH= args.robot_length  # robot length

    map_size, obstacles = imageToRects.imageToRects(args.world)
    # Note the obstacles are the two corner points of a rectangle
    # Each obstacle is (x1,y1), (x2,y2), making for 4 points
    XMAX = map_size[0]
    YMAX = map_size[1]

    G = [[0], []]  # nodes, edges
    vertices = [[args.start_pos_x, args.start_pos_y], [args.start_pos_x, args.start_pos_y + 10]]

    # goal/target
    tx = args.target_pos_x
    ty = args.target_pos_y

    # start
    sigmax_for_randgen = XMAX / 2.0
    sigmay_for_randgen = YMAX / 2.0
    nodes = 0
    edges = 1
    main()
