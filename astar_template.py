import numpy as np
from utils import get_collision_fn_PR2, load_env, execute_trajectory, draw_sphere_marker
from pybullet_tools.utils import connect, disconnect, get_joint_positions, wait_if_gui, set_joint_positions, joint_from_name, get_link_pose, link_from_name
from pybullet_tools.pr2_utils import PR2_GROUPS
import time
### YOUR IMPORTS HERE ###
from queue import PriorityQueue
#########################

def main(screenshot=False):
    # initialize PyBullet
    connect(use_gui=True)
    # load robot and obstacle resources
    robots, obstacles = load_env('pr2doorway.json')

    # define active DoFs
    base_joints = [joint_from_name(robots['pr2'], name) for name in PR2_GROUPS['base']]

    collision_fn = get_collision_fn_PR2(robots['pr2'], base_joints, list(obstacles.values()))
    # Example use of collision checking
    # print("Robot colliding? ", collision_fn((0.5, -1.3, -np.pi/2)))

    # Example use of setting body poses
    # set_pose(obstacles['ikeatable6'], ((0, 0, 0), (1, 0, 0, 0)))

    # Example of draw 
    # draw_sphere_marker((0, 0, 1), 0.1, (1, 0, 0, 1))
    
    start_config = tuple(get_joint_positions(robots['pr2'], base_joints))
    goal_config = (2.6, -1.3, -np.pi/2)
    path = []
    start_time = time.time()
    ### YOUR CODE HERE ###
    final_path_points = set() # just visualization for the markers
    collision_points = set() # just visualization for the markers
    non_collision_points = set() # just visualization for the markers
    class Node:
        def __init__(self, x, y, theta):
            self.x = x
            self.y = y
            self.theta = theta
            self.cost = float('inf')
            self.heuristic = 0
            self.parent = None

        def __lt__(self, other):
            return self.cost + self.heuristic < other.cost + other.heuristic

        def __eq__(self, other):
            return self.x == other.x and self.y == other.y and self.theta == other.theta

        def __hash__(self):
            return hash((self.x, self.y, self.theta))

        def __str__(self):
            return f"Node(x={self.x}, y={self.y}, theta={self.theta})"
        
    def action_cost(n, m):
        xy = (n.x - m.x)**2 + (n.y - m.y)**2    
        t = (min(abs(n.theta - m.theta), ((2*np.pi)- abs(n.theta - m.theta))))**2
        return np.round(np.sqrt(xy + t),3)


    def heuristic(n, g):
        xy = (n.x - g.x)**2 + (n.y - g.y)**2
        t = (min(abs(n.theta - g.theta), ((2*np.pi)- abs(n.theta - g.theta))))**2
        return np.round(np.sqrt(xy + t),3)


    def get_four_connect(node):
        x, y, theta = node.x, node.y, node.theta 
        del_d = 0.1
        det_theta = np.pi/2
        neighbors = [
            Node(round(x+del_d,3), y, theta),
            Node(round(x-del_d,3), y, theta),
            Node(x, round(y+del_d,3), theta),
            Node(x, round(y-del_d,3), theta),
            Node(x, y, round((theta+det_theta)%(2*np.pi)-np.pi,3)),
            Node(x, y, round((theta-det_theta)%(2*np.pi)-np.pi,3))
        ]
        return neighbors

    def get_eight_connect(node):
        x, y, theta = node.x, node.y, node.theta
        del_dx = 0.3 #0.5 and 0.3 works
        del_dy = 0.1
        det_theta = np.pi/2

        neighbors = [
            Node(np.round(x-del_dx, 3), np.round(y-del_dy, 3), np.round(theta, 3)),
            Node(np.round(x-del_dx, 3), np.round(y, 3), np.round(theta, 3)),
            Node(np.round(x-del_dx, 3), np.round(y+del_dy, 3), np.round(theta, 3)),
            Node(np.round(x, 3), np.round(y-del_dy, 3), np.round(theta, 3)),
            Node(np.round(x, 3), np.round(y+del_dy, 3), np.round(theta, 3)),
            Node(np.round(x+del_dx, 3), np.round(y-del_dy, 3), np.round(theta, 3)),
            Node(np.round(x+del_dx, 3), np.round(y, 3), np.round(theta, 3)),
            Node(np.round(x+del_dx, 3), np.round(y+del_dy, 3), np.round(theta, 3)),

            Node(np.round(x-del_dx, 3), y, np.round((theta+det_theta)%(2*np.pi) - np.pi, 3)),
            Node(np.round(x-del_dx, 3), y, np.round((theta-det_theta)%(2*np.pi) - np.pi, 3)),
            Node(np.round(x, 3), y, np.round((theta+det_theta)%(2*np.pi) - np.pi, 3)),
            Node(np.round(x, 3), y, np.round((theta-det_theta)%(2*np.pi) - np.pi, 3)),
            Node(np.round(x+del_dx, 3), y, np.round((theta+det_theta)%(2*np.pi) - np.pi, 3)),
            Node(np.round(x+del_dx, 3), y, np.round((theta-det_theta)%(2*np.pi) - np.pi, 3)),

            Node(x, np.round(y-del_dy, 3), np.round((theta+det_theta)%(2*np.pi) - np.pi, 3)),
            Node(x, np.round(y-del_dy, 3), np.round((theta-det_theta)%(2*np.pi) - np.pi, 3)),
            Node(x, np.round(y, 3), np.round((theta+det_theta)%(2*np.pi) - np.pi, 3)),
            Node(x, np.round(y, 3), np.round((theta-det_theta)%(2*np.pi) - np.pi, 3)),
            Node(x, np.round(y+del_dy, 3), np.round((theta+det_theta)%(2*np.pi) - np.pi, 3)),
            Node(x, np.round(y+del_dy, 3), np.round((theta-det_theta)%(2*np.pi) - np.pi, 3)),

            Node(np.round(x-del_dx, 3), np.round(y-del_dy, 3), np.round((theta+det_theta)%(2*np.pi) - np.pi, 3)),
            Node(np.round(x-del_dx, 3), np.round(y-del_dy, 3), np.round((theta-det_theta)%(2*np.pi) - np.pi, 3)),
            Node(np.round(x+del_dx, 3), np.round(y+del_dy, 3), np.round((theta+det_theta)%(2*np.pi) - np.pi, 3)),
            Node(np.round(x+del_dx, 3), np.round(y+del_dy, 3), np.round((theta-det_theta)%(2*np.pi) - np.pi, 3)),
            Node(np.round(x+del_dx, 3), np.round(y-del_dy, 3), np.round((theta+det_theta)%(2*np.pi) - np.pi, 3)),
            Node(np.round(x-del_dx, 3), np.round(y+del_dy, 3), np.round((theta-det_theta)%(2*np.pi) - np.pi, 3))
        ]

        return neighbors

    def close_enough(node1, node2, threshold=0.001):
        return (abs(node1.x - node2.x) <= threshold and 
                abs(node1.y - node2.y) <= threshold and 
                abs(node1.theta - node2.theta) <= threshold)
    
    # astar main code
    start = Node(np.round(start_config[0],3), np.round(start_config[1],3), np.round(start_config[2],3))
    goal = Node(np.round(goal_config[0],3), np.round(goal_config[1],3), np.round(goal_config[2],3))

    open_list = PriorityQueue()
    open_dict = {start: np.round(start.cost,3)}  
    closed_list = set()

    start.cost = 0
    start.heuristic = heuristic(start, goal)
    open_list.put(start)

    while not open_list.empty():
        current_node = open_list.get()
        # print(f"cuurrent{np.round(current_node.cost,3), current_node.x,current_node.y, current_node.theta} goal{goal.x, goal.y, goal.theta}, len {open_list.qsize(), close_enough(current_node, goal)}")


        if current_node in closed_list:
            continue

        closed_list.add(current_node)

        if close_enough(current_node, goal):
            while current_node is not None:
                final_path_points.add((current_node.x, current_node.y, 0.5)) # just visualization for the markers
                path.append((current_node.x, current_node.y, current_node.theta))
                current_node = current_node.parent
            break

        # neighbors = get_eight_connect(current_node)

        neighbors = get_four_connect(current_node)
        for neighbor in neighbors:
            if neighbor in closed_list:
                continue

            if not collision_fn((neighbor.x, neighbor.y, neighbor.theta)):
                tentative_g = np.round(current_node.cost,3) + action_cost(current_node, neighbor)
                if neighbor not in open_dict or tentative_g < open_dict[neighbor]:
                    non_collision_points.add((current_node.x, current_node.y, 0.5)) # just visualization for the markers
                    neighbor.parent = current_node
                    neighbor.cost = tentative_g
                    neighbor.heuristic = heuristic(neighbor, goal)
                    open_list.put(neighbor)
                    open_dict[neighbor] = tentative_g
            else:
                collision_points.add((current_node.x, current_node.y, 0.5)) # just visualization for the markers
                

    path.reverse()  

    cost_final_path = 0
    count = 0
    print(len(path))
    for i in range(len(path)-1):
        n = Node(path[i][0], path[i][1], path[i][2])
        m = Node(path[i+1][0], path[i+1][1], path[i+1][2])
        cost_final_path+=action_cost(n, m)
        count += 1
    print("final cost", cost_final_path)
    print("Planner run time: ", time.time() - start_time)



    for point in final_path_points:
        draw_sphere_marker(point, 0.09, (0, 0, 0, 1))
    for point in collision_points:
        if point not in final_path_points:
            draw_sphere_marker(point, 0.09, (1, 0, 0, 1))
    for point in non_collision_points:
        if point not in final_path_points and point not in collision_points:
            draw_sphere_marker(point, 0.09, (0, 0, 3, 1))
    if path:
        print("Path Found")
    else:
        print("No solution found")
    
    ######################
    print("Planner run time: ", time.time() - start_time)
    # Execute planned path
    execute_trajectory(robots['pr2'], base_joints, path, sleep=0.2)
    # Keep graphics window opened
    wait_if_gui()
    disconnect()

if __name__ == '__main__':
    main()