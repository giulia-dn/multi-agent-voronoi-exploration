import matplotlib.pyplot as plt
import numpy as np
import cv2 
from skimage.draw import line
import random


class GridWorld:
    def __init__(self, grid_size=(500, 500)):
        self.grid_size = grid_size
        self.height, self.width = grid_size
        self.real_map = np.full((self.height, self.width, 3), 255, dtype=np.uint8)  

        cv2.rectangle(self.real_map, (0, 0), (self.width-1, self.height-1), (0, 0, 0), 4)
        cv2.rectangle(self.real_map, (140, 40), (200, 300), (0, 0, 0), -1) 
        cv2.rectangle(self.real_map, (280, 150), (450, 220), (0, 0, 0), -1)
        cv2.rectangle(self.real_map, (100, 400), (400, 460), (0, 0, 0), -1)
        cv2.circle(self.real_map, (400, 80), 45, (0, 0, 0), -1)
        cv2.circle(self.real_map, (250, 320), 30, (0, 0, 0), -1)
        cv2.circle(self.real_map, (450, 400), 35, (0, 0, 0), -1)
        cv2.circle(self.real_map, (60, 250), 40, (0, 0, 0), -1)
        cv2.rectangle(self.real_map, (5, 5), (110, 110), (255, 255, 255), -1)

        self.shared_map = np.full((self.height, self.width, 3), 127, dtype=np.uint8)  

    def get_pixel_color(self, x, y):
        return self.real_map[int(y), int(x)]

    def get_full_map(self):
        return self.real_map
    
    def final_check(self):
        free_space = (self.get_full_map() == [255, 255, 255]).all(axis=-1)
        total_free_pixels = np.sum(free_space)
        if total_free_pixels > 0:
            discovered_free_pixels = np.sum(np.all(self.shared_map[free_space] == [255, 255, 255], axis=-1))
            exploration_percentage = discovered_free_pixels / total_free_pixels
            if exploration_percentage >= 0.99995:
                return True
        return False
        
class Agent:
    def __init__(self, agent_id, start_pos, sensor_range):
        self.agent_id = agent_id
        self.pos = tuple(start_pos)  
        self.sensor_range = sensor_range
        self.voronoi_mask = None
        self.is_stuck = False
        
        self.total_distance = 0
        self.point_list = [self.pos]  
        self.parent_nodes = {self.pos: None}  

        self.current_path = []    
        self.active_goal = None  

    
    def scan_world(self, gridworld):
        x, y = self.pos
        x_min = max(0, x - self.sensor_range)
        x_max = min(gridworld.width, x + self.sensor_range + 1)
        y_min = max(0, y - self.sensor_range)
        y_max = min(gridworld.height, y + self.sensor_range + 1)

        p_y, p_x = np.ogrid[y_min:y_max, x_min:x_max]  
        dist_quad = (p_x - x) ** 2 + (p_y - y) ** 2
        mask = dist_quad <= self.sensor_range ** 2  

        gridworld.shared_map[y_min:y_max, x_min:x_max][mask] = gridworld.real_map[y_min:y_max, x_min:x_max][mask]  

    
    def move(self, x_goal, y_goal):
        dist = np.hypot(x_goal - self.pos[0], y_goal - self.pos[1])
        self.total_distance += dist
        self.pos = (x_goal, y_goal)

    
    def move_to_goal(self, gridworld, path, voronoi_mask=None, window_name=None, all_agents=None, closest_agent_map=None, video_writer=None):
        if path is None:
            return
        for i in range(1, len(path)):
            start_node = path[i-1]
            end_node = path[i]

            rr, cc = line(int(start_node[1]), int(start_node[0]), int(end_node[1]), int(end_node[0])) 
            step_size = max(1, self.sensor_range // 4) 

            for j in range(0, len(rr), step_size):
                sub_x, sub_y = cc[j], rr[j]

                if np.all(gridworld.get_pixel_color(sub_x, sub_y) == [0, 0, 0]) or (voronoi_mask is not None and not voronoi_mask[sub_y, sub_x]): 
                    return  
                                   
                self.move(sub_x, sub_y)
                self.scan_world(gridworld) 

                if window_name:
                    frontiers = extract_frontiers(gridworld.shared_map)
                    if all_agents is not None:
                        current_goals=[]
                        for a in all_agents:
                            current_goals.append(a.active_goal)
                    else:
                        current_goals = [self.active_goal]                
                    img = render_scene(gridworld, all_agents, closest_agent_map, frontiers, current_goals, agent_idx=self.agent_id)
                    cv2.imshow(window_name, img)
                    if video_writer is not None:
                        video_writer.write(img)
                    cv2.waitKey(10)

                look_ahead = min(len(rr), j + 3)
                future_pts = gridworld.shared_map[rr[j:look_ahead], cc[j:look_ahead]] 
                if np.any(np.all(future_pts == [0, 0, 0], axis=-1)): 
                    return 
                
                yield sub_x, sub_y
                
            self.move(end_node[0], end_node[1])
            self.scan_world(gridworld)
            yield end_node[0], end_node[1]
            
        self.active_goal = None

    
    def step_to_goal(self, start_node, end_node, step_size, limited_step=True, voronoi_mask=None, shared_map=None):
        distance = np.hypot(end_node[0]-start_node[0], end_node[1]-start_node[1]) 
        if limited_step: 
            if distance < step_size: 
                goal = int(end_node[0]), int(end_node[1])
            else: 
                theta = np.arctan2(end_node[1]-start_node[1], end_node[0]-start_node[0]) 
                goal = int(start_node[0]+step_size*np.cos(theta)), int(start_node[1]+step_size*np.sin(theta))

            rr, cc = line(int(start_node[1]), int(start_node[0]), int(goal[1]), int(goal[0])) 
            for j in range(len(rr)):
                if np.all(shared_map[rr[j], cc[j]] == [0, 0, 0]) or (voronoi_mask is not None and not voronoi_mask[rr[j], cc[j]]): 
                    index = max(0, j-3)
                    return (int(cc[index]), int(rr[index]))
            return goal
        else: 
            rr, cc = line(int(start_node[1]), int(start_node[0]), int(end_node[1]), int(end_node[0]))
            for j in range(len(rr)):
                if np.all(shared_map[rr[j], cc[j]] == [0, 0, 0]) or (voronoi_mask is not None and not voronoi_mask[rr[j], cc[j]]):
                    index = max(0, j-3)
                    return (int(cc[index]), int(rr[index]))
            return (int(end_node[0]), int(end_node[1]))

    def find_goal(self, gridworld, show_frontier=False, voronoi_mask=None):
        shared_map = gridworld.shared_map
        is_black = (shared_map == [0, 0, 0]).all(axis=-1).astype(np.uint8)
        is_white = (shared_map == [255, 255, 255]).all(axis=-1)
        is_grey = (shared_map == [127, 127, 127]).all(axis=-1)
        
        dilated_grey = cv2.dilate(is_grey.astype(np.uint8), np.ones((3,3)))
        danger_zone = cv2.dilate(is_black.astype(np.uint8), np.ones((5,5)))
        
        frontier_mask = ((is_white) & (dilated_grey > 0) & (danger_zone == 0))
        
        if voronoi_mask is not None:
        
            local_frontier_mask = frontier_mask & voronoi_mask
            valid_points = np.argwhere(local_frontier_mask)

            if valid_points.size == 0:
                global_points = np.argwhere(frontier_mask)
                if global_points.size > 0:
                    best_global_yx = find_closest_node(self.pos[0], self.pos[1], global_points)
                    best_global_xy = (int(best_global_yx[1]), int(best_global_yx[0]))

                    if voronoi_mask is not None:
                        my_cell_pixels = np.argwhere(voronoi_mask & is_white)
                        if my_cell_pixels.size > 0:
                            sub_goal_yx = find_closest_node(best_global_xy[0], best_global_xy[1], my_cell_pixels)
                            return (int(sub_goal_yx[1]), int(sub_goal_yx[0]))

                if voronoi_mask is not None: 
                    my_cell_pixels = np.argwhere(voronoi_mask & is_white)
                    if my_cell_pixels.size > 0:
                        target_yx = find_closest_node(self.pos[0], self.pos[1], my_cell_pixels)
                        return (int(target_yx[1]), int(target_yx[0]))
                return None
        else:
            valid_points = np.argwhere(frontier_mask)

        if show_frontier:
            return valid_points

        if valid_points.size > 0:
            target_yx = find_closest_node(self.pos[0], self.pos[1], valid_points)
            goal = (int(target_yx[1]), int(target_yx[0]))
            return goal
        else:
            dangerless_mask = is_white & dilated_grey
            if voronoi_mask is not None:
                dangerless_mask = dangerless_mask & voronoi_mask
            
            last_points = np.argwhere(dangerless_mask)
            if last_points.size == 0 and voronoi_mask is not None:
                last_points = np.argwhere(is_white & dilated_grey)

            if last_points.size > 0:
                target_yx = find_closest_node(self.pos[0], self.pos[1], last_points)
                if target_yx is None:
                    return None
                goal = (int(target_yx[1]), int(target_yx[0]))
                return goal
            return None

    def path_to_goal(self, target_node):
        path = []
        current_node = target_node
        while current_node is not None:
            path.append(current_node)
            current_node = self.parent_nodes.get(current_node)
        path.reverse()
        self.current_path = path
        return path
    
    def update_voronoi_mask(self, voronoi_mask):
        self.voronoi_mask = voronoi_mask  

def find_closest_node(x, y, pointlist):
    if pointlist is None or len(pointlist) == 0:
        return None
    points = np.array(pointlist)
    if isinstance(pointlist, list):
        x_p, y_p = points[:, 0], points[:, 1]
    else:
        y_p, x_p = points[:, 0], points[:, 1]
    
    dist = (x - x_p)**2 + (y - y_p)**2
    idx_min = np.argmin(dist)
    return pointlist[idx_min]

def collision_check(x1, y1, x2, y2, shared_map, voronoi_mask=None): 
    rr, cc = line(int(y1), int(x1), int(y2), int(x2))
    rr = np.clip(rr, 0, shared_map.shape[0] - 1)
    cc = np.clip(cc, 0, shared_map.shape[1] - 1)
    
    line_points = shared_map[rr, cc]
    if np.any(np.all(line_points == [0, 0, 0], axis=-1)) or np.any(np.all(line_points == [127, 127, 127], axis=-1)): 
        return True

    if voronoi_mask is not None:
        if np.any(voronoi_mask[rr, cc] == False):  
            return True
    return False

def extract_frontiers_mask(shared_map):
    is_unknown = np.all(shared_map == [127, 127, 127], axis=-1)
    is_free = np.all(shared_map == [255, 255, 255], axis=-1).astype(np.uint8) 
    kernel = np.ones((3, 3), np.uint8) 
    dilated_free = cv2.dilate(is_free, kernel, iterations=1)
    return is_unknown & (dilated_free > 0) 

def extract_frontiers(shared_map):
    return np.argwhere(extract_frontiers_mask(shared_map)) 

def planned_exploration(agent, gridworld, window_name="RRT", limited_step=True, all_agents=None, voronoi_mask=None, closest_agent_map=None, video_writer=None):
    if agent.active_goal is not None:
        return

    goal = agent.find_goal(gridworld, voronoi_mask=voronoi_mask) 
    if not goal:
        return
    
    agent.active_goal = goal  
    frontier_points = agent.find_goal(gridworld, show_frontier=True, voronoi_mask=voronoi_mask)
    start_node = (int(agent.pos[0]), int(agent.pos[1]))

    if np.hypot(goal[0] - start_node[0], goal[1] - start_node[1]) <= agent.sensor_range: 
        if not collision_check(start_node[0], start_node[1], goal[0], goal[1], gridworld.shared_map, voronoi_mask=voronoi_mask): 
            path = [start_node, goal] 
            list(agent.move_to_goal(gridworld, path, voronoi_mask=voronoi_mask, window_name=window_name, all_agents=all_agents, closest_agent_map=closest_agent_map, video_writer=video_writer))
            return

    agent.point_list = [start_node]
    agent.parent_nodes = {start_node: None}

    if voronoi_mask is not None:
        cell_y, cell_x = np.where(voronoi_mask)
    else:
        cell_y, cell_x = [], []

    max_rrt_iterations = 1000
    if voronoi_mask is not None:
        is_grey = (gridworld.shared_map == [127, 127, 127]).all(axis=-1)
        unknown_in_cell = is_grey & voronoi_mask
        if not np.any(unknown_in_cell): 
            max_rrt_iterations = 50  

    for i in range(max_rrt_iterations):
        if i % 10 == 0: 
            x_rand, y_rand = goal 
        elif len(cell_x) > 0: 
            idx = np.random.randint(0, len(cell_x))
            x_rand, y_rand = cell_x[idx], cell_y[idx]
        else: 
            x_rand = np.random.randint(0, gridworld.width)
            y_rand = np.random.randint(0, gridworld.height)

        closest = find_closest_node(x_rand, y_rand, agent.point_list) 
        next_node = agent.step_to_goal(closest, (x_rand, y_rand), step_size=agent.sensor_range // 2, limited_step=limited_step, voronoi_mask=voronoi_mask, shared_map=gridworld.shared_map) 
        if next_node is None:
            continue

        new_node = (int(np.clip(next_node[0], 0, gridworld.width - 1)), int(np.clip(next_node[1], 0, gridworld.height - 1))) 

        if np.all(gridworld.shared_map[new_node[1], new_node[0]] == [255, 255, 255]): 
            if not collision_check(closest[0], closest[1], new_node[0], new_node[1], gridworld.shared_map, voronoi_mask=voronoi_mask): 
                if new_node not in agent.parent_nodes: 
                    agent.point_list.append(new_node) 
                    agent.parent_nodes[new_node] = closest 

                    if window_name is not None:
                        if all_agents is not None: 
                            current_goals = [None] * len(all_agents)
                            current_goals[agent.agent_id] = goal

                        else:
                            current_goals = [goal]
                        img = render_scene(gridworld, all_agents, closest_agent_map, frontier_points, current_goals, agent_idx=agent.agent_id) 
                        for child_pt, parent_pt in agent.parent_nodes.items(): 
                            if parent_pt is not None:
                                cv2.line(img, (int(parent_pt[0]), int(parent_pt[1])), (int(child_pt[0]), int(child_pt[1])), (0, 255, 0), 1)
                        cv2.imshow(window_name, img)
                        
                        if video_writer is not None:
                            video_writer.write(img)
                        
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            return

                    if np.hypot(new_node[0] - goal[0], new_node[1] - goal[1]) <= (agent.sensor_range // 4): 
                        if not collision_check(new_node[0], new_node[1], goal[0], goal[1], gridworld.shared_map, voronoi_mask=voronoi_mask):
                            path = agent.path_to_goal(new_node)
                            path.append(goal) 

                            list(agent.move_to_goal(gridworld, path, voronoi_mask=voronoi_mask, window_name=window_name, all_agents=all_agents, closest_agent_map=closest_agent_map, video_writer=video_writer)) 
                            return

    agent.active_goal = None 
    agent.is_stuck = True 

    angle = random.uniform(0, 2 * np.pi)
    step_escape = agent.sensor_range // 2
    escape_x = int(np.clip(agent.pos[0] + step_escape * np.cos(angle), 0, gridworld.width - 1))
    escape_y = int(np.clip(agent.pos[1] + step_escape * np.sin(angle), 0, gridworld.height - 1))

    if not collision_check(agent.pos[0], agent.pos[1], escape_x, escape_y, gridworld.shared_map, voronoi_mask=voronoi_mask):
        agent.move(escape_x, escape_y)
        agent.scan_world(gridworld)
        agent.is_stuck = False  
    return

def compute_voronoi(agents, gridworld):
    height, width = gridworld.height, gridworld.width
    grid_y, grid_x = np.indices((height, width))  

    distances = []
    for agent in agents:
        agent_x, agent_y = agent.pos
        dist = np.hypot(grid_x - agent_x, grid_y - agent_y)  
        distances.append(dist)
    distances = np.stack(distances, axis=0)  
    closest_agent_map = np.argmin(distances, axis=0) 

    for i, agent in enumerate(agents):
        agent.update_voronoi_mask(closest_agent_map == i)  
    return closest_agent_map 


def render_scene(gridworld, all_agents, closest_agent_map, frontier_points, active_goals=None, agent_idx=0):
    img = gridworld.shared_map.copy()

    if closest_agent_map is not None:
        laplacian = cv2.Laplacian(closest_agent_map.astype(np.float32), cv2.CV_32F)
        img[np.abs(laplacian) > 0] = [0, 255, 255]  

    if frontier_points is not None and len(frontier_points) > 0:
        pts = np.atleast_2d(frontier_points)
        for pt in pts:
            cv2.circle(img, (int(pt[1]), int(pt[0])), 1, (255, 0, 255), -1)

    if all_agents is not None:
        for idx, a in enumerate(all_agents):
            rx, ry = int(a.pos[0]), int(a.pos[1])
            cv2.circle(img, (rx, ry), 5, (255, 0, 0), -1)
            cv2.circle(img, (rx, ry), a.sensor_range, (255, 0, 0), 1)
            cv2.putText(img, f"R{idx}", (rx - 8, ry - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1, cv2.LINE_AA)

    if active_goals is not None:
        for idx, g in enumerate(active_goals):
            if g is not None:
                gx, gy = int(g[0]), int(g[1])
                cv2.circle(img, (gx, gy), 5, (0, 0, 255), 2)
                cv2.putText(img, f"G{idx}", (gx + 6, gy + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv2.LINE_AA)

    return img

def run_simulation(agents, gridworld, window_name="Multi Agent Exploration", video_filename=None):
    if video_filename is None:
        video_filename = f"multi_agent_exploration_{len(agents)}_agents.mp4"
    for agent in agents:
        agent.scan_world(gridworld)
        
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL) 

    high, width = gridworld.height, gridworld.width
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    fps = 30
    out = cv2.VideoWriter(video_filename, fourcc, fps, (width, high))



    for _ in range(1500):
        if gridworld.final_check():
            print("Esplorazione completata con successo!")
            break

        closest_map = compute_voronoi(agents, gridworld) 

        safety_margin = 5
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2 * safety_margin + 1, 2 * safety_margin + 1))

        voronoi_masks = {}
        for idx in range(len(agents)):
            raw_mask = (closest_map == idx).astype(np.uint8)
            eroded_mask = cv2.erode(raw_mask, kernel).astype(bool)
            voronoi_masks[idx] = eroded_mask

        for idx, agent in enumerate(agents):
            planned_exploration(
                agent, 
                gridworld, 
                window_name=window_name, 
                limited_step=True,
                all_agents=agents,
                voronoi_mask=voronoi_masks[idx],
                closest_agent_map=closest_map,
                video_writer=out
            )
                
        frontiers = extract_frontiers(gridworld.shared_map) 
        active_goals = [] 
        for a in agents:
            active_goals.append(a.active_goal) 
        frame = render_scene(gridworld, agents, closest_map, frontiers, active_goals)

        out.write(frame) 
        
        cv2.imshow(window_name, frame) 
        if cv2.waitKey(100) & 0xFF == ord('q'): 
            print("Simulazione interrotta dall'utente.")
            break

    out.release()
    cv2.destroyAllWindows()
    print("Video della simulazione salvato come:", video_filename)


def run_joint_experiment(agent_list_counts, num_runs=3):
    results_iterations = {}
    results_distances = {}
    
    for num_agents in agent_list_counts:
        run_iters = []
        run_dists = []
        print(f"\n--- Test con {num_agents} agenti ---")
        
        for run in range(num_runs):
            gridworld = GridWorld(grid_size=(500, 500))
            start_positions = []
            
            for i in range(num_agents):
                while True:
                    ex = random.randint(-60, 60)
                    ey = random.randint(-60, 60)
                    x = 250 + ex
                    y = 250 + ey
                    pixel_value = gridworld.get_pixel_color(x, y)
                    is_white = np.all(pixel_value == [255, 255, 255])
                    secure_distance = all(np.linalg.norm(np.array((x, y)) - np.array(pos)) >= 20 for pos in start_positions)
                    if is_white and secure_distance:
                        break
                start_positions.append((x, y))
            
            agents = [Agent(agent_id=i, start_pos=start_positions[i], sensor_range=30) for i in range(num_agents)]
            
            for agent in agents:
                agent.scan_world(gridworld)
                
            completed_in_iterations = 1500
            for iteration in range(1500):
                if gridworld.final_check():
                    completed_in_iterations = iteration + 1
                    break
                
                
                closest_map = compute_voronoi(agents, gridworld)

                safety_margin = 5
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2 * safety_margin + 1, 2 * safety_margin + 1))

                voronoi_masks = {}
                for idx in range(len(agents)):
                    raw_mask = (closest_map == idx).astype(np.uint8)
                    eroded_mask = cv2.erode(raw_mask, kernel).astype(bool)
                    voronoi_masks[idx] = eroded_mask
                for idx, agent in enumerate(agents):
                    planned_exploration(
                        agent, 
                        gridworld, 
                        window_name=None,
                        limited_step=True,
                        all_agents=agents,
                        voronoi_mask=voronoi_masks[idx],
                        closest_agent_map=closest_map
                    )
            
            total_team_distance = sum(agent.total_distance for agent in agents)
            
            run_iters.append(completed_in_iterations)
            run_dists.append(total_team_distance)
            print(f"Run {run + 1}/{num_runs}: Iterazioni = {completed_in_iterations}, Distanza = {total_team_distance:.2f}")
            
        results_iterations[num_agents] = np.mean(run_iters)
        results_distances[num_agents] = np.mean(run_dists)
        
    return results_iterations, results_distances

def plot_combined_results(res_iters, res_dists):
    agents_count = list(res_iters.keys())
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    
    ax1.plot(agents_count, list(res_iters.values()), marker='o', color='purple', linewidth=2, markersize=8)
    ax1.set_title('Scalabilità: Agenti vs Iterazioni', fontsize=12)
    ax1.set_xlabel('Numero di Agenti (N)', fontsize=10)
    ax1.set_ylabel('Iterazioni Medie di Completamento', fontsize=10)
    ax1.grid(True, linestyle='--', alpha=0.7)
    ax1.set_xticks(agents_count)
    

    ax2.plot(agents_count, list(res_dists.values()), marker='s', color='blue', linewidth=2, markersize=8)
    ax2.set_title('Efficienza: Agenti vs Distanza Totale', fontsize=12)
    ax2.set_xlabel('Numero di Agenti (N)', fontsize=10)
    ax2.set_ylabel('Distanza Totale Cumulativa Media (pixel)', fontsize=10)
    ax2.grid(True, linestyle='--', alpha=0.7)
    ax2.set_xticks(agents_count)
    
    plt.tight_layout()
    plt.savefig('scalability_and_efficiency_analysis.png', dpi=300)
    plt.show()



   

if __name__ == "__main__":

    NUM_AGENTS = 4 #10 ok #20 #2 #5
    START_POSITIONS = []
    gridworld = GridWorld(grid_size=(500, 500))
    
    for i in range(NUM_AGENTS):
        while True:
            ex = random.randint(-60, 60)
            ey = random.randint(-60, 60)
            x = 250 + ex
            y = 250 + ey
            pixel_value = gridworld.get_pixel_color(x, y)

            is_white = np.all(pixel_value == [255, 255, 255])
            secure_distance = all(np.linalg.norm(np.array((x, y)) - np.array(pos)) >= 20 for pos in START_POSITIONS)

            if is_white and secure_distance:
                break
        START_POSITIONS.append((x, y))
    
    agents = [] 
    for i in range(NUM_AGENTS):
        pos = START_POSITIONS[i % len(START_POSITIONS)]
        agent = Agent(agent_id=i, start_pos=pos, sensor_range=30)
        agents.append(agent)
        
    # ESECUZIONE SIMULAZIONE MULTI-AGENTE
    print("Avvio della simulazione di esplorazione multi-agente...")
    run_simulation(agents, gridworld)
    
    # ESECUZIONE ESPERIMENTO DI SCALABILITÀ
    #agent_counts_to_test = [1, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20]
    #iters_data, dists_data = run_joint_experiment(agent_counts_to_test, num_runs=3)
    #plot_combined_results(iters_data, dists_data)
    
  