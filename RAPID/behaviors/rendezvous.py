import numpy as np
from scipy.optimize import linear_sum_assignment
from sklearn.cluster import KMeans
from copy import deepcopy

from ..utils import find_frontier_cells, euclidian_distance, a_star_cost, find_nearest_free
from .common import return_home_or_finish

def rendezvous(selfrobot): #from Bramblett, 2022
        def cluster_env():
            # unknowns = np.column_stack(np.where(selfrobot.belief_space["occupancy_grid"]==-1))
            robots = list(selfrobot.belief_space["robot_informations"].keys())
            frontiers = find_frontier_cells(selfrobot.belief_space["occupancy_grid"], traversable_types=selfrobot.traversable_types)
            kmeans_clusters = KMeans(n_clusters=len(robots), random_state=0, n_init="auto").fit(frontiers)

            return kmeans_clusters

        def explore_subbehavior(delta = 1):
            selfrobot.delta_replan = 10
            art_found = []
            if "artifacts" in selfrobot.belief_space: #check artifacts for pi4 condition
                single_arts = [art for art in selfrobot.belief_space["artifacts"] if selfrobot.belief_space["artifacts"][art]["needed_robots"] < 2]
                if len(single_arts) > 0:
                    art_labels = selfrobot.current_clustering.predict([selfrobot.belief_space["artifacts"][art]["coordinates"] for art in single_arts])
                    for art, artprediction in zip(single_arts, art_labels):
                        art_cluster = (int(selfrobot.current_clustering.cluster_centers_[artprediction][0]), int(selfrobot.current_clustering.cluster_centers_[artprediction][1]))
                        if selfrobot.belief_space["artifacts"][art]["status"] not in ["done", "destroyed"] and art_cluster == selfrobot.allocated_cluster:
                            art_found.append({"id": art, "type":selfrobot.belief_space["artifacts"][art]["type"], "coordinates": selfrobot.belief_space["artifacts"][art]["coordinates"]})

            #Pi1 condition in the paper
            if np.abs(selfrobot.env.step - selfrobot.rdvtime) < 1.5 * selfrobot.max_speed.x * a_star_cost(selfrobot.belief_space["occupancy_grid"], start = (int(selfrobot.transform.x), int(selfrobot.transform.y)), goal = (int(selfrobot.rdvspot[0]), int(selfrobot.rdvspot[1])), env_ease=selfrobot.env_ease, traversable_types=selfrobot.traversable_types): #if the time until rdvtime is shorter than 1.5* time to go for it, then, pass in rdv mode
                selfrobot.rdvstate = "rendezvous"
            #Pi4 condition in the paper
            elif len(art_found) > 0:
                selfrobot.action_to_perform = art_found[0] #TODO : Use equation 10 of the paper to decide if and which task to use????
                selfrobot.target = art_found[0]["coordinates"]
                selfrobot.last_plan_time = selfrobot.env.step
                selfrobot.rdvstate = "exploit"#"search"

            elif selfrobot.target == None: #else, we stay in the explore state and recompute a target if necessary
                #sobel detection for frontier
                frontiers = find_frontier_cells(selfrobot.belief_space["occupancy_grid"], traversable_types= selfrobot.traversable_types)
                if len(frontiers)>0:
                    labels = selfrobot.current_clustering.predict(frontiers)
                    centers = selfrobot.current_clustering.cluster_centers_[labels]
                    centers_int = np.column_stack((centers[:, 0].astype(int), centers[:, 1].astype(int)))
                    in_allocated_cluster = np.all(centers_int == np.array(selfrobot.allocated_cluster), axis=1)
                    position = (selfrobot.transform.x, selfrobot.transform.y)
                    base_cost = euclidian_distance(position, frontiers.T) #eq.6, case2
                    alloc_cluster_coords = (int(selfrobot.allocated_cluster[0]), int(selfrobot.allocated_cluster[1]))
                    fcosts = np.where(in_allocated_cluster, base_cost, base_cost + delta*euclidian_distance(frontiers.T, alloc_cluster_coords)) #eq.6, case1

                    explopoint = frontiers[np.argmin(fcosts)] #eq. 7
                    selfrobot.target  = (int(explopoint[0]), int(explopoint[1]))
                    selfrobot.last_plan_time = selfrobot.env.step
                else:
                    selfrobot.rdvstate = "rendezvous"
                #print(selfrobot.target)

            #explo plus proche, OU ALORS, on garde le kmeans et on fait un predict sur les nouvelles frontieres du sobel???

            #explore until rdv time limitation

        def rendezvous_subbehavior():

            # Set a goal point?
            #check if others are here (with the rdv time limitation)
            missing_robot = []
            if selfrobot.env.step < 10: #pour le tout debut de mission, que les robots aient le temps de se donner l'info qu'ils existent^^
                return
        
            for robot in selfrobot.belief_space["robot_informations"]:
                #if (euclidian_distance((selfrobot.transform.x, selfrobot.transform.y), selfrobot.belief_space["robot_informations"][robot]["position"])<selfrobot.communication_range/2 ) and  (selfrobot.belief_space["robot_informations"][robot]["step"] > selfrobot.env.step - 40): #we check that the robot is actually here and not an old position
                if (selfrobot.belief_space["robot_informations"][robot]["step"] > selfrobot.env.step - 20): #test de condition sans distance, juste info recente
                    continue
                else:
                    missing_robot.append(robot) #if it's not here, he is missing at this point.
            if len(missing_robot) > 0 and euclidian_distance((selfrobot.transform.x, selfrobot.transform.y), selfrobot.rdvspot) > selfrobot.communication_range/2: #goto rdv 
                #selfrobot.treshold_for_target = selfrobot.communication_range/2
                selfrobot.target = selfrobot.rdvspot
                return

            if len(missing_robot) == 0 or selfrobot.rdvtime <= selfrobot.env.step :
                #then with the robots that are here : 
                #identify actions and explo clusters
                if selfrobot.bid == None or (selfrobot.env.step - selfrobot.bid["step"]) >= 200:#no bid, or bid too old.
                    frontier_bids = {}
                    artifacts_bids = {}
                    frontiers = find_frontier_cells(selfrobot.belief_space["occupancy_grid"], traversable_types = selfrobot.traversable_types)
                    if len(frontiers) > len(selfrobot.belief_space["robot_informations"]):
                        
                        selfrobot.current_clustering = cluster_env() #KMeans cluster object
                        cluster_centers = selfrobot.current_clustering.cluster_centers_ 

                        for cc in cluster_centers:
                            cost = euclidian_distance((selfrobot.transform.x, selfrobot.transform.y), (int(cc[0]), int(cc[1])))
                            frontier_bids.update({(int(cc[0]), int(cc[1])):(1/cost)}) #then high cost will make a small bid.
                    if "artifacts" in selfrobot.belief_space:
                        robots_in_rdv = len(selfrobot.belief_space["robot_informations"]) - len(missing_robot)
                        sorted_arts = dict(sorted(selfrobot.belief_space["artifacts"].items(), key=lambda item: item[1]["discovery_time"]))
                        chosen_arts = []
                        sum_tasks = 0
                        for art in sorted_arts:
                            if selfrobot.belief_space["artifacts"][art]["status"] not in ["destroyed", "done"]:
                                if sum_tasks + selfrobot.belief_space["artifacts"][art]["needed_robots"] <=robots_in_rdv :
                                    chosen_arts.append(art)
                                    sum_tasks += selfrobot.belief_space["artifacts"][art]["needed_robots"]
                                else:
                                    break
                        

                        for art in chosen_arts: #selfrobot.belief_space["artifacts"]:
                            #check capability
                            if selfrobot.belief_space["artifacts"][art]["status"] not in ["destroyed", "done"]:
                                type = selfrobot.belief_space["artifacts"][art]["type"]
                                coords = selfrobot.belief_space["artifacts"][art]["coordinates"]
                                capability = selfrobot.competences[type]["capability"]
                                cost = euclidian_distance((selfrobot.transform.x, selfrobot.transform.y), (int(coords[0]), int(coords[1]))) #+ selfrobot.belief_space["artifacts"][art]["discovery_time"]
                                #print(capability)
                                artifacts_bids.update({art:(capability/cost)})
                    selfrobot.bid = {"frontiers":frontier_bids, "artifacts":artifacts_bids, "step": selfrobot.env.step}
                    selfrobot.belief_space["robot_informations"][selfrobot.robot_id].update({"bids":selfrobot.bid})
                    #avec ca, la communication devrait automatiquement partager les bids, vu que les robots envoient leurs infos du belief space en entier.
                    #print(selfrobot.bid)
                missing_bids = False
                for robot in selfrobot.belief_space["robot_informations"]: #TODO : Harmoniser les bids entre les robots.
                    if robot in missing_robot:
                        continue
                    if not("bids" in list(selfrobot.belief_space["robot_informations"][robot].keys())):
                        missing_bids = True
                        break
                    elif selfrobot.belief_space["robot_informations"][robot]["bids"]["step"] <= selfrobot.env.step - 100:
                            missing_bids = True
                            break
                if not missing_bids: #here, we use hungarian algorithm, so we have a one-shot auction
                    #TODO : faire une matrice avec les bids des robot non-missing, puis identifier la tâche à effectuer pour le robot faisant le calcul.
                    robots_present = []
                    for r in selfrobot.belief_space["robot_informations"]:
                        if not(r in missing_robot):
                            robots_present.append(r)
                    #robots_present = [r for r in selfrobot.belief_space["robot_informations"] if r not in missing_robot]
                    tasks_frontiers = list(selfrobot.bid["frontiers"].keys())
                    #tasks_artifacts = list(selfrobot.bid["artifacts"].keys())
                    tasks_artifacts = []
                    #test multirobottask
                    for art in list(selfrobot.bid["artifacts"].keys()):
                        awared_robots = len(selfrobot.belief_space["artifacts"][art]["awared"])
                        needed_robots = selfrobot.belief_space["artifacts"][art]["needed_robots"]
                        if awared_robots >= needed_robots:
                            for i in range(needed_robots):
                                tasks_artifacts.append(art)
                    # Initialiser la matrice d'affectation (bids)
                    exploration_matrix = np.zeros((len(robots_present), len(tasks_frontiers)))
                    artifact_matrix = np.zeros((len(robots_present), len(tasks_artifacts)))
                    # Remplir la matrice avec les bids des robots pour chaque tâche
                    # print(tasks_artifacts)
                    # print(selfrobot.bid)
                    #print(selfrobot.belief_space["artifacts"])



                    for r in range(len(robots_present)): 
                        for t in range(len(tasks_artifacts)):
                            if tasks_artifacts[t] in selfrobot.belief_space["robot_informations"][robots_present[r]]["bids"]["artifacts"]:
                                artifact_matrix[r][t] =  selfrobot.belief_space["robot_informations"][robots_present[r]]["bids"]["artifacts"][tasks_artifacts[t]]



                    if len(tasks_artifacts) > 0:
                        artifact_indices = zip(*linear_sum_assignment(artifact_matrix, maximize=True))
                        for r_idx, t_idx in artifact_indices:
                            if robots_present[r_idx] == selfrobot.robot_id: #if robots_present[r_idx]+1 == selfrobot.robot_id:
                                task_id = tasks_artifacts[t_idx]
                                selfrobot.action_to_perform = {
                                    "id": task_id,
                                    "type": selfrobot.belief_space["artifacts"][task_id]["type"],
                                    "coordinates": selfrobot.belief_space["artifacts"][task_id]["coordinates"]
                                }
                                selfrobot.rdvstate = "exploit"
                                selfrobot.delta_replan = 2*selfrobot.env.width
                                selfrobot.target = selfrobot.action_to_perform["coordinates"]
                                selfrobot.bid = None
                                selfrobot.last_plan_time = selfrobot.env.step
                                #print(selfrobot.action_to_perform)
                                #print(selfrobot.target)
                                break
                    # print(f"robot id : {selfrobot.robot_id} : {selfrobot.action_to_perform}")
                    if len(tasks_frontiers) > 0:
                        bid_keys = {robot: list(selfrobot.belief_space["robot_informations"][robot]["bids"]["frontiers"]) for robot in robots_present}
                        all_keys = list(dict.fromkeys(tasks_frontiers + [k for keys in bid_keys.values() for k in keys]))
                        key_labels = dict(zip(all_keys, selfrobot.current_clustering.predict(all_keys)))
                        for i, robot in enumerate(robots_present):
                            robot_bids = selfrobot.belief_space["robot_informations"][robot]["bids"]["frontiers"]
                            for j, task in enumerate(tasks_frontiers):
                                if task not in robot_bids:
                                    for cluster in bid_keys[robot]:
                                        if key_labels[cluster] == key_labels[task]:
                                            exploration_matrix[i,j] = robot_bids[cluster]
                                else:
                                    exploration_matrix[i,j] = robot_bids[task]

                    if len(tasks_frontiers)==0 and len(tasks_artifacts)==0 : #if there is no frontier and no task anymore, we finish
                        selfrobot.rdvstate = "finish"
                        return

                    # explo
                    if len(tasks_frontiers) >0:
                        cluster_indices = zip(*linear_sum_assignment(exploration_matrix, maximize=True))
                        for r_idx, t_idx in cluster_indices:
                            if robots_present[r_idx] == selfrobot.robot_id:
                                task_id = tasks_frontiers[t_idx]
                                selfrobot.allocated_cluster = task_id
                                if selfrobot.rdvstate != "exploit":
                                    selfrobot.rdvstate = "explore"
                                    selfrobot.bid = None
                                break
                    
                    
                    #SETUP NEXT RDV SPOT
                    
                    # print(selfrobot.rdvstate)

                    partition = np.zeros(selfrobot.belief_space["occupancy_grid"].shape, dtype=int)

                    # partition[selfrobot.belief_space["occupancy_grid"] == -1] = selfrobot.current_clustering.labels_ + 1
                    frontiers = find_frontier_cells(selfrobot.belief_space["occupancy_grid"], traversable_types=selfrobot.traversable_types)

                    if len(frontiers) > 0 and selfrobot.current_clustering is not None:

                        frows = [f[0] for f in frontiers]
                        fcols = [f[1] for f in frontiers]
                        partition[frows, fcols] = -1

                        unknown_mask = partition == -1
                        unknown_coords = np.argwhere(unknown_mask)  # cellules inconnues actuelles

                        partition[unknown_mask] = selfrobot.current_clustering.predict(unknown_coords) + 1 # test
                        #je traduis du mieux que je peux le code matlab de bramblett sur le gitub. Elle a l'air de faire une moyenne ponderee des centroides
                        #par la taille des partitions.

                        
                        unk_part = partition[partition != 0] 
                        labels, a_counts = np.unique(unk_part, return_counts=True)
                        c_loc = selfrobot.current_clustering.cluster_centers_ 

                        np_rdvspot = np.round(np.sum(c_loc[labels - 1] * a_counts[:, np.newaxis], axis=0) / len(unk_part)).astype(int) #TODO, il faut que je ne prenne plus en compte les zones inconnues inaccessibles. Il faudrait que je les purge en fait...
                        selfrobot.rdvspot = (int(np_rdvspot[0]), int(np_rdvspot[1]))
                        if not(selfrobot.belief_space["occupancy_grid"][selfrobot.rdvspot] in selfrobot.traversable_types):
                            selfrobot.rdvspot = find_nearest_free(selfrobot.belief_space["occupancy_grid"], selfrobot.rdvspot, traversable_types=selfrobot.traversable_types) #si le rdv est un mur ou une case inconnue, alors, on 
            
                    
                    #Note :  je fais les rdv de manière decentralisee, normalement chaque robot attends d'avoir l'info que les autres sont dans le cluster
                    #donc EN THEORIE tout le monde a la meme map, les clusters et donc les points de rdv devraient etre les memes.....
                    #en pratique, on verra^^
                    selfrobot.rdvtime = selfrobot.env.step + np.max(selfrobot.belief_space["occupancy_grid"].shape)*2 #TODO maybe set a better incrementation value.
            
        def search_subbehavior():
            artifact_coordinates = selfrobot.belief_space["artifacts"][selfrobot.action_to_perform["id"]]["coordinates"]
            if euclidian_distance((int(selfrobot.transform.x), int(selfrobot.transform.y)), artifact_coordinates) < selfrobot.vision_range:
                selfrobot.rdvstate = "exploit"
            else:
                selfrobot.target = artifact_coordinates

        def exploit_subbehavior():
            if selfrobot.action_to_perform == None:
                selfrobot.rdvstate = "explore"
            #make the job until done or rdv time limitation

        def finish_subbehavior():
            return_home_or_finish(selfrobot)

        if not hasattr(selfrobot, "rdvstate"): #for initialisation, we set rdv originally
            selfrobot.rdvstate = "rendezvous"
            selfrobot.bid = None
            selfrobot.rdvspot = (int(selfrobot.transform.x), int(selfrobot.transform.y))
            selfrobot.rdvtime = 20
            selfrobot.allocated_cluster = None
            selfrobot.current_clustering = None

            #we have to cheat here cause this method requires the robot to know each other at the beggining of the mission.
            for r in selfrobot.env.agents:
                if r.robot_id != selfrobot.robot_id:
                    BS_copy = deepcopy(selfrobot.belief_space)
                    r.recieve_belief(BS_copy)

        if selfrobot.rdvstate == "explore":
            explore_subbehavior()
        elif selfrobot.rdvstate == "rendezvous":
            rendezvous_subbehavior()
        elif selfrobot.rdvstate == "search":
            search_subbehavior()
        elif selfrobot.rdvstate == "exploit":
            exploit_subbehavior()
        elif selfrobot.rdvstate == "finish":
            finish_subbehavior()