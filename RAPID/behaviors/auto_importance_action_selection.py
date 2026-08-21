from ..utils import *

def ai_action_selection(selfrobot): 
    #reshape importance of communication depending of the time from last communication:
    #print(f"robot {self.robot_id} : last com : {self.time_from_last_communication}")



    
    # fonction a part, je met un parametre en string "default" par defaut et un mode pour chaque expe tentée?
    importance_online_configuraton(selfrobot)

    #selfrobot.check_communication_importance()

    interest_points = [] #we will add all of our interest points here
    #interest points identification -----------------------------------------------------------
    #exploration frontiers ----------------------------
    frontiers = find_frontier_cells(selfrobot.belief_space["occupancy_grid"], traversable_types=selfrobot.traversable_types) #from utils
    if list(frontiers) != None or len(list(frontiers))!=0:
        cluster_centers = cluster_frontier_cells(selfrobot.belief_space["occupancy_grid"], frontiers, int(selfrobot.vision_range/2), traversable_types=selfrobot.traversable_types) #from utils : make cluster of fontiers to reduce computation time
        for cc in cluster_centers:
            interest_points.append({"type":"exploration","coordinates":cc, "needed_robots":1})
    #--------------------------------------

    #Artifacts ----------------------------
    if "artifacts" in selfrobot.belief_space:
        for art in selfrobot.belief_space["artifacts"]:
            #IMPORTANCE CHECK
            if art +1 <= len(selfrobot.env.interest_points["artifacts"]): #ouais c'est degueu
                selfrobot.env.interest_points["artifacts"][art].check_importance(selfrobot)

            #INTEREST POINT CREATION
            if selfrobot.belief_space["artifacts"][art]["status"] not in ["done", "destroyed"] :
                if euclidian_distance( (selfrobot.init_transform.x, selfrobot.init_transform.y) , selfrobot.belief_space["artifacts"][art]["coordinates"]) >= selfrobot.competences[selfrobot.belief_space["artifacts"][art]["type"]]["distance_treshold"]: #we verify that the treshold is respected
                    interest_points.append({"type": selfrobot.belief_space["artifacts"][art]["type"] ,
                                            "coordinates":selfrobot.belief_space["artifacts"][art]["coordinates"], 
                                            "id":art, 
                                            "needed_robots": selfrobot.belief_space["artifacts"][art]["needed_robots"],
                                            "step": selfrobot.belief_space["artifacts"][art]["step"],
                                            "discovery_time": selfrobot.belief_space["artifacts"][art]["discovery_time"],
                                            "awared": selfrobot.belief_space["artifacts"][art]["awared"]})#adding directly the artifacts in the interest points
    #--------------------------------------
    #------------------------------------------------------------------------------------------


    #barycentre de communications----------
    #liste de toutes les positions des robots

    robots_pos_list = [] #list of float xy position of all robots
    for robot_id in selfrobot.belief_space["robot_informations"]:
        if robot_id != selfrobot.robot_id and selfrobot.belief_space["robot_informations"][robot_id]["status"]!= "finishing":  # ComImportance : est ce que je ferais pas un truc spécifique aux robots?
            if selfrobot.communication_range*3/selfrobot.max_speed.x <= selfrobot.env.step - selfrobot.belief_space["robot_informations"][robot_id]["step"] < 4 * ( np.max(selfrobot.belief_space["occupancy_grid"].shape)/selfrobot.max_speed.x ) : #time = dist/speed
                if euclidian_distance((selfrobot.transform.x, selfrobot.transform.y) ,selfrobot.belief_space["robot_informations"][robot_id]["position"]) >=  selfrobot.competences["communication"]["distance_treshold"]:
                    robots_pos_list.append(selfrobot.belief_space["robot_informations"][robot_id]["position"])
        if len(robots_pos_list)>0:
            if euclidian_distance((selfrobot.transform.x, selfrobot.transform.y) , selfrobot.last_given_position) >=  selfrobot.competences["communication"]["distance_treshold"]:
                    robots_pos_list.append(selfrobot.last_given_position)# TODO ComInfo, la last given position, c'est a double trnchant, je sais pas trop

    communication_clusters = simple_clustering(robots_pos_list, selfrobot.communication_range) #from utils: make simple clusters of robot based on communication range, will return the center of clusters
    for cc in communication_clusters:
            #if euclidian_distance( (selfrobot.init_transform.x, selfrobot.init_transform.y) , cc) >= selfrobot.competences["communication"]["distance_treshold"]: #we verify that the distance treshold is respected
            interest_points.append({"type":"communication","coordinates":cc, "needed_robots":1})#adding those clusters in the communication points
            #TODO MultiRobotTask : try different values of needed robots


    #--------------------------------------        

    #if we have no interest point anymore, we consider the mission done.*
    if len(interest_points) == 0: # or (len(interest_points)==1 and interest_points[0]["type"] == "base_station_com"):
        if euclidian_distance((int(selfrobot.transform.x),int(selfrobot.transform.y)), (int(selfrobot.init_transform.x),int(selfrobot.init_transform.y))) > selfrobot.treshold_for_target:
            selfrobot.status = "finishing"
            selfrobot.target = (int(selfrobot.init_transform.x),int(selfrobot.init_transform.y))
            selfrobot.last_plan_time = selfrobot.env.step
            return None
        else:
            selfrobot.finish()
            return None

    #utility calculation-----------------------------------------------------------------------            
    for ip in interest_points:
        #individual utility
        #cost = euclidian_distance(ip["coordinates"], (selfrobot.transform.x, selfrobot.transform.y)) #euclidian distance for the moment (C in the model)
        #cost = a_star_cost(selfrobot.belief_space["occupancy_grid"], (int(selfrobot.transform.x), int(selfrobot.transform.y)), (int(ip["coordinates"][0]), int(ip["coordinates"][1])), selfrobot.env_ease, traversable_types=selfrobot.traversable_types)
        cost = cost_distance_calculation((int(selfrobot.transform.x), int(selfrobot.transform.y)), (int(ip["coordinates"][0]), int(ip["coordinates"][1])), selfrobot.belief_space["occupancy_grid"], selfrobot.env_ease,  traversable_types=selfrobot.traversable_types, mode = selfrobot.cost_calculation_mode)
        if cost < 1:
            cost = 1 #avoid divide by 0

        capability = selfrobot.competences[ip["type"]]["capability"] #I'll cnsider that the type of the IP will be named the same than the competence (mu in the model)

        individual_utility = capability/cost
        #global feasability
        other_individual_values = np.array([])
        for robot in selfrobot.belief_space["robot_informations"]: #the key value of this dict is robot id
            if ip["type"] == "communication": #if the task is communication, no need to add collective sufficiency, we'll place it to (our own util, the task will depend only on importance).
                #other_individual_values = np.append(other_individual_values, individual_utility)
                other_individual_values = np.append(other_individual_values, 1.0)
                break
            #TODO TEST cette modif
            if len(selfrobot.belief_space["robot_informations"]) <=1: #if the robot believes he's alone
                other_individual_values = np.append(other_individual_values, 1.0)
                break
            if robot == selfrobot.robot_id :
                continue
            else:
                #ligne de l'enfer sorry
                other_robot_pos = (int(selfrobot.belief_space["robot_informations"][robot]["position"][0]),int(selfrobot.belief_space["robot_informations"][robot]["position"][1]))

                #ocost = euclidian_distance(ip["coordinates"], other_robot_pos)
                ocost = cost_distance_calculation(other_robot_pos, (int(ip["coordinates"][0]), int(ip["coordinates"][1])), selfrobot.belief_space["occupancy_grid"], selfrobot.belief_space["robot_informations"][robot]["env_ease"],  traversable_types=selfrobot.belief_space["robot_informations"][robot]["traversable_types"], mode = selfrobot.cost_calculation_mode)
                if ocost <1:
                    ocost = 1 #avoid divide by 0

                ocapability = selfrobot.belief_space["robot_informations"][robot]["competences"][ip["type"]]["capability"]

                oobsolecence = selfrobot.env.step - selfrobot.belief_space["robot_informations"][robot]["step"] 

                #TODO, MultiRobotTask check ca
                if ip["needed_robots"] <=1:
                    other_individual_values = np.append(other_individual_values, (ocapability/ocost))
                else:
                    #selfrobot.belief_space["robot_informations"][robot]["step"]
                    if  robot in ip["awared"] : #check if the robot knows about the task or not
                        other_individual_values = np.append(other_individual_values, (ocapability/ocost))
            
                    #capacite des autres sur l'ip 
        
        #collective_sufficiency = float(np.max(other_individual_values)) #backup

        #for MultiRobotTask::
        if len(other_individual_values) >= ip["needed_robots"]:
            collective_sufficiency = float(max_k(other_individual_values, ip["needed_robots"]))
        elif len(other_individual_values) == ip["needed_robots"]-1:
            collective_sufficiency = 1 #je fais ca parce que sinon je peux pas calculer le max_k, mais ca revient au meme  meme "si je suis le plus loin"
        else:
            collective_sufficiency = np.inf
        #collective_sufficiency = testproduct

        if collective_sufficiency == 0:
            collective_sufficiency = 1e-8 #avoid divide by 0



        bests_others = []
        required_assist = 0
        if ip["needed_robots"] > 1: 
            # #required_assist = 1
            if len(other_individual_values) >= ip["needed_robots"]-1:
                nbcloser = 0
                for i in range (len(selfrobot.belief_space["robot_informations"])):
                    value = float(max_k(other_individual_values, i+1))
                    if i < ip["needed_robots"] -1:
                        bests_others.append(value)
                    #print(f"value indiv : {individual_utility} /// Other : {value}")
                    if value > individual_utility:
                        nbcloser+=1
                #print(nbcloser)
                if nbcloser >= ip["needed_robots"]:
                    required_assist = -np.inf
                else:
                    required_assist = float(np.sum(bests_others)) #- ((selfrobot.env.step - ip["discovery_time"]))/selfrobot.env.step) #- (1+nbcloser - ip["needed_robots"])) * ((selfrobot.env.step - ip["discovery_time"])/selfrobot.env.step) #TODO ajuster le delta discovery
                    #required_assist = (float(np.sum(bests_others)) - (1+nbcloser - ip["needed_robots"])) * ((selfrobot.env.step - ip["discovery_time"])/selfrobot.env.step) #TODO ajuster le delta discovery
            else:
                required_assist = - np.inf
            
            #required_assist = 1 + float(max_k(other_individual_values, ip["needed_robots"] -1)) #equivalent to the commented above...
        

        # if required_assist !=0:
        #     required_assist += selfrobot.env.step - ip["step"]
    
        
        utility = ((individual_utility + required_assist) / (collective_sufficiency ))
        #utility = ( selfrobot.competences[ip["type"]]["importance"] * individual_utility) / collective_sufficiency

        ip.update({"utility":utility})
        #ip.update({"utility":collective_utility})

    #------------------------------------------------------------------------------------------
    best_action = None
    best_weighted_utility = -np.inf
    for ip in interest_points:
        #tuning params-----------------------------------------------------------------------------
        weighted_utility = ip["utility"] * selfrobot.competences[ip["type"]]["importance"]

        if weighted_utility >= best_weighted_utility:
            best_weighted_utility = weighted_utility
            best_action = ip
    
    #action perform
    if best_action != None:
        selfrobot.action_to_perform = best_action
        selfrobot.target = (int(selfrobot.action_to_perform["coordinates"][0]), int(selfrobot.action_to_perform["coordinates"][1]))
        selfrobot.last_plan_time = selfrobot.env.step
    else:
        print("problem")


def importance_online_configuraton(selfrobot):
    #TODO Faire une satisfaction de la completion de la tache???

    satisfactions = {}
    G_satisfaction = 0
    for task_type in selfrobot.competences:
        task_satisfaction = selfrobot.competences[task_type]["satisfaction_calculation"](selfrobot)
        satisfactions.update({task_type: task_satisfaction})
        pass # TODO
        G_satisfaction += task_satisfaction

    # TODO REWARD WITH THE OLD_G_SATISFACTION, STILL NEED TO INIT THE FIRST

    #TODO Faire le calcul de nouvelle importance a t en fonction
    importance = 0 #TODO

    #TODO MAJ de l'importance, a adapter...
    capability = selfrobot.competences[CHANGER]["capability"] #same, doesnt change
    distance_treshold = selfrobot.competences[CHANGER]["distance_treshold"]
    selfrobot.shape_competence("communication", capability=capability , importance=importance, distance_treshold=distance_treshold)


def cost_distance_calculation(pointA, pointB, grid, env_ease, traversable_types, mode="euclidian"):
    if mode == "astar":
        return float(a_star_cost(grid, pointA, pointB, env_ease, traversable_types))
    elif mode == "euclidian":
        return float(euclidian_distance(pointA, pointB))
    elif mode == "manhathan":
        return float(manhathan_distance(pointA, pointB))
