from RAPID.Environment import Environment, TargetPointEnvironment, ExplorationEnvironment #import of the different environments
from RAPID import Agents

from PIL import Image

import os
import logging
import random
import json
import itertools



config_file = "/home/erwan/Documents/tests_simulations/RAPID/examples/config_examples/config_variation_heterogeneous_example.json"

with open(config_file, "r") as outfile:
    json_from_file = outfile.read()
    
config =  json.loads(json_from_file)

def get_param_values(config, param_path):
    keys = param_path.split('/')
    value = config
    for key in keys:
        value = value[key]
    return value

def set_param_value(config, param_path, value):
    keys = param_path.split('/')
    current = config
    for key in keys[:-1]:
        current = current[key]
    current[keys[-1]] = value


def main():
    PARAM_VARIATION = config["PARAM_VARIATION"]

    param_values = [get_param_values(config, param) for param in PARAM_VARIATION]

    total_number_of_experiments = 1
    for values in param_values:
        total_number_of_experiments *= len(values)

    #TODO mettre dans des variables et afficher dans la boucle
    print(f"number of experiments : {total_number_of_experiments}")
    print(f"number of simulations : {total_number_of_experiments * config["NB_SIMULATION"]}")
    print("continue ? Y/N")
    confirmation = input()
    if confirmation != "Y" and confirmation != "y":
        exit()

    total_number_of_simulations = total_number_of_experiments * config["NB_SIMULATION"]

    # Générer toutes les combinaisons d'expériences
    experiment_combinations = list(itertools.product(*param_values))


    #create the main folder
    if not os.path.exists(config["RESULT_PATH"]+config["GROUP_EXPERIMENT_NAME"]):
        os.mkdir(config["RESULT_PATH"]+config["GROUP_EXPERIMENT_NAME"])
    else:
        print("this group experiment already exists, are you sure? Y/N")
        confirmation = input()
        if confirmation == "N":
            exit()
        

    experiment_counter = 0
    simulation_counter = 0
    # Afficher chaque combinaison d'expérience    
    for combination in experiment_combinations:
        experiment_counter += 1
        config_iteration = config.copy()
        for param, value in zip(PARAM_VARIATION, combination):
            set_param_value(config_iteration, param, value)
        



        exp_specificity = "_".join([f"{param.replace('/', '_')}{value}" for param, value in zip(PARAM_VARIATION, combination)]) #genere le nom de l'experience

        #create the experiment folder
        if not os.path.exists(config_iteration["RESULT_PATH"]+config_iteration["GROUP_EXPERIMENT_NAME"]):
            os.mkdir(config_iteration["RESULT_PATH"]+config_iteration["GROUP_EXPERIMENT_NAME"])

        #env Image loading
        if config_iteration["ENV_IMAGE_PATH"]:
            img = Image.open(config_iteration["ENV_IMAGE_PATH"])
        else:
            img = None

        logging.basicConfig(level=logging.INFO)
        #print(config_iteration)

        #check init_pose for agents
        for group in config_iteration["AGENTS_GROUPS"]:
            if config_iteration["AGENTS_GROUPS"][group]["NB_AGENTS"] == len(config_iteration["AGENTS_GROUPS"][group]["INIT_POSITION"]):
                continue
            else:
                print(f"wrong initialisation of the init positions for group {group}. there should be the same number of init position as there is agents in the group")
                exit()



        #run loop of the experiments, you can change the environment type in the environment definition.
        for s in range(config_iteration["NB_SIMULATION"]):
            simulation_counter += 1
            print(f"experience {experiment_counter}/{total_number_of_experiments} || Simulation {simulation_counter}/{total_number_of_simulations}")
            #ENVIRONMENT DEFINITION------------

            #env = ExplorationEnvironment(width=SCREEN_WIDTH, height=SCREEN_HEIGHT, background_color= BACKGROUND_COLOR) #example of an empty exploration environment defined by size
            #env = TargetPointEnvironment(background_color= BACKGROUND_COLOR, env_image=img, amount_of_agents_goal=2) #example definition of a target point environment
            #env = ExplorationEnvironment(render= True, background_color= BACKGROUND_COLOR, env_image=img, full_knowledge=False) #example of exploration environment without rendering.

            
            env = ExplorationEnvironment(width=config_iteration["SCREEN_WIDTH"],
                                        height=config_iteration["SCREEN_HEIGHT"],
                                        render= config_iteration["RENDER"], 
                                        background_color= config_iteration["BACKGROUND_COLOR"],
                                        env_image=img,
                                        limit_of_steps=config_iteration["STEP_LIMITATION"],
                                        scaling_factor=config_iteration["SCALING_FACTOR"],
                                        communication_mode=config_iteration["COMMUNICATION_MODE"],
                                        end_at_full_exploation= config_iteration["END_AT_FULL_EXPLORATION"]
                                        ) #example of exploration environment with rendering (there is rendering by default)
            
            #---------------------------------

            #AGENTS Definition----------------
            # AGENTS Definition
            for group_name, group_config in config_iteration["AGENTS_GROUPS"].items():
                for i in range(group_config["NB_AGENTS"]):
                    agents_init_pos = group_config["INIT_POSITION"][i] if group_config["INIT_POSITION"] else (random.randrange(0, env.width), random.randrange(0, env.height), random.uniform(0, 2 * 3.14))
                    agent_class = getattr(Agents, group_config["AGENTS_TYPE"])
                    env.add_agent(agent_class(
                        env,
                        robot_id=len(env.agents) + 1,
                        init_transform=agents_init_pos,
                        max_speed=group_config["MAX_SPEED"],
                        behavior_to_use=group_config["BEHAVIOR"],
                        communication_range=group_config["COMMUNICATION_RANGE"],
                        communication_period=group_config["COMMUNICATION_PERIOD"],
                        vision_range=group_config["VISION_RANGE"],
                    ))
            #---------------------------------

            env.run()
            #make data from the sim.
            create_simulation_data(env, s+1, exp_specificity)
    
    

    #-------------------------------------------------------------------------------------------------------------------------------------------


def create_simulation_data(env, sim_number:int, exp_name): #TODO, passer en CSV sera plus opti si on a beaucoup de données
    result_file_path = config["RESULT_PATH"]+config["GROUP_EXPERIMENT_NAME"]+exp_name+".json"
    if sim_number==1: #if file doesn't exists:
        with open(result_file_path, "w") as outfile:
            outfile.write(json.dumps({})) #then we create an empty json file

    sim_data = { #Here are detailed the different env metrics.
        f"simulation{sim_number}":{ 
        "total_steps":env.step, 
        "goal_reached":env.goal_condition(),
        "agents_data":{},
        "communication_mode": env.communication_mode,
        "size":[env.width, env.height]
        }
    }
    for a in env.agents: #here are detailed the different agent metrics.
        sim_data[f"simulation{sim_number}"]["agents_data"].update({
                                                                        f"robot_{a.robot_id}":{
                                                                            "type":a.__class__.__name__,
                                                                            "total_distance_made" : a.total_distance_made,
                                                                            "communication_range" : a.communication_range,
                                                                            "communication_period": a.communication_period,
                                                                            "vision_range": a.vision_range,
                                                                            "max_speed": (a.max_speed.x, a.max_speed.y, a.max_speed.w),
                                                                            "BS_occupancy_grid": a.belief_space["occupancy_grid"]
                                                                        }
                                                                    })
    
    with open(result_file_path, "r") as outfile:
        json_from_file = outfile.read()
    
    file_obj =  json.loads(json_from_file)
    file_obj.update(sim_data) #adding the simulation data to the global experiment data

    json_exp_data = json.dumps(file_obj, indent=4) 

    with open(result_file_path, "w") as outfile:
        outfile.write(json_exp_data) #save in the file


def init_corner_positions(environment, total_nb_agent, num_agent, num_simulation, space_between_agents = 10):
    possible_spawns = [
        (0 + space_between_agents*total_nb_agent - space_between_agents*num_agent,0, 0), #top left
        (0, environment.height - space_between_agents*total_nb_agent + space_between_agents*num_agent, 0), #bottom_left
        (environment.width - space_between_agents*total_nb_agent + space_between_agents*num_agent, 0, 0),#top right
        (environment.width - space_between_agents*total_nb_agent + space_between_agents*num_agent, environment.height -space_between_agents*total_nb_agent, 0), #bot right
        (int(environment.width/2 + space_between_agents*num_agent), int(environment.height/2), 0), #center
    ]
    position = possible_spawns[num_simulation%len(possible_spawns)]
    return position



if __name__ == "__main__":
    main()