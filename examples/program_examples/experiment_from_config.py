from RAPID.Environment import Environment, TargetPointEnvironment, ExplorationEnvironment #import of the different environments
from RAPID import Agents

from PIL import Image

import os
import logging
import random
import json
import itertools


config_file = "/home/erwan/Documents/tests_simulations/RAPID/examples/config_examples/exp_config.json"

with open(config_file, "r") as outfile:
    json_from_file = outfile.read()
    
config =  json.loads(json_from_file)


def main():
    VARIABLES_TO_VARIATE = ["NB_GROUND_AGENTS", "GROUND_AGENTS_COMMUNICATION_RANGE", "GROUND_AGENTS_COMMUNICATION_FREQUENCY"] #ce sont des tableaux dans le fichier de config.

    

    total_number_of_experiments =  1
    for f in VARIABLES_TO_VARIATE:
        total_number_of_experiments *= len(config[f])

    #TODO mettre dans des variables et afficher dans la boucle
    print(f"number of experiments : {total_number_of_experiments}")
    print(f"number of simulations : {total_number_of_experiments * config["NB_SIMULATION"]}")

    total_number_of_simulations = total_number_of_experiments * config["NB_SIMULATION"]

    # Générer toutes les combinaisons d'expériences
    experiment_combinations = list(itertools.product(*[config[var] for var in VARIABLES_TO_VARIATE]))

    #print(experiment_combinations)

    #create the main folder
    if not os.path.exists(config["RESULT_PATH"]+config["GROUP_EXPERIMENT_NAME"]):
            os.mkdir(config["RESULT_PATH"]+config["GROUP_EXPERIMENT_NAME"])

    experiment_counter = 0
    simulation_counter = 0
    # Afficher chaque combinaison d'expérience
    for i, combination in enumerate(experiment_combinations):
        experiment_counter += 1
        config_iteration = config.copy()
        config_iteration.update(dict(zip(VARIABLES_TO_VARIATE, combination)))
        # print(f"Experiment {i+1}: {dict(zip(VARIABLES_TO_VARIATE, combination))}")
        
        #FILE MAKING

        #EXPERIMENT NAME SPECIFICITY
        exp_specificity = ""
        for f in list(zip(VARIABLES_TO_VARIATE, combination)):
            exp_specificity += str(f[0])+str(f[1]) + "_"
        
        exp_specificity = exp_specificity.removesuffix("_")

        #create the experiment folder
        if not os.path.exists(config_iteration["RESULT_PATH"]+config_iteration["GROUP_EXPERIMENT_NAME"]):
            os.mkdir(config_iteration["RESULT_PATH"]+config_iteration["GROUP_EXPERIMENT_NAME"])

        #env Image loading
        if config_iteration["ENV_IMAGE_PATH"]:
            img = Image.open(config_iteration["ENV_IMAGE_PATH"]).convert("L")
        else:
            img = None

        logging.basicConfig(level=logging.INFO)
        #print(config_iteration)

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
                                        caption= config_iteration["EXPERIMENT_NAME"],
                                        background_color= config_iteration["BACKGROUND_COLOR"],
                                        env_image=img,
                                        limit_of_steps=config_iteration["STEP_LIMITATION"],
                                        scaling_factor=config_iteration["SCALING_FACTOR"],
                                        communication_mode=config_iteration["COMMUNICATION_MODE"],
                                        ) #example of exploration environment with rendering (there is rendering by default)
            
            #---------------------------------

            #AGENTS Definition----------------
            for i in range(config_iteration["NB_GROUND_AGENTS"]):
                #random init position if it is not defined
                if config_iteration["GROUND_AGENTS_INIT_POSITION"] == None:
                    agents_init_pos = (random.randrange(0,env.width), random.randrange(0,env.height), random.uniform(0, 2*3.14)) 
                else:
                    agents_init_pos = config_iteration["GROUND_AGENTS_INIT_POSITION"]

                env.add_agent(Agents.Ground(
                                            env, 
                                            robot_id=len(env.agents)+1, 
                                            init_transform=agents_init_pos,
                                            max_speed=config_iteration["GROUND_AGENTS_MAX_SPEED"],
                                            behavior_to_use=config_iteration["GROUND_AGENTS_BEHAVIOR"],
                                            communication_range=config_iteration["GROUND_AGENTS_COMMUNICATION_RANGE"],
                                            communication_frequency=config_iteration["GROUND_AGENTS_COMMUNICATION_FREQUENCY"],
                                            vision_range=config_iteration["GROUND_AGENTS_VISION_RANGE"],
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
                                                                            "total_distance_made" : a.total_distance_made,
                                                                            "communication_range" : a.communication_range,
                                                                            "communication_frequency": a.communication_frequency,
                                                                            "vision_range": a.vision_range,
                                                                            "max_speed": (a.max_speed.x, a.max_speed.y, a.max_speed.w),
                                                                        }
                                                                    })
    
    with open(result_file_path, "r") as outfile:
        json_from_file = outfile.read()
    
    file_obj =  json.loads(json_from_file)
    file_obj.update(sim_data) #adding the simulation data to the global experiment data

    json_exp_data = json.dumps(file_obj, indent=4) 

    with open(result_file_path, "w") as outfile:
        outfile.write(json_exp_data) #save in the file




if __name__ == "__main__":
    main()