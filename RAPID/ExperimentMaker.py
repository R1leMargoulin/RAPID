from .Environment import Environment, TargetPointEnvironment
from .Agents import Robot, Ground, Aerial

AVAILABLE_ROBOT_TYPE = ["Ground", "Aerial"]

import os
import json
import logging
import random
from PIL import Image

class ExperimentMaker():
    def __init__(self, config_file):
        #existing_metrics_list = ["agents_travel_distance", "goal_completion_steps", "global_distance", "distance_per_agents"]#TODO, a voir si on a besoin de ca.

        with open(config_file, 'r') as file:
            self.config = json.load(file)
        
        #verification of the config content:
        if(not(("name" in self.config) and ("env_image" in self.config or "env_size" in self.config) and "agents" in self.config)):
            logging.error(f"you need in your configuration at least : \n - name\n - env_image or env_size\n - agents\n\n Those requirements are not respected in the given configuration")
            exit()
        
        if "name" in self.config:
            self.name = self.config["name"]
        else :
            logging.error("'name' missing in the config file")
            exit()
        
        if "agents" in self.config:
            self.agent_groups = self.config["agents"]
        else :
            logging.error("'agents' missing in the config file")
            exit()

        if "env_image" in self.config :
            self.image = Image.open(self.config["env_image"]).convert("L")
        elif "env_size" in self.config:
            self.env_size = self.config["env_size"]
        else:
            logging.error("'env_image' or 'env_size' missing in the config file.")
            exit()
        
        if "env_type" in self.config:
            if(self.config["env_type"] == "TargetPointEnvironment"): #TODO : a voir comment on implemente ca en vrai
                self.env_type = "TargetPointEnvironment"
            else:
                logging.error("env_type not known.")
                exit()
        else:
            logging.error("'env_type' missing in the config file.")
            exit()

        if("nb_simulations" in self.config):
            self.nb_simulations = self.config["nb_simulations"]
        else : 
            self.nb_simulations = 1

        if "result_path" in self.config:
            self.result_file_path = f"{self.config["result_path"]}{self.name}.json"

        if "limit_of_steps" in self.config:
            self.step_limit = self.config["limit_of_steps"]
        else:
            self.step_limit = 10000

        self.env = None

    def make_experiment(self):
        
        for i in range(self.nb_simulations):
            self.init_simulation()
            print(f"run simulation : {i+1}")
            self.run_simulation()
            print(f"took {self.env.step} steps, goal reached : {self.env.goal_condition()}")
            print(f"simulation {i+1} finished, adding data to the experiment file.")
            self.create_simulation_data(exp_number=i+1)
        print(f"Experiment done, data saved in {self.result_file_path}")


    def init_simulation(self):
        #Environment setup
        if(self.env_type == "TargetPointEnvironment"):
            amount_of_agents_needed = 1
            if "env_params" in self.config:
                if "amount_of_agents_on_goal" in self.config["env_params"]:
                    amount_of_agents_needed = self.config["env_params"]["amount_of_agents_on_goal"]

            if self.image:
                self.env = TargetPointEnvironment(env_image=self.image, amount_of_agents_goal=amount_of_agents_needed, limit_of_steps=self.step_limit)
            else:
                self.env = TargetPointEnvironment(width=self.env_size[0], height=self.env_size[1], amount_of_agents_goal=amount_of_agents_needed, limit_of_steps=self.step_limit)



        for agent_type in self.agent_groups:
            #verification of agent config content
            if "type" in self.agent_groups[agent_type]:
                type = self.agent_groups[agent_type]["type"]
            else : 
                logging.error(f"type missing in {agent_type} configuation")
                exit()

            if "parameters" in self.agent_groups[agent_type]:
                if "color" in self.agent_groups[agent_type]["parameters"]:
                    color = tuple(self.agent_groups[agent_type]["parameters"]["color"])
                else: 
                    color = None

                if "init_transform" in self.agent_groups[agent_type]["parameters"]:
                    init_transform = self.agent_groups[agent_type]["parameters"]["init_transform"]
                else: 
                    init_transform = None
                
                if "max_speed" in self.agent_groups[agent_type]["parameters"]:
                    max_speed = self.agent_groups[agent_type]["parameters"]["max_speed"]
                else: 
                    max_speed = None
                if "behavior_to_use" in self.agent_groups[agent_type]["parameters"]:
                    behavior_to_use = self.agent_groups[agent_type]["parameters"]["behavior_to_use"]
                else: 
                    behavior_to_use = None


            for agent in range(self.agent_groups[agent_type]["amount"]): #TODO peut etre mettre ca dan une fonction add_agents a partir des agents
                if init_transform == "random":
                    init_transform = (random.randrange(0,self.env.width), random.randrange(0,self.env.height), random.uniform(0, 2*3.14))

                if type == "Ground":
                    self.env.add_agent(Ground(
                                                self.env, 
                                                robot_id=len(self.env.agents)+1,
                                                color=color,
                                                init_transform=tuple(init_transform),
                                                max_speed=tuple(max_speed),
                                                behavior_to_use=behavior_to_use
                                            ))
                elif type == "Aerial":
                    pass
                else:
                    logging.error(f"type of robot {agent_type} is not known")

    def run_simulation(self):
        self.env.run()

    def create_simulation_data(self, exp_number:int,): #TODO, passer en CSV sera plus opti si on a beaucoup de données
        if exp_number==1: #if file doesn't exists:
            with open(self.result_file_path, "w") as outfile:
                outfile.write(json.dumps({})) #then we create an empty json file

        exp_data = {
            f"experiment{exp_number}":{ 
            "total_steps":self.env.step, 
            "goal_reached":self.env.goal_condition(),
            "agents_data":{}
            }
        }
        for a in self.env.agents:
            exp_data[f"experiment{exp_number}"]["agents_data"].update({
                f"robot_{a.robot_id}":{
                    "total_distance_made" : a.total_distance_made
                }
            })
        
        with open(self.result_file_path, "r") as outfile:
            json_from_file = outfile.read()
        
        file_obj =  json.loads(json_from_file)
        file_obj.update(exp_data) #adding the simulation data to the global experiment data

        json_exp_data = json.dumps(file_obj, indent=4) 

        with open(self.result_file_path, "w") as outfile:
            outfile.write(json_exp_data) #save in the file