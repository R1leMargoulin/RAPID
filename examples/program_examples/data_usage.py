import json

#For this program specifically, you will need matplotlib, pandas and seaborn installed
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns




result_file_path = "/home/erwan/Documents/tests_simulations/RAPID/experiments/" #CHANGE PATH

#example for an empty env 100x100, where we just variated the number of heterogeneous robot using a nearest frontier behavior with a blackboard com method.

experiment_group_name = "empty_env_100x100"
experiments_filenames = ["1_robots.json", "2_robots.json", "3_robots.json", "4_robots.json", "5_robots.json"]



#let's plot the number of step for reaching the goal in function of the number of agents.
#I'll take the mean of all simulations ran for each experiments:

nb_steps_df = pd.DataFrame({
    "robot_nb" : [],
    "mean_steps_to_goal": []
})

for e in experiments_filenames:
    filename = result_file_path+experiment_group_name+"/"+e
    #open the json file, place it in a json obj
    with open(filename, "r") as file:
        result_json = file.read()
    result_obj = json.loads(result_json)

    number_of_simulations = len(result_obj.keys())

    robot_number = None
    total_steps = 0
    for sim in result_obj :
        robot_number = len( result_obj[sim]["agents_data"].keys() ) #TODO y'a peut etre plus propre
        total_steps += result_obj[sim]["total_steps"]

    mean_steps = total_steps/number_of_simulations
    data = {"robot_nb":robot_number, "mean_steps_to_goal": mean_steps }
    nb_steps_df.loc[len(nb_steps_df)] = data




sns.set_theme()
sns.barplot(nb_steps_df, x="robot_nb", y="mean_steps_to_goal") 
plt.show()
