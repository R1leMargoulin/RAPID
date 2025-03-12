import os
import json

#For this program specifically, you will need matplotlib, pandas and seaborn installed
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import seaborn as sns




result_file_path = "/home/erwan/Documents/tests_simulations/RAPID/experiments/" #CHANGE PATH

#example for an empty env 100x100, where we just variated the number of heterogeneous robot using a nearest frontier behavior with a blackboard com method.

experiment_group_name = "empty_env_100x100" #name of the folder where all datafiles are

experiments_filenames = os.listdir(result_file_path+experiment_group_name) #get all files in the group of experiments.


nb_steps_df = pd.DataFrame({
    "robot_nb" : [],
    "total_steps": []
})



for e in experiments_filenames:
    filename = result_file_path+experiment_group_name+"/"+e
    #open the json file, place it in a json obj
    with open(filename, "r") as file:
        result_json = file.read()
    result_obj = json.loads(result_json)

    robot_number = None
    for sim in result_obj : #for all simulations
        robot_number = len( result_obj[sim]["agents_data"].keys() ) #we take the amount of robots
        total_steps = result_obj[sim]["total_steps"] #we take the total_steps made until the environment's goal is reached.
        
        data = {"robot_nb":robot_number, "total_steps": total_steps } #then we add the data to our dataframe
        nb_steps_df.loc[len(nb_steps_df)] = data



#we display the data in a lineplot.
sns.set_theme()
sns.lineplot(nb_steps_df, x="robot_nb", y="total_steps") 
plt.show()
