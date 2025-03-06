from RAPID.ExperimentMaker import ExperimentMaker


config_file = "/home/erwan/Documents/tests_simulations/RAPID/config_file.json"

exp = ExperimentMaker(config_file)

exp.make_experiment()