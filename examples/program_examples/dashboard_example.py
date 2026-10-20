"""Reduced simulation example with the dashboard (4 ground robots, alpha adaptation action selection).

Keys in the dashboard window : 1 importances, 2 tasks, 3 knowledge, 4 all, left/right change robot,
p pause display, n normalise alphas, t true positions of the believed robots.
The dashboard runs in a separate process (spawn) : the `if __name__ == "__main__":` guard is mandatory.
"""
import os
from PIL import Image
from RAPID.Environment import ExplorationEnvironment
from RAPID import Agents

ENV_IMAGE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "env_images_example", "labyrinth_100_100.png")
POS_LIST = [[5, 5, 0], [5, 9, 0], [5, 13, 0], [5, 17, 0]]


def main():
    img = Image.open(ENV_IMAGE_PATH).convert("RGB")
    env = ExplorationEnvironment(env_image=img, full_knowledge=False, scaling_factor=4, communication_mode="limited",
                                 render=True, end_at_full_exploation=False,
                                 display_dashboard=True, dashboard_mode="knowledge", dashboard_period=10)
    for pos in POS_LIST:
        env.add_agent(Agents.Ground(env, robot_id=len(env.agents) + 1, init_transform=pos, max_speed=(1, 1, 0.5),
                                    behavior_to_use="alpha_adapt_action_selection", vision_range=25,
                                    communication_range=20, communication_period=5, energy_amount=1e6, delta_replan=30))
        env.agents[-1].shape_competence("exploration", 1, 1)
    env.run()


if __name__ == "__main__":
    main()
