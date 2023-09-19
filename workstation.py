import numpy as np
import pandas as pd
from manager import Manager

class Workstation():

    def __init__(self, manager):
        self.manager = manager
        self.job = pd.DataFrame()

    def step(self):
        self.job = self.manager.request_job()
        return self.job


if __name__ == "__main__":
    json_path = "/home/felipe/Documents/isaac_sim_grasping/grasp_data/Grasps_dataset.json"
    grippers_path = "/home/felipe/Documents/isaac_sim_grasping/grippers"
    objects_path = "/home/felipe/Documents/isaac_sim_grasping/objects"

    G_manager = Manager(json_path, grippers_path, objects_path)
    w1 = Workstation(G_manager)
    w2 = Workstation(G_manager)

    print(w1.step())
    print(w2.step())
    print(w2.step())
    print(w2.step())
    print(G_manager.task_pointer)
