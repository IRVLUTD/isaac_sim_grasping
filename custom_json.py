import numpy as np
import pandas as pd 
import os

if __name__ == "__main__":
    # Use complete paths due to python.sh directory
    path_to_json = "/home/felipe/Documents/isaac_sim_grasping/grasp_data"
    json_files = [pos_json for pos_json in os.listdir(path_to_json) if pos_json.endswith('.json')]
    print(json_files)  

    #Samples to get from .json files
    samples= 5
    df = pd.DataFrame()
    for i in json_files:
        tmp = pd.read_json(os.path.join(path_to_json, i))
        if df.empty:
            df = tmp[:5]
        else:
            df = pd.concat([df,tmp[:5]],ignore_index= True)
    print(df)

    result = df.to_json(os.path.join(path_to_json, "Grasps_dataset.json"))
        #print(tmp.head(4))


