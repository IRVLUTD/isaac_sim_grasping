import numpy as np
import pandas as pd 
import os

if __name__ == "__main__":
    # Use complete paths due to python.sh directory
    path_to_json = "/home/felipe/Documents/cmp"
    output_path = "/home/felipe/Documents/tests"
    json_files = [pos_json for pos_json in os.listdir(path_to_json) if pos_json.endswith('.json')]
    print(json_files)  
    #json_files = ["/home/felipe/Documents/isaac_sim_grasping/grasp_data/out.json"]
    #Samples to get from .json files
    samples= 100
    df = pd.DataFrame()
    for i in json_files: 
        tmp = pd.read_json(os.path.join(path_to_json, i))
        print(i)

        result = tmp[:samples].to_json(os.path.join(output_path, i))
        if df.empty:
            df = tmp[:samples]
        else:
            df = pd.concat([df,tmp[:samples]],ignore_index= True)
    for i in range(20):
        #print(df.iloc[i]['gripper'])
        #print(df.iloc[i]['grasps'])
        pass
        
    
    
    print(len(df))
    zero = np.zeros(len(df))
    print(not any(zero))
    for i,r in df.iterrows():
        print(r["grasps"]["dofs"])

    df.to_json(output_path)
    


