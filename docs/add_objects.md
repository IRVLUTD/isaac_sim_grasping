# Adding Objects
Import the object to Isaac Sim with the use of the GUI and save it as .usd. Refer to the [Isaac Sim manual](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html) for more information about the supported formats. 

## Steps to Add objects:
1) Import the Object and save as .usd. Preferably using instanceable meshes.

![](https://github.com/IRVLUTD/isaac_sim_grasping/blob/main/media/AO1.png)


2) Prepare the .usd object by checking that the attributes are correct, i.e. colliders, correct masses or densities, etc.
3) Test the object's collider preferably by visualizing within the simulation.

![](https://github.com/IRVLUTD/isaac_sim_grasping/blob/main/media/AO2.gif)

Note: the directories used by Isaac Sim may become invalid when using objects transformed to .usds in other PCs, make sure to use relative directories instead of complete directories. (i.e Albedo maps in .usd files)

Important: the standalone checks for the objects .usd files within a directory with a specific structure: 

https://github.com/IRVLUTD/isaac_sim_grasping/blob/6b43f3369395127aa0e532aed0129e3df6c7b422/manager.py#L167C4-L179

