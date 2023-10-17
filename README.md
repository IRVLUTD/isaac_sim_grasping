# isaac_sim_grasping
Graspit Grasping Filtering in Isaac Sim

# Command to run standalone:
 ./python.sh ~/Documents/isaac_sim_grasping/set_upv0.py

Notes: 
- You can change the command to the different set_up files we have. set_upv0 uses Cloner and set_upv1 creates the multiple grippers within one environment
- Need to be from the issac sim directory
- The set up files contain global directories that are not yet implemented as args (To use you need to change manually within the file)
- Deactivate conda if you have an active environment, it may cause some errors when running isaac sim
- Always use complete paths, errors may occur otherwise

# Gripper Adding
The .usd grippers must be tested and made ready for the use within the simulation. To do so the user_examples folder was added to the repository. You should copy it to "~/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/isaac/examples" that way it can be loaded and used to test .usd files faster with the GUI. To load in the GUI go to the Isaac Examples tab >  ScratchPad > Testing.

Note:
- Readability is very fragile for examples in Isaac Sim, you should only modify Test.py.
- Don't use the stop button to reset example, instead use the reset below loading example.

## Isaac Sim Manual
https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html

## Helpful code for python standalones (code snippet samples)
https://docs.omniverse.nvidia.com/isaacsim/latest/reference_python_snippets.html

