First, install Anaconda

Then, in the terminal run
```bash
conda create -n pb_elena python conda-forge::pybullet
```
to build the Python requirements for using PyBullet. Activate the new environment you've built using

```bash
conda activate pb_elena
```


Now, running the example (python .\load_robot_pb.py) should show the robot being simulated (and dropping to the floor instantly, whoops)



# Install gemini
---
```bash
conda create --name genesis python==3.10 
conda activate genesis
pip install git+https://github.com/Genesis-Embodied-AI/Genesis.git
pip install torch torchvision torchaudio 
```