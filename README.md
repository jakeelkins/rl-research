# rl-research
Where I put all my research code for future reference. There's (going to be) three main veins:

## 42_work/
- where I built a python API to NASA's (Eric Stoneking of GSFC) attitude control simulator "42". This has been scrapped partially for now--couldn't get the fidelity in control I wanted based on the socket connectivity.

## ac_gym_v0/
- this is my custom OpenAI gym env for research how RL can control a spacecraft's attitude. v0 allows the agent to control the raw torque of the spacecraft in the +/- x, y, and z in the body-frame.

## RL/
- this is where I will put my RL plug-n-play scripts to research which algorithms perform the best for various tasks.
