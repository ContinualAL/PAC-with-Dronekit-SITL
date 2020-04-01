# PAC-with-Dronekit-SITL <br>
This Github resources are the simulation codes of our research paper submitted on IEEE Transaction. The preprint document are made available online in the arXiv that you can access here: <https://arxiv.org/abs/1907.08619>. To run the simulation, you are required to install a realistic drone simulator provided by ArduPilot namely Dronekit-SITL.
<br>
## Dronekit-SITL Python Installation 
<br>
Please follow the instructions below to install dronekit-sitl in your machine:

1. Make sure you have installed Python 2.7 on your machine (for Windows users it should be associated with your Command Prompt or terminal).
1. If you do not have python 2.7, you can download from this link: <https://www.python.org/downloads/>.
1. Install matplotlib package using pip: <br>
    `pip install matplotlib`
1. Open your Command Prompt and it is installed (or updated) on all platforms using the command: <br>
    `pip install dronekit-sitl -UI`
1. Make sure your installation is successful by run: <br>
    `dronekit-sitl copter` <br>
Then, you will get some states from your terminal as shown below: <br>
`os: win, apm: copter, release: stable` <br>
`SITL already Downloaded and Extracted.` <br>
`Ready to boot.` <br>
`Execute: C:\Users\Abdul Hady\.dronekit\sitl\copter-3.3\apm.exe --home=-35.363261,149.165230,584,353 --model=quad` <br>
`bind port 5760 for 0` <br>
`Started model quad at -35.363261,149.165230,584,353 at speed 1.0` <br>
`Serial port 0 on TCP port 5760` <br>
`Starting sketch 'ArduCopter'` <br>
`Starting SITL input` <br>
`Waiting for connection ....` <br>
