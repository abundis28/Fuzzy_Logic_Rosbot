# Fuzzy_Logic_Rosbot
Fuzzy logic based control system that allowed a Rosbot to perform two behaviors automatically: right-edge following, obstacle avoidance and a combination of both.

The PID.py file contains a right-edge following PID-based program used as a baseline to measure the fuzzy logic performance. <br>
OA_FLC.py contains the obstacle avoidance fuzzy logic controller. <br>
REF_FLC.py contains the right-edge following fuzzy logic controller. <br>
COMBINED_FLC.py contains the higher fuzzy logic controller the combines both behaviour depending on a set of conditions detected by the sensors.
