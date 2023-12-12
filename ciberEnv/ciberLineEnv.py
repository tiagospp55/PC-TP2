from gym import Env,spaces
import numpy as np
import subprocess
import socket
import time
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import PPO

from ciberEnv1.ciberEnv1 import CiberEnv1

SIM_IP = "127.0.0.1"
SIM_PORT = 6000

class CiberLineEnv(CiberEnv1):
    def __init__(self) -> None:
        super().__init__(spaces.MultiBinary(7),
                         spaces.Box(low=-0.15, high=0.15,shape=(2,),dtype=np.float32),
                         ["--param","../Labs/rmi-2223/C1-env-config.xml",
                          "--lab","../Labs/rmi-2223/C1-lab.xml", "--grid", "../Labs/rmi-2223/C1-grid.xml",
                          "--scoring","4"]
        )

        
    def step(self, action):
        self.agentapi.driveMotors(action[0],action[1])
        self.agentapi.readSensors()

        obsl = [int(x) for x in self.agentapi.measures.lineSensor]

        obs = np.array(obsl)

        done = self.agentapi.measures.time == self.agentapi.simTime \
               or self.agentapi.measures.score < 0

        reward = self.agentapi.measures.score - self.prev_score
        self.prev_score = self.agentapi.measures.score

        if done:
            print("SCORE ENV", self.agentapi.measures.score)

        return obs, reward, done, {"score":self.agentapi.measures.score}


    def reset(self):
        super().reset()

        obsl = [int(x) for x in self.agentapi.measures.lineSensor]
        obs = np.array(obsl)

        return obs

    def close(self):
        self.sim_proc.terminate()








##main

# Create Environment
c_env = CiberLineEnv()

# Learn model
model = PPO("MlpPolicy", c_env, verbose=1)
model.learn(200000)

# Save model
model.save("ciberlinenev_ppo_1")
