from gym_grasper.controller.MujocoController import MJ_Controller
from Agent import Agent
import ikpy
import utils

############ CONFIG ##############
home_pos = (-2.595, -4.255, 1.491, -1.485, -0.920, -2.040)
home_pos2 = (-2.595, -4.255, 1.491, -1.485, -0.920, -1.040)
home_pos3 = (-2.595, -4.255, 1.491, -1.485, -0.920, -0.040)
home_pos0 = (0, 0, 0, 0, 0, 0)
place_pose = (-1.643, -3.509, 1.643, -2.070, -1.512, -2.016)
place_pose2 = [-1.6461, -3.5756, 1.7393, -2.1001, -1.5096, -2.8140]
#################################


controller = MJ_Controller()
agent = Agent(controller)
agent.movej(home_pos, relative=False)
controller.stay(1000)
print(agent.getl())

agent.movel([0, 0, 0], relative=True)
controller.stay(1000)
agent.movel([-0.1, 0, 0], relative=True)

controller.stay(1000)
agent.movel([0.1, 0, 0], relative=True)
controller.stay(1000)

agent.movel([0, -0.1, 0], relative=True)
controller.stay(1000)
agent.movel([0, 0.1, 0], relative=True)
controller.stay(1000)

agent.movel([0, 0, 0.1], relative=True)
controller.stay(1000)
agent.movel([0, 0, -0.1], relative=True)
controller.stay(1000)



# agent.movej(home_pos2, relative=False)
# controller.stay(1000)
# print(agent.getl())


# todo
## controller.get_image_data(width=200, height=200, show=True)
## agent.movel([0.1, 0, 0], relative=True)

# Wait before finishing
controller.stay(20000)