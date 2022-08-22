from gym_grasper.controller.MujocoController import MJ_Controller
from Agent import Agent

############ CONFIG ##############
home_pos = (-2.595, -4.255, 1.491, -1.485, -0.920, -2.040)
home_pos2 = (0, 0, 0, 0, 0, 0)
place_pose = (-1.643, -3.509, 1.643, -2.070, -1.512, -2.016)
place_pose2 = [-1.6461, -3.5756, 1.7393, -2.1001, -1.5096, -2.8140]
#################################


controller = MJ_Controller()
agent = Agent(controller)
agent.movej(home_pos, relative=False)
# controller.get_image_data(show=True)
controller.stay(2000)
# agent.movej([0.5, 0, 0, 0, 0, 0], relative=True)
# controller.get_image_data(show=True)
controller.stay(2000)
# controller.get_image_data(show=True)




print(agent.getl())
# cur_tcp_pos = agent.getl()
# target_pos = cur_tcp_pos + [0, 0.1, 0]
# controller.move_ee(target_pos, plot=True, marker=True)
# controller.stay(2000)
# target_pos = cur_tcp_pos
# controller.move_ee(target_pos, plot=True, marker=True)
# controller.stay(2000)
# target_pos = cur_tcp_pos + [0.1, 0, 0]
# controller.move_ee(target_pos, plot=True, marker=True)
# controller.stay(2000)


# Move ee to position above the object, plot the trajectory to an image file, show a marker at the target location
#

# Wait before finishing
controller.stay(20000)

# controller.get_image_data(width=200, height=200, show=True)