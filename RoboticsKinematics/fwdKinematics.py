
import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.setGravity(0,0,-9.81)
ur5Id = p.loadURDF("/home/omkharat/Desktop/IvLabs/RoboticsKinematics/git-repo-cloned/urdf/ur5.urdf", [0,0,0], useFixedBase = 1)
joints = p.getNumJoints(ur5Id)
p.setJointMotorControlArray(ur5Id, range(joints), p.POSITION_CONTROL, targetPositions=[1.2,0.4,1,0.2,0.1,0.8,0.5,1.2])
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./24.)