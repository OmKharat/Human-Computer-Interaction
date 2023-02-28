
import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.setGravity(0,0,-9.81)
ur5Id = p.loadURDF("/home/omkharat/Desktop/IvLabs/RoboticsKinematics/git-repo-cloned/urdf/ur5.urdf", [0,0,0], useFixedBase = 1)
joints = p.getNumJoints(ur5Id)
orientation = p.getQuaternionFromEuler([3.14, 0.0 , 3.14 ])
target = p.calculateInverseKinematics(ur5Id, (joints-2), [0.1, 0.1, 0.4], targetOrientation = orientation)
print(target)
p.setJointMotorControlArray(ur5Id, range(joints-2), p.POSITION_CONTROL, targetPositions= target)
for i in range (100):
    p.stepSimulation()
    time.sleep(1./24.)