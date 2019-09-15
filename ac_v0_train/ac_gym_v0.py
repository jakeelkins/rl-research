import pybullet as p
import pybullet_utils.bullet_client as bc
import numpy as np
import time
import pybullet_data
import gym
from gym import spaces, logger


class AttitudeControlEnv(gym.Env):

    #----------toolbox----------

    def _invertQuaternion(self, q):
        return np.array((-q[0], -q[1], -q[2], q[3]))

    def _getAngleFromVectors(self, v1, v2):
        #these are always unit vectors when used

        #note: clips at -1 to 1 just in case, roundoff was giving dot products >1 (invalid arccos)
        angle = np.arccos(np.clip(np.dot(v1, v2), -1, 1))
        return angle

    def _getAngleFromOrn(self, Orn1, Orn2):
        # given two quaternions (like pybullet outputs), calculates 
        # the minimum angle between the two vectors in the +z-axis body frame

        # (this helps with reward)
        
        v = (0, 0, 1) #we want to use body z-axis

        #do q dot v

        #sep the orientation into axisangle
        #q = [vec, scalar]
        v2 = np.array(v)
        v1 = np.array((Orn1[0], Orn1[1], Orn1[2]))
        s2 = 0
        s1 = Orn1[3]

        qdotv = ((s1*v2)+(s2*v1)+(np.cross(v1,v2)), (s1*s2)-(np.dot(v1,v2)))

        #now do (q dot v) * qinverse
        Orn1inverse = self._invertQuaternion(Orn1)

        v1 = qdotv[0]
        s1 = qdotv[1]

        v2 = np.array((Orn1inverse[0], Orn1inverse[1], Orn1inverse[2]))
        s2 = Orn1inverse[3]

        newVector1 = ((s1*v2)+(s2*v1)+(np.cross(v1,v2)), (s1*s2)-(np.dot(v1,v2)))[0]

        #-------do it again------
        v2 = np.array(v)
        v1 = np.array((Orn2[0], Orn2[1], Orn2[2]))
        s2 = 0
        s1 = Orn2[3]

        qdotv = ((s1*v2)+(s2*v1)+(np.cross(v1,v2)), (s1*s2)-(np.dot(v1,v2)))

        #now do (q dot v) * qinverse
        Orn2inverse = self._invertQuaternion(Orn2)

        v1 = qdotv[0]
        s1 = qdotv[1]

        v2 = np.array((Orn2inverse[0], Orn2inverse[1], Orn2inverse[2]))
        s2 = Orn2inverse[3]

        newVector2 = ((s1*v2)+(s2*v1)+(np.cross(v1,v2)), (s1*s2)-(np.dot(v1,v2)))[0]

        #now we can do dot product between the 2 vectors to get cosine between two unit vectors

        #NOTE: was a big bug where roundoff error gave a dot here >1, so invalid arccos
        angle = np.arccos(np.clip(np.dot(newVector1, newVector2), -1, 1))
        
        return angle


    def _getVectorFromOrn(self, Orn):
        #gets the vector (x,y,z) of the vector of the pointer (+z-axis) of s/c

        v = (0, 0, 1) #we want to use body z-axis

        #do q dot v

        #sep the orientation into axisangle
        #q = [vec, scalar]
        v2 = np.array(v)
        v1 = np.array((Orn[0], Orn[1], Orn[2]))
        s2 = 0
        s1 = Orn[3]

        qdotv = ((s1*v2)+(s2*v1)+(np.cross(v1,v2)), (s1*s2)-(np.dot(v1,v2)))

        #now do (q dot v) * qinverse
        Orninverse = self._invertQuaternion(Orn)

        v1 = qdotv[0]
        s1 = qdotv[1]

        v2 = np.array((Orninverse[0], Orninverse[1], Orninverse[2]))
        s2 = Orninverse[3]

        newVector = ((s1*v2)+(s2*v1)+(np.cross(v1,v2)), (s1*s2)-(np.dot(v1,v2)))[0]
        
        return newVector

    #-------end toolbox, start actual env-------













    def __init__(self):

        #self.physicsClient = p.connect(p.DIRECT)
        self.physicsClient = bc.BulletClient()
        #self.physicsClient = bc.BulletClient(connection_mode=p.DIRECT)

        self.physicsClient.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.physicsClient.setGravity(0,0,0)

        self.startPos = [0,0,1]

        self.startOrientation = self.physicsClient.getQuaternionFromEuler([0,0,0])

        self.scID = self.physicsClient.loadURDF("lm50.urdf", self.startPos, self.startOrientation)

        high = np.array([np.pi, np.pi, np.pi, 1, 1, 1])

        self.action_space = spaces.Discrete(7)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

        self.nsteps = 0

        #---thresholds for episode-----

        self.maxsteps = 500

        self.maxAngVelo = 10

        #---------------------------

        #initial command/goal
        self.goalEuler = (np.random.randint(-np.pi, high=np.pi, size=3))

        while np.array_equal(self.goalEuler, np.array([0, 0, 0])):
            self.goalEuler = (np.random.randint(-np.pi, high=np.pi, size=3))

        self.goalQuat = self.physicsClient.getQuaternionFromEuler(self.goalEuler)

        self.goalVec = self._getVectorFromOrn(self.goalQuat)

        self.initialAngle = self._getAngleFromOrn(self.startOrientation, self.goalQuat)

        #avoid div by zero, just in case
        if self.initialAngle < 0.0001:
            self.initialAngle = 0.0001


        #getting state. state vector will be (goalVec, currVec), so (1x6) of floats
        # (Xgoal, Ygoal, Zgoal; Xcurr, Ycurr, Zcurr)
        scPosition, scOrientation = self.physicsClient.getBasePositionAndOrientation(self.scID)
        
        currVec = self._getVectorFromOrn(scOrientation)

        scTransVelo, scAngVelo = self.physicsClient.getBaseVelocity(self.scID) #leave this out for now

        self.state = (self.goalVec[0], self.goalVec[1], self.goalVec[2], currVec[0], currVec[1], currVec[2])

        self.steps_beyond_done = None




    def step(self, action):

        if type(action) != int:
            action = np.argmax(action)

        assert self.action_space.contains(action), f'action {action} is invalid, must be int [0 - 6]'

        if action == 0:
            #do nothing
            self.physicsClient.applyExternalTorque(self.scID, -1, [0, 0, 0], p.LINK_FRAME)
        if action == 1:
            # -1 in x
            self.physicsClient.applyExternalTorque(self.scID, -1, [-1, 0, 0], p.LINK_FRAME)
        if action == 2:
            # +1 in x
            self.physicsClient.applyExternalTorque(self.scID, -1, [+1, 0, 0], p.LINK_FRAME)
        if action == 3:
            # -1 in y
            self.physicsClient.applyExternalTorque(self.scID, -1, [0, -1, 0], p.LINK_FRAME)
        if action == 4:
            # +1 in y
            self.physicsClient.applyExternalTorque(self.scID, -1, [0, 1, 0], p.LINK_FRAME)
        if action == 5:
            # -1 in z
            self.physicsClient.applyExternalTorque(self.scID, -1, [0, 0, -1], p.LINK_FRAME)
        if action == 6:
            # +1 in z
            self.physicsClient.applyExternalTorque(self.scID, -1, [0, 0, 1], p.LINK_FRAME)

        #propagate dynamics
        self.physicsClient.stepSimulation()

        #BIG Q: how do we ensure timetables are correct? on client-side?

        scPosition, scOrientation = self.physicsClient.getBasePositionAndOrientation(self.scID)
        scEuler = self.physicsClient.getEulerFromQuaternion(scOrientation)

        scTransVelo, scAngVelo = self.physicsClient.getBaseVelocity(self.scID)

        self.nsteps += 1
        
        currVec = self._getVectorFromOrn(scOrientation)

        self.state = (self.goalVec[0], self.goalVec[1], self.goalVec[2], currVec[0], currVec[1], currVec[2])

        
        #currAngle = self._getAngleFromOrn(scOrientation, self.goalQuat)

        #faster
        currAngle = self._getAngleFromVectors(self.goalVec, currVec)

        #print(f'currAngle: {currAngle*180/np.pi}')

        done = abs(scAngVelo[0]) > self.maxAngVelo \
                or abs(scAngVelo[1]) > self.maxAngVelo \
                or abs(scAngVelo[2]) > self.maxAngVelo# \
                #or self.nsteps >= self.maxsteps

        done = bool(done)

        #--------REWARD---------
        if not done:
            #we calculate current angle's difference from initial angle (in radians)
            reward = np.maximum(0, ((self.initialAngle-currAngle)/(self.initialAngle))**3)

        elif self.steps_beyond_done is None:
            # epsiode just ended
            self.steps_beyond_done = 0
            reward = np.maximum(0, ((self.initialAngle-currAngle)/(self.initialAngle))**3)

            #print('disconnecting')
            #self.physicsClient.disconnect()
        
        else:
            if self.steps_beyond_done == 0:
                logger.warn("You are calling 'step()' even though this environment has already returned done = True. You should always call 'reset()' once you receive 'done = True' -- any further steps are undefined behavior.")
            self.steps_beyond_done += 1
            reward = 0.0
        #print(f'reward from ep: {reward}')
        return np.array(self.state), reward, done, {}





    def reset(self):

        #self.physicsClient = p.connect(p.DIRECT)
        self.physicsClient = bc.BulletClient()

        self.physicsClient.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.physicsClient.setGravity(0,0,0)

        self.scID = self.physicsClient.loadURDF("lm50.urdf", self.startPos, self.startOrientation)

        self.nsteps = 0

        #new command/goal
        self.goalEuler = (np.random.randint(-np.pi, high=np.pi, size=3))

        while np.array_equal(self.goalEuler, np.array([0, 0, 0])):
            self.goalEuler = (np.random.randint(-np.pi, high=np.pi, size=3))

        self.goalQuat = self.physicsClient.getQuaternionFromEuler(self.goalEuler)

        self.goalVec = self._getVectorFromOrn(self.goalQuat)

        self.initialAngle = self._getAngleFromOrn(self.startOrientation, self.goalQuat)

        if self.initialAngle < 0.001:
            self.initialAngle = 0.001

        #getting state. state vector will be (goalVec, currVec), so (1x6) of floats
        # (Xgoal, Ygoal, Zgoal; Xcurr, Ycurr, Zcurr)
        scPosition, scOrientation = self.physicsClient.getBasePositionAndOrientation(self.scID)
        
        currVec = self._getVectorFromOrn(scOrientation)

        scTransVelo, scAngVelo = self.physicsClient.getBaseVelocity(self.scID) #leave this out for now

        self.state = (self.goalVec[0], self.goalVec[1], self.goalVec[2], currVec[0], currVec[1], currVec[2])

        self.steps_beyond_done = None

        return np.array(self.state)




            