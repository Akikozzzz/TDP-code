# This is the controller for red defender1
from controller import Robot, Motion, Supervisor
import math
import time

class Nao(Supervisor):  # Inherits Supervisor in order to get the location of other nodes.
    def loadMotionFiles(self):
        # load motions files
        try:
            self.forwards = Motion('../../motions/Forwards.motion')
            self.turnLeft30 = Motion('../../motions/TurnLeft30.motion')
            self.turnRight30 = Motion('../../motions/TurnRight30.motion')
            self.shoot = Motion('../../motions/Shoot.motion')
            self.handWave = Motion('../../motions/HandWave.motion')
            self.standUpFromFront = Motion('../../motions/StandUpFromFront.motion')
            self.standUpFromBack = Motion('../../motions/StandUpFromBack.motion')
            self.sideStepRight = Motion('../../motions/SideStepRight.motion')
            self.sideStepLeft = Motion('../../motions/SideStepLeft.motion')
            if not all([self.forwards, self.turnLeft30, self.turnRight30, self.shoot, self.handWave]):
                print("Error: Motion files could not be loaded. Check file paths.")
        except Exception as e:
            print("Error loading motion files:", e)

    def startMotion(self, motion):
        # Check if motion exists
        if motion is None:
            print("Error: No motion loaded.")
            return
        # Interrupt the current action and start a new one
        if self.currentlyPlaying:
            self.currentlyPlaying.stop()
        motion.play()
        self.currentlyPlaying = motion
    
    # Get and return the current GPS position of the robot
    def get_position(self):
        position = self.gps.getValues()
        return position
    
    # Get and return the current acceleration of the robot
    def get_acceleration(self):
        acceleration = self.accelerometer.getValues()
        return acceleration
        
    # Use the inertial unit to get the robot's orientation
    def get_orientation(self):      
        rpy = self.inertialUnit.getRollPitchYaw()
        yaw = rpy[2]  # Using yaw as robot orientation
        return yaw
        
    # Get time step
    def findAndEnableDevices(self):
        self.timeStep = int(self.getBasicTimeStep())

        # GPS device initialization
        self.gps = self.getDevice('gps')
        self.gps.enable(4 * self.timeStep)

        # Inertial unit device initialization
        self.inertialUnit = self.getDevice('inertial unit')
        self.inertialUnit.enable(self.timeStep)
        
         # Accelerometer device initialization
        self.accelerometer = self.getDevice('accelerometer')
        self.accelerometer.enable(self.timeStep)

    # Use the right call
    def __init__(self):
        super(Nao, self).__init__()  
        self.currentlyPlaying = None
        self.findAndEnableDevices()
        self.loadMotionFiles()
    
    # Calculate the distance between two points
    def calculateDistance(self, pos1, pos2):
        return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)
        
    # Calculate the angle between the current point and the target point
    def calculateAngle(self, pos1, pos2):
        dx = pos2[0] - pos1[0]
        dz = pos2[1] - pos1[1]
        return math.atan2(dz, dx)
        
    # Detecting if a robot has fallen from back
    def isFallenBack(self):       
        acceleration = self.accelerometer.getValues()       
        return 0.1 < abs(acceleration[2]) < 0.5 
    
    # Detecting if a robot has fallen from front
    def isFallenFront(self):
        acceleration = self.accelerometer.getValues()
        return 1 < abs(acceleration[2]) < 3 
        
    """
    Description:
    1. After each action starts executing, you need to add
       while not self.motions.isOver():
           self.step(self.timeStep)
       to make sure that while one motion is executing, 
       another motion is not conflicting or interrupting, 
       so that falls can be minimized.
    2. In order to ensure that each robot can do its own job, 
    the activity range of each robot is defined, 
    and if it exceeds the activity range, 
    it will stand by at a predetermined place, 
    so as to avoid the situation that multiple robots go to look for the ball at the same time 
    or no one is looking for the ball.
    3. This algorithm is not the optimal solution, 
    it can only complete the action temporarily, 
    and will be updated with more optimized algorithms in the future.
    """
        
    def run(self):
        # Get the ball node
        ball_node = self.getFromDef("ball")
        if ball_node is None:
            print("Error: Ball node not found.")
            return
        
        # Blue's goal range
        goal_x = -4.5 
        goal_center_y = 0
        goal_y_min = -1.34321
        goal_y_max = 1.34321
        approach_distance = 0.1
        
        # Blue's restricted area
        goal_area_x_min = -4.5
        goal_area_x_max = -3.5
        goal_area_y_min = -1.5
        goal_area_y_max = 1.5
        
        # move area for Blue defender1
        move_area_x_min = 1.25
        move_area_x_max = 4.5
        move_area_y_min = -3
        move_area_y_max = 3
        
        # initial position
        defender1_red_position = [2.5, 0, 0.334]
 
        while True:
            # Get the coordinates of the ball through the ball node
            target_position = ball_node.getField("translation").getSFVec3f()
            target_x, target_y = target_position[0], target_position[1]
            
            # Obtain basic data about the robot, including position, acceleration, yaw and distance from the ball.
            robot_position = self.get_position()
            robot_acc = self.get_acceleration()
            robot_yaw = self.get_orientation()
            distance_to_ball = self.calculateDistance(robot_position, (target_x, target_y, 0))
    
            # Determine if the ball is within the moveable range
            in_move_area = False
            if move_area_x_min <= target_x <= move_area_x_max:
                if move_area_y_min <= target_y <= move_area_y_max:
                    in_move_area = True
    
            # Detecting a fall
            if self.isFallenBack():
                self.startMotion(self.standUpFromBack)
                while not self.standUpFromBack.isOver():
                    self.step(self.timeStep)
                continue
    
            if self.isFallenFront():
                self.startMotion(self.standUpFromFront)
                while not self.standUpFromFront.isOver():
                    self.step(self.timeStep)
                continue
            
            # The ball is out of range, the goalkeeper returns to the initial position and faces the ball
            if not in_move_area:
                distance_to_initial = self.calculateDistance(robot_position, defender1_red_position)
                if distance_to_initial > 0.05:  # If not in initial position
                    # turn to initial position
                    angle_to_initial = self.calculateAngle(robot_position, defender1_red_position)
                    angle_diff_to_initial = (angle_to_initial - robot_yaw + math.pi) % (2 * math.pi) - math.pi
                    if abs(angle_diff_to_initial) > math.radians(30):
                        if angle_diff_to_initial > 0:
                            self.startMotion(self.turnLeft30)
                            while not self.turnLeft30.isOver():
                                self.step(self.timeStep)
                        else:
                            self.startMotion(self.turnRight30)
                            while not self.turnRight30.isOver():
                                self.step(self.timeStep)
                    else:
                        # move to initial position
                        self.startMotion(self.forwards)
                        while not self.forwards.isOver():
                            self.step(self.timeStep)
                else:
                    # turn to ball
                    angle_to_ball = self.calculateAngle(robot_position, (target_x, target_y, 0))
                    angle_diff = (angle_to_ball - robot_yaw + math.pi) % (2 * math.pi) - math.pi
                    if abs(angle_diff) > math.radians(15):
                        if angle_diff > 0:
                            self.startMotion(self.turnLeft30)
                            while not self.turnLeft30.isOver():
                                self.step(self.timeStep)
                        else:
                            self.startMotion(self.turnRight30)
                            while not self.turnRight30.isOver():
                                self.step(self.timeStep)
            else:
                # The ball is in the moveable area and the defender takes the initiative
                distance_to_ball = self.calculateDistance(robot_position, (target_x, target_y, 0))
                if distance_to_ball > 0.2:
                    # Adjust the angle of the robot and the ball
                    angle_to_ball = self.calculateAngle(robot_position, (target_x, target_y, 0))
                    angle_diff = (angle_to_ball - robot_yaw + math.pi) % (2 * math.pi) - math.pi
                    if abs(angle_diff) > math.radians(30):
                        if angle_diff > 0:
                            self.startMotion(self.turnLeft30)
                            while not self.turnLeft30.isOver():
                                self.step(self.timeStep)
                        else:
                            self.startMotion(self.turnRight30)
                            while not self.turnRight30.isOver():
                                self.step(self.timeStep)
                    else: # Adjusted and headed for the ball
                        self.startMotion(self.forwards)
                        while not self.forwards.isOver():
                            self.step(self.timeStep)
                else: # When kicking the ball out of the penalty area, try to face towards the opposing team's goal
                    angle_to_goal_center = self.calculateAngle((target_x, target_y, 0), (goal_x, goal_center_y))
                    angle_diff_to_goal = angle_to_goal_center - robot_yaw
                    angle_diff_to_goal = (angle_diff_to_goal + math.pi) % (2 * math.pi) - math.pi
    
                    if abs(angle_diff_to_goal) > math.radians(10):  
                        if angle_diff_to_goal < 0:
                            self.startMotion(self.sideStepLeft)                             
                            while not self.sideStepLeft.isOver():
                                self.step(self.timeStep)
                        else:
                            self.startMotion(self.sideStepRight) 
                            while not self.sideStepRight.isOver():
                                self.step(self.timeStep)
                    else: # pass the ball
                        initial_ball_position = target_position.copy()
                        self.startMotion(self.forwards)
                        while not self.forwards.isOver():
                            self.step(self.timeStep)
                        # Update the position of the ball and determine if the ball has moved
                        updated_ball_position = ball_node.getField("translation").getSFVec3f()
                        ball_movement = self.calculateDistance(initial_ball_position, updated_ball_position)
                         
                        if ball_movement < 0.05:  # Determine if the kick is empty
                            self.startMotion(self.sideStepRight)
                            while not self.sideStepRight.isOver():
                                self.step(self.timeStep)
    
            # Cyclic execution
            if self.step(self.timeStep) == -1:
                break 

# Create a Nao instance and run the main loop
robot = Nao()
robot.run()

