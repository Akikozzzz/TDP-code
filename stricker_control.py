from controller import Robot, Motion, Supervisor
import math
import time

class Nao(Supervisor):  # 继承 Supervisor 以便获取其他节点的位置
    def loadMotionFiles(self):
        # 加载前进、转向和停顿的动作文件
        try:
            self.forwards = Motion('../../motions/Forwards50.motion')
            self.turnLeft30 = Motion('../../motions/TurnLeft30.motion')
            self.turnRight30 = Motion('../../motions/TurnRight30.motion')
            self.shoot = Motion('../../motions/Shoot.motion')
            self.handWave = Motion('../../motions/HandWave.motion')
            self.standUpFromFront = Motion('../../motions/StandUpFromFront.motion')
            self.standUpFromBack = Motion('../../motions/StandUpFromBack.motion')
            self.sideStepRight = Motion('../../motions/SideStepRight.motion')
            if not all([self.forwards, self.turnLeft30, self.turnRight30, self.shoot, self.handWave]):
                print("Error: Motion files could not be loaded. Check file paths.")
        except Exception as e:
            print("Error loading motion files:", e)

    def startMotion(self, motion):
        # 检查 motion 是否存在
        if motion is None:
            print("Error: No motion loaded.")
            return
        # 中断当前动作并启动新动作
        if self.currentlyPlaying:
            self.currentlyPlaying.stop()
        motion.play()
        self.currentlyPlaying = motion

    def get_position(self):
        # 获取并返回机器人当前GPS位置
        position = self.gps.getValues()
        print('当前GPS位置: [x y z] = [%f %f %f]' % (position[0], position[1], position[2]))
        return position
        
    def get_acceleration(self):
        acceleration = self.accelerometer.getValues()
        print('当前acc值: [x y z] = [%f %f %f]' % (acceleration[0], acceleration[1], acceleration[2]))
        return acceleration

    def get_orientation(self):
        # 使用惯性单元获取机器人的朝向
        rpy = self.inertialUnit.getRollPitchYaw()
        yaw = rpy[2]  # 使用yaw作为机器人的朝向
        print('当前朝向: yaw = %f' % yaw)
        return yaw

    def findAndEnableDevices(self):
        # 获取时间步长
        self.timeStep = int(self.getBasicTimeStep())

        # GPS设备初始化
        self.gps = self.getDevice('gps')
        self.gps.enable(4 * self.timeStep)

        # 惯性单元设备初始化
        self.inertialUnit = self.getDevice('inertial unit')
        self.inertialUnit.enable(self.timeStep)
        
         # 加速度计设备初始化
        self.accelerometer = self.getDevice('accelerometer')
        self.accelerometer.enable(self.timeStep)

    def __init__(self):
        super(Nao, self).__init__()  # 使用正确的调用方式
        self.currentlyPlaying = None
        self.findAndEnableDevices()
        self.loadMotionFiles()

    def calculateDistance(self, pos1, pos2):
        # 计算两点之间的距离
        return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)

    def calculateAngle(self, pos1, pos2):
        # 计算当前点和目标点之间的角度
        dx = pos2[0] - pos1[0]
        dz = pos2[1] - pos1[1]
        return math.atan2(dz, dx)
    
    def isFallenBack(self):
        # 检测机器人是否摔倒（例如：根据加速度计的值）
        acceleration = self.accelerometer.getValues()
        # 简单判断，如果z轴加速度小于一个阈值，认为机器人摔倒
        return 0.5 < abs(acceleration[2]) < 1 # 阈值可根据实际情况调整
        
    def isFallenFront(self):
        # 检测机器人是否摔倒（例如：根据加速度计的值）
        acceleration = self.accelerometer.getValues()
        # 简单判断，如果z轴加速度小于一个阈值，认为机器人摔倒
        return 2.5 < abs(acceleration[2]) < 3 # 阈值可根据实际情况调整
        
    def run(self):
        # 获取球节点
        ball_node = self.getFromDef("ball")
        if ball_node is None:
            print("Error: Ball node not found.")
            return
    
        goal_x = 4.4985  # 球门x坐标
        goal_center_y = 0
        goal_y_min = -1.34321
        goal_y_max = 1.34321
        approach_distance = 0.2
        attacker_active = False
 
        while True:
            target_position = ball_node.getField("translation").getSFVec3f()
            target_x, target_y = target_position[0], target_position[1]
            print(f"目标位置 (球的位置): x = {target_x}, y = {target_y}")
    
            if target_x > goal_x and goal_y_min <= target_y <= goal_y_max:
                print("球已进入球门，停止踢球。")
                break
            
            if self.isFallenBack():
                print("机器人摔倒了，执行起身动作。")
                self.startMotion(self.standUpFromBack)
                while not self.standUpFromBack.isOver():
                    self.step(self.timeStep)
                print("机器人已起身，继续执行后续动作。")
                continue
            
            if self.isFallenFront():
                print("机器人摔倒了，执行起身动作。")
                self.startMotion(self.standUpFromFront)
                while not self.standUpFromFront.isOver():
                    self.step(self.timeStep)
                print("机器人已起身，继续执行后续动作。")
                continue
                
            robot_position = self.get_position()
            robot_acc = self.get_acceleration()
            robot_yaw = self.get_orientation()
            distance_to_ball = self.calculateDistance(robot_position, (target_x, target_y, 0))
            
            if distance_to_ball < 1 or attacker_active:
                attacker_active = True  # 一旦进攻者开始行动，保持此状态
    
                if distance_to_ball > approach_distance:
                    angle_to_ball = self.calculateAngle(robot_position, (target_x, target_y, 0))
                    angle_diff = angle_to_ball - robot_yaw
                    angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        
                    if abs(angle_diff) > math.radians(30):
                        if angle_diff > 0:
                            self.startMotion(self.turnLeft30)
                        else:
                            self.startMotion(self.turnRight30)
                    else:
                        self.startMotion(self.forwards)
                else:
                    angle_to_goal_center = self.calculateAngle((target_x, target_y, 0), (goal_x, goal_center_y))
                    angle_diff_to_goal = angle_to_goal_center - robot_yaw
                    angle_diff_to_goal = (angle_diff_to_goal + math.pi) % (2 * math.pi) - math.pi
        
                    if abs(angle_diff_to_goal) > math.radians(30):
                        if angle_diff_to_goal > 0:
                            self.startMotion(self.turnLeft30)
                        else:
                            self.startMotion(self.turnRight30)
                    else:
                        print("到达目标位置，执行踢球动作。")
                        
                        initial_ball_position = target_position.copy()
                        self.startMotion(self.shoot)
                        
                        while not self.shoot.isOver():
                            self.step(self.timeStep)
                        
                        updated_ball_position = ball_node.getField("translation").getSFVec3f()
                        ball_movement = self.calculateDistance(initial_ball_position, updated_ball_position)
                        
                        if ball_movement < 0.05:  # 检查球是否移动，若移动小于0.05则认为踢空
                            print("踢空了，向右侧移一步重新对准球。")
                            self.startMotion(self.sideStepRight)
                            while not self.sideStepRight.isOver():
                                self.step(self.timeStep)
                            print("侧移完成，重新尝试踢球。")
                        else:
                            print("踢球成功，重新检查球的位置。")
        
            if self.step(self.timeStep) == -1:
                break
# 创建Nao实例并运行主循环
robot = Nao()
# 运行程序，前往球的位置
robot.run()