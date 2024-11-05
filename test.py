from controller import Robot, Motion, Supervisor
import math
import time

class Nao(Supervisor):  # 继承 Supervisor 以便获取其他节点的位置
    def loadMotionFiles(self):
        # 加载前进、转向和停顿的动作文件
        try:
            self.forwards = Motion('../../motions/Forwards50.motion')
            self.turnLeft40 = Motion('../../motions/TurnLeft40.motion')
            self.turnRight40 = Motion('../../motions/TurnRight40.motion')
            self.shoot = Motion('../../motions/Shoot.motion')
            self.handWave = Motion('../../motions/HandWave.motion')
            self.standUp = Motion('../../motions/StandUpFromFront.motion')
            if not all([self.forwards, self.turnLeft40, self.turnRight40, self.shoot, self.handWave]):
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
    
    def isFallen(self):
        # 检测机器人是否摔倒（例如：根据加速度计的值）
        acceleration = self.accelerometer.getValues()
        # 简单判断，如果z轴加速度小于一个阈值，认为机器人摔倒
        return 1 < abs(acceleration[2]) < 1.5 # 阈值可根据实际情况调整

    def run(self):
    # 获取球的位置
        ball_node = self.getFromDef("ball")
        if ball_node is None:
            print("Error: Ball node not found.")
            return
       
        target_position = ball_node.getField("translation").getSFVec3f()
        target_x, target_y = target_position[0], target_position[1]

        print(f"目标位置 (球的位置): x = {target_x}, y = {target_y}")

        while True:
            if self.isFallen():
                print("机器人摔倒了，执行起身动作。")
                self.startMotion(self.standUp)
                while not self.standUp.isOver():
                    self.step(self.timeStep)  # 等待动作完成
                print("机器人已起身，继续执行后续动作。")
                continue  # 继续执行后续动作
                      
            robot_position = self.get_position()
            robot_acceleration = self.get_acceleration()
            distance_to_target = self.calculateDistance(robot_position, (target_x, target_y, 0))

            # 计算当前与目标位置的角度差
            angle_to_target = self.calculateAngle(robot_position, (target_x, target_y, 0))
            robot_yaw = self.get_orientation()

            # 计算角度差并归一化到[-pi, pi]
            angle_diff = angle_to_target - robot_yaw
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

            # 使用40度的旋转步幅来调整朝向
            if abs(angle_diff) > math.radians(40):  # 当角度误差大于40度时旋转
                if angle_diff > 0:
                    self.startMotion(self.turnLeft40)
                else:
                    self.startMotion(self.turnRight40)
            else:
                # 角度接近目标方向，开始前进
                self.startMotion(self.forwards)
                
            if distance_to_target < 0.18:
                print("到达目标位置")
                # 执行踢球动作
                self.startMotion(self.shoot)
                while not self.shoot.isOver():
                    self.step(self.timeStep)  # 等待动作完成
                print("踢球动作完成")
                break

            if self.step(self.timeStep) == -1:
                break

# 创建Nao实例并运行主循环
robot = Nao()
# 运行程序，前往球的位置
robot.run()



