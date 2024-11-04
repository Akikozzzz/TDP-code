from controller import Robot, Motion
import math

class Nao(Robot):
    def loadMotionFiles(self):
        # 加载前进、转向和停顿的动作文件
        try:
            self.forwards = Motion('../../motions/Forwards50.motion')
            self.turnLeft40 = Motion('../../motions/TurnLeft40.motion')
            self.turnRight40 = Motion('../../motions/TurnRight40.motion')
            if self.forwards is None or self.turnLeft60 is None or self.turnRight60 is None or self.stand is None:
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


    def __init__(self):
        Robot.__init__(self)
        self.currentlyPlaying = None
        self.findAndEnableDevices()
        self.loadMotionFiles()

    def calculateDistance(self, pos1, pos2):
        # 计算两点之间的距离
        return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[2] - pos2[2]) ** 2)

    def calculateAngle(self, pos1, pos2):
        # 计算当前点和目标点之间的角度
        dx = pos2[0] - pos1[0]
        dz = pos2[2] - pos1[2]
        return math.atan2(dz, dx)

    def run(self, target_x, target_z):
        target_position = (target_x, 0, target_z)  # 目标位置 (x, y, z)

        while True:
            robot_position = self.get_position()
            distance_to_target = self.calculateDistance(robot_position, target_position)

            if distance_to_target < 0.2:  # 如果距离小于设定的范围，停止
                print("到达目标位置")
                break

            # 计算当前与目标位置的角度差
            angle_to_target = self.calculateAngle(robot_position, target_position)
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
                
            if self.step(self.timeStep) == -1:
                break

# 创建Nao实例并运行主循环
robot = Nao()
# 指定目标坐标
target_x, target_z = -1.3, 0.3  # 示例目标坐标
robot.run(target_x, target_z)



