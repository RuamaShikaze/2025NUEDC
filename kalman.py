class KalmanFilter:
    def __init__(self, A=1, H=1, Q=0.01, R=0.1):
        self.A = A  # 状态转移矩阵
        self.H = H  # 观测矩阵
        self.Q = Q  # 过程噪声
        self.R = R  # 观测噪声
        self.P = 1  # 估计误差协方差
        self.x = 0  # 初始状态（误差）

    def predict(self):
        self.x = self.A * self.x
        self.P = self.A * self.P * self.A + self.Q

    def update(self, z):
        K = self.P * self.H / (self.H * self.P * self.H + self.R)
        self.x = self.x + K * (z - self.H * self.x)
        self.P = (1 - K * self.H) * self.P
        return self.x
