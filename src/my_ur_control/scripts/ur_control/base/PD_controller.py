import time

# PD控制器参数
Kp = 0.5  # 比例系数，可根据实际情况调整
Kd = 0.1  # 微分系数，可根据实际情况调整

# 目标位置（示例中假设为某个合适的行程位置值，单位根据实际情况确定）
target_position = 100
# 当前位置（初始值，后续会根据实际反馈更新）
current_position = 0

# 用于记录上一次的误差，用于计算微分部分
prev_error = 0


# 模拟获取当前位置的函数（实际中要替换成真实的从机器人获取反馈的接口）
def get_current_position():
    return current_position


# 模拟设置手爪位置的函数（实际中要替换成真实的向机器人发送控制命令的接口）
def set_hand_position(position):
    global current_position
    current_position = position
    print(f"Setting hand position to: {position}")


while True:
    # 计算当前误差（目标位置与当前实际位置的差值）
    error = target_position - get_current_position()

    # 计算误差的变化率（微分部分）
    error_diff = error - prev_error

    # 计算控制输出，PD控制律
    control_output = Kp * error + Kd * error_diff

    # 根据控制输出去设置手爪的位置（这里简单相加，实际可能需要按照机器人控制逻辑来处理）
    new_position = current_position + control_output

    # 设置手爪位置
    set_hand_position(new_position)

    # 更新上一次的误差
    prev_error = error

    # 适当延时，避免过于频繁控制，具体时间根据实际情况调整
    time.sleep(0.01)

    # 判断是否已经达到目标位置附近（定义一个合适的误差范围作为收敛条件）
    if abs(error) < 1:
        print("Reached the target position.")
        break