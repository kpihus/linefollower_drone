def fix_yaw(yaw):
    return (yaw + 360) % 360

print(fix_yaw(0))