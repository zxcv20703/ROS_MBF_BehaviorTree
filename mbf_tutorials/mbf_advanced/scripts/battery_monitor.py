import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

def callback(data):
    # 將接收到的字串轉換成浮點數
    voltage = float(data.data) / 10
    
    battery_capacity = 33  # 電池容量，單位為 Ah
    full_voltage = 29.4  # 滿電壓，單位為 V
    empty_voltage = 21  # 空電壓，單位為 V
    current_capacity = battery_capacity * (voltage - empty_voltage) / (full_voltage - empty_voltage)
    percentage = current_capacity / battery_capacity * 100
    
    # 創建Float64型別的訊息
    msg = Float64()
    # 將浮點數賦值給訊息的data欄位
    msg.data = voltage
    
    with open('/home/ros/catkin_ws/src/mbf_tutorials/mbf_advanced/param/battery_level.txt', 'w') as file:
        # 将电压值和百分比写入文件
        file.write('%.1f, %.1f\n' % (voltage, percentage))
    
    # 印出訊息
    rospy.loginfo("電壓data: %.1f V, 電量百分比: %.1f %%", voltage, percentage)
    
    
    # 印出訊息
    rospy.loginfo("電壓data: %.1f V, 電量百分比: %.1f %%", voltage, percentage)

def voltage_listener():
    # 初始化節點
    rospy.init_node('voltage_listener', anonymous=True)
    # 訂閱名為"driver_real_voltage_state"的topic，並指定callback函數
    rospy.Subscriber("driver_real_voltage_state", String, callback)
    # spin()讓節點在等待訊息時不會退出
    rospy.spin()

if __name__ == '__main__':
    voltage_listener()

