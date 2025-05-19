import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import cv2
import os
import math
import serial
import time
from matplotlib.patches import Circle

# Thông số robot SCARA
L1, L2 = 140, 120  # Chiều dài các khâu (mm)
image_size = 400
workspace_size = 300
scale = 1.0

# Thiết lập kết nối Serial với Arduino Master
# Thay đổi 'COM3' thành cổng Arduino Master của bạn
try:
    arduino_serial = serial.Serial('COM3', 9600, timeout=1)
    print("Đã kết nối với Arduino qua Serial")
    arduino_connected = True
    time.sleep(2)  # Đợi Arduino khởi động
except:
    print("Không thể kết nối với Arduino, chế độ mô phỏng sẽ được kích hoạt")
    arduino_connected = False

# Biến lưu trữ vị trí và góc hiện tại
x_drawn, y_drawn = [], []
theta1_list, theta2_list = [], []
pen_down = True  # Mặc định bút được hạ xuống để vẽ

def pen_up_func():
    """Nhấc bút lên để di chuyển mà không vẽ"""
    global pen_down
    pen_down = False
    print("Bút đã được nhấc lên")
    if arduino_connected:
        arduino_serial.write(b'U')  # Gửi lệnh nhấc bút lên

def pen_down_func():
    """Hạ bút xuống để bắt đầu vẽ"""
    global pen_down
    pen_down = True
    print("Bút đã được hạ xuống")
    if arduino_connected:
        arduino_serial.write(b'D')  # Gửi lệnh hạ bút xuống

def toggle_pen():
    """Đảo trạng thái bút (từ lên xuống và ngược lại)"""
    global pen_down
    pen_down = not pen_down
    if pen_down:
        pen_down_func()
    else:
        pen_up_func()

def find_image_files():
    """Tìm và hiển thị các file ảnh trong thư mục hiện tại"""
    current_dir = os.getcwd()
    print(f"Thư mục hiện tại: {current_dir}")
    
    image_files = [f for f in os.listdir(current_dir) if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif'))]
    
    if image_files:
        print(f"Tìm thấy {len(image_files)} file ảnh:")
        for i, file in enumerate(image_files, 1):
            print(f"  {i}. {file}")
    else:
        print("Không tìm thấy file ảnh nào trong thư mục hiện tại!")
    
    return image_files

def extract_drawing_coordinates(image_path, scale=1.0):
    """Trích xuất tọa độ từ hình ảnh với tỷ lệ scale và thêm chức năng nhấc bút"""
    # Kiểm tra đường dẫn file
    if not os.path.exists(image_path):
        print(f"Không tìm thấy file: {image_path}")
        return None, []
        
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"Không thể đọc hình ảnh: {image_path}")
        return None, []
        
    # Áp dụng blur để làm mịn ảnh và loại bỏ nhiễu
    img_blur = cv2.GaussianBlur(img, (5, 5), 0)
    
    # Trích xuất cạnh với Canny
    edges = cv2.Canny(img_blur, 50, 150)
    
    # Tìm contour với phương pháp giản lược để có ít điểm hơn
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Sắp xếp contours theo kích thước (lớn đến nhỏ)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    
    # Phát hiện các điểm nên nhấc bút
    path_with_lifts = detect_pen_lift_points(contours)
    
    # Danh sách điểm và lệnh nhấc/hạ bút
    drawing_points = []
    pen_commands = []
    
    # Xử lý các lệnh và điểm
    for cmd, point in path_with_lifts:
        if cmd == "move" and point:
            x, y = point
            drawing_points.append((x * scale, y * scale))
            pen_commands.append(True if pen_down else False)  # True = vẽ, False = không vẽ
        elif cmd == "pen_up":
            pen_up_func()
        elif cmd == "pen_down":
            pen_down_func()
    
    return img, drawing_points, pen_commands

def convert_to_robot_coords(image_points):
    """Chuyển đổi từ tọa độ ảnh sang tọa độ robot với tỉ lệ scale"""
    robot_coords = []
    
    for x_img, y_img in image_points:
        # Dịch chuyển gốc tọa độ từ góc trên bên trái sang giữa, đồng thời đổi chiều y
        x_robot = (x_img - image_size / 2) * scale  
        y_robot = (image_size / 2 - y_img) * scale  
        robot_coords.append((x_robot, y_robot))
    
    return robot_coords

def inverse_kinematics(x, y):
    """Tính toán động học nghịch"""
    # Tính toán khoảng cách từ gốc đến điểm đích
    d = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    
    # Kiểm tra xem điểm có nằm trong phạm vi không
    if d < -1 or d > 1:
        return None  # Điểm nằm ngoài phạm vi hoạt động
    
    # Tính góc khớp 2 (khớp khuỷu)
    theta2 = np.arccos(np.clip(d, -1.0, 1.0))  
    
    # Tính góc khớp 1 (khớp vai)
    theta1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))
    
    return np.degrees(theta1), np.degrees(theta2)

def forward_kinematics(theta1, theta2):
    """Tính toán động học thuận"""
    # Chuyển đổi góc từ độ sang radian
    theta1_rad = np.radians(theta1)
    theta2_rad = np.radians(theta2)
    
    # Tính tọa độ của khớp thứ nhất
    x1 = L1 * np.cos(theta1_rad)
    y1 = L1 * np.sin(theta1_rad)
    
    # Tính tọa độ của khớp thứ hai (điểm cuối)
    x2 = x1 + L2 * np.cos(theta1_rad + theta2_rad)
    y2 = y1 + L2 * np.sin(theta1_rad + theta2_rad)
    
    return (x1, y1), (x2, y2)

def update(frame):
    """Hàm cập nhật cho animation"""
    if frame >= len(robot_points):
        return []
        
    x, y = robot_points[frame]
    
    # Kiểm tra xem điểm có nằm trong phạm vi làm việc không
    d = math.sqrt(x**2 + y**2)
    if d > L1 + L2 or d < abs(L1 - L2):
        print(f"Điểm ({x:.1f}, {y:.1f}) nằm ngoài phạm vi hoạt động, bỏ qua.")
        return []
        
    result = inverse_kinematics(x, y)
    if result is None:
        return []
        
    theta1, theta2 = result

    # Gửi góc tới Arduino nếu được kết nối
    if arduino_connected:
        # Chuyển thành chuỗi với định dạng "theta1,theta2,pen_state"
        # pen_state: 1 = bút xuống, 0 = bút lên
        command = f"{theta1:.2f},{theta2:.2f},{1 if pen_down else 0}\n"
        arduino_serial.write(command.encode())
        # Đợi phản hồi từ Arduino
        response = arduino_serial.readline().decode().strip()
        if response:
            print(f"Arduino phản hồi: {response}")

    # Tính toán tọa độ Cartesian của các khớp
    (x1, y1), (x2, y2) = forward_kinematics(theta1, theta2)

    # Cập nhật danh sách các điểm đã vẽ chỉ khi bút được hạ xuống
    if pen_down:
        x_drawn.append(x2)
        y_drawn.append(y2)
    elif x_drawn and y_drawn:
        # Thêm None để tạo đoạn ngắt trong đường vẽ khi nhấc bút
        x_drawn.append(None)
        y_drawn.append(None)
        
    theta1_list.append(theta1)
    theta2_list.append(theta2)

    # Xóa và vẽ lại các axes
    ax1.clear()
    ax1.imshow(image_original, cmap="gray")
    ax1.set_title("Ảnh Gốc (Trắng Đen)")
    ax1.axis("off")

    ax2.clear()
    ax2.set_xlim(-workspace_size, workspace_size)
    ax2.set_ylim(-workspace_size, workspace_size)
    
    # Vẽ vòng tròn giới hạn vùng làm việc
    workspace_circle = Circle((0, 0), L1 + L2, fill=False, color='red', linestyle='--', alpha=0.3)
    min_workspace_circle = Circle((0, 0), abs(L1 - L2), fill=False, color='red', linestyle='--', alpha=0.3)
    ax2.add_patch(workspace_circle)
    ax2.add_patch(min_workspace_circle)

    # Vẽ cánh tay robot
    ax2.plot([0, x1], [0, y1], "ro-", lw=4, label="Link 1")  
    ax2.plot([x1, x2], [y1, y2], "bo-", lw=4, label="Link 2")  
    
    # Hiển thị đầu bút với hình dạng khác nhau tùy thuộc vào trạng thái
    if pen_down:
        ax2.scatter(x2, y2, c="g", s=50, label="End Effector (Vẽ)")  
    else:
        ax2.scatter(x2, y2, c="orange", s=50, marker="^", label="End Effector (Không vẽ)")  
    
    # Vẽ lại đường đã vẽ
    # Xử lý các đoạn ngắt (khi x_drawn[i] là None)
    i = 0
    while i < len(x_drawn):
        if x_drawn[i] is None:
            i += 1
            continue
            
        line_x, line_y = [], []
        while i < len(x_drawn) and x_drawn[i] is not None:
            line_x.append(x_drawn[i])
            line_y.append(y_drawn[i])
            i += 1
            
        if line_x:
            ax2.plot(line_x, line_y, "k-", lw=2)
    
    ax2.legend(loc="upper right")
    ax2.set_title(f"Bước {frame+1}/{len(robot_points)} - Góc 1: {theta1:.2f}° - Góc 2: {theta2:.2f}°")
    ax2.grid(True)

    # Vẽ đồ thị các góc
    ax3.clear()
    ax3.plot(range(len(theta1_list)), theta1_list, "r-", label="Theta1 (°)")
    ax3.plot(range(len(theta2_list)), theta2_list, "b-", label="Theta2 (°)")
    ax3.set_ylim(-180, 180)
    ax3.legend()
    ax3.set_title("Góc quay của Robot")
    ax3.grid(True)
    
    # Thêm thời gian trễ để đồng bộ với robot thực tế
    if arduino_connected:
        plt.pause(0.1)  # Đợi robot thực hiện
    
    return []

def optimize_path(points, max_points=200):
    """Tối ưu hóa đường đi để có ít điểm hơn và thêm chức năng nhấc bút"""
    if len(points) <= max_points:
        return points
        
    # Lấy mẫu các điểm với khoảng cách đều
    indices = np.linspace(0, len(points) - 1, max_points, dtype=int)
    return [points[i] for i in indices]

def detect_pen_lift_points(contours, min_distance=50):
    """Phát hiện các điểm nên nhấc bút dựa trên khoảng cách giữa các contour"""
    path_with_lifts = []
    
    # Xử lý từng contour một
    for i, contour in enumerate(contours):
        # Bỏ qua các contour quá nhỏ
        if cv2.arcLength(contour, True) < 20:
            continue
            
        # Giảm số điểm với Douglas-Peucker
        epsilon = 0.005 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # Thêm lệnh nhấc bút nếu không phải contour đầu tiên
        if i > 0 and path_with_lifts:
            path_with_lifts.append(("pen_up", None))
            
        # Thêm lệnh đặt bút
        path_with_lifts.append(("pen_down", None))
        
        # Thêm các điểm của contour hiện tại
        for point in approx:
            x, y = point[0]
            path_with_lifts.append(("move", (x, y)))
    
    return path_with_lifts

def export_to_file(robot_points, pen_commands, filename="robot_path.txt"):
    """Xuất dữ liệu tọa độ và lệnh bút ra file để có thể tải lên Arduino"""
    with open(filename, "w") as f:
        f.write("# Robot SCARA drawing path\n")
        f.write("# Format: x,y,pen_state (1=down, 0=up)\n")
        
        for i, (x, y) in enumerate(robot_points):
            pen_state = 1 if (i < len(pen_commands) and pen_commands[i]) else 0
            f.write(f"{x:.2f},{y:.2f},{pen_state}\n")
    
    print(f"Đã xuất dữ liệu vẽ ra file {filename}")

def main():
    global x_drawn, y_drawn, theta1_list, theta2_list, ax1, ax2, ax3, image_original, robot_points, pen_down
    
    # Khởi tạo biến toàn cục
    x_drawn, y_drawn = [], []
    theta1_list, theta2_list = [], []
    pen_down = True  # Mặc định bút được hạ xuống
    
    # Hiển thị thông tin tỉ lệ vẽ
    print(f"Hình sẽ được vẽ với scale = {scale:.1f}")
    
    # Hiển thị các file ảnh trong thư mục
    image_files = find_image_files()
    
    # Yêu cầu người dùng nhập đường dẫn ảnh
    default_image = "tải xuống.png" if "tải xuống.png" in image_files else (image_files[0] if image_files else "")
    
    try:
        image_path = input(f"Nhập đường dẫn đến hình ảnh (Enter để dùng '{default_image}'): ") or default_image
    except:
        image_path = default_image
    
    if not image_path:
        print("Không có file ảnh để xử lý.")
        return
    
    # Xử lý ảnh và trích xuất các điểm
    print(f"Đang xử lý ảnh: {image_path}")
    try:
        image_original, image_points, pen_commands = extract_drawing_coordinates(image_path)
    except Exception as e:
        print(f"Lỗi khi xử lý ảnh: {e}")
        # Thử cách khác nếu lỗi
        image_original, image_points = extract_drawing_coordinates(image_path)
        pen_commands = [True] * len(image_points)  # Mặc định tất cả các điểm đều được vẽ
    
    if image_original is None or not image_points:
        print("Không thể xử lý ảnh hoặc không tìm thấy điểm nào.")
        return
    
    # Tối ưu hóa đường đi
    image_points = optimize_path(image_points)
    
    # Kiểm tra xem pen_commands có tồn tại không
    if 'pen_commands' in locals() and len(pen_commands) > 0:
        pen_commands = pen_commands[:len(image_points)]  # Cắt ngắn lại nếu cần
        print(f"Số lần nhấc/hạ bút: {pen_commands.count(False)}/{pen_commands.count(True)}")
    
    # Chuyển đổi sang tọa độ robot
    robot_points = convert_to_robot_coords(image_points)
    
    print(f"Số điểm cần vẽ ban đầu: {len(image_points)}")
    print(f"Số điểm sau khi tối ưu: {len(robot_points)}")
    
    # Xuất tọa độ ra file để tải lên Arduino (tùy chọn)
    export_to_file(robot_points, pen_commands)
    
    # Tạo figure và axes
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 5))
    
    ax1.axis("off")  
    ax2.set_xlim(-workspace_size, workspace_size)
    ax2.set_ylim(-workspace_size, workspace_size)
    ax3.set_ylim(-180, 180)
    
    # Hỏi người dùng có muốn tiếp tục không
    if arduino_connected:
        proceed = input("Arduino đã được kết nối. Bạn có muốn bắt đầu vẽ? (y/n): ")
        if proceed.lower() != 'y':
            print("Đã hủy vẽ.")
            return
    
    # Chạy animation với thời gian interval ngắn hơn để vẽ nhanh hơn
    ani = animation.FuncAnimation(fig, update, frames=len(robot_points), interval=5, repeat=False, blit=True)
    plt.tight_layout()
    plt.show()
    
    # Đóng kết nối Serial khi kết thúc
    if arduino_connected:
        arduino_serial.close()
        print("Đã đóng kết nối với Arduino")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import traceback
        print(f"Lỗi: {e}")
        print("Chi tiết lỗi:")
        traceback.print_exc()
        
        # Đảm bảo đóng kết nối Serial nếu có lỗi
        if 'arduino_serial' in globals() and arduino_connected:
            arduino_serial.close()
            print("Đã đóng kết nối với Arduino do lỗi")