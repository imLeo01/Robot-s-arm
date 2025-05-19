import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
import math
from matplotlib.patches import Circle

# Thông số robot SCARA
L1 = 145 
L2 = 130 
workspace_size = 200  
image_size = 500  
scale = workspace_size / image_size  

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

def extract_drawing_coordinates(image_path, scale=0.4):
    """Trích xuất tọa độ từ hình ảnh sử dụng thuật toán từ file gốc"""
    # Kiểm tra đường dẫn file
    if not os.path.exists(image_path):
        print(f"Không tìm thấy file: {image_path}")
        return None, []
        
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"Không thể đọc hình ảnh: {image_path}")
        return None, []
    
    # Cải thiện độ tương phản
    img = cv2.equalizeHist(img)
    
    # Áp dụng blur để làm mịn ảnh và loại bỏ nhiễu
    img_blur = cv2.GaussianBlur(img, (5, 5), 0)
    
    # Trích xuất cạnh với Canny
    edges = cv2.Canny(img_blur, 50, 150)
    
    # Tìm contour với phương pháp giản lược
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Sắp xếp contours theo kích thước
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    
    drawing_points = []
    for contour in contours[:10]:  # Chỉ lấy 10 contour lớn nhất
        # Bỏ qua các contour quá nhỏ
        if cv2.arcLength(contour, True) < 20:
            continue
            
        # Sử dụng Douglas-Peucker để giảm số điểm
        epsilon = 0.005 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        for point in approx:
            x, y = point[0]
            drawing_points.append((x * scale, y * scale))

    return img, drawing_points

def convert_to_robot_coords(image_points):
    """Chuyển đổi từ tọa độ ảnh sang tọa độ robot"""
    robot_coords = []
    for x_img, y_img in image_points:
        # Dịch chuyển gốc tọa độ từ góc trên bên trái sang giữa, đồng thời đổi chiều y
        x_robot = (x_img - image_size / 2) * scale  
        y_robot = (image_size / 2 - y_img) * scale  
        robot_coords.append((x_robot, y_robot))
    return robot_coords

def inverse_kinematics(x, y):
    """Tính toán động học nghịch"""
    # Tính khoảng cách từ gốc đến điểm
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

def optimize_path(points, max_points=200):
    """Tối ưu hóa đường đi để có ít điểm hơn"""
    if len(points) <= max_points:
        return points
        
    # Lấy mẫu các điểm với khoảng cách đều
    indices = np.linspace(0, len(points) - 1, max_points, dtype=int)
    return [points[i] for i in indices]

def visualize_robot_simulation(img, robot_points, angles):
    """Mô phỏng chuyển động của robot vẽ hình"""
    if not robot_points or not angles:
        print("Không có dữ liệu để mô phỏng")
        return
        
    # Tạo figure với 2 trục
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 7))
    
    # Hiển thị ảnh gốc ở bên trái
    if img is not None:
        ax1.imshow(img, cmap='gray')
    ax1.set_title('Ảnh Gốc')
    ax1.axis('off')
    
    # Thiết lập trục cho cánh tay robot
    ax2.set_xlim(-workspace_size, workspace_size)
    ax2.set_ylim(-workspace_size, workspace_size)
    ax2.set_aspect('equal')
    ax2.grid(True)
    ax2.set_title('Mô phỏng cánh tay robot')
    ax2.set_xlabel('X (mm)')
    ax2.set_ylabel('Y (mm)')
    
    # Vẽ vòng tròn giới hạn vùng làm việc
    workspace_circle = Circle((0, 0), L1 + L2, fill=False, color='red', linestyle='--', alpha=0.3)
    min_workspace_circle = Circle((0, 0), abs(L1 - L2), fill=False, color='red', linestyle='--', alpha=0.3)
    ax2.add_patch(workspace_circle)
    ax2.add_patch(min_workspace_circle)
    
    # Khởi tạo biến lưu vết
    x_drawn, y_drawn = [], []
    
    # Hàm cập nhật animation
    def update(frame):
        # Xóa trục vẽ cũ
        ax2.clear()
        
        # Thiết lập lại các thuộc tính của trục
        ax2.set_xlim(-workspace_size, workspace_size)
        ax2.set_ylim(-workspace_size, workspace_size)
        ax2.set_aspect('equal')
        ax2.grid(True)
        ax2.set_title(f'Bước {frame+1}/{len(robot_points)}')
        ax2.set_xlabel('X (mm)')
        ax2.set_ylabel('Y (mm)')
        
        # Vẽ lại giới hạn vùng làm việc
        workspace_circle = Circle((0, 0), L1 + L2, fill=False, color='red', linestyle='--', alpha=0.3)
        min_workspace_circle = Circle((0, 0), abs(L1 - L2), fill=False, color='red', linestyle='--', alpha=0.3)
        ax2.add_patch(workspace_circle)
        ax2.add_patch(min_workspace_circle)
        
        # Lấy tọa độ và góc hiện tại
        x, y = robot_points[frame]
        theta1, theta2 = angles[frame]
        
        # Tính toán vị trí các khớp
        (x1, y1), (x2, y2) = forward_kinematics(theta1, theta2)
        
        # Thêm điểm vào đường vẽ
        x_drawn.append(x2)
        y_drawn.append(y2)
        
        # Vẽ cánh tay robot
        ax2.plot([0, x1], [0, y1], 'ro-', linewidth=3, label='Link 1')
        ax2.plot([x1, x2], [y1, y2], 'bo-', linewidth=3, label='Link 2')
        
        # Vẽ các khớp
        ax2.plot(0, 0, 'ko', markersize=8)  # Khớp gốc
        ax2.plot(x1, y1, 'ko', markersize=6)  # Khớp giữa
        ax2.plot(x2, y2, 'go', markersize=6)  # Điểm đầu cuối
        
        # Vẽ đường di chuyển của điểm đầu cuối
        ax2.plot(x_drawn, y_drawn, 'g-', linewidth=1)
        
        # Hiển thị thông tin góc
        ax2.set_title(f'Bước {frame+1}/{len(robot_points)} - θ1: {theta1:.2f}°, θ2: {theta2:.2f}°')
        ax2.legend(loc='upper right')
        
        return []
    
    # Tạo animation
    from matplotlib.animation import FuncAnimation
    ani = FuncAnimation(fig, update, frames=len(robot_points), interval=20, blit=True, repeat=False)
    plt.tight_layout()
    plt.show()
    
    return ani

def visualize_results(img, image_points, robot_points, angles):
    """Hiển thị kết quả tính toán"""
    # Tạo 3 subplots
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))
    
    # Hiển thị ảnh gốc với các điểm được trích xuất
    if img is not None:
        ax1.imshow(img, cmap='gray')
        # Vẽ các điểm trên ảnh gốc
        x_vals = [p[0]/0.4 for p in image_points]
        y_vals = [p[1]/0.4 for p in image_points]
        ax1.scatter(x_vals, y_vals, c='r', s=5)
        ax1.plot(x_vals, y_vals, 'g-', linewidth=1, alpha=0.5)
    ax1.set_title(f'Ảnh Gốc và Đường Viền ({len(image_points)} điểm)')
    ax1.axis('off')
    
    # Hiển thị tọa độ robot (tọa độ Cartesian)
    ax2.set_xlim(-workspace_size, workspace_size)
    ax2.set_ylim(-workspace_size, workspace_size)
    ax2.grid(True)
    ax2.set_aspect('equal')
    
    # Vẽ giới hạn vùng làm việc
    workspace_circle = Circle((0, 0), L1 + L2, fill=False, color='red', linestyle='--', alpha=0.3)
    min_workspace_circle = Circle((0, 0), abs(L1 - L2), fill=False, color='red', linestyle='--', alpha=0.3)
    ax2.add_patch(workspace_circle)
    ax2.add_patch(min_workspace_circle)
    
    # Vẽ các điểm tọa độ robot
    x_vals = [p[0] for p in robot_points]
    y_vals = [p[1] for p in robot_points]
    ax2.scatter(x_vals, y_vals, c='b', s=5)
    ax2.plot(x_vals, y_vals, 'm-', linewidth=1)
    ax2.set_title(f'Tọa độ Robot ({len(robot_points)} điểm)')
    ax2.set_xlabel('X (mm)')
    ax2.set_ylabel('Y (mm)')
    
    # Hiển thị góc khớp
    theta1_vals = [a[0] for a in angles]
    theta2_vals = [a[1] for a in angles]
    
    ax3.plot(range(len(theta1_vals)), theta1_vals, 'r-', label='Theta1 (°)')
    ax3.plot(range(len(theta2_vals)), theta2_vals, 'b-', label='Theta2 (°)')
    ax3.set_ylim(-180, 180)
    ax3.grid(True)
    ax3.legend()
    ax3.set_title('Góc Quay Khớp')
    ax3.set_xlabel('Điểm')
    ax3.set_ylabel('Góc (°)')
    
    plt.tight_layout()
    plt.show()

def main():
    # Hiển thị các file ảnh trong thư mục
    image_files = find_image_files()
    
    # Yêu cầu người dùng nhập đường dẫn ảnh
    default_image = "doraemon.png" if "doraemon.png" in image_files else (image_files[0] if image_files else "")
    
    try:
        image_path = input(f"Nhập đường dẫn đến hình ảnh (Enter để dùng '{default_image}'): ") or default_image
    except:
        image_path = default_image
    
    if not image_path:
        print("Không có file ảnh để xử lý.")
        return
    
    # Xử lý ảnh và trích xuất các điểm
    print(f"Đang xử lý ảnh: {image_path}")
    img, image_points = extract_drawing_coordinates(image_path)
    
    if img is None or not image_points:
        print("Không thể xử lý ảnh hoặc không tìm thấy điểm nào.")
        return
    
    # Tối ưu hóa đường đi
    image_points = optimize_path(image_points)
    
    # Chuyển đổi sang tọa độ robot
    robot_points = convert_to_robot_coords(image_points)
    
    # Tính góc khớp cho mỗi điểm
    angles = []
    valid_points = []
    for point in robot_points:
        result = inverse_kinematics(point[0], point[1])
        if result is not None:
            angles.append(result)
            valid_points.append(point)
    
    robot_points = valid_points  # Cập nhật lại chỉ giữ các điểm hợp lệ
    
    print(f"Số điểm cần vẽ ban đầu: {len(image_points)}")
    print(f"Số điểm hợp lệ sau khi tính toán: {len(robot_points)}")