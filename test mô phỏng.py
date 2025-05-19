import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import math
import time
import os
from matplotlib.animation import FuncAnimation

# Thông số cánh tay robot (đơn vị mm)
LINK1_LENGTH = 100  # Độ dài link 1
LINK2_LENGTH = 80   # Độ dài link 2

# Kích thước không gian làm việc (mm)
WORKSPACE_WIDTH = 200
WORKSPACE_HEIGHT = 150

# Biến lưu trữ vị trí và góc hiện tại
current_angles = [0, 0]
current_position = [0, 0]
pen_down = False
drawing_points = []

# Chuyển đổi tọa độ Cartesian (x, y) sang góc khớp (inverse kinematics)
def inverse_kinematics(x, y):
    # Tính khoảng cách từ gốc đến điểm (x, y)
    d = math.sqrt(x**2 + y**2)
    
    # Kiểm tra xem điểm có nằm trong phạm vi hoạt động không
    if d > LINK1_LENGTH + LINK2_LENGTH or d < abs(LINK1_LENGTH - LINK2_LENGTH):
        print(f"Điểm ({x}, {y}) nằm ngoài phạm vi hoạt động!")
        return None
    
    # Định lý cosine để tính góc
    cos_angle2 = (d**2 - LINK1_LENGTH**2 - LINK2_LENGTH**2) / (2 * LINK1_LENGTH * LINK2_LENGTH)
    # Giới hạn giá trị trong khoảng [-1, 1] để tránh lỗi do làm tròn số
    cos_angle2 = min(1, max(-1, cos_angle2))
    
    # Góc khớp 2 (khớp khuỷu)
    angle2 = math.acos(cos_angle2)
    angle2_deg = math.degrees(angle2)
    
    # Góc khớp 1 (khớp vai)
    beta = math.atan2(y, x)
    alpha = math.acos((LINK1_LENGTH**2 + d**2 - LINK2_LENGTH**2) / (2 * LINK1_LENGTH * d))
    angle1 = beta - alpha
    angle1_deg = math.degrees(angle1)
    
    return angle1_deg, angle2_deg

# Chuyển đổi từ góc khớp sang tọa độ Cartesian (forward kinematics)
def forward_kinematics(angle1, angle2):
    # Chuyển đổi góc từ độ sang radian
    angle1_rad = math.radians(angle1)
    angle2_rad = math.radians(angle2)
    
    # Tính tọa độ của khớp thứ nhất
    x1 = LINK1_LENGTH * math.cos(angle1_rad)
    y1 = LINK1_LENGTH * math.sin(angle1_rad)
    
    # Tính tọa độ của khớp thứ hai (điểm cuối)
    x2 = x1 + LINK2_LENGTH * math.cos(angle1_rad + angle2_rad)
    y2 = y1 + LINK2_LENGTH * math.sin(angle1_rad + angle2_rad)
    
    return (x1, y1), (x2, y2)

# Vẽ cánh tay robot
def plot_arm(angle1, angle2, ax):
    # Tính tọa độ các khớp
    (x1, y1), (x2, y2) = forward_kinematics(angle1, angle2)
    
    # Vẽ các link
    ax.plot([0, x1], [0, y1], 'b-', linewidth=3, label='Link 1')
    ax.plot([x1, x2], [y1, y2], 'g-', linewidth=3, label='Link 2')
    
    # Vẽ các khớp
    ax.plot(0, 0, 'ro', markersize=10)  # Khớp gốc
    ax.plot(x1, y1, 'ro', markersize=8)  # Khớp giữa
    
    # Vẽ điểm đầu cuối (bút)
    if pen_down:
        ax.plot(x2, y2, 'ko', markersize=6)  # Bút đang ở vị trí hạ xuống
    else:
        ax.plot(x2, y2, 'ko', markersize=6, mfc='none')  # Bút đang ở vị trí nhấc lên
    
    return (x2, y2)

# Di chuyển cánh tay đến vị trí xác định (mô phỏng)
def move_to_position(x, y):
    global current_angles, current_position, drawing_points
    
    angles = inverse_kinematics(x, y)
    if angles:
        current_angles = angles
        current_position = (x, y)
        
        # Nếu bút đang được hạ xuống, ghi lại điểm vẽ
        if pen_down:
            drawing_points.append((x, y))
        
        return True
    return False

# Hạ bút xuống (mô phỏng)
def pen_down_sim():
    global pen_down, drawing_points, current_position
    pen_down = True
    # Thêm điểm hiện tại vào danh sách điểm vẽ
    drawing_points.append(current_position)
    print("Bút đã được hạ xuống")

# Nhấc bút lên (mô phỏng)
def pen_up_sim():
    global pen_down, drawing_points
    pen_down = False
    # Thêm None để ngắt đường vẽ
    if drawing_points and drawing_points[-1] is not None:
        drawing_points.append(None)
    print("Bút đã được nhấc lên")

# Kiểm tra đường dẫn tệp
def check_file_path(file_path):
    # Thử với đường dẫn chính xác
    if os.path.exists(file_path):
        return file_path
    
    # Thử với r prefix
    r_path = r"{}".format(file_path)
    if os.path.exists(r_path):
        return r_path
    
    # Thử thay thế dấu gạch chéo ngược bằng dấu gạch chéo thuận
    forward_slash_path = file_path.replace('\\', '/')
    if os.path.exists(forward_slash_path):
        return forward_slash_path
    
    # Thử xóa dấu ngoặc kép nếu có
    if file_path.startswith('"') and file_path.endswith('"'):
        clean_path = file_path[1:-1]
        if os.path.exists(clean_path):
            return clean_path
    
    print(f"Không thể tìm thấy tệp: {file_path}")
    print(f"Đường dẫn đầy đủ: {os.path.abspath(file_path)}")
    return None

# Xử lý hình ảnh
def process_image(image_path, threshold_value=127, min_contour_length=10):
    # Kiểm tra và sửa đường dẫn
    valid_path = check_file_path(image_path)
    if valid_path is None:
        return None
    
    print(f"Đang đọc tệp từ: {valid_path}")
    
    # Đọc hình ảnh
    img = cv2.imread(valid_path)
    if img is None:
        print(f"OpenCV không thể đọc hình ảnh từ {valid_path}")
        # Thử cách khác sử dụng thư viện PIL nếu có
        try:
            from PIL import Image
            import numpy as np
            pil_img = Image.open(valid_path)
            img = np.array(pil_img.convert('RGB'))
            # Chuyển từ RGB sang BGR (định dạng OpenCV)
            img = img[:, :, ::-1].copy()
            print("Đã đọc hình ảnh thành công bằng PIL")
        except Exception as e:
            print(f"Không thể đọc hình ảnh bằng PIL: {e}")
            return None
    
    # Hiển thị kích thước hình ảnh
    print(f"Kích thước hình ảnh: {img.shape}")
    
    # Chuyển đổi sang ảnh grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Làm mờ ảnh để giảm nhiễu
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Phát hiện cạnh bằng Canny
    edges = cv2.Canny(blur, threshold_value / 2, threshold_value)
    
    # Tìm các đường viền
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    
    # Lọc các đường viền nhỏ
    filtered_contours = [cnt for cnt in contours if cv2.arcLength(cnt, False) > min_contour_length]
    
    # Hiển thị số lượng đường viền
    print(f"Đã tìm thấy {len(filtered_contours)} đường viền")
    
    # Hiển thị kết quả
    result_img = img.copy()
    cv2.drawContours(result_img, filtered_contours, -1, (0, 255, 0), 2)
    
    # Hiển thị hình ảnh gốc và hình ảnh đã phát hiện đường viền
    plt.figure(figsize=(12, 6))
    plt.subplot(1, 2, 1)
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.title('Hình ảnh gốc')
    plt.subplot(1, 2, 2)
    plt.imshow(cv2.cvtColor(result_img, cv2.COLOR_BGR2RGB))
    plt.title('Đường viền đã phát hiện')
    plt.tight_layout()
    plt.show()
    
    return filtered_contours

# Vẽ mô phỏng các đường viền
def simulate_drawing(contours):
    global drawing_points
    
    if not contours:
        print("Không có đường viền để vẽ")
        return
    
    # Reset drawing points
    drawing_points = []
    
    # Tính toán kích thước hình ảnh
    all_points = np.vstack([cnt.reshape(-1, 2) for cnt in contours])
    min_x, min_y = all_points.min(axis=0)
    max_x, max_y = all_points.max(axis=0)
    
    # Hiển thị thông tin kích thước
    print(f"Kích thước hình: Chiều rộng={max_x-min_x}px, Chiều cao={max_y-min_y}px")
    
    # Tỷ lệ để chuyển đổi từ pixel sang kích thước thực tế (mm)
    scale_x = WORKSPACE_WIDTH / (max_x - min_x)
    scale_y = WORKSPACE_HEIGHT / (max_y - min_y)
    scale = min(scale_x, scale_y) * 0.9  # Sử dụng 90% không gian làm việc
    
    print(f"Tỷ lệ scale: {scale} mm/px")
    
    # Tạo hình vẽ
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))
    
    # Thiết lập trục cho cánh tay robot
    ax1.set_xlim(-WORKSPACE_WIDTH/2, WORKSPACE_WIDTH/2)
    ax1.set_ylim(-WORKSPACE_HEIGHT/2, WORKSPACE_HEIGHT/2)
    ax1.set_aspect('equal')
    ax1.grid(True)
    ax1.set_title('Mô phỏng cánh tay robot')
    ax1.set_xlabel('X (mm)')
    ax1.set_ylabel('Y (mm)')
    
    # Thiết lập trục cho hình vẽ
    ax2.set_xlim(-WORKSPACE_WIDTH/2, WORKSPACE_WIDTH/2)
    ax2.set_ylim(-WORKSPACE_HEIGHT/2, WORKSPACE_HEIGHT/2)
    ax2.set_aspect('equal')
    ax2.grid(True)
    ax2.set_title('Hình vẽ')
    ax2.set_xlabel('X (mm)')
    ax2.set_ylabel('Y (mm)')
    
    # Vẽ giới hạn vùng làm việc
    workspace_circle = Circle((0, 0), LINK1_LENGTH + LINK2_LENGTH, 
                             fill=False, color='red', linestyle='--', alpha=0.3)
    min_workspace_circle = Circle((0, 0), abs(LINK1_LENGTH - LINK2_LENGTH), 
                                 fill=False, color='red', linestyle='--', alpha=0.3)
    ax1.add_patch(workspace_circle)
    ax1.add_patch(min_workspace_circle)
    
    # Vị trí ban đầu
    move_to_position(LINK1_LENGTH, 0)
    pen_up_sim()
    
    # Vẽ cánh tay robot ở vị trí ban đầu
    plot_arm(current_angles[0], current_angles[1], ax1)
    
    # Danh sách các điểm cần vẽ
    drawing_sequence = []
    
    # Vị trí ban đầu (trung tâm của vùng làm việc)
    offset_x = 0
    offset_y = 0
    
    # Chuẩn bị dữ liệu vẽ từng đường viền
    for contour in contours:
        # Tối ưu hóa đường viền để giảm số điểm
        epsilon = 0.01 * cv2.arcLength(contour, True)
        approx_contour = cv2.approxPolyDP(contour, epsilon, False)
        
        # Lấy điểm đầu tiên
        start_point = approx_contour[0][0]
        x_start = (start_point[0] - min_x) * scale - WORKSPACE_WIDTH/2
        y_start = (start_point[1] - min_y) * scale - WORKSPACE_HEIGHT/2
        
        # Thêm lệnh nhấc bút và di chuyển
        drawing_sequence.append(("move", (x_start, y_start)))
        drawing_sequence.append(("pen_down", None))
        
        # Thêm các điểm còn lại
        for point in approx_contour[1:]:
            x = (point[0][0] - min_x) * scale - WORKSPACE_WIDTH/2
            y = (point[0][1] - min_y) * scale - WORKSPACE_HEIGHT/2
            drawing_sequence.append(("move", (x, y)))
        
        # Thêm lệnh nhấc bút
        drawing_sequence.append(("pen_up", None))
    
    # Chỉ số hiện tại trong dãy lệnh
    current_idx = 0
    
    # Hàm cập nhật cho animation
    def update(frame):
        nonlocal current_idx
        
        if current_idx < len(drawing_sequence):
            command, point = drawing_sequence[current_idx]
            
            # Xóa trục hiện tại để vẽ lại
            ax1.clear()
            ax1.set_xlim(-WORKSPACE_WIDTH/2, WORKSPACE_WIDTH/2)
            ax1.set_ylim(-WORKSPACE_HEIGHT/2, WORKSPACE_HEIGHT/2)
            ax1.set_aspect('equal')
            ax1.grid(True)
            ax1.set_title('Mô phỏng cánh tay robot')
            ax1.set_xlabel('X (mm)')
            ax1.set_ylabel('Y (mm)')
            
            # Vẽ lại giới hạn vùng làm việc
            workspace_circle = Circle((0, 0), LINK1_LENGTH + LINK2_LENGTH, 
                                     fill=False, color='red', linestyle='--', alpha=0.3)
            min_workspace_circle = Circle((0, 0), abs(LINK1_LENGTH - LINK2_LENGTH), 
                                         fill=False, color='red', linestyle='--', alpha=0.3)
            ax1.add_patch(workspace_circle)
            ax1.add_patch(min_workspace_circle)
            
            # Thực hiện lệnh
            if command == "move" and point:
                x, y = point
                if move_to_position(x, y):
                    # Vẽ cánh tay robot ở vị trí mới
                    end_pos = plot_arm(current_angles[0], current_angles[1], ax1)
            elif command == "pen_down":
                pen_down_sim()
                # Vẽ lại cánh tay
                plot_arm(current_angles[0], current_angles[1], ax1)
            elif command == "pen_up":
                pen_up_sim()
                # Vẽ lại cánh tay
                plot_arm(current_angles[0], current_angles[1], ax1)
            
            # Vẽ hình vẽ
            ax2.clear()
            ax2.set_xlim(-WORKSPACE_WIDTH/2, WORKSPACE_WIDTH/2)
            ax2.set_ylim(-WORKSPACE_HEIGHT/2, WORKSPACE_HEIGHT/2)
            ax2.set_aspect('equal')
            ax2.grid(True)
            ax2.set_title('Hình vẽ')
            ax2.set_xlabel('X (mm)')
            ax2.set_ylabel('Y (mm)')
            
            # Vẽ lại các điểm đã vẽ
            x_points = []
            y_points = []
            for point in drawing_points:
                if point is None:
                    # Vẽ đường nối các điểm đã thu thập
                    if x_points:
                        ax2.plot(x_points, y_points, 'k-')
                        x_points = []
                        y_points = []
                else:
                    # Thu thập điểm
                    x_points.append(point[0])
                    y_points.append(point[1])
            
            # Vẽ đường cuối cùng
            if x_points:
                ax2.plot(x_points, y_points, 'k-')
            
            current_idx += 1
        
        return []
    
    # Tạo animation
    ani = FuncAnimation(fig, update, frames=len(drawing_sequence) + 10, blit=True, interval=100)
    plt.tight_layout()
    plt.show()

# Hàm chính
def main():
    try:
        # Kiểm tra đường dẫn tệp trước
        test_path = r"C:\Users\USER\Documents\ROBOTICS 1\tải xuống.png"
        file_exists = os.path.exists(test_path)
        print(f"Kiểm tra đường dẫn ban đầu: {test_path}")
        print(f"Tệp tồn tại: {file_exists}")
        
        if not file_exists:
            print("Đường dẫn tuyệt đối:", os.path.abspath(test_path))
            # Thử tìm kiếm các tệp hình ảnh trong thư mục hiện tại
            current_dir = os.getcwd()
            print(f"Thư mục hiện tại: {current_dir}")
            image_files = [f for f in os.listdir(current_dir) if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif'))]
            if image_files:
                print("Các tệp hình ảnh trong thư mục hiện tại:")
                for img_file in image_files:
                    print(f"  - {img_file}")
        
        # Nhập đường dẫn đến hình ảnh
        image_path = input("Nhập đường dẫn đến hình ảnh: ")
        
        # Xử lý hình ảnh
        contours = process_image(image_path)
        if contours:
            # Hỏi người dùng có muốn mô phỏng không
            sim_choice = input("Bạn có muốn mô phỏng việc vẽ hình ảnh này không? (y/n): ")
            if sim_choice.lower() == 'y':
                simulate_drawing(contours)
                print("Mô phỏng hoàn tất!")
        
    except Exception as e:
        import traceback
        print(f"Lỗi: {e}")
        print("Chi tiết lỗi:")
        traceback.print_exc()

if __name__ == "__main__":
    main()