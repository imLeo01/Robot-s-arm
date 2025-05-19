import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from PIL import Image, ImageTk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
import cv2
import os
import serial
import time
import threading

class RobotArmController:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Drawing Controller")
        self.root.geometry("1200x700")
        self.root.configure(bg="#f0f0f0")
        
        # Thiết lập biến
        self.arduino = None
        self.is_connected = False
        self.is_drawing = False
        self.stop_drawing = False
        
        # Robot parameters
        self.L1, self.L2 = 140, 120  # Chiều dài link 1 và 2 (mm)
        self.image_size = 400
        self.workspace_size = 300
        self.scale = self.workspace_size / self.image_size
        self.step_per_mm = 10  # Số bước/mm
        
        # COM port and baudrate
        self.com_port = tk.StringVar(value="COM14")
        self.baudrate = tk.IntVar(value=115200)
        
        # Ảnh mẫu - khởi tạo trước khi gọi setup_ui
        self.available_images = self.find_image_files()
        self.image_choice = tk.StringVar(value=self.available_images[0] if self.available_images else "")
        
        # Biến lưu ảnh và đường dẫn
        self.original_image = None
        self.drawing_path = []
        self.robot_path = []
        self.current_image = None
        self.prev_angles = [0, 0]
        
        # Khởi tạo UI sau khi các biến đã được chuẩn bị
        self.setup_ui()
        
        # Cập nhật danh sách ảnh
        self.update_image_list()
        
        # Animation variables
        self.animation = None
        self.current_frame = 0
        self.drawing_thread = None
        
    def find_image_files(self):
        """Tìm tất cả các file ảnh trong thư mục hiện tại"""
        extensions = ['.png', '.jpg', '.jpeg', '.bmp', '.gif']
        image_files = []
        
        for file in os.listdir('.'):
            if any(file.lower().endswith(ext) for ext in extensions):
                image_files.append(file)
        
        return image_files if image_files else ["sample.png"]
    
    def setup_ui(self):
        """Thiết lập giao diện người dùng"""
        # Tạo style
        self.style = ttk.Style()
        self.style.configure('TButton', font=('Arial', 11))
        self.style.configure('TLabel', font=('Arial', 11))
        self.style.configure('TFrame', background="#f0f0f0")
        self.style.configure('Emergency.TButton', foreground='white', background='red', font=('Arial', 12, 'bold'))
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Chia thành 3 phần: điều khiển (trái), hiển thị (giữa), thông tin (phải)
        control_frame = ttk.LabelFrame(main_frame, text="Điều khiển", padding=10)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        
        display_frame = ttk.LabelFrame(main_frame, text="Hiển thị", padding=10)
        display_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        info_frame = ttk.LabelFrame(main_frame, text="Thông tin", padding=10)
        info_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)
        
        # ===== Phần điều khiển =====
        # Kết nối Serial
        conn_frame = ttk.LabelFrame(control_frame, text="Kết nối", padding=5)
        conn_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(conn_frame, text="COM Port:").grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Entry(conn_frame, textvariable=self.com_port, width=10).grid(row=0, column=1, sticky=tk.W, pady=2)
        
        # Nút kết nối
        self.connect_btn = ttk.Button(conn_frame, text="Kết nối", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=2, padx=5, pady=2)
        
        # Tình trạng kết nối
        self.status_var = tk.StringVar(value="Chưa kết nối")
        ttk.Label(conn_frame, textvariable=self.status_var, foreground="red").grid(row=1, column=0, columnspan=3, sticky=tk.W)
        
        # Chọn ảnh
        image_frame = ttk.LabelFrame(control_frame, text="Chọn ảnh", padding=5)
        image_frame.pack(fill=tk.X, pady=5)
        
        self.image_combo = ttk.Combobox(image_frame, textvariable=self.image_choice, state="readonly", width=25)
        self.image_combo.pack(fill=tk.X, pady=5)
        self.image_combo.bind("<<ComboboxSelected>>", self.show_image_preview)
        
        btn_frame = ttk.Frame(image_frame)
        btn_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(btn_frame, text="Tải ảnh", command=self.load_image).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Làm mới", command=self.update_image_list).pack(side=tk.LEFT, padx=5)
        
        # Cài đặt vẽ
        settings_frame = ttk.LabelFrame(control_frame, text="Cài đặt vẽ", padding=5)
        settings_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(settings_frame, text="Ngưỡng cắt:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.threshold_var = tk.IntVar(value=128)
        threshold_slider = ttk.Scale(settings_frame, from_=0, to=255, variable=self.threshold_var, orient=tk.HORIZONTAL, length=150)
        threshold_slider.grid(row=0, column=1, padx=5, pady=2)
        threshold_slider.bind("<ButtonRelease-1>", self.process_current_image)
        
        ttk.Label(settings_frame, text="Đảo màu:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.invert_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(settings_frame, variable=self.invert_var, command=self.process_current_image).grid(row=1, column=1, sticky=tk.W, pady=2)
        
        # Điều khiển vẽ
        draw_frame = ttk.LabelFrame(control_frame, text="Điều khiển vẽ", padding=5)
        draw_frame.pack(fill=tk.X, pady=5)
        
        btn_frame2 = ttk.Frame(draw_frame)
        btn_frame2.pack(fill=tk.X, pady=5)
        
        self.draw_btn = ttk.Button(btn_frame2, text="Bắt đầu vẽ", command=self.start_drawing)
        self.draw_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_btn = ttk.Button(btn_frame2, text="Dừng vẽ", command=self.stop_drawing_command, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút khẩn cấp
        ttk.Button(draw_frame, text="DỪNG KHẨN CẤP", command=self.emergency_stop, style="Emergency.TButton").pack(fill=tk.X, pady=10)
        
        # ===== Phần hiển thị =====
        # Preview frame
        preview_frame = ttk.Frame(display_frame)
        preview_frame.pack(fill=tk.BOTH, expand=True)
        
        # Tạo frame hiển thị ảnh gốc
        self.preview_frame = ttk.LabelFrame(preview_frame, text="Ảnh gốc", padding=5)
        self.preview_frame.grid(row=0, column=0, sticky=tk.NSEW, padx=5, pady=5)
        
        self.preview_label = tk.Label(self.preview_frame, bg="white")
        self.preview_label.pack(fill=tk.BOTH, expand=True)
        
        # Tạo frame hiển thị đường nét
        self.path_frame = ttk.LabelFrame(preview_frame, text="Đường nét trích xuất", padding=5)
        self.path_frame.grid(row=0, column=1, sticky=tk.NSEW, padx=5, pady=5)
        
        self.fig_path = plt.Figure(figsize=(5, 4), dpi=100)
        self.ax_path = self.fig_path.add_subplot(111)
        self.canvas_path = FigureCanvasTkAgg(self.fig_path, master=self.path_frame)
        self.canvas_path.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Tạo frame hiển thị mô phỏng
        self.robot_frame = ttk.LabelFrame(preview_frame, text="Mô phỏng robot", padding=5)
        self.robot_frame.grid(row=1, column=0, columnspan=2, sticky=tk.NSEW, padx=5, pady=5)
        
        self.fig_robot = plt.Figure(figsize=(8, 4), dpi=100)
        self.ax_robot = self.fig_robot.add_subplot(111)
        self.canvas_robot = FigureCanvasTkAgg(self.fig_robot, master=self.robot_frame)
        self.canvas_robot.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Configure grid
        preview_frame.columnconfigure(0, weight=1)
        preview_frame.columnconfigure(1, weight=1)
        preview_frame.rowconfigure(0, weight=1)
        preview_frame.rowconfigure(1, weight=1)
        
        # ===== Phần thông tin =====
        # Thông tin robot
        robot_info_frame = ttk.LabelFrame(info_frame, text="Thông tin robot", padding=5)
        robot_info_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(robot_info_frame, text="Link 1:").grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Label(robot_info_frame, text=f"{self.L1} mm").grid(row=0, column=1, sticky=tk.W, pady=2)
        
        ttk.Label(robot_info_frame, text="Link 2:").grid(row=1, column=0, sticky=tk.W, pady=2)
        ttk.Label(robot_info_frame, text=f"{self.L2} mm").grid(row=1, column=1, sticky=tk.W, pady=2)
        
        ttk.Label(robot_info_frame, text="Tỷ lệ bước/mm:").grid(row=2, column=0, sticky=tk.W, pady=2)
        ttk.Label(robot_info_frame, text=f"{self.step_per_mm}").grid(row=2, column=1, sticky=tk.W, pady=2)
        
        # Thông tin vẽ
        drawing_info_frame = ttk.LabelFrame(info_frame, text="Thông tin vẽ", padding=5)
        drawing_info_frame.pack(fill=tk.X, pady=5)
        
        self.points_var = tk.StringVar(value="Số điểm: 0")
        ttk.Label(drawing_info_frame, textvariable=self.points_var).pack(anchor=tk.W, pady=2)
        
        self.progress_var = tk.StringVar(value="Tiến độ: 0%")
        ttk.Label(drawing_info_frame, textvariable=self.progress_var).pack(anchor=tk.W, pady=2)
        
        self.progress = ttk.Progressbar(drawing_info_frame, orient=tk.HORIZONTAL, length=200, mode='determinate')
        self.progress.pack(fill=tk.X, pady=5)
        
        # Góc hiện tại
        angle_frame = ttk.LabelFrame(info_frame, text="Góc hiện tại", padding=5)
        angle_frame.pack(fill=tk.X, pady=5)
        
        self.theta1_var = tk.StringVar(value="θ1: 0.0°")
        ttk.Label(angle_frame, textvariable=self.theta1_var).pack(anchor=tk.W, pady=2)
        
        self.theta2_var = tk.StringVar(value="θ2: 0.0°")
        ttk.Label(angle_frame, textvariable=self.theta2_var).pack(anchor=tk.W, pady=2)
        
        # Hướng dẫn
        guide_frame = ttk.LabelFrame(info_frame, text="Hướng dẫn", padding=5)
        guide_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        guide_text = """1. Kết nối với cổng COM
2. Chọn ảnh để vẽ
3. Điều chỉnh ngưỡng và đảo màu nếu cần
4. Nhấn 'Bắt đầu vẽ' để vẽ ảnh
5. Có thể dừng quá trình vẽ bất cứ lúc nào
6. Nút DỪNG KHẨN CẤP sẽ dừng robot ngay lập tức

Chú ý: Đảm bảo robot đã về vị trí home trước khi vẽ."""
        
        guide_label = ttk.Label(guide_frame, text=guide_text, wraplength=250, justify=tk.LEFT)
        guide_label.pack(fill=tk.BOTH, expand=True)
        
        # Footer
        footer_frame = ttk.Frame(self.root)
        footer_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(footer_frame, text="© 2025 Robot Drawing Controller - HCMUTE", font=('Arial', 9)).pack(side=tk.RIGHT)
    
    def update_image_list(self):
        """Cập nhật danh sách ảnh"""
        self.available_images = self.find_image_files()
        self.image_combo['values'] = self.available_images
        
        if self.available_images and not self.image_choice.get():
            self.image_choice.set(self.available_images[0])
            self.show_image_preview()
    
    def load_image(self):
        """Mở hộp thoại chọn file ảnh"""
        file_path = filedialog.askopenfilename(
            title="Chọn ảnh",
            filetypes=[("Image files", "*.png *.jpg *.jpeg *.bmp *.gif")]
        )
        
        if file_path:
            # Lấy tên file
            file_name = os.path.basename(file_path)
            
            # Nếu file không trong thư mục hiện tại, sao chép vào
            if not os.path.exists(file_name):
                # Đọc file ảnh
                img = Image.open(file_path)
                # Lưu vào thư mục hiện tại
                img.save(file_name)
            
            # Cập nhật danh sách ảnh
            self.update_image_list()
            
            # Chọn file mới
            self.image_choice.set(file_name)
            self.show_image_preview()
    
    def show_image_preview(self, event=None):
        """Hiển thị xem trước ảnh"""
        image_path = self.image_choice.get()
        
        if image_path and os.path.exists(image_path):
            try:
                # Hiển thị ảnh gốc
                img = Image.open(image_path).convert("L")
                img = img.resize((250, 250), Image.LANCZOS)
                img_tk = ImageTk.PhotoImage(img)
                self.preview_label.configure(image=img_tk)
                self.preview_label.image = img_tk
                
                # Lưu ảnh hiện tại
                self.current_image = image_path
                
                # Xử lý ảnh
                self.process_current_image()
            except Exception as e:
                messagebox.showerror("Lỗi", f"Không thể mở ảnh: {str(e)}")
                self.preview_label.configure(image=None, text="Lỗi hiển thị ảnh")
        else:
            self.preview_label.configure(image=None, text="Không tìm thấy ảnh.")
    
    def process_current_image(self, event=None):
        """Xử lý ảnh hiện tại để trích xuất đường nét"""
        if not self.current_image or not os.path.exists(self.current_image):
            return
        
        try:
            # Trích xuất đường nét từ ảnh
            threshold = self.threshold_var.get()
            invert = self.invert_var.get()
            
            self.original_image, self.drawing_path = self.extract_drawing_path(
                self.current_image, threshold, invert
            )
            
            # Chuyển sang tọa độ robot
            _, self.robot_path = self.convert_to_robot_coords(self.drawing_path)
            
            # Hiển thị đường nét
            self.show_drawing_path()
            
            # Cập nhật thông tin
            self.points_var.set(f"Số điểm: {len(self.robot_path)}")
            self.progress_var.set("Tiến độ: 0%")
            self.progress['value'] = 0
        except Exception as e:
            messagebox.showerror("Lỗi", f"Không thể xử lý ảnh: {str(e)}")
    
    def extract_drawing_path(self, image_path, threshold=128, invert=True):
        """Trích xuất đường nét từ ảnh"""
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        img_blur = cv2.GaussianBlur(img, (5, 5), 0)
        
        # Áp dụng ngưỡng
        if invert:
            _, binary = cv2.threshold(img_blur, threshold, 255, cv2.THRESH_BINARY_INV)
        else:
            _, binary = cv2.threshold(img_blur, threshold, 255, cv2.THRESH_BINARY)
        
        # Tìm contour
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Tạo danh sách điểm
        drawing_path = []
        for contour in contours:
            # Bỏ qua contour quá nhỏ
            if cv2.contourArea(contour) < 10:
                continue
                
            # Đơn giản hóa contour một chút
            epsilon = 0.002 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            for point in approx:
                x, y = point[0]
                drawing_path.append((x, y))
        
        return img, drawing_path
    
    def convert_to_robot_coords(self, drawing_path):
        """Chuyển đường nét từ tọa độ ảnh sang tọa độ robot"""
        robot_coords = []
        
        # Tìm kích thước ảnh
        if self.original_image is not None:
            height, width = self.original_image.shape
        else:
            height, width = self.image_size, self.image_size
        
        # Tỷ lệ chuyển đổi
        scale = self.workspace_size / max(width, height)
        
        # Điểm trước để kiểm tra, bắt đầu với pen_up
        prev_point = None
        
        for point in drawing_path:
            x_img, y_img = point
            
            # Chuyển sang tọa độ robot (gốc ở giữa)
            x_robot = (x_img - width/2) * scale
            y_robot = (height/2 - y_img) * scale
            
            # Xác định trạng thái bút (1: bút xuống, 0: bút lên)
            if prev_point is None:
                pen_state = 0  # Điểm đầu tiên, bút lên
            else:
                # Tính khoảng cách từ điểm trước
                dist = np.sqrt((x_img - prev_point[0])**2 + (y_img - prev_point[1])**2)
                if dist > 10:  # Nếu khoảng cách > 10 pixel, bút lên
                    robot_coords.append((x_robot, y_robot, 0))  # Thêm điểm với bút lên
                    pen_state = 1  # Điểm tiếp theo bút xuống
                else:
                    pen_state = 1  # Bút xuống để vẽ
            
            robot_coords.append((x_robot, y_robot, pen_state))
            prev_point = (x_img, y_img)
        
        # Kết thúc với bút lên
        if robot_coords:
            last_x, last_y, _ = robot_coords[-1]
            robot_coords.append((last_x, last_y, 0))
        
        return self.original_image, robot_coords
    
    def show_drawing_path(self):
        """Hiển thị đường nét trích xuất"""
        self.ax_path.clear()
        
        if not self.drawing_path:
            return
        
        # Vẽ đường nét
        x_vals = [p[0] for p in self.drawing_path]
        y_vals = [p[1] for p in self.drawing_path]
        
        self.ax_path.plot(x_vals, y_vals, 'b-', linewidth=1)
        self.ax_path.set_aspect('equal')
        self.ax_path.axis('off')
        
        self.canvas_path.draw()
    
    def simulate_robot_arm(self, robot_coords, frame_idx):
        """Mô phỏng cánh tay robot"""
        self.ax_robot.clear()
        
        if not robot_coords or frame_idx >= len(robot_coords):
            return
        
        # Lấy tọa độ và trạng thái bút
        x, y, pen = robot_coords[frame_idx]
        
        # Tính góc từ tọa độ bằng động học ngược
        angles = self.inverse_kinematics(x, y)
        if not angles:
            return
        
        theta1, theta2 = angles
        self.theta1_var.set(f"θ1: {theta1:.1f}°")
        self.theta2_var.set(f"θ2: {theta2:.1f}°")
        
        # Tính toán vị trí điểm nối và điểm cuối
        x1 = self.L1 * np.cos(np.radians(theta1))
        y1 = self.L1 * np.sin(np.radians(theta1))
        
        x2 = x1 + self.L2 * np.cos(np.radians(theta1 + theta2))
        y2 = y1 + self.L2 * np.sin(np.radians(theta1 + theta2))
        
        # Vẽ cánh tay
        self.ax_robot.plot([0, x1], [0, y1], 'ro-', linewidth=3, markersize=8)  # Link 1
        self.ax_robot.plot([x1, x2], [y1, y2], 'bo-', linewidth=3, markersize=8)  # Link 2
        
        # Hiển thị trạng thái bút
        pen_status = "Down" if pen == 1 else "Up"
        self.ax_robot.set_title(f"Mô phỏng robot - Điểm {frame_idx+1}/{len(robot_coords)} - Bút: {pen_status}")
        
        # Thiết lập giới hạn trục
        limit = self.L1 + self.L2 + 20
        self.ax_robot.set_xlim(-limit, limit)
        self.ax_robot.set_ylim(-limit, limit)
        self.ax_robot.set_aspect('equal')
        
        # Vẽ vùng làm việc
        circle = plt.Circle((0, 0), self.L1 + self.L2, fill=False, color='gray', linestyle='--')
        self.ax_robot.add_patch(circle)
        
        if abs(self.L1 - self.L2) > 1:
            inner_circle = plt.Circle((0, 0), abs(self.L1 - self.L2), fill=False, color='gray', linestyle='--')
            self.ax_robot.add_patch(inner_circle)
        
        # Vẽ trục tọa độ
        self.ax_robot.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        self.ax_robot.axvline(x=0, color='k', linestyle='-', alpha=0.3)
        
        # Cập nhật canvas
        self.canvas_robot.draw()
    
    def inverse_kinematics(self, x, y):
        """Tính động học ngược (x, y) -> (theta1, theta2)"""
        d = (x**2 + y**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        
        if abs(d) > 1:  # Điểm ngoài tầm với
            return None
            
        theta2 = np.arccos(d)
        theta1 = np.arctan2(y, x) - np.arctan2(self.L2 * np.sin(theta2), self.L1 + self.L2 * np.cos(theta2))
        
        # Chuyển từ radian sang độ
        return np.degrees(theta1), np.degrees(theta2)
    
    def toggle_connection(self):
        """Kết nối/ngắt kết nối với Arduino"""
        if self.is_connected:
            # Ngắt kết nối
            if self.arduino:
                self.arduino.close()
                self.arduino = None
                
            self.is_connected = False
            self.connect_btn.config(text="Kết nối")
            self.status_var.set("Đã ngắt kết nối")
        else:
            # Kết nối
            port = self.com_port.get()
            baudrate = self.baudrate.get()
            
            try:
                self.arduino = serial.Serial(port, baudrate)
                time.sleep(2)  # Chờ Arduino sẵn sàng
                
                self.is_connected = True
                self.connect_btn.config(text="Ngắt kết nối")
                self.status_var.set(f"Đã kết nối với {port}")
                
                # Gửi lệnh home
                self.send_command("HOME")
            except Exception as e:
                messagebox.showerror("Lỗi kết nối", f"Không thể kết nối với Arduino: {str(e)}")
    
    def send_command(self, command):
        """Gửi lệnh đến Arduino"""
        if not self.is_connected or not self.arduino:
            messagebox.showwarning("Cảnh báo", "Chưa kết nối với Arduino!")
            return False
            
        try:
            # Đảm bảo lệnh kết thúc bằng ký tự xuống dòng
            if not command.endswith('\n'):
                command += '\n'
                
            self.arduino.write(command.encode())
            return True
        except Exception as e:
            messagebox.showerror("Lỗi", f"Không thể gửi lệnh: {str(e)}")
            return False
    
    def move_physical_robot(self, prev_angles, theta1, theta2, pen):
        """Điều khiển robot thực tế"""
        if not self.is_connected or not self.arduino:
            return False
        
        try:
            # Tính delta góc và chuyển thành steps
            dx = int((theta1 - prev_angles[0]) * self.step_per_mm)
            dy = int((theta2 - prev_angles[1]) * self.step_per_mm)
            
            # Gửi lệnh quay
            self.send_command(f"G {dx} {dy}")
            
            # Điều khiển servo nhấc bút
            if pen == 1:
                self.send_command("PD")  # Hạ bút
            else:
                self.send_command("PU")  # Nâng bút
                
            # Chờ một chút để robot thực hiện lệnh
            time.sleep(0.05)
            
            return True
        except Exception as e:
            print(f"Lỗi điều khiển robot: {str(e)}")
            return False
    
    def start_drawing(self):
        """Bắt đầu quá trình vẽ"""
        if not self.is_connected:
            messagebox.showwarning("Cảnh báo", "Vui lòng kết nối với Arduino trước khi vẽ!")
            return
        
        if not self.robot_path:
            messagebox.showwarning("Cảnh báo", "Không có đường nét để vẽ. Vui lòng chọn ảnh!")
            return
        
        # Kiểm tra xem đang vẽ không
        if self.is_drawing:
            messagebox.showinfo("Thông báo", "Đang trong quá trình vẽ!")
            return
        
        # Hỏi người dùng xác nhận
        result = messagebox.askquestion("Xác nhận", "Bắt đầu quá trình vẽ? Đảm bảo robot đã ở vị trí home.")
        if result != 'yes':
            return
        
        # Cập nhật trạng thái
        self.is_drawing = True
        self.stop_drawing = False
        
        # Cập nhật nút
        self.draw_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        
        # Reset góc hiện tại
        self.prev_angles = [0, 0]
        
        # Bắt đầu vẽ trong một thread riêng biệt
        self.drawing_thread = threading.Thread(target=self.drawing_process)
        self.drawing_thread.daemon = True
        self.drawing_thread.start()
    
    def drawing_process(self):
        """Quá trình vẽ (chạy trong thread riêng)"""
        try:
            total_points = len(self.robot_path)
            print(f"Bắt đầu vẽ {total_points} điểm")
            
            # Lệnh về home trước khi bắt đầu
            self.send_command("HOME")
            self.send_command("PU")  # Nâng bút lên
            time.sleep(1)
            
            # Lặp qua từng điểm trong đường đi robot
            for i, (x, y, pen) in enumerate(self.robot_path):
                # Kiểm tra dừng
                if self.stop_drawing:
                    break
                
                # Hiển thị mô phỏng
                self.root.after(0, lambda idx=i: self.simulate_robot_arm(self.robot_path, idx))
                
                # Tính toán góc
                angles = self.inverse_kinematics(x, y)
                
                if not angles:
                    print(f"Bỏ qua điểm {i}: Ngoài tầm với ({x}, {y})")
                    continue
                
                theta1, theta2 = angles
                
                # Điều khiển robot thực tế
                self.move_physical_robot(self.prev_angles, theta1, theta2, pen)
                
                # Cập nhật góc hiện tại
                self.prev_angles = [theta1, theta2]
                
                # Cập nhật tiến độ
                progress = (i + 1) / total_points * 100
                self.root.after(0, lambda p=progress: self.update_progress(p))
                
                # Chờ một chút giữa các điểm
                time.sleep(0.05)
            
            # Nâng bút khi kết thúc
            self.send_command("PU")
            
            # Về home sau khi vẽ
            self.send_command("HOME")
            
        except Exception as e:
            self.root.after(0, lambda: messagebox.showerror("Lỗi", f"Lỗi trong quá trình vẽ: {str(e)}"))
        finally:
            # Cập nhật trạng thái
            self.is_drawing = False
            self.root.after(0, self.reset_drawing_ui)
    
    def update_progress(self, progress):
        """Cập nhật thanh tiến độ"""
        self.progress_var.set(f"Tiến độ: {progress:.1f}%")
        self.progress['value'] = progress
    
    def reset_drawing_ui(self):
        """Reset giao diện sau khi vẽ xong"""
        self.draw_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        
        if self.stop_drawing:
            messagebox.showinfo("Thông báo", "Quá trình vẽ đã bị dừng!")
        else:
            messagebox.showinfo("Thành công", "Vẽ hoàn thành!")
            self.progress_var.set("Tiến độ: 100%")
            self.progress['value'] = 100
    
    def stop_drawing_command(self):
        """Dừng quá trình vẽ"""
        if not self.is_drawing:
            return
            
        result = messagebox.askquestion("Xác nhận", "Bạn có chắc muốn dừng quá trình vẽ?")
        if result != 'yes':
            return
            
        self.stop_drawing = True
        
        # Dừng các động cơ
        self.send_command("PU")  # Nâng bút
    
    def emergency_stop(self):
        """Dừng khẩn cấp"""
        self.stop_drawing = True
        
        # Gửi lệnh dừng khẩn cấp
        self.send_command("STOP")
        
        messagebox.showwarning("Dừng khẩn cấp", "Lệnh dừng khẩn cấp đã được gửi!")
        
        # Reset UI
        self.reset_drawing_ui()


# Chạy ứng dụng
if __name__ == "__main__":
    root = tk.Tk()
    app = RobotArmController(root)
    root.mainloop()