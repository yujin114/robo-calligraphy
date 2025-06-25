import os
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import Int32, Bool

from PIL import Image, ImageDraw, ImageFont, ImageTk
import tkinter as tk
from tkinter import Entry, Button, Radiobutton, IntVar, Label, Frame


class TextToImageApp(Node):
    def __init__(self):
        super().__init__('text_to_image_node')
        self.get_logger().info('Node 생성됨!')

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(Int32, 'font_style', qos)

        # 종료버튼 통신
        self.shutdown_pub = self.create_publisher(Bool, '/shutdown_signal', qos)


        # GUI 설정
        self.root = tk.Tk()
        self.root.title("텍스트 이미지 생성기")
        self.root.geometry("550x600")
        self.root.configure(bg='#f0f0f0')

        self.font_style_var = IntVar()
        self.font_style_var.set(0)

        Label(self.root, text="텍스트 입력:", bg='#f0f0f0', font=("Arial", 14, "bold")).pack(pady=5)
        self.entry = Entry(self.root, font=("Arial", 16))
        self.entry.pack(pady=5, ipadx=10, ipady=7, fill='x', padx=20)

        # 굵기 선택
        frame = Frame(self.root, bg='#f0f0f0')
        frame.pack(pady=5)

        self.radio_thin = Radiobutton(frame, text="얇게", variable=self.font_style_var, value=0, bg='#f0f0f0', font=("Arial", 13))
        self.radio_bold = Radiobutton(frame, text="굵게", variable=self.font_style_var, value=1, bg='#f0f0f0', font=("Arial", 13))
        self.radio_thin.pack(side='left', padx=10)
        self.radio_bold.pack(side='left', padx=10)

        # 글자 크기 선택
        self.font_size_var = IntVar()
        self.font_size_var.set(72)

        Label(self.root, text="글자 크기 선택:", bg='#f0f0f0', font=("Arial", 14, "bold")).pack(pady=5)
        size_frame = Frame(self.root, bg='#f0f0f0')
        size_frame.pack(pady=5)

        Radiobutton(size_frame, text="40pt", variable=self.font_size_var, value=40, bg='#f0f0f0', font=("Arial", 13)).pack(side='left', padx=10)
        Radiobutton(size_frame, text="50pt", variable=self.font_size_var, value=50, bg='#f0f0f0', font=("Arial", 13)).pack(side='left', padx=10)
        Radiobutton(size_frame, text="60pt", variable=self.font_size_var, value=60, bg='#f0f0f0', font=("Arial", 13)).pack(side='left', padx=10)

        self.button = Button(self.root, text="이미지 생성", command=self.on_button_click,
                             bg="#4CAF50", fg="white", font=("Arial", 14, "bold"))
        self.button.pack(pady=10, ipadx=12, ipady=6)

        # 종료 버튼
        self.quit_button = Button(self.root, text="종료", command=self.on_quit_click,
                          bg="#f44336", fg="white", font=("Arial", 14, "bold"))
        self.quit_button.pack(pady=5, ipadx=12, ipady=6)


        # 이미지 표시 프레임
        self.image_frame = Frame(self.root, bg='white', bd=2, relief='groove')
        self.image_frame.pack(pady=10, padx=20, fill='both', expand=True)

        self.image_label = Label(self.image_frame, bg='white')
        self.image_label.pack(padx=10, pady=10)

        # 이미지 저장 경로
        self.image_path = "/home/yujin/f6_ws/src/wr_robot/image/picture.png"
        
        os.makedirs(os.path.dirname(self.image_path), exist_ok=True)

        # 폰트 경로
        self.font_path_regular = "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"
        self.font_path_bold =   "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"


        # ROS spin
        self.spin_thread = threading.Thread(target=self.ros_spin_loop, daemon=True)
        self.spin_thread.start()

    def ros_spin_loop(self):
        self.get_logger().info("ROS spin 루프 시작됨")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def on_button_click(self):
        style = self.font_style_var.get()
        msg = Int32()
        msg.data = style
        self.publisher.publish(msg)
        self.get_logger().info(f"퍼블리시함: {msg.data}")

        text = self.entry.get()
        img_width, img_height = 400, 200  
        img = Image.new('RGB', (img_width, img_height), color='white')
        draw = ImageDraw.Draw(img)

        font_size = self.font_size_var.get()
        try:
            font = ImageFont.truetype(
                self.font_path_bold if style == 1 else self.font_path_regular,
                font_size
            )
        except IOError:
            font = ImageFont.load_default()
            self.get_logger().warn("기본 폰트 사용 중 (NanumGothic을 찾을 수 없음)")

        try:
            bbox = draw.textbbox((0, 0), text, font=font)
            text_width = bbox[2] - bbox[0]
            text_height = bbox[3] - bbox[1]
        except Exception as e:
            self.get_logger().error(f"textbbox 오류: {e}")
            return

        text_x = (img_width - text_width) / 2
        text_y = (img_height - text_height) / 2
        draw.text((text_x, text_y), text, fill='black', font=font)

        try:
            img.save(self.image_path)
            self.get_logger().info(f"이미지를 저장함: {self.image_path}")
        except Exception as e:
            self.get_logger().error(f"이미지 저장 실패: {e}")
            return

        img_tk = ImageTk.PhotoImage(img)
        self.image_label.configure(image=img_tk)
        self.image_label.image = img_tk  # 참조 유지

    def run(self):
        self.root.mainloop()

    def on_quit_click(self):
        msg = Bool()
        msg.data = True
        self.shutdown_pub.publish(msg)
        self.get_logger().info("📢 종료 신호 퍼블리시 완료 (/shutdown_signal)")
        self.root.destroy()

def main():
    rclpy.init()
    app = TextToImageApp()
    app.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
