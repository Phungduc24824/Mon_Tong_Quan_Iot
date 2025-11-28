#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from bottle import get, post, request, response, static_file, default_app
from AlphaBot import AlphaBot
from config import BOT_TOKEN, CHAT_ID

import threading
import requests
import io
import time
import socket
import subprocess
import RPi.GPIO as GPIO

from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput


# ===================== AUTO WIFI CONNECTION WITH FALLBACK AP =====================

# Danh sách WiFi ưu tiên (tự động thử kết nối)
WIFI_NETWORKS = [
    {"ssid": "Ego", "password": "123456789"},
    # Thêm WiFi khác nếu cần:
    # {"ssid": "Home_WiFi", "password": "password123"},
    # {"ssid": "Office_WiFi", "password": "office456"},
]

# Cấu hình Access Point (hotspot phát từ Pi)
AP_SSID = "AlphaBot_Config"
AP_PASSWORD = "alphabot123"
AP_CHANNEL = "6"
AP_IP = "192.168.4.1"

def check_wifi_connected():
    """Kiểm tra xem đã kết nối WiFi chưa"""
    try:
        result = subprocess.run(
            ["iwgetid", "-r"],
            capture_output=True,
            text=True,
            timeout=3
        )
        ssid = result.stdout.strip()
        if ssid:
            print(f"[WIFI] Đã kết nối: {ssid}")
            return True, ssid
        return False, None
    except Exception as e:
        print(f"[WIFI] Lỗi kiểm tra: {e}")
        return False, None

def connect_to_wifi(ssid, password, timeout=20):
    """Kết nối tới một WiFi cụ thể"""
    print(f"[WIFI] Đang thử kết nối tới '{ssid}'...")
    
    try:
        # Dừng các tiến trình WiFi cũ
        subprocess.run(["sudo", "killall", "wpa_supplicant"], 
                      stderr=subprocess.DEVNULL, timeout=3)
        time.sleep(1)
        
        # Tạo file cấu hình wpa_supplicant
        wpa_config = f'''ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=VN

network={{
    ssid="{ssid}"
    psk="{password}"
    key_mgmt=WPA-PSK
    scan_ssid=1
}}
'''
        
        config_file = "/tmp/wpa_temp.conf"
        with open(config_file, "w") as f:
            f.write(wpa_config)
        
        # Khởi động wpa_supplicant
        subprocess.run(
            ["sudo", "wpa_supplicant", "-B", "-i", "wlan0", 
             "-c", config_file, "-D", "nl80211,wext"],
            timeout=5
        )
        time.sleep(2)
        
        # Lấy IP bằng dhcpcd
        subprocess.run(["sudo", "dhcpcd", "wlan0"], timeout=8)
        
        # Kiểm tra kết nối
        start_time = time.time()
        while time.time() - start_time < timeout:
            connected, current_ssid = check_wifi_connected()
            if connected and current_ssid == ssid:
                # Lấy IP để hiển thị
                try:
                    ip_result = subprocess.run(
                        ["hostname", "-I"],
                        capture_output=True,
                        text=True,
                        timeout=3
                    )
                    ip = ip_result.stdout.strip().split()[0]
                    print(f"[WIFI] ✓ Kết nối thành công!")
                    print(f"[WIFI] SSID: {ssid}")
                    print(f"[WIFI] IP: {ip}")
                except:
                    print(f"[WIFI] ✓ Kết nối thành công tới {ssid}")
                return True
            time.sleep(2)
        
        print(f"[WIFI] ✗ Timeout kết nối tới '{ssid}'")
        return False
        
    except Exception as e:
        print(f"[WIFI] ✗ Lỗi kết nối '{ssid}': {e}")
        return False

def start_access_point():
    """Tạo Access Point (hotspot) từ Raspberry Pi"""
    print(f"[AP] Đang khởi động Access Point '{AP_SSID}'...")
    
    try:
        # Dừng các service WiFi
        subprocess.run(["sudo", "killall", "wpa_supplicant"], 
                      stderr=subprocess.DEVNULL, timeout=3)
        subprocess.run(["sudo", "killall", "hostapd"], 
                      stderr=subprocess.DEVNULL, timeout=3)
        subprocess.run(["sudo", "killall", "dnsmasq"], 
                      stderr=subprocess.DEVNULL, timeout=3)
        time.sleep(1)
        
        # Cấu hình IP tĩnh cho wlan0
        subprocess.run([
            "sudo", "ifconfig", "wlan0", AP_IP, 
            "netmask", "255.255.255.0", "up"
        ], timeout=5)
        
        # Tạo file cấu hình hostapd
        hostapd_conf = f"""interface=wlan0
driver=nl80211
ssid={AP_SSID}
hw_mode=g
channel={AP_CHANNEL}
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase={AP_PASSWORD}
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
"""
        
        hostapd_file = "/tmp/hostapd.conf"
        with open(hostapd_file, "w") as f:
            f.write(hostapd_conf)
        
        # Tạo file cấu hình dnsmasq (DHCP server)
        dnsmasq_conf = f"""interface=wlan0
dhcp-range=192.168.4.2,192.168.4.20,255.255.255.0,24h
domain=local
address=/alphabot.local/{AP_IP}
"""
        
        dnsmasq_file = "/tmp/dnsmasq.conf"
        with open(dnsmasq_file, "w") as f:
            f.write(dnsmasq_conf)
        
        # Khởi động hostapd
        subprocess.Popen(
            ["sudo", "hostapd", hostapd_file],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        time.sleep(3)
        
        # Khởi động dnsmasq
        subprocess.Popen(
            ["sudo", "dnsmasq", "-C", dnsmasq_file, "--no-daemon"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        time.sleep(2)
        
        print(f"[AP] ✓ Access Point đã khởi động!")
        print(f"[AP] SSID: {AP_SSID}")
        print(f"[AP] Password: {AP_PASSWORD}")
        print(f"[AP] IP: {AP_IP}")
        print(f"[AP] Truy cập: http://{AP_IP}:8000")
        print(f"[AP] Hoặc: http://alphabot.local:8000")
        return True
        
    except Exception as e:
        print(f"[AP] ✗ Lỗi khởi động AP: {e}")
        return False

def smart_wifi_connect():
    """
    Tự động kết nối WiFi thông minh:
    1. Kiểm tra xem đã kết nối chưa
    2. Thử kết nối các WiFi trong danh sách
    3. Nếu thất bại -> tạo Access Point
    """
    print("="*50)
    print("[WIFI] Bắt đầu quá trình kết nối WiFi...")
    print("="*50)
    
    # Bước 1: Kiểm tra đã kết nối chưa
    connected, current_ssid = check_wifi_connected()
    if connected:
        print(f"[WIFI] ✓ Đã kết nối sẵn tới '{current_ssid}'")
        return True
    
    # Bước 2: Thử kết nối các WiFi trong danh sách
    print(f"[WIFI] Tìm thấy {len(WIFI_NETWORKS)} WiFi để thử...")
    for wifi in WIFI_NETWORKS:
        if connect_to_wifi(wifi["ssid"], wifi["password"]):
            print("[WIFI] ✓ Kết nối WiFi thành công!")
            return True
        time.sleep(2)
    
    # Bước 3: Không kết nối được -> tạo Access Point
    print("[WIFI] ✗ Không kết nối được WiFi nào!")
    print("[WIFI] → Chuyển sang chế độ Access Point...")
    
    if start_access_point():
        print("\n" + "="*50)
        print("🔥 QUAN TRỌNG:")
        print(f"1. Kết nối điện thoại/laptop tới WiFi: {AP_SSID}")
        print(f"2. Mật khẩu: {AP_PASSWORD}")
        print(f"3. Mở trình duyệt: http://{AP_IP}:8000")
        print("="*50 + "\n")
        return False
    
    print("[WIFI] ✗ Không thể khởi động Access Point!")
    return False

# Tự động kết nối WiFi khi khởi động
smart_wifi_connect()


# ===================== MJPEG STREAMING =====================

class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        super().__init__()
        self.frame = None
        self.condition = threading.Condition()

    def write(self, buf):
        with self.condition:
            self.frame = bytes(buf)
            self.condition.notify_all()

    def flush(self):
        pass


output = StreamingOutput()
picam2 = None
CAMERA_OK = False

try:
    picam2 = Picamera2()
    video_config = picam2.create_video_configuration(main={"size": (640, 480)})
    picam2.configure(video_config)
    picam2.start_recording(JpegEncoder(), FileOutput(output))
    CAMERA_OK = True
    print("[CAM] Camera started.")
except Exception as e:
    print("[CAM] Camera init error:", e)
    picam2 = None
    CAMERA_OK = False


# ===================== GPIO / ROBOT BASE =====================

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

car = AlphaBot()

# Tốc độ manual (điều khiển tay)
cur_speed = 50  # 20–100

# Tốc độ cho chế độ AUTO
AUTO_BASE_SPEED = 40       # tiến / rẽ
AUTO_BASE_SPEED_NEAR = 30  # lùi / tiến chậm khi quá gần


# ===================== SERVO (PAN / TILT) =====================

SERVO_PIN_PAN = 27   # chỉnh lại nếu servo bạn nối chân khác
SERVO_PIN_TILT = 22

GPIO.setup(SERVO_PIN_PAN, GPIO.OUT)
GPIO.setup(SERVO_PIN_TILT, GPIO.OUT)

servo_pan_pwm = GPIO.PWM(SERVO_PIN_PAN, 50)   # 50Hz
servo_tilt_pwm = GPIO.PWM(SERVO_PIN_TILT, 50)

servo_pan_pwm.start(0)
servo_tilt_pwm.start(0)

current_pan = 90
current_tilt = 90


def _angle_to_duty(angle: int) -> float:
    """Chuyển góc (0–180) sang duty (xấp xỉ 2.5–12.5%)."""
    return 2.5 + (angle / 180.0) * 10.0


def set_pan_angle(angle: int):
    global current_pan
    angle = max(0, min(180, int(angle)))
    duty = _angle_to_duty(angle)
    print(f"[SERVO PAN] angle={angle}, duty={duty:.2f}")
    servo_pan_pwm.ChangeDutyCycle(duty)
    time.sleep(0.25)
    servo_pan_pwm.ChangeDutyCycle(0)
    current_pan = angle


def set_tilt_angle(angle: int):
    global current_tilt
    angle = max(0, min(180, int(angle)))
    duty = _angle_to_duty(angle)
    print(f"[SERVO TILT] angle={angle}, duty={duty:.2f}")
    servo_tilt_pwm.ChangeDutyCycle(duty)
    time.sleep(0.25)
    servo_tilt_pwm.ChangeDutyCycle(0)
    current_tilt = angle


# Đưa về giữa khi khởi động
set_pan_angle(90)
set_tilt_angle(90)


# ===================== SENSORS (ULTRASONIC ONLY) =====================

PIN_TRIG = 17        # TRIG -> GPIO17
PIN_ECHO = 5         # ECHO -> GPIO5

GPIO.setup(PIN_TRIG, GPIO.OUT)
GPIO.setup(PIN_ECHO, GPIO.IN)


def measure_distance(max_wait: float = 0.05):
    """
    Đo khoảng cách bằng HY-SRF05 / HC-SR04.
    Trả về: distance (cm) hoặc None nếu timeout.
    Có in thêm log [ULTRA] để debug.
    """
    # đảm bảo TRIG thấp
    GPIO.output(PIN_TRIG, GPIO.LOW)
    time.sleep(0.002)

    # xung 10 µs
    GPIO.output(PIN_TRIG, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(PIN_TRIG, GPIO.LOW)

    # chờ ECHO lên (LOW -> HIGH)
    start_wait = time.time()
    timeout = start_wait + max_wait
    while GPIO.input(PIN_ECHO) == 0:
        if time.time() > timeout:
            print("[ULTRA] timeout: ECHO vẫn LOW")
            return None

    start = time.time()

    # chờ ECHO xuống lại (HIGH -> LOW)
    timeout = start + max_wait
    while GPIO.input(PIN_ECHO) == 1:
        if time.time() > timeout:
            print("[ULTRA] timeout: ECHO vẫn HIGH")
            return None

    end = time.time()
    elapsed = end - start
    distance_cm = (elapsed * 34300.0) / 2.0

    # Log để thấy cảm biến có đọc được không
    print(f"[ULTRA] dist={distance_cm:.1f} cm")
    return distance_cm


# ===================== AUTO FOLLOW STATE =====================

AUTO_MODE = False
_auto_thread = None
_auto_lock = threading.Lock()


def auto_follow_loop():
    """
    Vòng lặp theo người kết hợp:
    - Siêu âm: tiến/lùi giữ khoảng cách
    - IR trái/phải: né vật cản, rẽ trái hoặc phải
    """
    global AUTO_MODE, AUTO_BASE_SPEED, AUTO_BASE_SPEED_NEAR

    TARGET_DIST = 25.0
    TOLERANCE   = 5.0
    TOO_CLOSE   = 10.0
    MAX_DETECT  = 80.0
    LOOP_DT = 0.05

    last_state = None
    print("[AUTO] auto_follow_loop started (ultrasonic + IR)")

    try:
        while True:
            if not AUTO_MODE:
                break

            base_speed      = max(20, min(100, AUTO_BASE_SPEED))
            base_speed_near = max(20, min(base_speed, AUTO_BASE_SPEED_NEAR))

            # --- ĐỌC CẢM BIẾN ---
            dist = measure_distance(max_wait=0.015)
            ir_left, ir_right = read_ir_sensors()

            # ===== QUYẾT ĐỊNH HÀNH ĐỘNG =====
            if ir_left and ir_right:
                state = "stop"
            elif ir_left and not ir_right:
                state = "turn_right_ir"
            elif ir_right and not ir_left:
                state = "turn_left_ir"
            else:
                # Không vật cản IR → dùng siêu âm
                if dist is None:
                    state = "stop"
                elif dist < TOO_CLOSE:
                    state = "back_slow"
                elif (TARGET_DIST - TOLERANCE) <= dist <= (TARGET_DIST + TOLERANCE):
                    state = "stop"
                elif dist > (TARGET_DIST + TOLERANCE) and dist <= MAX_DETECT:
                    state = "forward"
                else:
                    state = "stop"

            # ===== THỰC THI LỆNH =====
            if state != last_state:
                d_str = "???" if dist is None else f"{dist:4.1f}cm"
                print(f"[AUTO] state: {last_state} -> {state}, dist={d_str}, speed={base_speed}")

                if state == "forward":
                    car.setPWMA(base_speed)
                    car.setPWMB(base_speed)
                    car.forward()
                elif state == "back_slow":
                    car.setPWMA(base_speed_near)
                    car.setPWMB(base_speed_near)
                    car.backward()
                elif state == "turn_right_ir":
                    car.setPWMA(base_speed)
                    car.setPWMB(base_speed)
                    car.right()
                elif state == "turn_left_ir":
                    car.setPWMA(base_speed)
                    car.setPWMB(base_speed)
                    car.left()
                elif state == "stop":
                    car.stop()

                last_state = state

            time.sleep(LOOP_DT)

    finally:
        car.stop()
        print("[AUTO] auto_follow_loop stopped")


def start_auto_mode():
    """
    Bật chế độ AUTO:
    - Đặt AUTO_MODE = True
    - Tạo thread chạy auto_follow_loop() ở nền
    """
    global AUTO_MODE, _auto_thread
    with _auto_lock:
        if AUTO_MODE:
            return
        AUTO_MODE = True
        _auto_thread = threading.Thread(
            target=auto_follow_loop,
            daemon=True,
        )
        _auto_thread.start()
        print("[AUTO] mode ON")


def stop_auto_mode():
    """
    Tắt chế độ AUTO:
    - Đặt AUTO_MODE = False
    - Dừng xe
    """
    global AUTO_MODE
    with _auto_lock:
        if not AUTO_MODE:
            return
        AUTO_MODE = False
        print("[AUTO] mode OFF flag set")
    car.stop()


# ===================== SNAPSHOT / TELEGRAM =====================

_last_capture = None
_last_lock = threading.Lock()


def send_to_telegram(image_bytes: bytes):
    if not BOT_TOKEN:
        return False, "Thiếu BOT_TOKEN trong config.py"
    if not CHAT_ID:
        return False, "Thiếu CHAT_ID trong config.py"

    url = f"https://api.telegram.org/bot{BOT_TOKEN}/sendPhoto"
    files = {"photo": ("alphabot.jpg", image_bytes, "image/jpeg")}
    data = {"chat_id": CHAT_ID, "caption": "📸 Ảnh chụp từ AlphaBot"}

    try:
        r = requests.post(url, data=data, files=files, timeout=10)
        print("[TG]", r.status_code, r.text[:200])
        if r.status_code == 200:
            return True, "Đã gửi ảnh lên Telegram."
        else:
            return False, r.text
    except Exception as e:
        return False, f"Lỗi kết nối: {e}"


# ===================== ROUTES =====================

@get("/")
def index():
    return static_file("index.html", root=".")


@get("/video")
def video():
    if not CAMERA_OK:
        response.status = 500
        return "Camera not available"

    response.content_type = "multipart/x-mixed-replace; boundary=frame"

    def generate():
        while True:
            with output.condition:
                output.condition.wait()
                frame = output.frame
            if not frame:
                continue
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            )

    return generate()


# ----- MOTOR (manual) -----
@post("/motor")
def motor_cmd():
    global cur_speed

    # điều khiển tay -> tắt AUTO
    stop_auto_mode()

    code = request.forms.get("code") or ""
    speed = request.forms.get("speed")

    if speed:
        try:
            cur_speed = max(20, min(100, int(speed)))
            print("[MOTOR] Speed ->", cur_speed)
        except Exception as e:
            print("[MOTOR] speed parse error:", e)

    if not code:
        return "OK"

    # ĐẢO CHIỀU: forward trên web -> car.backward(), backward -> car.forward()
    if code == "forward":
        car.setPWMA(cur_speed)
        car.setPWMB(cur_speed)
        car.backward()
        print("[MOTOR] forward (mapped to car.backward)")

    elif code == "backward":
        car.setPWMA(cur_speed)
        car.setPWMB(cur_speed)
        car.forward()
        print("[MOTOR] backward (mapped to car.forward)")

    elif code == "turnleft":
        car.setPWMA(cur_speed)
        car.setPWMB(cur_speed)
        car.left()
        print("[MOTOR] left")

    elif code == "turnright":
        car.setPWMA(cur_speed)
        car.setPWMB(cur_speed)
        car.right()
        print("[MOTOR] right")

    elif code == "stop":
        car.stop()
        print("[MOTOR] stop")

    return "OK"



# ----- SERVO -----
def _servo_handler():
    global current_pan, current_tilt

    # lấy pan/tilt hoặc code từ GET/POST
    pan = request.params.get("pan")    # -1,0,+1
    tilt = request.params.get("tilt")  # -1,0,+1
    code = request.params.get("code")  # left/right/up/down/center

    print("[SERVO] raw pan, tilt, code =", pan, tilt, code)

    if code:
        c = code.lower()
        step = 5
        if "left" in c:
            set_pan_angle(current_pan - step)
        elif "right" in c:
            set_pan_angle(current_pan + step)
        elif "up" in c:
            set_tilt_angle(current_tilt - step)
        elif "down" in c:
            set_tilt_angle(current_tilt + step)
        elif c in ("center", "middle", "home", "reset"):
            set_pan_angle(90)
            set_tilt_angle(90)
        return "OK"

    if pan not in (None, ""):
        try:
            step = int(pan)
        except ValueError:
            step = 0
        if step == 0:
            set_pan_angle(90)
        else:
            set_pan_angle(current_pan + step * 5)

    if tilt not in (None, ""):
        try:
            step_tilt = int(tilt)
        except ValueError:
            step_tilt = 0
        if step_tilt == 0:
            set_tilt_angle(90)
        else:
            set_tilt_angle(current_tilt + step_tilt * 5)

    return "OK"


@post("/servo")
def servo_cmd_post():
    return _servo_handler()


@get("/servo")
def servo_cmd_get():
    return _servo_handler()


# ----- AUTO ON/OFF -----
def _auto_handler():
    cmd = (
        request.forms.get("cmd")
        or request.forms.get("mode")
        or request.query.get("cmd")
        or request.query.get("mode")
        or ""
    ).lower()

    if cmd in ("on", "start", "1", "auto", "follow"):
        start_auto_mode()
        return "AUTO_ON"
    elif cmd in ("off", "stop", "0", "manual"):
        stop_auto_mode()
        return "AUTO_OFF"
    elif cmd == "":
        return (
            "<h2>AlphaBot AUTO mode</h2>"
            "<p>Dùng:</p>"
            "<ul>"
            "<li><code>/auto?cmd=on</code> &rarr; bật auto follow</li>"
            "<li><code>/auto?cmd=off</code> &rarr; tắt auto follow</li>"
            "</ul>"
        )
    else:
        response.status = 400
        return f"Unknown cmd: {cmd}"


@post("/auto")
def auto_cmd_post():
    return _auto_handler()


@get("/auto")
def auto_cmd_get():
    return _auto_handler()


# ----- AUTO SPEED -----
@post("/auto_speed")
def auto_speed():
    """
    POST /auto_speed với body: speed=40..100
    """
    global AUTO_BASE_SPEED, AUTO_BASE_SPEED_NEAR

    sp = request.forms.get("speed") or request.query.get("speed")
    if not sp:
        return "NO_SPEED"

    try:
        v = int(sp)
    except ValueError:
        response.status = 400
        return "INVALID_SPEED"

    v = max(20, min(100, v))
    AUTO_BASE_SPEED = v
    AUTO_BASE_SPEED_NEAR = max(20, v - 10)

    print(f"[AUTO] set speed: BASE={AUTO_BASE_SPEED}, NEAR={AUTO_BASE_SPEED_NEAR}")
    return "AUTO_SPEED_OK"


# ----- SNAPSHOT / PREVIEW / TELEGRAM -----
@get("/preview.jpg")
def preview_jpg():
    global _last_capture
    with _last_lock:
        data = _last_capture
    if not data:
        response.status = 404
        return "No capture yet"
    response.content_type = "image/jpeg"
    return data


@post("/snapshot")
def snapshot():
    global _last_capture
    if not CAMERA_OK:
        response.status = 500
        return "Camera không sẵn sàng."

    with output.condition:
        frame = output.frame
    if not frame:
        response.status = 500
        return "Không lấy được frame từ camera."

    with _last_lock:
        _last_capture = bytes(frame)
    return "SNAP_OK"


@post("/send_last_capture")
def send_last_capture():
    with _last_lock:
        data = _last_capture
    if not data:
        response.status = 400
        return "Chưa có ảnh để gửi. Hãy nhấn Chụp trước."

    ok, msg = send_to_telegram(data)
    if ok:
        return msg
    response.status = 500
    return msg


# ----- WIFI CONFIG PAGE -----
@get("/wifi_config")
def wifi_config_page():
    """Trang cấu hình WiFi"""
    return static_file("wifi_config.html", root=".")


@post("/connect_wifi")
def connect_wifi_handler():
    """Xử lý kết nối WiFi mới"""
    ssid = request.forms.get("ssid") or ""
    password = request.forms.get("password") or ""
    
    if not ssid or not password:
        response.status = 400
        return "ERROR: Thiếu SSID hoặc password"
    
    print(f"[WIFI] Nhận yêu cầu kết nối tới '{ssid}'")
    
    # Thêm WiFi mới vào danh sách
    new_wifi = {"ssid": ssid, "password": password}
    if new_wifi not in WIFI_NETWORKS:
        WIFI_NETWORKS.insert(0, new_wifi)  # Thêm vào đầu danh sách
    
    # Lưu vào file để dùng lần sau
    try:
        import json
        with open("/home/pi/wifi_networks.json", "w") as f:
            json.dump(WIFI_NETWORKS, f, indent=2)
        print("[WIFI] Đã lưu cấu hình WiFi")
    except Exception as e:
        print(f"[WIFI] Không thể lưu cấu hình: {e}")
    
    # Thử kết nối
    if connect_to_wifi(ssid, password):
        return "SUCCESS: Đã kết nối WiFi"
    else:
        response.status = 500
        return "ERROR: Không thể kết nối WiFi"


@get("/scan_wifi")
def scan_wifi_handler():
    """Quét các WiFi xung quanh"""
    try:
        result = subprocess.run(
            ["sudo", "iwlist", "wlan0", "scan"],
            capture_output=True,
            text=True,
            timeout=10
        )
        
        # Parse SSID từ kết quả
        networks = []
        for line in result.stdout.split('\n'):
            if 'ESSID:' in line:
                ssid = line.split('ESSID:"')[1].split('"')[0]
                if ssid and ssid not in networks:
                    networks.append(ssid)
        
        response.content_type = 'application/json'
        import json
        return json.dumps(networks)
        
    except Exception as e:
        print(f"[WIFI] Lỗi quét WiFi: {e}")
        response.status = 500
        return json.dumps([])


# ----- STATIC FILES (CSS/JS/HÌNH) -----
@get("/<filename:path>")
def static_files(filename):
    # Cho phép truy cập các file tĩnh khác (css, js, img, ...)
    return static_file(filename, root=".")


# ===================== ENTRYPOINT =====================

if __name__ == "__main__":
    from wsgiref.simple_server import make_server, WSGIServer
    from socketserver import ThreadingMixIn

    class ThreadingWSGIServer(ThreadingMixIn, WSGIServer):
        daemon_threads = True

    try:
        # lấy IP LAN của Pi để in ra cho tiện
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            localhost = s.getsockname()[0]
            s.close()
        except Exception:
            localhost = "127.0.0.1"

        app = default_app()
        httpd = make_server("0.0.0.0", 8000, app, server_class=ThreadingWSGIServer)
        print(f"[HTTP] Threaded server on http://{localhost}:8000")
        httpd.serve_forever()
    finally:
        try:
            stop_auto_mode()
        except Exception:
            pass
        try:
            if CAMERA_OK and picam2 is not None:
                picam2.stop_recording()
                picam2.close()
        except Exception:
            pass
        try:
            servo_pan_pwm.stop()
        except Exception:
            pass
        try:
            servo_tilt_pwm.stop()
        except Exception:
            pass
        car.stop()
        try:
            GPIO.cleanup()
        except Exception:
            pass