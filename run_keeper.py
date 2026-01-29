import csv
import irsim
import math
import numpy as np
import ekf_module  

# 1. Setup Env & EKF
env = irsim.make('scenarios_keeper/s1_forward.yaml')
ekf = ekf_module.EKFBall() 

# Parameter Gawang (Sesuai deskripsimu)
X_GOAL = 1.0        # Garis gawang di x=1.0
Y_MIN = 2.0         # Tiang bawah
Y_MAX = 3.7         # Tiang atas
Y_CENTER = 2.85     # Titik tengah gawang

is_initialized = False

while True:
    env.step()
    robot = env.robot_list[0]
    bola = env.obstacle_list[0]

    # State Robot
    rx, ry, rtheta = robot.state[0].item(), robot.state[1].item(), robot.state[2].item()
    # State Bola (Ground Truth untuk noise)
    bx, by = bola.state[0].item(), bola.state[1].item()

    if not is_initialized:
        # Bola datang dari kanan (x=4) ke kiri (x=1), maka arahnya pi (3.14)
        ekf.init(bx, by, 0.0, math.pi)
        is_initialized = True

    # 2. Update EKF (Membersihkan noise)
    z = [float(bx + np.random.normal(0, 0.02)), float(by + np.random.normal(0, 0.02))]
    ekf.predict(0.1)
    ekf.update(z)
    
    bx_est, by_est = ekf.getPosition()
    vx_est, vy_est = ekf.getVelocity()

    # 3. LOGIKA PREDIKSI ARAH & KECEPATAN (INTERCEPT)
    # Persamaan garis lintasan bola: y = mx + c
    # Kita cari y saat x = X_GOAL
    if vx_est < -0.1: # Bola bergerak ke arah gawang
        # Waktu yang dibutuhkan bola untuk sampai ke garis gawang
        t_hit = (X_GOAL - bx_est) / vx_est
        
        if t_hit > 0:
            # Prediksi posisi Y saat bola sampai di X_GOAL
            y_predicted = by_est + (vy_est * t_hit)
            
            # Beri sedikit offset agar robot tidak pas di garis gawang (mencegah tabrakan dengan jaring)
            # Kita targetkan robot berhenti sedikit di depan garis gawang (misal x=1.1)
            y_target = y_predicted
        else:
            y_target = by_est
    else:
        # Jika bola diam atau menjauh, kembali ke tengah
        y_target = Y_CENTER

    # BATAST: Jangan biarkan target melebihi tiang gawang agar tidak menabrak
    # Dikurangi radius robot (0.15) agar tidak mentok tiang
    y_target = np.clip(y_target, Y_MIN + 0.15, Y_MAX - 0.15)

    # 4. KONTROL GERAK LANGSUNG (P-Controller)
    # Robot menghadap atas (1.57 rad), maka linear_vel mengontrol sumbu Y
    
    # Kontrol Sudut: Harus tetap tegak lurus (1.57 rad)
    target_theta = 1.57
    angle_error = math.atan2(math.sin(target_theta - rtheta), math.cos(target_theta - rtheta))
    
    # Kontrol Posisi Y: Seberapa jauh robot dari y_target
    error_y = y_target - ry
    
    # Kecepatan linear proporsional dengan error (makin dekat makin pelan agar tidak overshoot)
    # Gunakan gain yang cukup tinggi (misal 5.0) agar responsif
    linear_vel = 5.0 * error_y 
    angular_vel = 8.0 * angle_error

    # 5. SAFETY: Jika robot melenceng dari garis X=1.0, paksa kembali
    error_x = X_GOAL - rx
    # Jika robot terlalu jauh ke depan/belakang gawang, gunakan rotasi sedikit untuk kembali
    angular_vel += 2.0 * error_x

    # Batasi kecepatan maksimal
    linear_vel = np.clip(linear_vel, -1.2, 1.2)
    angular_vel = np.clip(angular_vel, -3.0, 3.0)

    # Set velocity [linear, angular]
    robot.set_velocity([linear_vel, angular_vel])

    # Render
    env.render(0.05)
    if env.done(): break

env.end()