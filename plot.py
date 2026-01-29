import pandas as pd
import matplotlib.pyplot as plt

def plot_hasil():
    # Baca file csv
    try:
        data = pd.read_csv('data_simulasi.csv')
    except FileNotFoundError:
        print("File data_simulasi.csv tidak ditemukan. Run ir-sim dulu!")
        return

    plt.figure(figsize=(10, 6))

    # Plot Bola Asli
    plt.plot(data['true_x'], data['true_y'], 'g-', label='Jalur Asli (True)', linewidth=2)
    
    # Plot Noisy (Non-EKF)
    plt.scatter(data['noisy_x'], data['noisy_y'], c='r', alpha=0.3, s=10, label='Sensor Berisik (Non-EKF)')
    
    # Plot EKF
    plt.plot(data['ekf_x'], data['ekf_y'], 'b--', label='Hasil Filter (EKF)', linewidth=2)

    plt.title('Analisis Performa EKF pada Pergerakan Bola')
    plt.xlabel('Posisi X (m)')
    plt.ylabel('Posisi Y (m)')
    plt.legend()
    plt.grid(True)
    
    # Simpan gambar hasil plot
    plt.savefig('grafik_ekf.png')
    print("Grafik disimpan sebagai grafik_ekf.png")
    plt.show()

if __name__ == "__main__":
    plot_hasil()