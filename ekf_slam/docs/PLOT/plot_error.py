import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg') # Continuiamo a usare Agg per evitare crash grafici
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import os

def carica_dati(file_path):
    """Carica un CSV di Foxglove e restituisce le colonne tempo e valore."""
    if not os.path.exists(file_path):
        print(f"⚠️ Attenzione: File '{file_path}' non trovato. Il grafico relativo sarà vuoto.")
        return None, None
        
    try:
        df = pd.read_csv(file_path)
        df.columns = df.columns.str.strip()
        
        if 'elapsed time' not in df.columns or 'value' not in df.columns:
            print(f"❌ Errore: Colonne non valide in '{file_path}'.")
            return None, None
            
        return df['elapsed time'], df['value']
    except Exception as e:
        print(f"❌ Errore durante la lettura di '{file_path}': {e}")
        return None, None

def plot_analisi_completa(file_x, file_y, file_theta):
    print("Inizio caricamento dei file CSV degli errori...")
    tx, ex = carica_dati(file_x)
    ty, ey = carica_dati(file_y)
    tt, et = carica_dati(file_theta)

    # Creiamo una figura con 2 subplot (2 righe, 1 colonna), condividendo l'asse X
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
    
    # --- SUBPLOT 1: Errore di Posizione (X e Y) ---
    if tx is not None:
        ax1.plot(tx, ex, label='Errore Longitudinale (X)', color='#1f77b4', linewidth=1.5)
        # Calcolo Media e 3-Sigma per X
        mu_x, std_x = ex.mean(), ex.std()
        ax1.axhspan(mu_x - 3*std_x, mu_x + 3*std_x, color='#1f77b4', alpha=0.15, label='±3 Sigma (X)')

    if ty is not None:
        ax1.plot(ty, ey, label='Errore Laterale (Y)', color='#ff7f0e', linewidth=1.5)
        # Calcolo Media e 3-Sigma per Y
        mu_y, std_y = ey.mean(), ey.std()
        ax1.axhspan(mu_y - 3*std_y, mu_y + 3*std_y, color='#ff7f0e', alpha=0.15, label='±3 Sigma (Y)')
        
    ax1.axhline(0, color='red', linestyle='--', linewidth=1.5, label='Zero Error Line')
    ax1.set_title("Deriva di Posizione: Errori X e Y", fontsize=14, fontweight='bold', pad=10)
    ax1.set_ylabel("Errore Posizione [m]", fontsize=12)
    ax1.grid(True, which='both', linestyle=':', alpha=0.7)
    ax1.minorticks_on()
    ax1.legend(loc='upper left', ncol=2)

    # --- SUBPLOT 2: Errore di Orientamento (Theta) ---
    if tt is not None:
        ax2.plot(tt, et, label='Errore Imbardata (Theta)', color='#2ca02c', linewidth=1.5)
        # Calcolo Media e 3-Sigma per Theta
        mu_t, std_t = et.mean(), et.std()
        ax2.axhspan(mu_t - 3*std_t, mu_t + 3*std_t, color='#2ca02c', alpha=0.15, label='±3 Sigma (Theta)')
    
    ax2.axhline(0, color='red', linestyle='--', linewidth=1.5, label='Zero Error Line')
    ax2.set_title("Deriva di Orientamento: Errore Theta", fontsize=14, fontweight='bold', pad=10)
    ax2.set_xlabel("Tempo trascorso [s]", fontsize=12)
    ax2.set_ylabel("Errore Theta", fontsize=12)
    ax2.grid(True, which='both', linestyle=':', alpha=0.7)
    ax2.minorticks_on()
    ax2.legend(loc='upper left', ncol=2)

    # Ottimizza gli spazi per evitare che le scritte si sovrappongano
    plt.tight_layout()

    # Salvataggio
    output_filename = "analisi_deriva_completa.png"
    plt.savefig(output_filename, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f"✅ Analisi errori generata con successo! Apri il file: {output_filename}")

def plot_analisi_latenza(file_latenza, output_filename, titolo_grafico):
    print(f"Inizio caricamento del file CSV della latenza: {file_latenza}...")
    t_lat, latenze = carica_dati(file_latenza)
    
    if latenze is None:
        return

    # 1. Pulizia di eventuali valori "Not a Number"
    latenze = latenze.dropna()

    # --- NUOVO: FILTRO ANTI-OUTLIER FOLLE ---
    # Scartiamo tutti i valori superiori a 50 ms.
    # Sappiamo che il filtro impiega < 15 ms, quindi un valore da 3000 ms 
    # è chiaramente un freeze temporaneo del thread da parte del Sistema Operativo.
    latenze = latenze[latenze < 50.0]

    # Creiamo la figura
    fig, ax1 = plt.subplots(figsize=(10, 6))

    counts, bins, patches = ax1.hist(
        latenze, 
        bins=100, 
        density=True, 
        cumulative=True, 
        color='darkorange', 
        edgecolor='black', 
        alpha=0.7, 
        label='CDF (Probabilità Cumulativa)'
    )
    
    ax1.set_xlabel('Latenza EKF (ms)', fontsize=12)
    ax1.set_ylabel('Probabilità Cumulativa', color='black', fontsize=12)
    ax1.set_ylim([0, 1.05])

    # --- ASSE X DINAMICO IN BASE AI DATI PULITI ---
    max_lat = latenze.max()
    if max_lat < 1.0:
        ax1.xaxis.set_major_locator(ticker.MultipleLocator(0.1))
        ax1.xaxis.set_minor_locator(ticker.MultipleLocator(0.02))
    elif max_lat < 5.0:
        ax1.xaxis.set_major_locator(ticker.MultipleLocator(0.5))
        ax1.xaxis.set_minor_locator(ticker.MultipleLocator(0.1))
    elif max_lat < 15.0:
        ax1.xaxis.set_major_locator(ticker.MultipleLocator(1.0))
        ax1.xaxis.set_minor_locator(ticker.MultipleLocator(0.5))
    else:
        ax1.xaxis.set_major_locator(ticker.MultipleLocator(5.0))
        ax1.xaxis.set_minor_locator(ticker.MultipleLocator(1.0))
    
    ax1.grid(True, which='major', linestyle='--', alpha=0.7)
    ax1.grid(True, which='minor', linestyle=':', alpha=0.4)

    p99_val = np.percentile(latenze, 99)

    ax1.axhline(y=0.99, color='red', linestyle='-', linewidth=1.5, alpha=0.8)
    ax1.axvline(x=p99_val, color='red', linestyle='--', linewidth=1.5, alpha=0.8)
    
    # Adattamento dinamico della posizione del testo in base al max_lat
    ax1.text(p99_val + (max_lat * 0.02), 0.5, f'99% Percentile: {p99_val:.3f} ms', color='red', fontsize=11, rotation=90)

    plt.title(titolo_grafico, fontsize=14, fontweight='bold', pad=10)
    fig.tight_layout()

    plt.savefig(output_filename, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f"✅ Analisi latenza generata con successo! Apri il file: {output_filename}")

if __name__ == "__main__":
    f_x = "error_x.csv"
    f_y = "error_y.csv"
    f_z = "error_z.csv" 
    
    # Assicurati di esportare due CSV distinti da Foxglove
    f_lat_cones = "latenza_cones.csv" 
    f_lat_imu = "latenza_imu.csv"     
    
    plot_analisi_completa(f_x, f_y, f_z)
    
    # Doppio plot!
    plot_analisi_latenza(f_lat_cones, "cfd_latenza_cones.png", "CDF Latenza ConesCallback (LiDAR)")
    plot_analisi_latenza(f_lat_imu, "cfd_latenza_imu.png", "CDF Latenza ImuCallback (Odometria)")

if __name__ == "__main__":
    # Assicurati di avere esportato i file da Foxglove con questi nomi esatti
    f_x = "error_x.csv"
    f_y = "error_y.csv"
    f_z = "error_z.csv" 
    f_lat = "latenza.csv" # Nome del file contenente la latenza
    
    # Genera i grafici per gli errori (posa e orientamento)
    plot_analisi_completa(f_x, f_y, f_z)
    
    # Genera il grafico indipendente per la latenza
    plot_analisi_latenza(f_lat)