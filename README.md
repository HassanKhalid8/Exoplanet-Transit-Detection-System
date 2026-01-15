# Planet Transit Detection System 🌌
**Signals and Systems Term Project - 5th Semester**

This project demonstrates a modular pipeline developed in MATLAB to detect exoplanetary transits using the transit photometry method. The system simulates the light curve of a distant star and extracts the planet's orbital characteristics from noisy time-series data using periodic correlation.

## 📡 Signals and Systems Concepts
This project applies fundamental engineering principles to astrophysical "curve light signals":

* **Signal Detrending:** Utilizing polynomial fitting to remove low-frequency "drift" from non-stationary signals. This acts as a High-Pass Filter operation to level the baseline.
* **Time-Domain Filtering:** Applying **Savitzky-Golay filters** to suppress high-frequency white noise. This is preferred over standard moving averages because it preserves the sharp "Ingress" and "Egress" points (edges) of the transit.
* **Correlation & Periodicity:** Implementing the **Box Least Squares (BLS) algorithm**, which acts as a **Matched Filter**. It correlates the data with a square-wave template to find periodic rectangular pulses.
* **System Characterization:** Mapping the "Output Signal" (flux) back to the "Input System" (orbital mechanics and planetary physics).

## 🛠️ System Architecture
The pipeline follows a modular architecture to ensure signal integrity:

1. **Normalization:** Raw flux is converted into a relative scale centered at 1.0 to easily identify small transit dips.
2. **Conditioning:** The signal is processed through detrending and Savitzky-Golay smoothing to remove stellar noise and sensor drift.
3. **Discovery (BLS Engine):** A **Modulo Operation** (Phase Folding) wraps the time-series data onto itself. If the period is correct, signals add constructively (Coherent Integration).
4. **Characterization:** Signal depth and period are converted into physical units like Planet Radius and Astronomical Units (AU).

## 📐 Mathematical Foundation
The system utilizes three primary formulas for characterization:

* **Transit Depth ($\delta$):** $\delta = \frac{\Delta Flux}{Flux_{baseline}} = (\frac{R_{planet}}{R_{star}})^{2}$
* **Kepler's Third Law:** $a = \sqrt[3]{\frac{GM_{star}P^{2}}{4\pi^{2}}}$
* **Signal-to-Noise Ratio (SNR):** $SNR = \frac{\delta}{\sigma}\sqrt{N_{transits}}$

## 💻 Usage
1. Open the project in MATLAB (Signal Processing Toolbox required).
2. Run the main function: `ExoplanetDetectionSystem`.
3. Use the **"Generate Test Data"** button to simulate a known planet like **WASP-12b** to verify the system's accuracy.
4. Analyze the **Periodogram** for the power spike at the detected period.

## 👥 Authors (FCSE)
* **Hassan Khalid** (2023435)
* **Saad Mirza** (2023498)
* **Moiz Kakakhel** (2023315)

---
**Supervised by:** Sir Zaheer  
**Course Instructor:** Dr. Hanif
