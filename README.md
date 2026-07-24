 # WiFi CSI Indoor Localization for Autonomous Drones
//
This repository contains the investigation and implementation of WiFi Channel State Information (CSI) for GPS-denied indoor drone navigation. Traditional methods like cameras and IMUs degrade in low-light or dusty environments, such as warehouses. This project explores extracting per-subcarrier measurements from WiFi signals to map environmental geometry, identifies the terminal limitations of existing software-based CSI algorithms for moving targets, and proposes a custom hardware-accelerated Time Difference of Arrival (TDoA) ASIC architecture.
//
// ---
//
## Part 1: Literature Review & Evaluation
//
Current research heavily focuses on extracting spatial data from commodity WiFi Network Interface Cards (NICs). Two prominent methodologies are SpotFi and DeepFi.
//
### 1. SpotFi (Kotaru et al.)
SpotFi utilizes a modified 2D Multiple Signal Classification (MUSIC) algorithm to extract both Angle of Arrival (AoA) and Time of Flight (ToF) from the raw CSI matrix.
//
Technique:
It smooths the CSI matrix across antennas and subcarriers to compute a spatial covariance matrix. It then searches a mathematical grid for peaks where the signal vector is orthogonal to the noise subspace:


$$P_{MUSIC}(\theta, \tau) = \frac{1}{\mathbf{a}^H(\theta, \tau) \mathbf{E}_n \mathbf{E}_n^H \mathbf{a}(\theta, \tau)}$$


SpotFi applies a Gaussian likelihood function across consecutive packets to filter out multipath reflections and isolate the direct Line-of-Sight (LoS) path.

 Advantages:
 * Achieves sub-meter accuracy for static targets without requiring prior environmental mapping.
 * Mathematically filters out multipath interference rather than being confused by it.
//
 ### 2. DeepFi (Wang et al.)
 DeepFi treats indoor localization as an image classification problem using Deep Belief Networks (DBNs).
//
 Technique:
 It relies entirely on CSI amplitude data, ignoring phase. During an offline phase, it records the CSI amplitude signature (fingerprint) at various physical coordinates. A deep neural network is trained to associate these multipath signatures with specific locations.
//
 Advantages:
 * Bypasses complex geometric calculations and hardware phase unsynchronization.
 * Functions effectively in Non-Line-of-Sight (NLoS) environments where direct paths do not exist.
//
// ---
//
 ## The Drone Problem: Why Software CSI Breaks Down
//
 While SpotFi and DeepFi are functional for static smartphones or laptops, they structurally fail when the target collecting or transmitting the CSI is a dynamic drone.
//
 1. SpotFi & Drone Dynamics: AoA requires strict spatial coherency across the antenna array. A drone experiences severe motor vibration, rapid pitch/roll tilting, and spatial translation. This constantly alters the geometric baseline of the antennas, destroying the phase relationships required for the MUSIC spectrum. Furthermore, 2.4 GHz signal wavelengths ($\approx 12.5\text{ cm}$) are too large to form a reliable spatial array on a compact drone frame without spatial aliasing.
 2. DeepFi & Dynamic Environments: DeepFi maps a specific multipath environment. A moving drone introduces Doppler shifts and alters the multipath environment via propeller turbulence. Fingerprinting requires exhaustive, constant recalibration.
 3. The AGC Hardware Quirk: Commodity WiFi modules use Automatic Gain Control (AGC) which artificially inflates weak signals and attenuates strong ones before they hit the ADC. This invalidates the inverse-square law, making raw CSI amplitude mathematically useless for absolute distance ranging.
//
// ---
//
 ## Proposed Direction: Macroscopic Hardware TDoA (ASIC Architecture)
//
 To reliably track a drone in a closed space, we must abandon compact AoA arrays and machine learning fingerprinting. Instead, we propose a macroscopic Time Difference of Arrival (TDoA) multilateration system.
//
// Standard 40 MHz WiFi modules can only sample time in $25\text{ ns}$ increments (a $7.5\text{ meter}$ resolution). To achieve sub-nanosecond precision, we propose designing a custom peripheral Time-to-Digital Converter (TDC) ASIC. These passive sniffer chips will be placed at the extremities of the tracking volume (e.g., room corners).
//
 ### 1. The Multi-Phase Clocking Core
// Instead of clocking the ASIC at impossible GHz frequencies to get high precision, the architecture relies on multi-phase clock generation.
 * A base $100\text{ MHz}$ clock provides a $10\text{ ns}$ coarse timing period.
 * A high-performance internal Delay-Locked Loop (DLL) or Phase-Locked Loop (PLL) splits this base clock into 64 equally phase-delayed $100\text{ MHz}$ clocks.
 * This provides a timing resolution of $10\text{ ns} / 64 = \mathbf{156.25\text{ ps}}$. In physical space, this translates to a spatial resolution of roughly $4.68\text{ cm}$ ($c \times 156.25\text{ ps}$).
//
 ### 2. Lookup Table (LUT) Preamble Trigger
 The ASIC acts as a passive physical-layer sniffer. It downconverts the 2.4 GHz RF signal and digitizes the baseband.
 * The incoming digital sequence is continuously fed into a high-speed hardware correlator.
 * This correlator compares the incoming data stream against a hard-coded Lookup Table (LUT) containing the expected synchronization words.
 * The moment a strict match occurs, the correlator fires a synchronized, single-cycle strobe signal.
//
 ### 3. The Shadow Register Transfer
 * The 64 phase-delayed clocks are continuously driving a 64-bit sampling bus.
 * When the LUT strobe fires, the instantaneous state of all 64 clock lines is latched directly into a 64-bit shadow register.
 * By analyzing the sampled bits in this shadow register, the ASIC determines exactly which of the 64 clock phases the preamble arrived on, yielding the 156-picosecond timestamp.
//
 ### 4. Required Software Modifications
 For this hardware to function, a strict modification must be made to the drone's transmission software. Standard WiFi protocol headers are highly variable. To ensure the ASIC's LUT correlator triggers accurately and consistently, all drone communications must be forced to begin with a predetermined, fixed preamble word sequence. This guarantees a sharp correlation peak across all distributed corner nodes simultaneously.
//
 ### 5. FPGA Prototyping and ASIC Tape-Out
 This architecture is designed for eventual ASIC fabrication (tape-out) to serve as a cheap, agnostic peripheral for any microcontroller (STM32, ESP32) via SPI/I2C. However, initial prototyping and proof-of-concept (PoC) validation can be achieved using existing high-end FPGAs (such as Xilinx Kintex-7 or Zynq Ultrascale+). By utilizing the FPGA's native ISERDES blocks and PLL phase-shifting capabilities, we can emulate the 64-phase clock architecture and test the shadow-register transfer in a real-world warehouse environment prior to committing to costly silicon fabrication.
//
// ---
//
 ## Centralized Multilateration Math (Spherical Intersection)
//
 A macroscopic TDoA system fails if the distributed $100\text{ MHz}$ clocks drift. A master unit routes a centralized $10\text{ MHz}$ sine wave and a 1 Pulse-Per-Second (1PPS) digital sync line to all nodes to discipline their PLLs.
//
 Once synchronized timestamps ($t_1, t_2, \dots, t_N$) are captured and pulled from the shadow registers, they are sent to a central processor. The time differences ($\Delta t_{i1} = t_i - t_1$) map to distance differences ($R_{i1} = c \cdot \Delta t_{i1}$).
//
 The hyperbolic TDoA equation is non-linear:
// 

$$R_{i1} = \sqrt{(x - x_i)^2 + (y - y_i)^2 + (z - z_i)^2} - \sqrt{(x - x_1)^2 + (y - y_1)^2 + (z - z_1)^2}$$


//
 To execute this instantly on an embedded microcontroller, we linearize the system by squaring the distances to form a standard $\mathbf{A}\mathbf{y} = \mathbf{b}$ matrix:
// 

$$\begin{bmatrix} (x_2 - x_1) & (y_2 - y_1) & (z_2 - z_1) & R_{21} \\ \vdots & \vdots & \vdots & \vdots \\ (x_N - x_1) & (y_N - y_1) & (z_N - z_1) & R_{N1} \end{bmatrix} \begin{bmatrix} x \\ y \\ z \\ R_1 \end{bmatrix} = \frac{1}{2} \begin{bmatrix} \vert{}\vert{}\mathbf{x}_2\vert{}\vert{}^2 - \vert{}\vert{}\mathbf{x}_1\vert{}\vert{}^2 - R_{21}^2 \\ \vdots \\ \vert{}\vert{}\mathbf{x}_N\vert{}\vert{}^2 - \vert{}\vert{}\mathbf{x}_1\vert{}\vert{}^2 - R_{N1}^2 \end{bmatrix}$$


//
 The target coordinate is solved using an Ordinary Least Squares (OLS) estimator:
// 

$$\mathbf{y} = (\mathbf{A}^T \mathbf{A})^{-1} \mathbf{A}^T \mathbf{b}$$


//
// ---
//
 ## Methodology Comparison
//
 | Feature | SpotFi | DeepFi | Custom Hardware TDoA |
 | :--- | :--- | :--- | :--- |
 | Primary Metric | Phase (AoA) & ToF | Amplitude (Fingerprinting) | Absolute Arrival Time (TDoA) |
 | Algorithm | 2D MUSIC / Covariance | Deep Belief Networks | Multi-Phase Clocking / Multilateration |
 | Drone Suitability| Poor (Tilt/Vibration ruins phase) | Poor (Dynamic layout alters fingerprints)| Excellent (Phase-agnostic, immune to dynamics) |
 | Hardware Required| 3+ Antennas on Target | Single Antenna on Target | Passive Distributed Sniffer ASIC |
 | System Overhead | Low | Very High (Exhaustive training) | Moderate (Requires node sync & software mod) |
