 # WiFi CSI Indoor Localization for Autonomous Drones

This repository contains the investigation and implementation of WiFi Channel State Information (CSI) for GPS-denied indoor drone navigation. Traditional methods like cameras and IMUs degrade in low-light or dusty environments, such as warehouses. This project explores extracting per-subcarrier measurements from WiFi signals to map environmental geometry, identifies the terminal limitations of existing software-based CSI algorithms for moving targets, and proposes a custom hardware-accelerated Time Difference of Arrival (TDoA) ASIC architecture.

 ---

## Part 1: Literature Review & Evaluation

Current research heavily focuses on extracting spatial data from commodity WiFi Network Interface Cards (NICs). Two prominent methodologies are SpotFi and DeepFi.

### 1. SpotFi (Kotaru et al.)
SpotFi utilizes a modified 2D Multiple Signal Classification (MUSIC) algorithm to extract both Angle of Arrival (AoA) and Time of Flight (ToF) from the raw CSI matrix.

Technique:
It smooths the CSI matrix across antennas and subcarriers to compute a spatial covariance matrix. It then searches a mathematical grid for peaks where the signal vector is orthogonal to the noise subspace:


$$P_{MUSIC}(\theta, \tau) = \frac{1}{\mathbf{a}^H(\theta, \tau) \mathbf{E}_n \mathbf{E}_n^H \mathbf{a}(\theta, \tau)}$$


SpotFi applies a Gaussian likelihood function across consecutive packets to filter out multipath reflections and isolate the direct Line-of-Sight (LoS) path.

 Advantages:
 * Achieves sub-meter accuracy for static targets without requiring prior environmental mapping.
 * Mathematically filters out multipath interference rather than being confused by it.

 ### 2. DeepFi (Wang et al.)
 DeepFi treats indoor localization as an image classification problem using Deep Belief Networks (DBNs).

 Technique:
 It relies entirely on CSI amplitude data, ignoring phase. During an offline phase, it records the CSI amplitude signature (fingerprint) at various physical coordinates. A deep neural network is trained to associate these multipath signatures with specific locations.

 Advantages:
 * Bypasses complex geometric calculations and hardware phase unsynchronization.
 * Functions effectively in Non-Line-of-Sight (NLoS) environments where direct paths do not exist.

 ---

 ## The Drone Problem: Why Software CSI Breaks Down

 While SpotFi and DeepFi are functional for static smartphones or laptops, they structurally fail when the target collecting or transmitting the CSI is a dynamic drone.

 1. SpotFi & Drone Dynamics: AoA requires strict spatial coherency across the antenna array. A drone experiences severe motor vibration, rapid pitch/roll tilting, and spatial translation. This constantly alters the geometric baseline of the antennas, destroying the phase relationships required for the MUSIC spectrum. Furthermore, 2.4 GHz signal wavelengths ($\approx 12.5\text{ cm}$) are too large to form a reliable spatial array on a compact drone frame without spatial aliasing.
 2. DeepFi & Dynamic Environments: DeepFi maps a specific multipath environment. A moving drone introduces Doppler shifts and alters the multipath environment via propeller turbulence. Fingerprinting requires exhaustive, constant recalibration.
 3. The AGC Hardware Quirk: Commodity WiFi modules use Automatic Gain Control (AGC) which artificially inflates weak signals and attenuates strong ones before they hit the ADC. This invalidates the inverse-square law, making raw CSI amplitude mathematically useless for absolute distance ranging.

 ---

 ## Proposed Direction: Macroscopic Hardware TDoA (ASIC Architecture)

 To reliably track a drone in a closed space, we must abandon compact AoA arrays and machine learning fingerprinting. Instead, we propose a macroscopic Time Difference of Arrival (TDoA) multilateration system.

 Standard 40 MHz WiFi modules can only sample time in $25\text{ ns}$ increments (a $7.5\text{ meter}$ resolution). To achieve sub-nanosecond precision, we propose designing a custom peripheral Time-to-Digital Converter (TDC) ASIC. These passive sniffer chips will be placed at the extremities of the tracking volume (e.g., room corners).

 ### 1. The Multi-Phase Clocking Core
 Instead of clocking the ASIC at impossible GHz frequencies to get high precision, the architecture relies on multi-phase clock generation.
 * A base $100\text{ MHz}$ clock provides a $10\text{ ns}$ coarse timing period.
 * A high-performance internal Delay-Locked Loop (DLL) or Phase-Locked Loop (PLL) splits this base clock into 64 equally phase-delayed $100\text{ MHz}$ clocks.
 * This provides a timing resolution of $10\text{ ns} / 64 = \mathbf{156.25\text{ ps}}$. In physical space, this translates to a spatial resolution of roughly $4.68\text{ cm}$ ($c \times 156.25\text{ ps}$).

 ### 2. Lookup Table (LUT) Preamble Trigger
 The ASIC acts as a passive physical-layer sniffer. It downconverts the 2.4 GHz RF signal and digitizes the baseband.
 * The incoming digital sequence is continuously fed into a high-speed hardware correlator.
 * This correlator compares the incoming data stream against a hard-coded Lookup Table (LUT) containing the expected synchronization words.
 * The moment a strict match occurs, the correlator fires a synchronized, single-cycle strobe signal.

 ### 3. The Shadow Register Transfer
 * The 64 phase-delayed clocks are continuously driving a 64-bit sampling bus.
 * When the LUT strobe fires, the instantaneous state of all 64 clock lines is latched directly into a 64-bit shadow register.
 * By analyzing the sampled bits in this shadow register, the ASIC determines exactly which of the 64 clock phases the preamble arrived on, yielding the 156-picosecond timestamp.

 ### 4. Required Software Modifications
 For this hardware to function, a strict modification must be made to the drone's transmission software. Standard WiFi protocol headers are highly variable. To ensure the ASIC's LUT correlator triggers accurately and consistently, all drone communications must be forced to begin with a predetermined, fixed preamble word sequence. This guarantees a sharp correlation peak across all distributed corner nodes simultaneously.

 ### 5. FPGA Prototyping and ASIC Tape-Out
 This architecture is designed for eventual ASIC fabrication (tape-out) to serve as a cheap, agnostic peripheral for any microcontroller (STM32, ESP32) via SPI/I2C. However, initial prototyping and proof-of-concept (PoC) validation can be achieved using existing high-end FPGAs (such as Xilinx Kintex-7 or Zynq Ultrascale+). By utilizing the FPGA's native ISERDES blocks and PLL phase-shifting capabilities, we can emulate the 64-phase clock architecture and test the shadow-register transfer in a real-world warehouse environment prior to committing to costly silicon fabrication.

 ---

 ## Centralized Multilateration Math (Spherical Intersection)

 A macroscopic TDoA system fails if the distributed $100\text{ MHz}$ clocks drift. A master unit routes a centralized $10\text{ MHz}$ sine wave and a 1 Pulse-Per-Second (1PPS) digital sync line to all nodes to discipline their PLLs.

 Once synchronized timestamps ($t_1, t_2, \dots, t_N$) are captured and pulled from the shadow registers, they are sent to a central processor. The time differences ($\Delta t_{i1} = t_i - t_1$) map to distance differences ($R_{i1} = c \cdot \Delta t_{i1}$).

 The hyperbolic TDoA equation is non-linear:


$$R_{i1} = \sqrt{(x - x_i)^2 + (y - y_i)^2 + (z - z_i)^2} - \sqrt{(x - x_1)^2 + (y - y_1)^2 + (z - z_1)^2}$$



 To execute this instantly on an embedded microcontroller, we linearize the system by squaring the distances to form a standard $\mathbf{A}\mathbf{y} = \mathbf{b}$ matrix:
 

$$\begin{bmatrix} (x_2 - x_1) & (y_2 - y_1) & (z_2 - z_1) & R_{21} \\ \vdots & \vdots & \vdots & \vdots \\ (x_N - x_1) & (y_N - y_1) & (z_N - z_1) & R_{N1} \end{bmatrix} \begin{bmatrix} x \\ y \\ z \\ R_1 \end{bmatrix} = \frac{1}{2} \begin{bmatrix} \vert{}\vert{}\mathbf{x}_2\vert{}\vert{}^2 - \vert{}\vert{}\mathbf{x}_1\vert{}\vert{}^2 - R_{21}^2 \\ \vdots \\ \vert{}\vert{}\mathbf{x}_N\vert{}\vert{}^2 - \vert{}\vert{}\mathbf{x}_1\vert{}\vert{}^2 - R_{N1}^2 \end{bmatrix}$$


 The target coordinate is solved using an Ordinary Least Squares (OLS) estimator:
 

$$\mathbf{y} = (\mathbf{A}^T \mathbf{A})^{-1} \mathbf{A}^T \mathbf{b}$$



 ---

 ## Methodology Comparison

 | Feature | SpotFi | DeepFi | Custom Hardware TDoA |
 | :--- | :--- | :--- | :--- |
 | Primary Metric | Phase (AoA) & ToF | Amplitude (Fingerprinting) | Absolute Arrival Time (TDoA) |
 | Algorithm | 2D MUSIC / Covariance | Deep Belief Networks | Multi-Phase Clocking / Multilateration |
 | Drone Suitability| Poor (Tilt/Vibration ruins phase) | Poor (Dynamic layout alters fingerprints)| Excellent (Phase-agnostic, immune to dynamics) |
 | Hardware Required| 3+ Antennas on Target | Single Antenna on Target | Passive Distributed Sniffer ASIC |
 | System Overhead | Low | Very High (Exhaustive training) | Moderate (Requires node sync & software mod) |



 ---


 ## Part 2: Implementation and Drone Motion Analysis
 
 To answer the fundamental question of how WiFi CSI behaves differently on a moving drone compared to a static router, I implemented a dual-pipeline machine learning architecture. Rather than just running a standard classifier, I built a system designed to compare a traditional static spatial fingerprinting approach (a 1-Nearest Neighbor baseline) against a biologically-inspired Spiking Neural Network (SNN) that explicitly processes *continuous change*.The codebase handles data loading, feature extraction, classification, and a custom physics-based "stress test" module that mathematically corrupts static data to simulate drone flight.
 ### 1. The Dataset and Label Extraction
 For this assignment, I utilized the Widar 3.0 dataset, publicly available on the IEEE Dataport. Widar contains raw Intel 5300 format .dat files collected using static laptops and antennas.Data Labeling from FilenamesWidar does not use a separate metadata CSV for labels. Instead, every piece of ground truth is baked directly into the filename string using a strict convention: `user-gesture-location-orientation-repetition-receiver.dat`.To extract the location label (which is the target for our classifier), I implemented a regex parser during the data loading phase:
 ```python
FILENAME_RE = re.compile(
r"user(?P\d+)-(?P\d+)-(?P\d+)-"
r"(?P\d+)-(?P\d+)-r(?P\d+).dat"
)def parse_label_from_filename(path):
m = FILENAME_RE.search(os.path.basename(path))
if m is None: return None
return int(m.group("location"))

```
The Zero-Index TrapA critical detail in data preprocessing involved label remapping. The raw `location` field in Widar runs from 1 to 5. However, PyTorch's `CrossEntropyLoss` is strictly zero-indexed—passing a label of `5` to a 5-neuron output layer throws a fatal `IndexError: Target 5 is out of bounds`. To ensure pipeline stability, the labels are passed through a `remap_labels` function that maps `{1, 2, 3, 4, 5}` to `{0, 1, 2, 3, 4}` before reaching any classifier.
### 2. Feature Extraction: Static vs. Motion ViewsThe raw CSI tensor extracted via `csiread` has the shape `(Time, Antennas, Subcarriers)`. The foundation of this experiment relies on extracting two radically different perspectives from this raw amplitude tensor.
#### A. The Static Fingerprint (`amplitude_snapshot`)The "amplitude snapshot" is the cornerstone of traditional WiFi CSI fingerprinting systems like DeepFi, RADAR, and Horus. In this pipeline, it serves as the feature extractor for the baseline $k$-NN classifier. It mathematically squashes the `Time` dimension to create a single, static spatial fingerprint of the environment.


```python
def amplitude_snapshot(amp_seq):
"""(T, n_antennas, n_subcarriers) -> flattened, unit-norm static
fingerprint of shape (n_antennas * n_subcarriers,)."""
mean_amp = amp_seq.mean(axis=0)                # (antennas, subcarriers)
flat = mean_amp.flatten()
norm = np.linalg.norm(flat) + 1e-8
return flat / norm 
```
The Mechanics of the Snapshot:
\1. Time-Averaging (`mean(axis=0)`): The function takes a burst of packets collected over a short window and averages their amplitudes, explicitly destroying temporal information.
\2. Flattening: It converts the `(Antennas, Subcarriers)` matrix into a 1D vector (e.g., $3 \times 30 = 90$ features).
\3. L2 Normalization: It divides the vector by its L2 norm. This makes the feature invariant to absolute transmission power—if the router suddenly transmits at half power, the normalized geometric "shape" of the fingerprint remains identical.Because this feature explicitly destroys all temporal information to create a static spatial map, it profoundly affected how the $k$-NN classifier responded to different conditions later in the stress tests.#### B. The Event-Based Motion View (`delta_sequence`)Drones are defined by movement. To test if we can classify locations based on multipath *dynamics* rather than static signatures, the SNN pipeline takes the first-order difference between consecutive packets.
```python
def delta_sequence(amp_seq):
"""packet-to-packet amplitude deltas, flattened and scaled to unit std."""
diffs = np.diff(amp_seq, axis=0)                # (T-1, antennas, subcarriers)
flat = diffs.reshape(diffs.shape[0], -1)        # (T-1, antennas*subcarriers)
return flat / (np.std(flat) + 1e-8)
```
By utilizing `np.diff`, the static environmental background is mathematically erased. The constant terms vanish. The SNN network is completely blind to absolute signal strength and must classify the location based purely on the frequency and structure of the amplitude fluctuations.
### 3. Baseline Classifier Performance (k-NN)
To satisfy the "simple classifier" requirement, I built a 1-Nearest Neighbor model running on the `amplitude_snapshot` features. Under standard, static conditions (collected by a stationary receiver), this feature is incredibly powerful. Because radio multipath fading creates a highly unique pattern of peaks and nulls across the 30 subcarriers at any specific location in a room, the average shape is a nearly perfect spatial barcode.Testing on a subset of 4,500 real data files, the k-NN model achieved 93.5% accuracy on real Widar data (and 100% on synthetic data).Final Confusion Matrix (Real Widar 3.0 Data):
```text
[[253   0   6   5   6]
[  1 258   2   1   8]
[  4   4 251   6   5]
[  1   2   1 254  12]
[  1   3   4  16 246]]
```
As shown, errors are minimal and evenly distributed. The model easily memorizes the static environmental geometries.
### 4. The Spiking Neural Network (DeltaSNN)
Processing the `delta_sequence` requires a temporal model. I implemented a multi-layer Leaky Integrate-and-Fire (LIF) network using `snntorch`.Adaptive Threshold EncodingReal CSI subcarriers have vastly different dynamic ranges—some sit near deep fades and swing wildly, others barely move. Applying a single global threshold to generate spikes starves quiet channels and saturates noisy ones. To fix this, I implemented a per-channel adaptive threshold, calibrated *strictly* on the training split's standard deviation:
```python
def compute_adaptive_thresholds(train_delta_seqs, k=1.0, min_threshold=1e-3):
all_deltas = np.concatenate(train_delta_seqs, axis=0)
per_channel_std = all_deltas.std(axis=0)
return np.maximum(k * per_channel_std, min_threshold).astype(np.float32)
```
Membrane Potential ReadoutTraining SNN classifiers with Backpropagation Through Time (BPTT) using surrogate gradients is prone to the "dead gradient" problem. If the network goes completely silent (zero spikes), learning halts permanently. To prevent this, the final classification reads out the continuous *membrane potential* trace of the output neurons instead of discrete spike counts, maintaining a live gradient signal:
```python
# Inside DeltaSNN forward pass:
return torch.stack(mem2_trace, dim=0).mean(dim=0)  # (batch, n_classes)
```
### 5. What Changes on a Drone?
Standard datasets like Widar are collected using fixed laptops. To answer the assignment's core question—*"what actually changes when the thing collecting CSI is a drone?"*—I engineered a `stress_test.py` module. This applies simulated physics corruptions to the static test data to observe how the assumptions of standard localization systems break down.Because the amplitude snapshot feature assumes the world—and the receiver—is entirely still, it profoundly affected how the baseline $k$-NN responded to different conditions.#### A. Orientation Shifts (The Tilt)Drones pitch and roll to translate. If a drone rotates, the geometric relationship between its antenna array and the incident incoming multipath signals changes completely.
```python
def orientation_shift(amp_seq, shift=1):
"""Roll along the antenna axis (axis=1)."""
return np.roll(amp_seq, shift=shift, axis=1)
```
* The Effect: The baseline $k$-NN failed spectacularly here, plummeting from 93.5% down to 20.1% (effectively random guessing).
* Why: The snapshot relies on a fixed geometric relationship between the antennas. If Antenna 1 usually sees a strong signal and Antenna 3 sees a weak one, rolling the drone swaps these positions. The `(Antennas, Subcarriers)` grid is physically shifted, and the $k$-NN algorithm fails to match this rotated array to its database of flat, static fingerprints. Systems like SpotFi, which rely on precise Angle-of-Arrival (AoA) calculations, will fail entirely if the drone's orientation is not strictly compensated for using IMU data.
#### B. Motor VibrationsDrone propellers operate at thousands of RPMs, causing high-frequency micro-vibrations across the airframe.
```python
def vibration_jitter(amp_seq, freq_hz=200, packet_rate_hz=1000, amplitude=0.15):
t = np.arange(T) / packet_rate_hz
ripple = amplitude * np.sin(2 * np.pi * freq_hz * t)
return amp_seq * (1 + ripple[:, None, None])
```
* The Effect: This breaks Hypothesis 1 of DeepFi—that CSI is stable over a short packet burst. However, the static $k$-NN was completely immune to this, maintaining its 93.5% accuracy. The event-based SNN, however, degraded severely.* Why: The vibration function applies a zero-mean high-frequency sine wave. Because the amplitude snapshot applies `amp_seq.mean(axis=0)`, it acts as a perfect mathematical low-pass filter. The rapid packet-to-packet vibrations cancel out to zero during the averaging step, making the classifier inherently blind to this specific type of drone motion. Conversely, because the SNN is highly sensitive to *change*, the artificial vibration floods the network with false spikes, masking the actual environmental multipath dynamics.#### C. Doppler Phase Ramps (Translation)As a drone moves through space, the path lengths of incoming signals continuously change during the brief packet observation window.
```python
def doppler_phase_ramp(amp_seq, ramp_strength=0.02):
ramp = np.linspace(0, ramp_strength * T, T)
return amp_seq * (1 + ramp[:, None, None])
```
* The Effect: Standard sanitization algorithms often fit and remove a constant linear-phase term, assuming the receiver is stationary. A translating drone introduces real Doppler shifts that confound static sanitization steps. The SNN struggled heavily here, as the constant translation creates a persistent DC offset in the delta sequence, triggering continuous false firings.### ConclusionTranslating WiFi CSI localization to drones is not a matter of simply mounting a receiver to a quadcopter.The amplitude snapshot feature assumes the receiver is entirely still. While this averaging makes it brilliantly robust against high-frequency zero-mean noise (like motor vibration), it is fatally brittle when deployed on a drone that tilts, yaws, and changes its physical orientation in 3D space. Conversely, motion-sensitive systems (like delta-encoded SNNs) are highly susceptible to the motor vibrations and constant translation inherent to flight.A successful drone-based CSI navigation system will likely require a multimodal approach: using the IMU to constantly un-rotate the CSI phase matrices (solving the tilt issue), coupled with specialized low-pass filters tuned to reject the drone's known motor RPM frequencies (solving the vibration issue) before the CSI data ever reaches the neural network.
