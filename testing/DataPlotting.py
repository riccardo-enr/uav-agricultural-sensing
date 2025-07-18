from pyulog import ULog
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable

def detect_steady_state(time, data, window_size=30, threshold_std=0.1, min_duration=20):
    """
    Detect steady state periods in time series data.
    
    Parameters:
    - time: time array
    - data: data array
    - window_size: size of rolling window for analysis
    - threshold_std: standard deviation threshold for steady state
    - min_duration: minimum duration (in seconds) for a valid steady state
    
    Returns:
    - steady_periods: list of tuples (start_idx, end_idx) for steady state periods
    - steady_stats: dictionary with statistics for each steady state period

    """
    if len(data) < window_size:
        return [], {}
    
    # Calculate rolling statistics
    rolling_std = np.full(len(data), np.nan)
    rolling_mean = np.full(len(data), np.nan)
    
    for i in range(window_size, len(data)):
        window_data = data[i-window_size:i]
        rolling_std[i] = np.std(window_data)
        rolling_mean[i] = np.mean(window_data)
    
    # Find periods where standard deviation is below threshold
    steady_mask = rolling_std < threshold_std
    steady_mask = np.nan_to_num(steady_mask, nan=False)
    
    # Find continuous steady periods
    steady_periods = []
    steady_stats = {}
    
    in_steady_state = False
    start_idx = None
    
    for i in range(len(steady_mask)):
        if steady_mask[i] and not in_steady_state:
            # Start of steady state
            start_idx = i
            in_steady_state = True
        elif not steady_mask[i] and in_steady_state:
            # End of steady state
            end_idx = i - 1
            duration = time[end_idx] - time[start_idx]
            
            if duration >= min_duration:
                steady_periods.append((start_idx, end_idx))
                
                # Calculate statistics for this period
                period_data = data[start_idx:end_idx+1]
                steady_stats[len(steady_periods)-1] = {
                    'start_time': time[start_idx],
                    'end_time': time[end_idx],
                    'duration': duration,
                    'mean': np.mean(period_data),
                    'std': np.std(period_data),
                    'min': np.min(period_data),
                    'max': np.max(period_data),
                    'median': np.median(period_data)
                }
            
            in_steady_state = False
    
    # Handle case where data ends in steady state
    if in_steady_state and start_idx is not None:
        end_idx = len(data) - 1
        duration = time[end_idx] - time[start_idx]
        
        if duration >= min_duration:
            steady_periods.append((start_idx, end_idx))
            period_data = data[start_idx:end_idx+1]
            steady_stats[len(steady_periods)-1] = {
                'start_time': time[start_idx],
                'end_time': time[end_idx],
                'duration': duration,
                'mean': np.mean(period_data),
                'std': np.std(period_data),
                'min': np.min(period_data),
                'max': np.max(period_data),
                'median': np.median(period_data)
            }
    
    return steady_periods, steady_stats

def plot_with_steady_state(time, data, steady_periods, steady_stats, title, ylabel, colormap, fig_size=(12, 6)):
    """
    Plot data with steady state regions highlighted.
    """
    # Create color-coded line plot
    norm = Normalize(vmin=data.min(), vmax=data.max())
    cmap = plt.get_cmap(colormap)
    
    points = np.array([time, data]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    lc = LineCollection(segments, cmap=cmap, norm=norm)
    lc.set_array(data)
    lc.set_linewidth(2)
    
    fig, ax = plt.subplots(figsize=fig_size)
    ax.add_collection(lc)
    
    
    # Highlight steady state periods
    for i, (start_idx, end_idx) in enumerate(steady_periods):
        ax.axvspan(time[start_idx], time[end_idx], alpha=0.3, color='yellow')
    
    ax.set_xlim(time.min(), time.max())
    ax.set_ylim(data.min() - abs(data.max() - data.min()) * 0.05, 
                data.max() + abs(data.max() - data.min()) * 0.05)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    
    
    # Add colorbar
    cbar = fig.colorbar(ScalarMappable(norm=norm, cmap=cmap), ax=ax, location='right', pad=0.01)
    cbar.set_label(ylabel)
    
    # Add grid
    ax.grid(True, which='major', axis='x', linestyle='-', linewidth=0.5, alpha=0.7)
    ax.minorticks_on()
    ax.grid(True, which='minor', axis='x', linestyle=':', linewidth=0.4, alpha=0.5)
    
  
  
    plt.tight_layout()
    return fig, ax

def print_steady_state_summary(steady_stats, data_name):
    """
    Print summary of steady state analysis.
    """
    print(f"\n=== {data_name} Steady State Analysis ===")
    
    if not steady_stats:
        print("No steady state periods detected.")
        return
    
    print(f"Number of steady state periods: {len(steady_stats)}")
    
    for i, stats in steady_stats.items():
        print(f"\nSteady State Period {i+1}:")
        print(f"  Time range: {stats['start_time']:.1f} - {stats['end_time']:.1f} s")
        print(f"  Duration: {stats['duration']:.1f} s")
        print(f"  Mean: {stats['mean']:.3f}")
        print(f"  Std Dev: {stats['std']:.3f}")
        print(f"  Range: {stats['min']:.3f} - {stats['max']:.3f}")
        print(f"  Median: {stats['median']:.3f}")

def main():
    # Load ULog data
    ulog = ULog('/Users/michelegiacalone/Downloads/log_276_2025-7-14-09-56-54.ulg')
    hygrometer_data = ulog.get_dataset("sensor_hygrometer")

    # Extract data
    timestamp = np.array(hygrometer_data.data["timestamp"])
    temperature = np.array(hygrometer_data.data["temperature"])
    humidity = np.array(hygrometer_data.data["humidity"])

    # Time conversion and masking
    time_sec = (timestamp - timestamp[0]) * 1e-6
    mask = (time_sec >= 0) & (time_sec <= 300)
    time_sec_filtered = time_sec[mask]
    temperature_filtered = temperature[mask]
    humidity_filtered = humidity[mask]
    time_shifted = time_sec_filtered - 10

    # Perform steady state analysis
    print("Performing steady state analysis...")

    # Temperature steady state analysis
    temp_steady_periods, temp_steady_stats = detect_steady_state(
        time_shifted, temperature_filtered, 
        window_size=30, threshold_std=0.1, min_duration=20
    )

    # Humidity steady state analysis
    hum_steady_periods, hum_steady_stats = detect_steady_state(
        time_shifted, humidity_filtered, 
        window_size=30, threshold_std=0.5, min_duration=20
    )

    # Print steady state summaries
    print_steady_state_summary(temp_steady_stats, "Temperature")
    print_steady_state_summary(hum_steady_stats, "Humidity")

    # Plot temperature with steady state analysis
    fig_temp, ax_temp = plot_with_steady_state(
        time_shifted, temperature_filtered, temp_steady_periods, temp_steady_stats,
        "Temperature vs Time with Steady State Analysis", "Temperature (°C)", "coolwarm"
    )

    # Plot humidity with steady state analysis
    fig_hum, ax_hum = plot_with_steady_state(
        time_shifted, humidity_filtered, hum_steady_periods, hum_steady_stats,
        "Humidity vs Time with Steady State Analysis", "Humidity (%)", "coolwarm_r"
    )

    # Additional analysis: Overall statistics
    print(f"\n=== Overall Statistics ===")
    print(f"Temperature:")
    print(f"  Overall mean: {np.mean(temperature_filtered):.3f} °C")
    print(f"  Overall std: {np.std(temperature_filtered):.3f} °C")
    print(f"  Overall range: {np.min(temperature_filtered):.3f} - {np.max(temperature_filtered):.3f} °C")

    print(f"\nHumidity:")
    print(f"  Overall mean: {np.mean(humidity_filtered):.3f} %")
    print(f"  Overall std: {np.std(humidity_filtered):.3f} %")
    print(f"  Overall range: {np.min(humidity_filtered):.3f} - {np.max(humidity_filtered):.3f} %")

    # Calculate percentage of time in steady state
    total_time = time_shifted[-1] - time_shifted[0]
    temp_steady_time = sum([stats['duration'] for stats in temp_steady_stats.values()])
    hum_steady_time = sum([stats['duration'] for stats in hum_steady_stats.values()])

    print(f"\n=== Steady State Coverage ===")
    print(f"Total measurement time: {total_time:.1f} s")
    print(f"Temperature steady state time: {temp_steady_time:.1f} s ({temp_steady_time/total_time*100:.1f}%)")
    print(f"Humidity steady state time: {hum_steady_time:.1f} s ({hum_steady_time/total_time*100:.1f}%)")

    plt.show()

if __name__ == "__main__":
    main()