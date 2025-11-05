import pandas as pd
import matplotlib.pyplot as plt
from tkinter import Tk, filedialog

# -----------------------------
# File selection dialog
# -----------------------------
root = Tk()
root.withdraw()  # Hide the main tkinter window
file_path = filedialog.askopenfilename(
    title="Select Telemetry CSV File",
    filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")]
)

if not file_path:
    print("❌ No file selected. Exiting.")
    exit()

print(f"📂 Selected file: {file_path}")

# -----------------------------
# Load the data
# -----------------------------
df = pd.read_csv(file_path)

# Try to detect the time column automatically
time_col = next((c for c in df.columns if "time" in c.lower()), None)
if time_col is None:
    raise ValueError("No time column found — please ensure one column contains 'Time'.")

# -----------------------------
# Define variable groups
# -----------------------------
groups = {
    "X Position": [
        "carriageXPosition"
    ],
    "Y Position": [
        "carriageYPosition"
    ],
    "X Angle": [
        "pendulumXAngle"
    ],
    "Y Angle": [
        "pendulumYAngle"
    ],
    "X Set Angle Control": [
        "xPositionError", "xSetsAngleP", "xSetsAngleI", "xSetsAngleD", "xSetPointAngle"
    ],
    "Y Set Angle Control": [
        "yPositionError", "ySetsAngleP", "ySetsAngleI", "ySetsAngleD", "ySetPointAngle"
    ],
    "X Set PWM Control": [
        "xAngleError", "xSetPWMP", "xSetPWMI", "xSetPWMD", "xPWM"
    ],
    "Y Set PWM Control": [
        "yAngleError", "ySetPWMP", "ySetPWMI", "ySetPWMD", "yPWM"
    ],
}

# -----------------------------
# Plot each group
# -----------------------------
for title, vars_to_plot in groups.items():
    plt.figure(figsize=(10, 6))
    found_any = False

    for var in vars_to_plot:
        if var in df.columns:
            plt.plot(df[time_col], df[var], label=var)
            found_any = True
        else:
            print(f"⚠️ Warning: {var} not found in CSV columns")

    if found_any:
        plt.title(title)
        plt.xlabel(time_col)
        plt.ylabel("Value")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
    else:
        plt.close()  # No valid variables, close empty plot

plt.show()
