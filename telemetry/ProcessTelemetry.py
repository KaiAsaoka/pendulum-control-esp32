import pandas as pd
import plotly.express as px
import tkinter as tk
from tkinter import filedialog

def is_numeric_row(row):
    """Return True if all values in the row are numeric."""
    for val in row:
        try:
            float(val)
        except:
            return False
    return True

def process():
    # === Prompt user for CSV ===
    root = tk.Tk()
    root.withdraw()
    filename = filedialog.askopenfilename(
        title="Select CSV file",
        filetypes=[("CSV Files", "*.csv")]
    )

    if not filename:
        print("No file selected. Exiting.")
        exit()

    # === Load CSV into strings first ===
    df_raw = pd.read_csv(filename, dtype=str)

    # === Detect bottom PID block ===
    mask_numeric = df_raw.apply(is_numeric_row, axis=1)
    cut_index = mask_numeric[mask_numeric == False].index.min()

    if pd.isna(cut_index):
        df_clean = df_raw.copy()
    else:
        df_clean = df_raw.iloc[:cut_index]

    # === Convert all numeric columns ===
    df = df_clean.apply(pd.to_numeric, errors='coerce')

    # === Get column names ===
    cols = df.columns.tolist()

    if len(cols) < 2:
        raise ValueError("CSV must contain at least 2 numeric columns.")

    # Use first column as X, plot all others as Y
    x_col = cols[0]
    y_cols = cols[1:]

    # Melt dataframe for multi-line plotting
    df_melted = df.melt(id_vars=x_col, value_vars=y_cols,
                        var_name="Variable", value_name="Value")

    # === Create plot with all columns ===
    fig = px.line(
        df_melted,
        x=x_col,
        y="Value",
        color="Variable",
        title=filename[:-4]
    )

    fig.update_layout(
        hovermode="x unified",
        template="plotly_dark"
    )

    fig.show()