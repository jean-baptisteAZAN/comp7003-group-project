#!/usr/bin/env python3
"""Generate performance comparison graphs from metrics CSVs for Task 3 report."""
import pandas as pd
import matplotlib.pyplot as plt
import os

METRICS_DIR = os.path.join("src", "path_planning", "metrics")
METRICS_STANDARD = os.path.join(METRICS_DIR, "metrics_standard.csv")
METRICS_SMOOTHED = os.path.join(METRICS_DIR, "metrics_smoothed.csv")
OUTPUT_DIR = os.path.join(METRICS_DIR, "graphs")


def load_metrics():
    frames = []
    for path in [METRICS_STANDARD, METRICS_SMOOTHED]:
        if os.path.exists(path):
            df = pd.read_csv(path)
            frames.append(df)
        else:
            print(f"Warning: {path} not found, skipping.")

    if not frames:
        return None

    df = pd.concat(frames, ignore_index=True)
    df["segment"] = df["segment"].astype(int)
    df["path_length"] = df["path_length"].astype(int)
    df["computation_time_s"] = df["computation_time_s"].astype(float)
    df["execution_time_s"] = pd.to_numeric(df["execution_time_s"], errors="coerce")
    return df


def bar_chart(df, metric, ylabel, title, filename):
    """Generic grouped bar chart: one group per segment, one bar per algorithm."""
    pivot = df.pivot(index="segment", columns="algorithm", values=metric)
    ax = pivot.plot(kind="bar", figsize=(8, 5), width=0.7)
    ax.set_xlabel("Segment")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend(title="Algorithm")
    plt.xticks(rotation=0)
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, filename), dpi=150)
    plt.close()
    print(f"Saved {filename}")


def summary_table(df):
    """Print a summary table to the console."""
    print("\n=== Performance Summary ===\n")
    print(df.to_string(index=False))
    print()

    # Per-algorithm totals
    totals = df.groupby("algorithm").agg(
        total_path_length=("path_length", "sum"),
        avg_computation_time=("computation_time_s", "mean"),
        total_execution_time=("execution_time_s", "sum"),
    )
    print("=== Per-algorithm totals ===\n")
    print(totals.to_string())
    print()


def main():
    has_standard = os.path.exists(METRICS_STANDARD)
    has_smoothed = os.path.exists(METRICS_SMOOTHED)

    if not has_standard and not has_smoothed:
        print(f"Error: No metrics files found in {METRICS_DIR}. Run the navigation first.")
        return

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    df = load_metrics()
    if df is None or df.empty:
        print("Error: No data loaded.")
        return

    summary_table(df)

    # Q1 - Path length comparison
    bar_chart(
        df,
        metric="path_length",
        ylabel="Path Length (grid cells)",
        title="Q1: Path Length per Segment",
        filename="q1_path_length.png",
    )

    # Q2 - Computation time comparison
    bar_chart(
        df,
        metric="computation_time_s",
        ylabel="Computation Time (s)",
        title="Q2: Computation Time per Segment",
        filename="q2_computation_time.png",
    )

    # Q3 - Execution time comparison
    if df["execution_time_s"].notna().any():
        bar_chart(
            df,
            metric="execution_time_s",
            ylabel="Execution Time (s)",
            title="Q3: Execution (Travel) Time per Segment",
            filename="q3_execution_time.png",
        )
    else:
        print("Skipping execution time graph (no data yet)")

    # Q4 - All metrics side by side (overview)
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    metrics = [
        ("path_length", "Path Length (cells)", "Path Length"),
        ("computation_time_s", "Comp. Time (s)", "Computation Time"),
        ("execution_time_s", "Exec. Time (s)", "Execution Time"),
    ]

    for ax, (metric, ylabel, title) in zip(axes, metrics):
        pivot = df.pivot(index="segment", columns="algorithm", values=metric)
        pivot.plot(kind="bar", ax=ax, width=0.7)
        ax.set_xlabel("Segment")
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.legend(title="Algorithm")
        ax.tick_params(axis="x", rotation=0)

    plt.suptitle("Q4: Performance Overview Across All Segments", fontsize=14)
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "q4_overview.png"), dpi=150)
    plt.close()
    print("Saved q4_overview.png")


if __name__ == "__main__":
    main()
