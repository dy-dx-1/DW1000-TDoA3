"""
This file is to verify geometric conditions for multilateration. 
Input the ranges or a position (and the anchor/tag positions) to plot 
2D projections of the XY, XZ, YZ planes and visualize intersection points. 
"""
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.path import Path
from matplotlib.patches import PathPatch
import numpy as np

# Anchor positions {anchor_id: (x,y,z), ...}
ANCHORS = {5: (0,0,18),
           3: (-57, 72, 127),
           4: (107, 180, 69),
           999: (200,0,200)} # SIMULATED ONE FOR ROOM TEST
# Ranges to anchor {anchor_id: range}
# Anchors without a range entry are still plotted but without range circle 
RANGES = {3: 145, 5: 116, 4:141, 999:283}
# Position in 3D space, set USE_POSITION to true to OVERWRITE defined RANGES 
# in that case, the range to each anchor from the position will be computed
POS = (250, 200, 100) 
USE_POSITION = False 
# Display the plot (if you are not running this remotely)
SHOW_PLOT = False
# Save the plot (None to not save, else specify path.png)
SAVE_PATH = "room_test.png" 

def plot_anchor_ranges(
    anchors,
    ranges=None,
    position=None,
    use_position=False,
    show=True,
    save_path=None,
    figsize=(18, 6),
    dpi=300):
    """
    Plot anchor positions and their range projections in XY, XZ, and YZ planes.

    Parameters
    ----------
    anchors : dict
        Mapping:
            {anchor_id: (x, y, z)}

        Example:
            {
                "A": (0, 0, 0),
                "B": (4, 2, 1),
                "C": (2, 5, 3),
            }

    ranges : dict or None, default=None
        Mapping:
            {anchor_id: range}

        Used when use_position=False.

        Anchors missing from this dictionary are still plotted, but have
        no range circle.

    position : tuple or None, default=None
        A 3D position `(x, y, z)`.

        Used when use_position=True. The range for each anchor is then
        calculated as the Euclidean distance from this position to the
        anchor.

    use_position : bool, default=False
        Selects how ranges are determined.

        False:
            Use the `ranges` dictionary.

        True:
            Ignore `ranges` and calculate the range from `position` to
            every anchor.

    show : bool, default=True
        If True, display the figure using plt.show().

    save_path : str or None, default=None
        If provided, save the figure as a PNG.

        Example:
            save_path="anchors.png"

    figsize : tuple, default=(18, 6)
        Figure size in inches.

    dpi : int, default=300
        Resolution used when saving the PNG.

    Returns
    -------
    fig : matplotlib.figure.Figure
        The generated figure.

    axes : numpy.ndarray
        The three matplotlib axes.
    """

    # ------------------------------------------------------------------
    # Validate inputs
    # ------------------------------------------------------------------

    if use_position:

        if position is None:
            raise ValueError(
                "position must be provided when use_position=True."
            )

        position = np.asarray(position, dtype=float)

        if position.shape != (3,):
            raise ValueError(
                "position must contain exactly three values: (x, y, z)."
            )

    else:

        if ranges is None:
            raise ValueError(
                "ranges must be provided when use_position=False."
            )

    # ------------------------------------------------------------------
    # Determine the ranges to use
    # ------------------------------------------------------------------

    if use_position:

        # Compute distance from the supplied position to every anchor.
        #
        # The resulting dictionary has exactly the same structure as
        # the normal `ranges` dictionary.
        computed_ranges = {}

        for anchor_id, anchor_position in anchors.items():

            anchor_position = np.asarray(
                anchor_position,
                dtype=float,
            )

            if anchor_position.shape != (3,):
                raise ValueError(
                    f"Position for anchor {anchor_id!r} must contain "
                    "exactly three values: (x, y, z)."
                )

            computed_ranges[anchor_id] = np.linalg.norm(
                anchor_position - position
            )

        ranges_to_plot = computed_ranges

    else:

        ranges_to_plot = ranges

    # ------------------------------------------------------------------
    # Colors
    # ------------------------------------------------------------------

    colors = plt.cm.tab10(
        np.linspace(0, 1, max(len(anchors), 1))
    )

    anchor_colors = {
        anchor_id: colors[i % len(colors)]
        for i, anchor_id in enumerate(anchors)
    }

    overlap_color = "#FFF2A8"

    # ------------------------------------------------------------------
    # Create figure
    # ------------------------------------------------------------------

    fig, axes = plt.subplots(
        1,
        3,
        figsize=figsize,
    )

    planes = [
        (0, 1, "XY Plane", "X", "Y"),
        (0, 2, "XZ Plane", "X", "Z"),
        (1, 2, "YZ Plane", "Y", "Z"),
    ]

    # ------------------------------------------------------------------
    # Plot each projection
    # ------------------------------------------------------------------

    for ax, (i, j, title, xlabel, ylabel) in zip(axes, planes):

        # --------------------------------------------------------------
        # Build circle information
        # --------------------------------------------------------------

        circle_data = []

        for anchor_id, anchor_position in anchors.items():

            # In normal range mode, anchors missing from ranges have
            # no circle.
            if anchor_id not in ranges_to_plot:
                continue

            radius = ranges_to_plot[anchor_id]

            if radius is None:
                continue

            if radius < 0:
                raise ValueError(
                    f"Range for anchor {anchor_id!r} cannot be negative."
                )

            anchor_position = np.asarray(
                anchor_position,
                dtype=float,
            )

            circle_data.append({
                "anchor_id": anchor_id,
                "x": anchor_position[i],
                "y": anchor_position[j],
                "radius": radius,
            })

        # --------------------------------------------------------------
        # Determine plotting bounds
        # --------------------------------------------------------------

        all_x = [
            position[i]
            for position in anchors.values()
        ]

        all_y = [
            position[j]
            for position in anchors.values()
        ]

        # Include circle extents.
        for circle in circle_data:

            all_x.extend([
                circle["x"] - circle["radius"],
                circle["x"] + circle["radius"],
            ])

            all_y.extend([
                circle["y"] - circle["radius"],
                circle["y"] + circle["radius"],
            ])

        if not all_x:
            all_x = [0]
            all_y = [0]

        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)

        dx = max_x - min_x
        dy = max_y - min_y

        margin_x = max(dx * 0.08, 0.5)
        margin_y = max(dy * 0.08, 0.5)

        plot_min_x = min_x - margin_x
        plot_max_x = max_x + margin_x
        plot_min_y = min_y - margin_y
        plot_max_y = max_y + margin_y

        # --------------------------------------------------------------
        # Draw overlap regions
        # --------------------------------------------------------------

        if len(circle_data) >= 2:

            resolution = 600

            grid_x = np.linspace(
                plot_min_x,
                plot_max_x,
                resolution,
            )

            grid_y = np.linspace(
                plot_min_y,
                plot_max_y,
                resolution,
            )

            X, Y = np.meshgrid(
                grid_x,
                grid_y,
            )

            coverage = np.zeros_like(
                X,
                dtype=np.uint16,
            )

            for circle in circle_data:

                inside = (
                    (X - circle["x"]) ** 2
                    + (Y - circle["y"]) ** 2
                    <= circle["radius"] ** 2
                )

                coverage += inside

            # Find the greatest number of overlapping circles anywhere
            # in the plot.
            max_coverage = coverage.max()

            # Shade ONLY points that are inside that maximum number
            # of circles.
            overlap = (
                (coverage == max_coverage)
                & (coverage > 1)
            )

            if np.any(overlap):

                overlap_mask = np.ma.masked_where(
                    ~overlap,
                    np.ones_like(
                        coverage,
                        dtype=float,
                    ),
                )

                ax.pcolormesh(
                    X,
                    Y,
                    overlap_mask,
                    shading="auto",
                    cmap=plt.matplotlib.colors.ListedColormap(
                        [overlap_color]
                    ),
                    alpha=0.85,
                    zorder=1,
                )

        # --------------------------------------------------------------
        # Draw circle outlines
        # --------------------------------------------------------------

        for circle in circle_data:

            anchor_id = circle["anchor_id"]

            ax.add_patch(
                Circle(
                    (
                        circle["x"],
                        circle["y"],
                    ),
                    circle["radius"],
                    fill=False,
                    edgecolor=anchor_colors[anchor_id],
                    linewidth=2,
                    zorder=3,
                )
            )

        # --------------------------------------------------------------
        # Draw ALL anchors
        # --------------------------------------------------------------

        for anchor_id, anchor_position in anchors.items():

            anchor_position = np.asarray(
                anchor_position,
                dtype=float,
            )

            ax.scatter(
                anchor_position[i],
                anchor_position[j],
                marker="*",
                s=180,
                color=anchor_colors[anchor_id],
                edgecolor="black",
                linewidth=0.7,
                zorder=4,
                label=anchor_id,
            )

            ax.annotate(
                anchor_id,
                (
                    anchor_position[i],
                    anchor_position[j],
                ),
                xytext=(6, 6),
                textcoords="offset points",
                fontsize=10,
                zorder=5,
            )

        # --------------------------------------------------------------
        # If using position mode, plot the position itself
        # --------------------------------------------------------------

        if use_position:

            ax.scatter(
                position[i],
                position[j],
                marker="x",
                s=140,
                color="black",
                linewidth=2.5,
                zorder=6,
                label="Position",
            )

        # --------------------------------------------------------------
        # Formatting
        # --------------------------------------------------------------

        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)

        ax.set_aspect(
            "equal",
            adjustable="box",
        )

        ax.set_xlim(
            plot_min_x,
            plot_max_x,
        )

        ax.set_ylim(
            plot_min_y,
            plot_max_y,
        )

        ax.grid(
            True,
            alpha=0.25,
        )

    # ------------------------------------------------------------------
    # Legend
    # ------------------------------------------------------------------

    handles, labels = axes[0].get_legend_handles_labels()

    if handles:

        fig.legend(
            handles,
            labels,
            loc="upper center",
            bbox_to_anchor=(0.5, 1.02),
            ncol=min(len(handles), 6),
        )

    fig.tight_layout()

    # ------------------------------------------------------------------
    # Save
    # ------------------------------------------------------------------

    if save_path is not None:

        fig.savefig(
            save_path,
            dpi=dpi,
            bbox_inches="tight",
            format="png",
        )

    # ------------------------------------------------------------------
    # Display
    # ------------------------------------------------------------------

    if show:
        plt.show()

    return fig, axes

if __name__ == "__main__": 
    plot_anchor_ranges(ANCHORS, RANGES, position=POS, use_position=USE_POSITION, show=SHOW_PLOT, save_path=SAVE_PATH) 