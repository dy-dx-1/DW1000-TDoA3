"""
This file is to verify geometric conditions for multilateration.

Input the ranges or a position (and the anchor/tag positions) to plot
2D projections of the XY, XZ, YZ planes and visualize the least-squares
TWR residual.

The highlighted region represents locations with the smallest achievable
3D least-squares range residual, rather than locations where the greatest
number of projected range circles overlap.

IMPORTANT:
The range circles shown in each 2D plot are orthogonal projections of
3D range spheres. They are therefore circles, not ellipses.

The true 3D position does not generally lie on the circumference of the
projected circle. If the anchor and position have different values in
the omitted coordinate, the projected position lies INSIDE the circle.

For example, in the XY plane:

    (x - x_anchor)^2 + (y - y_anchor)^2
        = range^2 - (z - z_anchor)^2

Therefore the projected position is only on the circle when:

    z == z_anchor
"""


import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Patch
from matplotlib.lines import Line2D
import numpy as np

<<<<<<< HEAD

# ----------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------

# Anchor positions {anchor_id: (x, y, z), ...}
ANCHORS = {
    5: (0, 0, 18),
    3: (-57, 72, 127),
    4: (107, 180, 69),
}

# Ranges to anchor {anchor_id: range}
# Anchors without a range entry are still plotted but without range circle
RANGES = {
    3: 145,
    5: 116,
    4: 141,
}

# Position in 3D space.
#
# Set USE_POSITION=True to overwrite RANGES and calculate ranges
# from this position.
POS = (250, 200, 100)

USE_POSITION = True

# Display the plot
SHOW_PLOT = True

# Save the plot (None to not save)
SAVE_PATH = None


# ----------------------------------------------------------------------
# Residual visualization settings
# ----------------------------------------------------------------------

# Number of points used for the 2D visualization grid.
RESOLUTION = 600

# Number of points searched along the omitted coordinate when finding
# the minimum 3D residual for each 2D grid location.
#
# Higher values give a more accurate projection of the 3D objective,
# at the expense of computation time.
OMITTED_COORDINATE_RESOLUTION = 200

# Highlight the lowest percentage of RMS residual values.
#
# For example:
#   5.0  -> highlight the lowest 5% of RMS residuals
#   10.0 -> highlight the lowest 10%
LOW_RESIDUAL_PERCENTILE = 5.0

# Color of the low-residual region.
RESIDUAL_COLOR = "#FFF2A8"

# Transparency of the highlighted region.
RESIDUAL_ALPHA = 0.85

=======
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
>>>>>>> 0b3ca437d1e7d18394df7173341256ae4915f98e

def plot_anchor_ranges(
    anchors,
    ranges=None,
    position=None,
    use_position=False,
    show=True,
    save_path=None,
    figsize=(18, 6),
    dpi=300,
):
    """
    Plot anchor positions, projected 3D range spheres, and
    least-squares residual projections in the XY, XZ, and YZ planes.

    The multilateration objective is defined in 3D as:

        J(x, y, z) =
            sum_i [
                sqrt(
                    (x - x_i)^2 +
                    (y - y_i)^2 +
                    (z - z_i)^2
                )
                - r_i
            ]^2

    where:

        (x_i, y_i, z_i) = position of anchor i
        r_i              = measured TWR range to anchor i

    Thus, for a candidate position, each individual range residual is:

        e_i = predicted_range_i - measured_range_i

    and the quantity minimized by least-squares multilateration is:

        J = sum_i e_i^2


    ------------------------------------------------------------------
    2D visualization of the 3D objective
    ------------------------------------------------------------------

    Because multilateration is a 3D problem but the figure contains
    three 2D planes, simply fixing the omitted coordinate would produce
    a slice through the 3D objective.

    For example, an XY slice at a fixed z would show:

        J(x, y, z_fixed)

    This can be misleading because the true low-residual region may
    occur at a different z.

    Instead, this function minimizes the 3D objective over the omitted
    coordinate for every point in the displayed plane.

    Therefore:

        XY plot:
            J_XY(x, y) = min_z J(x, y, z)

        XZ plot:
            J_XZ(x, z) = min_y J(x, y, z)

        YZ plot:
            J_YZ(y, z) = min_x J(x, y, z)

    Each 2D pixel therefore answers:

        "What is the smallest 3D least-squares residual that can be
         achieved at this location in the displayed plane, after
         allowing the omitted coordinate to vary?"

    This produces a minimized projection of the 3D least-squares
    objective rather than a fixed-coordinate slice.


    ------------------------------------------------------------------
    Range circles
    ------------------------------------------------------------------

    The circles shown in the 2D plots are NOT 2D range measurements.

    They are the orthogonal projections of the corresponding 3D
    range spheres.

    A 3D range sphere is:

        (x-x_i)^2 + (y-y_i)^2 + (z-z_i)^2 = r_i^2

    Its projection onto the XY plane is:

        (x-x_i)^2 + (y-y_i)^2 <= r_i^2

    and its boundary is therefore a circle.

    Consequently, the true 3D position may appear inside the projected
    circle rather than on its circumference.

    For example, in the XY plane:

        (x-x_i)^2 + (y-y_i)^2
            = r_i^2 - (z-z_i)^2

    so the projected position lies on the circumference only when
    z == z_i.

    The same principle applies to the XZ and YZ projections.

    Therefore the circles are retained as geometric references rather
    than being interpreted as exact 2D multilateration constraints.


    ------------------------------------------------------------------
    Highlighted region and contours
    ------------------------------------------------------------------

    The yellow highlighted region represents the lowest percentage
    of RMS residual values in each projection.

    The contour lines show progressively higher RMS residual regions,
    allowing the shape and ambiguity of the solution space to be
    visualized.

    The visualization does not identify a single solution point.
    Its purpose is to show the extent and shape of the low-residual
    solution space.


    Parameters
    ----------
    anchors : dict
        Mapping:

            {anchor_id: (x, y, z)}

    ranges : dict or None
        Mapping:

            {anchor_id: range}

        Used when use_position=False.

        Anchors missing from this dictionary are still plotted, but
        do not contribute to the multilateration residual.

    position : tuple or None
        A 3D position `(x, y, z)`.

        Used when use_position=True.

    use_position : bool, default=False
        If True, ignore the supplied ranges and calculate the range from
        `position` to every anchor.

    show : bool, default=True
        If True, display the figure using plt.show().

    save_path : str or None, default=None
        If provided, save the figure as a PNG.

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

        position = np.asarray(
            position,
            dtype=float,
        )

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
    # Validate and convert anchor positions
    # ------------------------------------------------------------------

    anchor_positions = {}

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

        anchor_positions[anchor_id] = anchor_position

    # ------------------------------------------------------------------
    # Determine ranges
    # ------------------------------------------------------------------

    if use_position:

        computed_ranges = {}

        for anchor_id, anchor_position in anchor_positions.items():

            computed_ranges[anchor_id] = np.linalg.norm(
                anchor_position - position
            )

        ranges_to_plot = computed_ranges

    else:

        ranges_to_plot = ranges

    # ------------------------------------------------------------------
    # Determine anchors with valid range measurements
    # ------------------------------------------------------------------

    measured_anchor_ids = []

    for anchor_id in anchors:

        if anchor_id not in ranges_to_plot:
            continue

        if ranges_to_plot[anchor_id] is None:
            continue

        radius = float(
            ranges_to_plot[anchor_id]
        )

        if radius < 0:
            raise ValueError(
                f"Range for anchor {anchor_id!r} cannot be negative."
            )

        measured_anchor_ids.append(anchor_id)

    if len(measured_anchor_ids) < 2:
        raise ValueError(
            "At least two anchors with valid ranges are required."
        )

    # ------------------------------------------------------------------
    # Colors
    # ------------------------------------------------------------------

    colors = plt.cm.tab10(
        np.linspace(
            0,
            1,
            max(len(anchors), 1),
        )
    )

    anchor_colors = {
        anchor_id: colors[i % len(colors)]
        for i, anchor_id in enumerate(anchors)
    }

    # ------------------------------------------------------------------
    # Create figure
    # ------------------------------------------------------------------

    fig, axes = plt.subplots(
        1,
        3,
        figsize=figsize,
    )

    # Each tuple contains:
    #
    #   horizontal coordinate
    #   vertical coordinate
    #   omitted coordinate
    #   title
    #   x-axis label
    #   y-axis label

    planes = [
        (0, 1, 2, "XY Plane", "X", "Y"),
        (0, 2, 1, "XZ Plane", "X", "Z"),
        (1, 2, 0, "YZ Plane", "Y", "Z"),
    ]

    # ------------------------------------------------------------------
    # Progress tracking
    # ------------------------------------------------------------------

    total_calculations = (
        len(planes)
        * OMITTED_COORDINATE_RESOLUTION
    )

    completed_calculations = 0

    print(
        "Calculating least-squares residual space..."
    )

    print(
        "Progress: 0%",
        end="",
        flush=True,
    )

    next_progress = 10

    # ------------------------------------------------------------------
    # Plot each projection
    # ------------------------------------------------------------------

    for ax, (i, j, k, title, xlabel, ylabel) in zip(
        axes,
        planes,
    ):

        # --------------------------------------------------------------
        # Build circle information
        # --------------------------------------------------------------

        circle_data = []

        for anchor_id in measured_anchor_ids:

            anchor_position = anchor_positions[anchor_id]

            radius = float(
                ranges_to_plot[anchor_id]
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
            anchor_positions[anchor_id][i]
            for anchor_id in anchors
        ]

        all_y = [
            anchor_positions[anchor_id][j]
            for anchor_id in anchors
        ]

        # Include projected range-circle extents.
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

        margin_x = max(
            dx * 0.08,
            0.5,
        )

        margin_y = max(
            dy * 0.08,
            0.5,
        )

        plot_min_x = min_x - margin_x
        plot_max_x = max_x + margin_x

        plot_min_y = min_y - margin_y
        plot_max_y = max_y + margin_y

        # --------------------------------------------------------------
        # Build 2D grid
        # --------------------------------------------------------------

        grid_x = np.linspace(
            plot_min_x,
            plot_max_x,
            RESOLUTION,
        )

        grid_y = np.linspace(
            plot_min_y,
            plot_max_y,
            RESOLUTION,
        )

        X, Y = np.meshgrid(
            grid_x,
            grid_y,
        )

        # --------------------------------------------------------------
        # Determine range of omitted coordinate
        # --------------------------------------------------------------

        omitted_anchor_values = np.array([
            anchor_positions[anchor_id][k]
            for anchor_id in measured_anchor_ids
        ])

        max_range = max(
            float(ranges_to_plot[anchor_id])
            for anchor_id in measured_anchor_ids
        )

        omitted_min = (
            omitted_anchor_values.min()
            - max_range
        )

        omitted_max = (
            omitted_anchor_values.max()
            + max_range
        )

        omitted_values = np.linspace(
            omitted_min,
            omitted_max,
            OMITTED_COORDINATE_RESOLUTION,
        )

        # --------------------------------------------------------------
        # Calculate minimized 3D least-squares objective
        # --------------------------------------------------------------

        min_objective = np.full(
            X.shape,
            np.inf,
            dtype=float,
        )

        for omitted_coordinate in omitted_values:

            objective = np.zeros_like(
                X,
                dtype=float,
            )

            for anchor_id in measured_anchor_ids:

                anchor_position = anchor_positions[anchor_id]

                measured_range = float(
                    ranges_to_plot[anchor_id]
                )

<<<<<<< HEAD
                predicted_range = np.sqrt(
                    (X - anchor_position[i]) ** 2
                    + (Y - anchor_position[j]) ** 2
                    + (
                        omitted_coordinate
                        - anchor_position[k]
                    ) ** 2
=======
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
>>>>>>> 0b3ca437d1e7d18394df7173341256ae4915f98e
                )

                residual = (
                    predicted_range
                    - measured_range
                )

                objective += residual ** 2

            # Keep only the lowest objective value found for each
            # 2D location.
            min_objective = np.minimum(
                min_objective,
                objective,
            )

            # ----------------------------------------------------------
            # Update progress
            # ----------------------------------------------------------

            completed_calculations += 1

            progress = int(
                completed_calculations
                / total_calculations
                * 100
            )

            if (
                progress >= next_progress
                and next_progress <= 100
            ):

                print(
                    f" -> {next_progress}%",
                    end="",
                    flush=True,
                )

                next_progress += 10

        # --------------------------------------------------------------
        # Convert sum of squared residuals to RMS residual
        # --------------------------------------------------------------

        rms_residual = np.sqrt(
            min_objective
            / len(measured_anchor_ids)
        )

        # --------------------------------------------------------------
        # Highlight the low-residual region
        # --------------------------------------------------------------

        threshold = np.percentile(
            rms_residual,
            LOW_RESIDUAL_PERCENTILE,
        )

        low_residual = (
            rms_residual <= threshold
        )

        residual_mask = np.ma.masked_where(
            ~low_residual,
            np.ones_like(rms_residual),
        )

        ax.pcolormesh(
            X,
            Y,
            residual_mask,
            shading="auto",
            cmap=plt.matplotlib.colors.ListedColormap(
                [RESIDUAL_COLOR]
            ),
            alpha=RESIDUAL_ALPHA,
            zorder=1,
        )

        # --------------------------------------------------------------
        # Draw residual contour lines
        # --------------------------------------------------------------

        levels = np.percentile(
            rms_residual,
            [5, 10, 20, 40, 60],
        )

        levels = np.unique(levels)

        if len(levels) > 1:

            ax.contour(
                X,
                Y,
                rms_residual,
                levels=levels,
                colors="darkgoldenrod",
                linewidths=0.7,
                alpha=0.45,
                zorder=2,
            )

        # --------------------------------------------------------------
        # Draw projected 3D range-sphere boundaries
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

        for anchor_id, anchor_position in anchor_positions.items():

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
        # If using position mode, plot the supplied position
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
    # Finish progress display
    # ------------------------------------------------------------------

    if next_progress <= 100:
        print(
            " -> 100%",
            end="",
            flush=True,
        )

    print(" -> Complete!")

    # ------------------------------------------------------------------
    # Legend
    # ------------------------------------------------------------------

    # Anchor entries
    handles, labels = axes[0].get_legend_handles_labels()

    # Yellow region
    residual_region_handle = Patch(
        facecolor=RESIDUAL_COLOR,
        edgecolor="none",
        alpha=RESIDUAL_ALPHA,
        label=(
            f"Lowest {LOW_RESIDUAL_PERCENTILE:g}% "
            "RMS residual region"
        ),
    )

    # Residual contours
    contour_handle = Line2D(
        [0],
        [0],
        color="darkgoldenrod",
        linewidth=1.2,
        alpha=0.65,
        label="Increasing RMS residual contours",
    )

    # Range-sphere projection
    range_circle_handle = Line2D(
        [0],
        [0],
        color="black",
        linewidth=2,
        label="Projected 3D range sphere",
    )

    handles.extend([
        residual_region_handle,
        contour_handle,
        range_circle_handle,
    ])

    labels.extend([
        (
            f"Lowest {LOW_RESIDUAL_PERCENTILE:g}% "
            "RMS residual region"
        ),
        "Increasing RMS residual contours",
        "Projected 3D range sphere",
    ])

    if handles:

        fig.legend(
            handles,
            labels,
            loc="upper center",
            bbox_to_anchor=(0.5, 1.02),
            ncol=min(
                len(handles),
                6,
            ),
        )

    # ------------------------------------------------------------------
    # Explanatory note
    # ------------------------------------------------------------------

    fig.text(
        0.5,
        0.015,
        "Range circles are 2D projections of 3D range spheres; "
        "the true 3D position may lie inside the projected circle.",
        ha="center",
        va="bottom",
        fontsize=9,
        color="dimgray",
    )

    # Leave room for the legend and explanatory note.
    fig.tight_layout(
        rect=(0, 0.045, 1, 0.94)
    )

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


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------

if __name__ == "__main__":

    plot_anchor_ranges(
        ANCHORS,
        RANGES,
        position=POS,
        use_position=USE_POSITION,
        show=SHOW_PLOT,
        save_path=SAVE_PATH,
    )
