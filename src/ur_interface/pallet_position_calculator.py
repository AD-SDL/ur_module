"""Pallet Position Calculator - Generalized Grid Position Calculations

This is a generalized calculator for any grid-based (pallet) positioning:
- Magazine slots
- Sample racks
- Plate positions
- Tray grids
- Any rectangular grid of positions
"""

import logging
from typing import Optional, Tuple


class PalletPositionCalculator:
    """Calculate positions in a rectangular grid (pallet)

    This is generalized for ANY grid-based positioning:
    - Magazines, racks, trays, plates, etc.
    - Just provide reference position and grid parameters
    """

    def __init__(self, logger: Optional[logging.Logger] = None):
        """Initialize calculator"""
        self.logger = logger or self._setup_logger()
        self._position_cache = {}

    def _setup_logger(self) -> logging.Logger:
        logger = logging.getLogger(__name__)
        logger.setLevel(logging.INFO)
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
            handler.setFormatter(formatter)
            logger.addHandler(handler)
        return logger

    def calculate_pallet_position(
        self,
        reference_position: list,
        index: int,
        x_gap_mm: float,
        y_gap_mm: float,
        num_x: int,
        num_y: int,
        index_mapping: Optional[list] = None,
    ) -> list:
        """Calculate position for a specific index in a pallet grid

        Args:
            reference_position: [x, y, z, rx, ry, rz] of index 0 position
            index: Which position to calculate (0 to num_x*num_y - 1)
            x_gap_mm: Spacing between columns (mm)
            y_gap_mm: Spacing between rows (mm)
            num_x: Number of columns
            num_y: Number of rows
            index_mapping: Optional custom index-to-grid mapping
                          (default: linear 0,1,2,... left-to-right, top-to-bottom)

        Returns:
            [x, y, z, rx, ry, rz] position for that index

        Example:
            # 5x4 magazine with 25mm spacing
            pos = calc.calculate_pallet_position(
                reference_position=[0.5, 0.2, 0.15, 0, 3.14, 0],
                index=7,
                x_gap_mm=25.0,
                y_gap_mm=25.0,
                num_x=5,
                num_y=4,
            )
            # Returns position of index 7 -> grid (2,1) -> [0.550, 0.225, 0.15, ...]
        """
        # Validate index
        total_positions = num_x * num_y
        if index < 0 or index >= total_positions:
            raise ValueError(f"Index {index} out of range (0 to {total_positions - 1})")

        # Get grid index for this position
        if index_mapping:
            if index >= len(index_mapping):
                raise ValueError(f"Index {index} not in mapping (length {len(index_mapping)})")
            grid_index = index_mapping[index]
        else:
            grid_index = index

        # Convert linear grid index to 2D coordinates
        x_idx = grid_index % num_x
        y_idx = grid_index // num_x

        # Calculate position
        position = reference_position.copy()
        position[0] += (x_idx * x_gap_mm) / 1000.0  # mm to meters
        position[1] += (y_idx * y_gap_mm) / 1000.0  # mm to meters

        self.logger.debug(f"Index {index} -> Grid ({x_idx},{y_idx}) -> Position {position[:3]}")

        return position

    def get_grid_coordinates(
        self,
        index: int,
        num_x: int,
        index_mapping: Optional[list] = None,
    ) -> Tuple[int, int]:
        """Convert index to grid coordinates

        Args:
            index: Position index
            num_x: Number of columns
            index_mapping: Optional custom mapping

        Returns:
            (x_index, y_index) tuple
        """
        if index_mapping:
            grid_index = index_mapping[index]
        else:
            grid_index = index

        x_idx = grid_index % num_x
        y_idx = grid_index // num_x

        return x_idx, y_idx

    def print_pallet_layout(
        self,
        num_x: int,
        num_y: int,
        x_gap_mm: float,
        y_gap_mm: float,
        reference_position: list,
        index_mapping: Optional[list] = None,
    ):
        """Print visual representation of pallet layout"""
        print(f"\nPallet Layout: {num_x} columns x {num_y} rows")
        print(f"Spacing: {x_gap_mm}mm (X) x {y_gap_mm}mm (Y)")
        print(f"Reference position (Index 0): {reference_position[:3]}")
        print("\nIndex layout:")

        for y in range(num_y):
            row = []
            for x in range(num_x):
                grid_idx = y * num_x + x
                # Find which index maps to this grid position
                if index_mapping:
                    try:
                        idx = index_mapping.index(grid_idx)
                        row.append(f"{idx:3d}")
                    except ValueError:
                        row.append("  -")
                else:
                    row.append(f"{grid_idx:3d}")
            print(" ".join(row))
        print()


# Singleton instance for convenience
_pallet_calculator = PalletPositionCalculator()


def calculate_pallet_position(
    reference_position: list,
    index: int,
    x_gap_mm: float,
    y_gap_mm: float,
    num_x: int,
    num_y: int,
    index_mapping: Optional[list] = None,
) -> list:
    """Convenience function to calculate pallet position

    This is a standalone function that doesn't require creating a calculator instance.
    """
    return _pallet_calculator.calculate_pallet_position(
        reference_position=reference_position,
        index=index,
        x_gap_mm=x_gap_mm,
        y_gap_mm=y_gap_mm,
        num_x=num_x,
        num_y=num_y,
        index_mapping=index_mapping,
    )


if __name__ == "__main__":
    """Example usage"""

    print("=" * 60)
    print("Pallet Position Calculator")
    print("=" * 60)

    calc = PalletPositionCalculator()

    # Example: 5x4 magazine
    print("\n=== Example 1: Magazine (5x4, 25mm spacing) ===")
    calc.print_pallet_layout(
        num_x=5,
        num_y=4,
        x_gap_mm=25.0,
        y_gap_mm=25.0,
        reference_position=[0.5, 0.2, 0.15, 0, 3.14, 0],
    )

    # Calculate some positions
    for idx in [0, 1, 5, 7, 19]:
        pos = calc.calculate_pallet_position(
            reference_position=[0.5, 0.2, 0.15, 0, 3.14, 0],
            index=idx,
            x_gap_mm=25.0,
            y_gap_mm=25.0,
            num_x=5,
            num_y=4,
        )
        x, y = pos[0], pos[1]
        print(f"Index {idx:2d}: x={x:.3f}m, y={y:.3f}m")

    # Example: 8x12 96-well plate (9mm spacing)
    print("\n=== Example 2: 96-Well Plate (8x12, 9mm spacing) ===")
    calc.print_pallet_layout(
        num_x=12,
        num_y=8,
        x_gap_mm=9.0,
        y_gap_mm=9.0,
        reference_position=[0.3, -0.4, 0.35, 0, 3.14, 0],
    )

    # Calculate well A1 (0) and H12 (95)
    well_a1 = calc.calculate_pallet_position(
        reference_position=[0.3, -0.4, 0.35, 0, 3.14, 0],
        index=0,
        x_gap_mm=9.0,
        y_gap_mm=9.0,
        num_x=12,
        num_y=8,
    )
    well_h12 = calc.calculate_pallet_position(
        reference_position=[0.3, -0.4, 0.35, 0, 3.14, 0],
        index=95,
        x_gap_mm=9.0,
        y_gap_mm=9.0,
        num_x=12,
        num_y=8,
    )
    print(f"Well A1  (index 0):  {well_a1}")
    print(f"Well H12 (index 95): {well_h12}")
