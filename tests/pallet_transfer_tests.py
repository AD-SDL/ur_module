"""Examples - Generalized Pallet Transfer Methods

Shows how the generalized transfer_from_pallet and transfer_to_pallet
work for various use cases.
"""

from ur_interface.ur import UR


def example_magazine_workflow():
    """Example: Magazine sample handling (original beamline use case)"""

    print("=" * 60)
    print("Example 1: Magazine Sample Handling")
    print("=" * 60)

    robot = UR(hostname="164.54.116.129")

    # Magazine configuration
    magazine_ref = [0.5, 0.2, 0.15, 0, 3.14, 0]  # First slot position
    stage_pos = [0.3, -0.4, 0.35, 0, 3.14, 0]  # Sample stage

    # Get sample from magazine slot 5
    print("\n=== Getting sample from magazine slot 5 ===")
    robot.transfer_from_pallet(
        pallet_reference=magazine_ref,
        pallet_index=5,  # Slot 5
        pallet_x_gap_mm=25.0,  # 25mm spacing
        pallet_y_gap_mm=25.0,
        pallet_num_x=5,  # 5x4 grid
        pallet_num_y=4,
        target=stage_pos,
    )

    # Do measurements...
    input("\nPress Enter when measurements complete...")

    # Return sample to magazine slot 5
    print("\n=== Returning sample to magazine slot 5 ===")
    robot.transfer_to_pallet(
        source=stage_pos,
        pallet_reference=magazine_ref,
        pallet_index=5,  # Same slot
        pallet_x_gap_mm=25.0,
        pallet_y_gap_mm=25.0,
        pallet_num_x=5,
        pallet_num_y=4,
    )

    robot.disconnect()


def example_96_well_plate():
    """Example: 96-well plate sample handling"""

    print("=" * 60)
    print("Example 2: 96-Well Plate Sampling")
    print("=" * 60)

    robot = UR(hostname="164.54.116.129")

    # 96-well plate configuration (8 rows x 12 columns, 9mm spacing)
    plate_ref = [0.3, -0.4, 0.35, 0, 3.14, 0]  # Well A1 position
    pipette_pos = [0.2, -0.3, 0.4, 0, 3.14, 0]  # Pipetting location

    # Sample from well D12 (index 47: row 3, column 11)
    print("\n=== Sampling from well D12 ===")
    robot.transfer_from_pallet(
        pallet_reference=plate_ref,
        pallet_index=47,  # Well D12
        pallet_x_gap_mm=9.0,  # 9mm well spacing
        pallet_y_gap_mm=9.0,
        pallet_num_x=12,  # 12 columns
        pallet_num_y=8,  # 8 rows
        target=pipette_pos,
        source_approach_distance=0.04,  # 40mm clearance
        target_approach_distance=0.04,
    )

    # Process sample...

    # Return to well
    print("\n=== Returning to well D12 ===")
    result = robot.transfer_to_pallet(
        source=pipette_pos,
        pallet_reference=plate_ref,
        pallet_index=47,
        pallet_x_gap_mm=9.0,
        pallet_y_gap_mm=9.0,
        pallet_num_x=12,
        pallet_num_y=8,
    )
    print(f"Result: {result}")

    robot.disconnect()
