import numpy as np
import ctypes
import os
from typing import Optional

# ---------------------------------------------------------------------------
# Constants & DLL Initialization
# ---------------------------------------------------------------------------

# Load the RoboVengeance Engine
# Update this path if the DLL is in a different subdirectory
DLL_PATH = os.path.join(os.path.dirname(__file__), r"C:\robogambit\sa\bbnnn_v5.dll")

try:
    engine = ctypes.CDLL(DLL_PATH)
    # The DLL expects: (int* board, bool is_white)
    engine.get_best_move.argtypes = [ctypes.POINTER(ctypes.c_int), ctypes.c_bool]
    engine.get_best_move.restype = ctypes.c_char_p
except Exception as e:
    print(f"CRITICAL ERROR: Could not load RoboVengeance Engine at {DLL_PATH}")
    print(f"Details: {e}")
    engine = None

BOARD_SIZE = 6

# ---------------------------------------------------------------------------
# Main Entry Point (Interfaced to C++ Engine)
# ---------------------------------------------------------------------------

def get_best_move(board: np.ndarray, playing_white: bool = True) -> Optional[str]:
    """
    Interfaces with the RoboVengeance C++ Engine to return the best move.
    
    Parameters
    ----------
    board         : 6×6 NumPy array (0=Empty, 1-5=White, 6-10=Black)
    playing_white : True if engine plays White, False for Black.

    Returns
    -------
    Move string: '<piece_id>:<src_cell>-><dst_cell>' (e.g., '4:C1->C4')
    None if the engine returns an empty string or fails.
    """
    if engine is None:
        return None

    # 1. Flatten the 6x6 NumPy array for the C++ engine
    # .astype(np.int32) ensures the memory layout matches the C++ 'int*'
    flattened_board = board.flatten().astype(np.int32)
    
    # 2. Create the C-compatible pointer
    board_ptr = flattened_board.ctypes.data_as(ctypes.POINTER(ctypes.c_int))
    
    # 3. Call the high-performance search
    # Note: RoboVengeance handles its internal 15-min clock automatically
    result_bytes = engine.get_best_move(board_ptr, playing_white)
    
    if not result_bytes:
        return None

    # 4. Convert bytes from DLL back to Python string
    move_str = result_bytes.decode('utf-8')
    
    # If the engine returned an empty string (Checkmate/Stalemate logic), return None
    if move_str == "":
        return None
        
    return move_str

# ---------------------------------------------------------------------------
# Quick Smoke-Test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # Standard starting position
    # Indices: [row][col] -> [0][0] is A1, [5][5] is F6
    initial_board = np.array([
        [ 2,  3,  4,  5,  3,  2],   # Row 1 (A1–F1)
        [ 1,  1,  1,  1,  1,  1],   # Row 2
        [ 0,  0,  0,  0,  0,  0],   # Row 3
        [ 0,  0,  0,  0,  0,  0],   # Row 4
        [ 6,  6,  6,  6,  6,  6],   # Row 5
        [ 7,  8,  9, 10,  8,  7],   # Row 6 (A6–F6)
    ], dtype=int)

    print("Initial Board State:")
    print(initial_board)
    
    move = get_best_move(initial_board, playing_white=True)
    print(f"\nRoboVengeance Engine Decision: {move}")