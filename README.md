🤖 RoboVengeance Engine: 6x6 Chess

A high-performance, tournament-ready chess engine written in C++ specifically for the 6x6 Regicide variant. This engine combines deep search heuristics with dynamic time management to dominate in 15-minute time-control formats.
🏆 Key Features
🧠 Advanced Search & Terminal Logic

    Iterative Deepening: Progressive search (up to Depth 25) ensures a valid "best move" is always ready.
    Terminal State Recognition: The engine is programmed to identify Checkmate and Stalemate instantly. If no legal moves exist, the engine gracefully terminates the turn by returning an empty string, preventing illegal moves in deadlocked positions.
    Mate-in-1 Detection: A specialized look-ahead system identifies instant winning moves before entering the main search, ensuring 100% tactical efficiency in the endgame.
    Zobrist Hashing: Detects and avoids Threefold Repetition using unique 64-bit board signatures.

⏱️ Dynamic Time Management

Built for the long game, RoboVengeance manages a 15-minute total game clock:

    Proactive Budgeting: Allocates 5% of the remaining total game time per move.
    Panic Mode: Automatically switches to high-speed (100ms) "blitz" moves if the total clock drops below 10 seconds.
    Internal Persistence: Tracks the global clock internally across DLL calls to mirror real-world tournament hardware constraints.

📊 Evaluation Heuristics

    Piece-Square Tables (PST): Optimized for a 6x6 grid to enforce center dominance.
    Knight Aggression: Specialized PST weights for Knights to maximize their fork potential in restricted space.
    King Safety: Rewards defensive shielding and penalizes exposure to open lines.

🎮 Tournament Rules

    Regicide: Victory is achieved by the physical capture of the King.
    Board: 6x6 squares (Files A-F, Ranks 1-6).
    Promotion: Automatic promotion to the most valuable piece currently off the board.

🛠️ Installation & Benchmarking
Compilation

Compile as a shared library (.dll or .so) with high-level optimizations:
Bash

g++ -O3 -march=native -shared -static -o RoboVengeance.dll RoboVengeance.cpp

Performance Benchmarking

To test the engine's node-per-second (NPS) performance or simulate a match, use the provided Python arena script:
Python

# Run a bot-vs-bot simulation
python bot_battle_arena.py

DLL Interface
C++

extern "C" __declspec(dllexport) const char* get_best_move(int* board36, bool is_white);

    Returns: ID:SRC->DST (e.g., 4:C1->C4) or "" if the game is over.

Developed for the March 2026 RoboGambit Tournament at IIT Delhi.
