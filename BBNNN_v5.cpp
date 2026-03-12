#include <vector>
#include <string>
#include <algorithm>
#include <cstring>
#include <chrono>
#include <random>

using namespace std;

typedef uint64_t Bitboard; 
const int INF = 30000;
const int KING_VAL = 25000;
const int p_vals[] = {0, 100, 320, 330, 900, 25000, 100, 320, 330, 900, 25000};

const int knight_aggro[36] = {
   -40, -20, -10, -10, -20, -40, -20,  10,  20,  20,  10, -20, -10,  25,  45,  45,  25, -10,
   -10,  25,  45,  45,  25, -10, -20,  10,  20,  20,  10, -20, -40, -20, -10, -10, -20, -40
};

const int king_mid[36] = {
    30,  40,  20,  20,  40,  30, 10,   5,   0,   0,   5,  10, -10, -20, -20, -20, -20, -10,
   -20, -30, -30, -30, -30, -20, -30, -40, -40, -40, -40, -30, -40, -50, -50, -50, -50, -40
};

struct Move {
    int id, from, to, promote_to, score;
    Move(int _id, int _f, int _t, int _p = 0, int _s = 0) : id(_id), from(_f), to(_t), promote_to(_p), score(_s) {}
    bool operator>(const Move& o) const { return score > o.score; }
};

class RoboVengeanceEngine {
public:
    Bitboard pieces[11]; 
    int history[36][36];
    uint64_t board_history[256], zobrist_table[36][11], zobrist_side, current_hash;
    int history_ply = 0;
    
    chrono::time_point<chrono::steady_clock> start_time;
    long long move_time_limit_ms = 0;
    long long total_time_remaining_ms = 15 * 60 * 1000; 
    bool stop_search = false;

    RoboVengeanceEngine() {
        mt19937_64 rng(42);
        for (int i = 0; i < 36; i++) for (int j = 1; j <= 10; j++) zobrist_table[i][j] = rng();
        zobrist_side = rng();
        memset(history, 0, sizeof(history));
    }

    void load(int* b_in, bool white) {
        memset(pieces, 0, sizeof(pieces)); current_hash = 0;
        for (int i = 0; i < 36; i++) {
            if (b_in[i]) { pieces[b_in[i]] |= (1ULL << i); current_hash ^= zobrist_table[i][b_in[i]]; }
        }
        if (!white) current_hash ^= zobrist_side;
    }

    int board_at(int sq) { for (int i = 1; i <= 10; i++) if ((pieces[i] >> sq) & 1) return i; return 0; }

    bool is_in_check(bool white) {
        Bitboard k_bb = pieces[white ? 5 : 10]; if (!k_bb) return true;
        int sq = __builtin_ctzll(k_bb);
        int p_dir = white ? -6 : 6;
        for (int s : {-1, 1}) {
            int f = sq + p_dir + s;
            if (f >= 0 && f < 36 && abs(f % 6 - sq % 6) == 1 && board_at(f) == (white ? 6 : 1)) return true;
        }
        static const int ko[] = {-13,-11,-8,-4,4,8,11,13};
        for (int o : ko) {
            int f = sq + o;
            if (f >= 0 && f < 36 && max(abs(f%6-sq%6), abs(f/6-sq/6)) <= 2 && board_at(f) == (white ? 7 : 2)) return true;
        }
        static const int ds[] = {7,5,-5,-7,1,-1,6,-6};
        for (int d = 0; d < 8; d++) {
            for (int s = 1; s < 6; s++) {
                int f = sq + ds[d] * s;
                if (f < 0 || f >= 36 || abs(f%6-(f-ds[d])%6) > 1) break;
                int p = board_at(f); if (p) {
                    if (white) { if (s==1 && p==10) return true; if (d<4 && (p==8||p==9)) return true; if (d>=4 && p==9) return true; }
                    else { if (s==1 && p==5) return true; if (d<4 && (p==3||p==4)) return true; if (d>=4 && p==4) return true; }
                    break;
                }
            }
        }
        return false;
    }

    int evaluate() {
        if (!pieces[5]) return -KING_VAL; if (!pieces[10]) return KING_VAL;
        int score = 0;
        for (int i = 0; i < 36; i++) {
            int p = board_at(i); if (!p) continue;
            int b = (p==2||p==7||p==4||p==9) ? knight_aggro[(p<=5)?i:(35-i)] : 0;
            if (p==5||p==10) b += king_mid[(p<=5)?i:(35-i)];
            score += (p <= 5) ? (p_vals[p] + b) : -(p_vals[p] + b);
        }
        return score;
    }

    int search(int depth, int alpha, int beta, bool max_p) {
        for (int i = 0; i < history_ply - 1; i++) if (board_history[i] == current_hash) return 0;
        static int nodes = 0;
        if ((++nodes & 127) == 0) {
            auto elapsed = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start_time).count();
            if (elapsed > move_time_limit_ms) stop_search = true;
        }
        if (stop_search || depth <= 0) return evaluate();
        if (!pieces[5]) return -KING_VAL; if (!pieces[10]) return KING_VAL;

        auto raw = gen_moves(max_p);
        if (raw.empty()) return evaluate();
        for (auto& m : raw) {
            int v = board_at(m.to); m.score = v ? (20000 + p_vals[v]) : history[m.from][m.to];
        }
        sort(raw.begin(), raw.end(), greater<Move>());

        int best = max_p ? -INF : INF;
        for (auto& m : raw) {
            int v = board_at(m.to); make_move(m); board_history[history_ply++] = current_hash;
            int score = search(depth - 1, alpha, beta, !max_p);
            history_ply--; unmake_move(m, v);
            if (stop_search) break;
            if (max_p) { best = max(best, score); alpha = max(alpha, best); }
            else { best = min(best, score); beta = min(beta, best); }
            if (beta <= alpha) break;
        }
        return best;
    }

    void make_move(const Move& m) {
        int v = board_at(m.to); if (v) { current_hash ^= zobrist_table[m.to][v]; pieces[v] &= ~(1ULL << m.to); }
        current_hash ^= zobrist_table[m.from][m.id]; pieces[m.id] &= ~(1ULL << m.from);
        int r = m.promote_to ? m.promote_to : m.id;
        current_hash ^= zobrist_table[m.to][r]; pieces[r] |= (1ULL << m.to);
        current_hash ^= zobrist_side;
    }

    void unmake_move(const Move& m, int v) {
        current_hash ^= zobrist_side;
        int r = m.promote_to ? m.promote_to : m.id;
        current_hash ^= zobrist_table[m.to][r]; pieces[r] &= ~(1ULL << m.to);
        current_hash ^= zobrist_table[m.from][m.id]; pieces[m.id] |= (1ULL << m.from);
        if (v) { current_hash ^= zobrist_table[m.to][v]; pieces[v] |= (1ULL << m.to); }
    }

    vector<Move> gen_moves(bool white) {
        vector<Move> moves; Bitboard self = 0, enemy = 0;
        for (int i = 1; i <= 5; i++) { self |= pieces[white ? i : i + 5]; enemy |= pieces[white ? i + 5 : i]; }
        Bitboard all = self | enemy;
        int start = white ? 1 : 6;
        for (int id = start; id < start + 5; id++) {
            Bitboard bb = pieces[id];
            while (bb) {
                int from = __builtin_ctzll(bb); bb &= (bb - 1);
                if (id == 3 || id == 8 || id == 4 || id == 9) {
                    static const int ds[] = {7,5,-5,-7,1,-1,6,-6};
                    int lim = (id == 3 || id == 8) ? 4 : 8;
                    for (int d = 0; d < lim; d++) {
                        for (int s = 1; s < 6; s++) {
                            int to = from + ds[d] * s;
                            if (to < 0 || to >= 36 || abs(to % 6 - (to - ds[d]) % 6) > 1) break;
                            if (!((self >> to) & 1)) moves.push_back(Move(id, from, to));
                            if ((all >> to) & 1) break;
                        }
                    }
                } else if (id == 2 || id == 7) {
                    static const int ko[] = {-13,-11,-8,-4,4,8,11,13};
                    for (int o : ko) {
                        int to = from + o; if (to >= 0 && to < 36 && max(abs(to % 6 - from % 6), abs(to / 6 - from / 6)) <= 2)
                            if (!((self >> to) & 1)) moves.push_back(Move(id, from, to));
                    }
                } else if (id == 1 || id == 6) {
                    int dir = white ? 6 : -6; int to = from + dir;
                    if (to >= 0 && to < 36 && !((all >> to) & 1)) add_pawn_move(moves, id, from, to, white);
                    for (int s : {-1, 1}) {
                        int c = to + s; if (c >= 0 && c < 36 && abs(c % 6 - from % 6) == 1 && ((enemy >> c) & 1)) add_pawn_move(moves, id, from, c, white);
                    }
                } else if (id == 5 || id == 10) {
                    static const int ko[] = {-7,-6,-5,-1,1,5,6,7};
                    for (int o : ko) {
                        int to = from + o; if (to >= 0 && to < 36 && abs(to % 6 - from % 6) <= 1)
                            if (!((self >> to) & 1)) moves.push_back(Move(id, from, to));
                    }
                }
            }
        }
        return moves;
    }

    void add_pawn_move(vector<Move>& mvs, int id, int f, int t, bool white) {
        if ((white && t / 6 == 5) || (!white && t / 6 == 0)) {
            if (__builtin_popcountll(pieces[white ? 4 : 9]) < 1) mvs.push_back(Move(id, f, t, white ? 4 : 9));
            if (__builtin_popcountll(pieces[white ? 3 : 8]) < 2) mvs.push_back(Move(id, f, t, white ? 3 : 8));
            if (__builtin_popcountll(pieces[white ? 2 : 7]) < 2) mvs.push_back(Move(id, f, t, white ? 2 : 7));
        } else mvs.push_back(Move(id, f, t));
    }
};

extern "C" {
    static RoboVengeanceEngine engine; static char out_s[64];
    __declspec(dllexport) const char* get_best_move(int* b_in, bool white) {
        engine.load(b_in, white); engine.start_time = chrono::steady_clock::now();
        engine.stop_search = false; engine.history_ply = 0;
        engine.board_history[engine.history_ply++] = engine.current_hash;

        // BUDGET: Use 5% of remaining bank
        engine.move_time_limit_ms = engine.total_time_remaining_ms / 20;
        if (engine.move_time_limit_ms < 100) engine.move_time_limit_ms = 100;

        auto raw = engine.gen_moves(white);

        // --- 1. INSTANT TERMINATION: Check if ALREADY in Checkmate ---
        // If we have no legal moves, the game is already over (Mate or Stalemate)
        vector<Move> legal;
        for (auto& m : raw) {
            int v = engine.board_at(m.to);
            engine.make_move(m);
            if (!engine.is_in_check(white)) legal.push_back(m);
            engine.unmake_move(m, v);
        }

        if (legal.empty()) {
            return ""; // RETURN NOTHING: We are under checkmate or stalemate
        }

        // --- 2. INSTANT KILL SYSTEM: Immediate King Capture ---
        for (auto& m : legal) {
            if (engine.board_at(m.to) == (white ? 10 : 5)) {
                auto end = chrono::steady_clock::now();
                engine.total_time_remaining_ms -= chrono::duration_cast<chrono::milliseconds>(end - engine.start_time).count();
                auto c2s = [](int s) { return string(1, 'A' + (s % 6)) + to_string((s / 6) + 1); };
                string res = to_string(m.id) + ":" + c2s(m.from) + "->" + c2s(m.to);
                if (m.promote_to) res += "=" + to_string(m.promote_to);
                strncpy(out_s, res.c_str(), 63); return out_s;
            }
        }

        // --- 3. MATE-IN-1 DETECTION ---
        for (auto& m : legal) {
            int v = engine.board_at(m.to);
            engine.make_move(m);
            auto responses = engine.gen_moves(!white);
            bool has_escape = false;
            for (auto& r : responses) {
                int rv = engine.board_at(r.to);
                engine.make_move(r);
                if (!engine.is_in_check(!white)) { has_escape = true; engine.unmake_move(r, rv); break; }
                engine.unmake_move(r, rv);
            }
            engine.unmake_move(m, v);
            if (!has_escape) {
                // Opponent has no legal moves after this -> This is MATE-IN-1
                auto end = chrono::steady_clock::now();
                engine.total_time_remaining_ms -= chrono::duration_cast<chrono::milliseconds>(end - engine.start_time).count();
                auto c2s = [](int s) { return string(1, 'A' + (s % 6)) + to_string((s / 6) + 1); };
                string res = to_string(m.id) + ":" + c2s(m.from) + "->" + c2s(m.to);
                if (m.promote_to) res += "=" + to_string(m.promote_to);
                strncpy(out_s, res.c_str(), 63); return out_s;
            }
        }

        // --- 4. ITERATIVE DEEPENING SEARCH ---
        Move best_m = legal[0];
        for (int d = 1; d <= 25; d++) {
            int a = -INF, b = INF, best_v = white ? -INF : INF; Move iter_best = best_m;
            for (auto& m : legal) {
                int v = engine.board_at(m.to); engine.make_move(m); 
                int s = engine.search(d - 1, a, b, !white); 
                engine.unmake_move(m, v);
                if (engine.stop_search) break;
                if (white && s > best_v) { best_v = s; iter_best = m; a = s; }
                if (!white && s < best_v) { best_v = s; iter_best = m; b = s; }
            }
            if (!engine.stop_search) best_m = iter_best; else break;
        }

        auto final_end = chrono::steady_clock::now();
        engine.total_time_remaining_ms -= chrono::duration_cast<chrono::milliseconds>(final_end - engine.start_time).count();
        auto c2s = [](int s) { return string(1, 'A' + (s % 6)) + to_string((s / 6) + 1); };
        string res = to_string(best_m.id) + ":" + c2s(best_m.from) + "->" + c2s(best_m.to);
        if (best_m.promote_to) res += "=" + to_string(best_m.promote_to);
        strncpy(out_s, res.c_str(), 63); return out_s;
    }
}