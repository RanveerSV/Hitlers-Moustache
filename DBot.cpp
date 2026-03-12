/*
 * ApexEngine.cpp  —  RoboGambit 6×6 Chess Engine
 * ================================================
 * DLL interface:  get_best_move(int* board36, bool is_white) -> const char*
 *
 * Board layout (matches tournament runner):
 *   Flat array of 36 ints, index 0 = A1, index 35 = F6
 *   sq = (rank-1)*6 + (file-'A')
 *   col = sq % 6    (0=A … 5=F)
 *   row = sq / 6    (0=rank1 … 5=rank6)
 *
 * Piece IDs:
 *   White: 1=Pawn  2=Knight  3=Bishop  4=Queen  5=King
 *   Black: 6=Pawn  7=Knight  8=Bishop  9=Queen 10=King
 *
 * Output format:  "<piece_id>:<src>-><dst>"   e.g. "1:A2->A3"
 *
 * Fixed vs RoboVengeanceEngine:
 *   1.  Transposition table (1M entries, exact/lower/upper)
 *   2.  Quiescence search with delta pruning
 *   3.  Null-move pruning (R=2, guarded for endgame/check)
 *   4.  Killer moves (2 per ply)
 *   5.  History heuristic updated correctly on beta cutoffs
 *   6.  MVV-LVA capture ordering
 *   7.  Richer eval: mobility, king safety, passed pawns, bishop pair
 *   8.  Endgame PST switching
 *   9.  Correct knight L-shape check (dr*dc==2)
 *  10.  Per-call node counter (no stale static state)
 *  11.  No variable-name collision in root (beta param != board)
 *  12.  history[] reset each call so TT stays warm but heuristics stay fresh
 */

#include <vector>
#include <string>
#include <algorithm>
#include <cstring>
#include <chrono>
#include <random>

using namespace std;
typedef uint64_t u64;

// ─── Constants ───────────────────────────────────────────────
const int INF      = 30000;
const int KING_VAL = 25000;
const int MAX_PLY  = 64;
const int TT_SIZE  = 1 << 20;   // 1 048 576 entries

const int P_VAL[11] = {0, 100, 320, 340, 950, KING_VAL,
                           100, 320, 340, 950, KING_VAL};

// ─── Piece-Square Tables (white, sq 0=A1 … 35=F6) ────────────
const int PST_PAWN[36] = {
     0,   0,   0,   0,   0,   0,
     5,   8,   8,   8,   8,   5,
    10,  15,  20,  20,  15,  10,
    20,  25,  30,  30,  25,  20,
    35,  40,  45,  45,  40,  35,
    90,  90,  90,  90,  90,  90,
};
const int PST_KNIGHT[36] = {
   -40, -25, -15, -15, -25, -40,
   -20,   5,  15,  15,   5, -20,
   -10,  20,  40,  40,  20, -10,
   -10,  20,  40,  40,  20, -10,
   -20,   5,  15,  15,   5, -20,
   -40, -25, -15, -15, -25, -40,
};
const int PST_BISHOP[36] = {
   -15,  -5,  -5,  -5,  -5, -15,
    -5,  10,  10,  10,  10,  -5,
    -5,  10,  20,  20,  10,  -5,
    -5,  10,  20,  20,  10,  -5,
    -5,  10,  10,  10,  10,  -5,
   -15,  -5,  -5,  -5,  -5, -15,
};
const int PST_QUEEN[36] = {
   -10,  -5,   0,   0,  -5, -10,
    -5,   5,  10,  10,   5,  -5,
     0,  10,  20,  20,  10,   0,
     0,  10,  20,  20,  10,   0,
    -5,   5,  10,  10,   5,  -5,
   -10,  -5,   0,   0,  -5, -10,
};
const int PST_KING_MID[36] = {
    30,  40,  20,  20,  40,  30,
    10,   5,   0,   0,   5,  10,
   -10, -20, -20, -20, -20, -10,
   -20, -30, -30, -30, -30, -20,
   -30, -40, -40, -40, -40, -30,
   -40, -50, -50, -50, -50, -40,
};
const int PST_KING_END[36] = {
   -10,  -5,   0,   0,  -5, -10,
    -5,  10,  15,  15,  10,  -5,
     0,  15,  25,  25,  15,   0,
     0,  15,  25,  25,  15,   0,
    -5,  10,  15,  15,  10,  -5,
   -10,  -5,   0,   0,  -5, -10,
};

// ─── Direction tables ────────────────────────────────────────
const int DIR_DIAG[4]     = { 7,  5, -5, -7};
const int DIR_STRAIGHT[4] = { 1, -1,  6, -6};
const int DIR_ALL[8]      = { 7,  5, -5, -7,  1, -1,  6, -6};
const int KNIGHT_OFF[8]   = {-13,-11, -8, -4,  4,  8, 11, 13};

// ─── Helpers ─────────────────────────────────────────────────
inline int sq_col(int sq) { return sq % 6; }
inline int sq_row(int sq) { return sq / 6; }
inline bool no_wrap(int a, int b) { return abs(sq_col(a) - sq_col(b)) <= 1; }
inline bool knight_ok(int a, int b) {
    int dr=abs(sq_row(a)-sq_row(b)), dc=abs(sq_col(a)-sq_col(b));
    return dr*dc==2;
}
static string sq2s(int sq) {
    return string(1,'A'+sq_col(sq)) + to_string(sq_row(sq)+1);
}

// ─── Move ────────────────────────────────────────────────────
struct Move {
    int id=0, from=0, to=0, promote_to=0, score=0;
    Move() = default;
    Move(int i,int f,int t,int p=0,int s=0):id(i),from(f),to(t),promote_to(p),score(s){}
    bool operator>(const Move& o) const { return score > o.score; }
};

// ─── Transposition Table ─────────────────────────────────────
enum TTFlag : uint8_t { TT_EXACT, TT_LOWER, TT_UPPER };
struct TTEntry {
    u64    hash  = 0;
    int    depth = 0;
    int    score = 0;
    TTFlag flag  = TT_EXACT;
    Move   best;
};
static TTEntry TT[TT_SIZE];

// ─────────────────────────────────────────────────────────────
class ApexEngine {
public:
    u64  pieces[11];
    u64  Z[36][11], Z_side, cur_hash;

    int  history[36][36];
    Move killers[MAX_PLY][2];
    // rep_table[0..game_ply-1] = actual game positions (persist across calls)
    // rep_table[game_ply..rep_ply-1] = current search tree
    u64  rep_table[512];
    int  rep_ply;        // top of rep stack (game + search)
    int  game_ply;       // how many game-history hashes are loaded
    int  search_nodes;
    bool stop_flag;
    chrono::time_point<chrono::steady_clock> t_start;

    ApexEngine() {
        mt19937_64 rng(0xDEADBEEFCAFEULL);
        for (int i=0;i<36;i++)
            for (int j=1;j<=10;j++)
                Z[i][j] = rng();
        Z_side = rng();
        game_ply = 0;
        rep_ply  = 0;
    }

    // ── Load ─────────────────────────────────────────────────
    void load(const int* b, bool white) {
        memset(pieces, 0, sizeof(pieces));
        cur_hash = 0;
        for (int i=0;i<36;i++) {
            int p=b[i];
            if (p>=1&&p<=10) {
                pieces[p] |= (1ULL<<i);
                cur_hash ^= Z[i][p];
            }
        }
        if (!white) cur_hash ^= Z_side;
    }

    // ── Piece at square ──────────────────────────────────────
    inline int at(int sq) const {
        for (int i=1;i<=10;i++)
            if ((pieces[i]>>sq)&1) return i;
        return 0;
    }

    // ── Make / Unmake ────────────────────────────────────────
    int make(const Move& m) {
        int cap=at(m.to);
        if (cap) { cur_hash^=Z[m.to][cap]; pieces[cap]&=~(1ULL<<m.to); }
        cur_hash^=Z[m.from][m.id]; pieces[m.id]&=~(1ULL<<m.from);
        int r=m.promote_to?m.promote_to:m.id;
        cur_hash^=Z[m.to][r]; pieces[r]|=(1ULL<<m.to);
        cur_hash^=Z_side;
        return cap;
    }

    void unmake(const Move& m, int cap) {
        cur_hash^=Z_side;
        int r=m.promote_to?m.promote_to:m.id;
        cur_hash^=Z[m.to][r]; pieces[r]&=~(1ULL<<m.to);
        cur_hash^=Z[m.from][m.id]; pieces[m.id]|=(1ULL<<m.from);
        if (cap) { cur_hash^=Z[m.to][cap]; pieces[cap]|=(1ULL<<m.to); }
    }

    // ── Check detection ──────────────────────────────────────
    bool in_check(bool white) const {
        u64 kb=pieces[white?5:10];
        if (!kb) return true;
        int sq=__builtin_ctzll(kb);
        int ep=white?6:1, en=white?7:2, eb=white?8:3, eq=white?9:4, ek=white?10:5;

        // Pawn
        int pd=white?6:-6;
        for (int dc:{-1,1}) {
            int f=sq+pd+dc;
            if (f>=0&&f<36&&abs(sq_col(f)-sq_col(sq))==1&&at(f)==ep) return true;
        }
        // Knight
        for (int off:KNIGHT_OFF) {
            int f=sq+off;
            if (f>=0&&f<36&&knight_ok(sq,f)&&at(f)==en) return true;
        }
        // Diagonal (bishop/queen)
        for (int d:DIR_DIAG) {
            for (int s=sq;;) {
                int f=s+d;
                if (f<0||f>=36||!no_wrap(s,f)) break;
                int p=at(f); if (p) { if (p==eb||p==eq) return true; break; }
                s=f;
            }
        }
        // Straight (queen)
        for (int d:DIR_STRAIGHT) {
            for (int s=sq;;) {
                int f=s+d;
                if (f<0||f>=36||!no_wrap(s,f)) break;
                int p=at(f); if (p) { if (p==eq) return true; break; }
                s=f;
            }
        }
        // King
        for (int d:DIR_ALL) {
            int f=sq+d;
            if (f>=0&&f<36&&no_wrap(sq,f)&&at(f)==ek) return true;
        }
        return false;
    }

    // ── Pawn helper ──────────────────────────────────────────
    void add_pawn(vector<Move>& mv, int id, int fr, int to, bool white) const {
        if ((white&&sq_row(to)==5)||(!white&&sq_row(to)==0)) {
            int q=white?4:9, b=white?3:8, n=white?2:7;
            if (__builtin_popcountll(pieces[q])<1) mv.push_back(Move(id,fr,to,q));
            if (__builtin_popcountll(pieces[b])<2) mv.push_back(Move(id,fr,to,b));
            if (__builtin_popcountll(pieces[n])<2) mv.push_back(Move(id,fr,to,n));
        } else mv.push_back(Move(id,fr,to));
    }

    // ── Move generation ──────────────────────────────────────
    vector<Move> gen(bool white) const {
        vector<Move> mv; mv.reserve(64);
        u64 self=0,enemy=0;
        for (int i=1;i<=5;i++){self|=pieces[white?i:i+5];enemy|=pieces[white?i+5:i];}
        u64 all=self|enemy;
        int base=white?1:6;

        for (int id=base;id<base+5;id++) {
            u64 bb=pieces[id];
            while (bb) {
                int fr=__builtin_ctzll(bb); bb&=bb-1;
                int pt=(id<=5)?id:id-5;

                if (pt==1) {
                    int dir=white?6:-6, to=fr+dir;
                    if (to>=0&&to<36&&!((all>>to)&1))
                        const_cast<ApexEngine*>(this)->add_pawn(mv,id,fr,to,white);
                    for (int dc:{-1,1}) {
                        int c=to+dc;
                        if (c>=0&&c<36&&abs(sq_col(c)-sq_col(fr))==1&&((enemy>>c)&1))
                            const_cast<ApexEngine*>(this)->add_pawn(mv,id,fr,c,white);
                    }
                } else if (pt==2) {
                    for (int off:KNIGHT_OFF) {
                        int to=fr+off;
                        if (to>=0&&to<36&&knight_ok(fr,to)&&!((self>>to)&1))
                            mv.push_back(Move(id,fr,to));
                    }
                } else if (pt==3) {
                    for (int d:DIR_DIAG) {
                        for (int s=fr;;) {
                            int to=s+d;
                            if (to<0||to>=36||!no_wrap(s,to)) break;
                            if ((self>>to)&1) break;
                            mv.push_back(Move(id,fr,to));
                            if ((all>>to)&1) break;
                            s=to;
                        }
                    }
                } else if (pt==4) {
                    for (int d:DIR_ALL) {
                        for (int s=fr;;) {
                            int to=s+d;
                            if (to<0||to>=36||!no_wrap(s,to)) break;
                            if ((self>>to)&1) break;
                            mv.push_back(Move(id,fr,to));
                            if ((all>>to)&1) break;
                            s=to;
                        }
                    }
                } else if (pt==5) {
                    for (int d:DIR_ALL) {
                        int to=fr+d;
                        if (to>=0&&to<36&&no_wrap(fr,to)&&!((self>>to)&1))
                            mv.push_back(Move(id,fr,to));
                    }
                }
            }
        }
        return mv;
    }

    // ── Evaluation ───────────────────────────────────────────
    bool is_endgame() const {
        return (__builtin_popcountll(pieces[4]|pieces[9])+
                __builtin_popcountll(pieces[3]|pieces[8])+
                __builtin_popcountll(pieces[2]|pieces[7]))<=2;
    }

    int pst_val(int p, int sq) const {
        bool eg=is_endgame();
        int idx=(p<=5)?sq:(35-sq);
        int pt=(p<=5)?p:p-5;
        switch(pt){
            case 1: return PST_PAWN[idx];
            case 2: return PST_KNIGHT[idx];
            case 3: return PST_BISHOP[idx];
            case 4: return PST_QUEEN[idx];
            case 5: return eg?PST_KING_END[idx]:PST_KING_MID[idx];
        }
        return 0;
    }

    int mobility(bool white) { return (int)gen(white).size(); }

    int king_safety(bool white) {
        u64 kb=pieces[white?5:10]; if (!kb) return 0;
        int sq=__builtin_ctzll(kb);
        u64 self=0; for (int i=1;i<=5;i++) self|=pieces[white?i:i+5];
        int def=0;
        for (int d:DIR_ALL){int f=sq+d;if(f>=0&&f<36&&no_wrap(sq,f)&&((self>>f)&1))def++;}
        return def*8-(in_check(white)?50:0);
    }

    int passed_pawns(bool white) {
        int bonus=0;
        int my_p=white?1:6, en_p=white?6:1;
        u64 mine=pieces[my_p], them=pieces[en_p];
        for (u64 tmp=mine;tmp;) {
            int sq=__builtin_ctzll(tmp); tmp&=tmp-1;
            int c=sq_col(sq), r=sq_row(sq); bool passed=true;
            for (u64 ep=them;ep;) {
                int esq=__builtin_ctzll(ep); ep&=ep-1;
                if (sq_col(esq)==c&&((white&&sq_row(esq)>r)||(!white&&sq_row(esq)<r)))
                { passed=false; break; }
            }
            if (passed) { int adv=white?r:(5-r); bonus+=15+adv*12; }
        }
        return bonus;
    }

    int evaluate() {
        if (!pieces[5])  return -KING_VAL;
        if (!pieces[10]) return  KING_VAL;
        int score=0;
        for (int i=0;i<36;i++){
            int p=at(i); if(!p) continue;
            int v=P_VAL[p]+pst_val(p,i);
            score+=(p<=5)?v:-v;
        }
        // Mobility removed from leaf eval (too slow in qsearch).
        // Approximated by piece activity in PSTs instead.
        score+=king_safety(true)-king_safety(false);
        score+=passed_pawns(true)-passed_pawns(false);
        if (__builtin_popcountll(pieces[3])>=2) score+=30;
        if (__builtin_popcountll(pieces[8])>=2) score-=30;
        return score;
    }

    // ── Move ordering ────────────────────────────────────────
    int mvv_lva(int att, int vic) const {
        return P_VAL[vic]*10 - P_VAL[(att<=5)?att:att-5];
    }

    void order(vector<Move>& mv, int ply, const Move& ttm) {
        for (auto& m:mv) {
            int vic=at(m.to);
            if (m.from==ttm.from&&m.to==ttm.to&&m.id==ttm.id) m.score=2000000;
            else if (vic)          m.score=1000000+mvv_lva(m.id,vic);
            else if (m.promote_to) m.score=950000;
            else if (ply<MAX_PLY&&killers[ply][0].from==m.from&&killers[ply][0].to==m.to)
                                   m.score=900000;
            else if (ply<MAX_PLY&&killers[ply][1].from==m.from&&killers[ply][1].to==m.to)
                                   m.score=800000;
            else                   m.score=history[m.from][m.to];
        }
    }

    // ── Time check ───────────────────────────────────────────
    void tick() {
        if ((++search_nodes&255)==0) {
            auto ms=chrono::duration_cast<chrono::milliseconds>(
                chrono::steady_clock::now()-t_start).count();
            if (ms>1000) stop_flag=true;
        }
    }

    // ── Quiescence ───────────────────────────────────────────
    int qsearch(int alpha, int beta, bool max_p) {
        if (!pieces[5])  return -KING_VAL;
        if (!pieces[10]) return  KING_VAL;
        int stand=evaluate();
        if (max_p){if(stand>=beta)return beta;if(stand>alpha)alpha=stand;}
        else      {if(stand<=alpha)return alpha;if(stand<beta)beta=stand;}

        auto raw=gen(max_p);
        vector<Move> caps; caps.reserve(16);
        for (auto& m:raw){
            int v=at(m.to);
            if (v){
                if (max_p&&stand+P_VAL[v]+200<alpha) continue;
                caps.push_back(Move(m.id,m.from,m.to,m.promote_to,mvv_lva(m.id,v)));
            }
        }
        sort(caps.begin(),caps.end(),greater<Move>());
        for (auto& m:caps){
            int cap=make(m);
            if (in_check(max_p)){unmake(m,cap);continue;}  // illegal — leaves own king in check
            int s=qsearch(alpha,beta,!max_p);
            unmake(m,cap);
            if (max_p){if(s>=beta)return beta;if(s>alpha)alpha=s;}
            else      {if(s<=alpha)return alpha;if(s<beta)beta=s;}
        }
        return max_p?alpha:beta;
    }

    // ── Alpha-beta ───────────────────────────────────────────
    int search(int depth, int alpha, int beta, bool max_p, int ply, bool null_ok) {
        // Repetition: cur_hash was just pushed at rep_table[rep_ply-1] by caller.
        // Check all prior entries [0..rep_ply-2] — includes game history AND search ancestors.
        // rep_table is always valid up to rep_ply-1 (write-guarded below).
        {
            int limit = min(rep_ply - 1, 511);
            for (int i = 0; i < limit; i++)
                if (rep_table[i] == cur_hash) return 0;
        }
        tick(); if (stop_flag) return evaluate();
        if (!pieces[5])  return -KING_VAL;
        if (!pieces[10]) return  KING_VAL;
        if (depth<=0)    return qsearch(alpha,beta,max_p);

        // TT probe
        int tidx=(int)(cur_hash&(TT_SIZE-1));
        TTEntry& tte=TT[tidx];
        Move ttm;
        if (tte.hash==cur_hash) {
            ttm=tte.best;
            if (tte.depth>=depth) {
                if (tte.flag==TT_EXACT)                    return tte.score;
                if (tte.flag==TT_LOWER&&tte.score>alpha)   alpha=tte.score;
                if (tte.flag==TT_UPPER&&tte.score<beta)    beta=tte.score;
                if (alpha>=beta)                            return tte.score;
            }
        }

        // Null-move pruning
        // Null move: push the flipped hash so the child's rep check sees it,
        // but with null_ok=false so it cannot chain null moves.
        if (null_ok&&depth>=3&&!in_check(max_p)&&!is_endgame()) {
            cur_hash^=Z_side;
            bool np = (rep_ply < 255);
            if (np) rep_table[rep_ply++] = cur_hash;
            int ns=search(depth-3,alpha,beta,!max_p,ply+1,false);
            if (np) rep_ply--;
            cur_hash^=Z_side;
            if (!stop_flag) {
                if ( max_p&&ns>=beta)  return beta;
                if (!max_p&&ns<=alpha) return alpha;
            }
        }

        auto raw=gen(max_p);
        if (raw.empty()) {
            return in_check(max_p) ? (max_p ? -(KING_VAL-ply) : (KING_VAL-ply)) : 0;
        }
        order(raw,ply,ttm);
        sort(raw.begin(),raw.end(),greater<Move>());

        int best=max_p?-INF:INF, orig_a=alpha;
        Move best_move; bool found=false;

        for (auto& m:raw) {
            int cap=make(m);
            if (in_check(max_p)){unmake(m,cap);continue;}
            found=true;
            bool pushed = (rep_ply < 255);
            if (pushed) rep_table[rep_ply++] = cur_hash;
            int s=search(depth-1,alpha,beta,!max_p,ply+1,true);
            if (pushed) rep_ply--;
            unmake(m,cap);
            if (stop_flag) break;

            if (max_p){if(s>best){best=s;best_move=m;}if(s>alpha)alpha=s;}
            else      {if(s<best){best=s;best_move=m;}if(s<beta)beta=s;}

            if (beta<=alpha) {
                // Check quiet move BEFORE unmake — at(m.to) is valid here since
                // unmake hasn't been called yet at this point in the flow.
                // Actually unmake was called above; use captured piece 'cap' instead.
                if (cap==0&&m.promote_to==0&&ply<MAX_PLY) {
                    killers[ply][1]=killers[ply][0]; killers[ply][0]=m;
                    history[m.from][m.to]+=depth*depth;
                }
                break;
            }
        }
        if (!found) {
            return in_check(max_p) ? (max_p ? -(KING_VAL-ply) : (KING_VAL-ply)) : 0;
        }

        if (!stop_flag&&best_move.id) {
            TTFlag flag=(best<=orig_a)?TT_UPPER:(best>=beta)?TT_LOWER:TT_EXACT;
            tte={cur_hash,depth,best,flag,best_move};
        }
        return best;
    }

    // ── Root ─────────────────────────────────────────────────
    Move root(bool white) {
        auto raw=gen(white);

        // Instant king capture
        for (auto& m:raw) if (at(m.to)==(white?10:5)) return m;

        // Legal filter
        vector<Move> legal; legal.reserve(raw.size());
        for (auto& m:raw){int c=make(m);bool ok=!in_check(white);unmake(m,c);if(ok)legal.push_back(m);}
        if (legal.empty()) return raw.empty()?Move(0,0,0):raw[0];

        Move best=legal[0];
        for (int d=1;d<=14;d++) {
            if (stop_flag) break;
            int a=-INF, bv=INF, bval=white?-INF:INF;
            Move iter=best;
            for (auto& m:legal) {
                int c=make(m);
                if (rep_ply < 511) rep_table[rep_ply] = cur_hash;
                rep_ply++;
                int s=search(d-1,a,bv,!white,1,true);
                rep_ply--; unmake(m,c);
                if (stop_flag) break;
                if (white &&s>bval){bval=s;iter=m;a=max(a,s);}
                if (!white&&s<bval){bval=s;iter=m;bv=min(bv,s);}
            }
            if (!stop_flag) best=iter;
        }
        return best;
    }
};

// ─── DLL export + game reset ─────────────────────────────────
#ifdef _WIN32
  #define EXPORT extern "C" __declspec(dllexport)
#else
  #define EXPORT extern "C" __attribute__((visibility("default")))
#endif

static ApexEngine engine;
static char       out_buf[128];

// Call between games to clear position history
EXPORT void reset_game() {
    engine.game_ply = 0;
    engine.rep_ply  = 0;
}

EXPORT const char* get_best_move(int* board, bool is_white) {
    engine.load(board, is_white);
    engine.t_start      = chrono::steady_clock::now();
    engine.stop_flag    = false;
    engine.search_nodes = 0;
    memset(engine.history, 0, sizeof(engine.history));
    memset(engine.killers, 0, sizeof(engine.killers));

    // Persist game history: add current position to game_ply table.
    // On first call game_ply==0; each call appends the position BEFORE our move.
    // This lets the search detect repetitions that span multiple game moves.
    if (engine.game_ply < 500) {
        engine.rep_table[engine.game_ply] = engine.cur_hash;
    }
    // Reset search stack to sit on top of game history
    engine.rep_ply = engine.game_ply + 1;

    Move m = engine.root(is_white);

    // Record the position AFTER our move into game history for future calls
    // (opponent will call get_best_move next; we record our resulting position)
    // The tournament applies our move to the board — advance game_ply by 1
    if (engine.game_ply < 500) engine.game_ply++;

    string res = to_string(m.id)+":"+sq2s(m.from)+"->"+sq2s(m.to);
    if (m.promote_to) res += "="+to_string(m.promote_to);
    strncpy(out_buf, res.c_str(), sizeof(out_buf)-1);
    out_buf[sizeof(out_buf)-1] = '\0';
    return out_buf;
}