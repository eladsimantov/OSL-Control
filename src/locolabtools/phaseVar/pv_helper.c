/*
 * pv_helper.c - 32-bit (armhf) bridge process for LocolabPhaseVariable.so
 *
 * Why: the LocoLab library is compiled for 32-bit ARM, but the Pi runs a 64-bit
 * Python, which cannot load it. This tiny program links the .so and serves it
 * over stdin/stdout, so 64-bit Python can use it through a pipe (see phase_variable.py).
 *
 * Binary protocol (little-endian, native doubles):
 *   'S' + 6 doubles (thighAngle_deg, thighVelocity_dps, Fz, time, incline, speed)
 *        -> reply 4 doubles (phase, stancePhase, swingPhase, state)
 *   'R' -> terminate + initialize (fresh state)   -> reply 'r'
 *   'P' -> ping                                   -> reply 'p'
 *   'Q' or EOF -> terminate and exit
 *
 * Build (on the Pi or any Linux PC):
 *   arm-linux-gnueabihf-gcc -O2 -o pv_helper pv_helper.c -L. -l:LocolabPhaseVariable.so \
 *       -Wl,-rpath,'$ORIGIN' -Wl,-z,max-page-size=0x10000 -Wl,-z,common-page-size=0x10000
 *   (the 64K alignment is required on 16K-page kernels, e.g. the Pi 5 default kernel)
 */
#include <unistd.h>

typedef struct { double thighAngle_deg, thighVelocity_dps, Fz, time, incline, speed; } In;
typedef struct { double phase, stancePhase, swingPhase, state; } Out;

void LocolabPhaseVariable_initialize(void);
void LocolabPhaseVariable_terminate(void);
void LocolabPhaseVariable(In *in, Out *out);

static int read_all(void *buf, size_t n) {
    char *p = (char *)buf;
    while (n > 0) {
        ssize_t r = read(0, p, n);
        if (r <= 0) return 0;
        p += r; n -= (size_t)r;
    }
    return 1;
}

static int write_all(const void *buf, size_t n) {
    const char *p = (const char *)buf;
    while (n > 0) {
        ssize_t w = write(1, p, n);
        if (w <= 0) return 0;
        p += w; n -= (size_t)w;
    }
    return 1;
}

int main(void) {
    In in; Out out; char op, ack;
    LocolabPhaseVariable_initialize();
    while (read_all(&op, 1)) {
        if (op == 'S') {
            if (!read_all(&in, sizeof in)) break;
            LocolabPhaseVariable(&in, &out);
            if (!write_all(&out, sizeof out)) break;
        } else if (op == 'R') {
            LocolabPhaseVariable_terminate();
            LocolabPhaseVariable_initialize();
            ack = 'r'; if (!write_all(&ack, 1)) break;
        } else if (op == 'P') {
            ack = 'p'; if (!write_all(&ack, 1)) break;
        } else if (op == 'Q') {
            break;
        }
    }
    LocolabPhaseVariable_terminate();
    return 0;
}
