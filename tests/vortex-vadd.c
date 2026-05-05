// vortex-vadd.c
//
// Vector-addition test for the VortexRoCC accelerator.
//
// The GPU kernel reads two input arrays (A and B) and writes their element-wise
// sum to a third array (C).  The host verifies the results after the GPU finishes.
//
// Build (from chipyard/tests/build/):
//   cmake -S .. -B . -DCMAKE_BUILD_TYPE=Release
//   cmake --build . --target vortex-vadd
//
// Run (from chipyard/sims/verilator/ or sims/vcs/):
//   make run-binary BINARY=../../tests/build/vortex-vadd.riscv \
//       CONFIG=VortexRocketConfig

#include <stdint.h>
#include <stdio.h>
#include "rocc.h"

// ---------------------------------------------------------------------------
// Shared data buffers (cache-line aligned so GPU cache accesses are clean)
// ---------------------------------------------------------------------------

#define N 32

static volatile uint32_t array_a[N] __attribute__((aligned(64)));
static volatile uint32_t array_b[N] __attribute__((aligned(64)));
static volatile uint32_t array_c[N] __attribute__((aligned(64)));

// Argument bundle passed to the kernel via the startup_arg pointer
typedef struct {
    uint32_t *a;
    uint32_t *b;
    uint32_t *c;
    uint32_t  n;
} VaddArgs;

static volatile VaddArgs vadd_args __attribute__((aligned(64)));

// ---------------------------------------------------------------------------
// GPU types and intrinsics
// ---------------------------------------------------------------------------

typedef union {
    struct { uint32_t x, y, z; };
    uint32_t m[3];
} dim3_t;

__thread dim3_t blockIdx;
__thread dim3_t threadIdx;
dim3_t blockDim;
dim3_t gridDim;

static inline void vx_tmc(int thread_mask) {
    __asm__ volatile (".insn r 0x0B, 0, 0, x0, %0, x0" :: "r"(thread_mask));
}

static inline void vx_wspawn(int num_warps, void *func_ptr) {
    __asm__ volatile (".insn r 0x0B, 1, 0, x0, %0, %1" :: "r"(num_warps), "r"(func_ptr));
}

static inline int vx_warp_id(void) {
    int r; __asm__ volatile ("csrr %0, 0xCC1" : "=r"(r)); return r;
}

static inline int vx_num_warps(void) {
    int r; __asm__ volatile ("csrr %0, 0xFC1" : "=r"(r)); return r;
}

// ---------------------------------------------------------------------------
// GPU register / TLS initialisation — mirrors vx_start.S
// ---------------------------------------------------------------------------

#define _XSTR(x) #x
#define XSTR(x)  _XSTR(x)

#define TLS_BLOCK_SIZE  32      // must cover blockIdx + threadIdx (2 * 12 bytes, round up)
#define STACK_SIZE      2048
#define NUM_HARTS_MAX   16      // 4 warps * 4 threads = 16 for default Vortex config

static char tls_pool  [NUM_HARTS_MAX * TLS_BLOCK_SIZE] __attribute__((aligned(16), used));
static char stack_pool[NUM_HARTS_MAX * STACK_SIZE]     __attribute__((aligned(16), used));

// Set sp and tp for the calling thread from mhartid; no stack frame (naked).
__attribute__((naked, used))
static void init_regs(void) {
    __asm__ (
        "csrr t0, 0xF14\n"
        "la   t1, stack_pool\n"
        "addi t2, t0, 1\n"
        "li   t3, " XSTR(STACK_SIZE) "\n"
        "mul  t2, t2, t3\n"
        "add  sp, t1, t2\n"     // sp = stack_pool + (hart_id+1) * STACK_SIZE
        "la   t1, tls_pool\n"
        "li   t3, " XSTR(TLS_BLOCK_SIZE) "\n"
        "mul  t2, t0, t3\n"
        "add  tp, t1, t2\n"     // tp = tls_pool + hart_id * TLS_BLOCK_SIZE
        "ret\n"
        
    );
}

// Spawned-warp stub: enable all threads, init each one, then exit.
__attribute__((naked, used))
static void init_regs_all(void) {
    __asm__ (
        "li   t0, -1\n"
        ".insn r 0x0B, 0, 0, x0, t0, x0\n"  // tmc(-1): activate all threads
        "call init_regs\n"                    // every thread sets its sp/tp
        ".insn r 0x0B, 0, 0, x0, x0, x0\n"  // tmc(0):  exit warp
        "ret\n"
    );
}

// ---------------------------------------------------------------------------
// Per-element kernel — blockIdx.x selects the element
// ---------------------------------------------------------------------------

static void kernel_vadd(VaddArgs *args)
{
    uint32_t i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < args->n)
        args->c[i] = args->a[i] + args->b[i];
}

// ---------------------------------------------------------------------------
// Warp stub — each spawned warp strides through tasks by warp ID
// ---------------------------------------------------------------------------

typedef struct {
    VaddArgs *args;
    uint32_t  num_tasks;
    uint32_t  num_warps;
} WarpSpawnArgs;

static WarpSpawnArgs g_warp_args;

__attribute__((noinline))
static void warp_stub(void) {
    WarpSpawnArgs *wa = &g_warp_args;
    uint32_t wid = (uint32_t)vx_warp_id();
    for (uint32_t i = wid; i < wa->num_tasks; i += wa->num_warps) {
        blockIdx.x  = i;
        blockDim.x  = 1;
        threadIdx.x = 0;
        kernel_vadd(wa->args);
    }
    vx_tmc(0);
}

// ---------------------------------------------------------------------------
// GPU entry point — sets up spawn args, activates all warps, warp 0 works too
// ---------------------------------------------------------------------------

__attribute__((noinline, used, section(".text.kernel")))
static void gpu_entry(VaddArgs *_unused)
{
    args = vadd_args;
    uint32_t nw = (uint32_t)vx_num_warps();
    g_warp_args.args      = args;
    g_warp_args.num_tasks = args->n;
    g_warp_args.num_warps = nw;
    gridDim.x  = args->n;
    blockDim.x = 1;
    vx_wspawn(nw, warp_stub);
    warp_stub();  // warp 0 participates as a worker
}

// ---------------------------------------------------------------------------
// GPU startup — mirrors _start in vx_start.S
//
// vortex_launch points here instead of gpu_entry.  This runs before any C
// runtime, so sp/tp are undefined; every function must be naked until
// init_regs has been called for the current thread.
// ---------------------------------------------------------------------------

__attribute__((naked, section(".text.kernel")))
static void vx_setup(void) {
    __asm__ (
        // Init warp 0 thread 0 first so we have a valid sp.
        "call  init_regs\n"
        // Spawn remaining warps to init_regs_all (each enables all threads,
        // runs init_regs for every thread, then dies via tmc(0)).
        "csrr  t0, 0xFC1\n"                         // num_warps
        "la    t1, init_regs_all\n"
        ".insn r 0x0B, 1, 0, x0, t0, t1\n"          // wspawn(nw, init_regs_all)
        // Enable all threads of warp 0 and initialise each one.
        "li    t0, -1\n"
        ".insn r 0x0B, 0, 0, x0, t0, x0\n"          // tmc(-1)
        "call  init_regs\n"
        "li    t0, 1\n"
        ".insn r 0x0B, 0, 0, x0, t0, x0\n"          // tmc(1): back to single thread
        // Pass &vadd_args explicitly because startup_arg is not forwarded by
        // VortexRoCC hardware (rs2 is unused).
        "la    a0, vadd_args\n"
        "call  gpu_entry\n"
        // gpu_entry returns after all warps finish; exit warp 0.
        ".insn r 0x0B, 0, 0, x0, x0, x0\n"          // tmc(0)
    );
}

// ---------------------------------------------------------------------------
// Host helper
// ---------------------------------------------------------------------------

static inline void vortex_launch(uintptr_t startup_addr, uintptr_t startup_arg)
{
    asm volatile (".insn i 0x73, 0, x0, %0, -64" :: "r"(startup_arg) : "memory");
    printf("kernel pointer: %p\n", (void *)startup_addr);
    printf("argument pointer: %p\n", (void *)startup_arg);
    ROCC_INSTRUCTION_SS(0, startup_addr, startup_arg, 0);
    asm volatile ("fence" ::: "memory");
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(void)
{
    // Initialise input arrays
    for (int i = 0; i < N; i++) {
        array_a[i] = (uint32_t)(i + 1);
        array_b[i] = (uint32_t)(i * 2);
        array_c[i] = 0;
    }

    vadd_args.a = (uint32_t *)array_a;
    vadd_args.b = (uint32_t *)array_b;
    vadd_args.c = (uint32_t *)array_c;
    vadd_args.n = N;

    printf("Starting vortex \n");
    vortex_launch((uintptr_t)vx_setup, (uintptr_t)&vadd_args);

    // Verify results
    for (int i = 0; i < N; i++) {
        uint32_t expected = array_a[i] + array_b[i];
        if (array_c[i] != expected) {
            printf("FAIL at index %d: got %u, expected %u\n",
                   i, (unsigned)array_c[i], (unsigned)expected);
            return 1;
        }
    }

    printf("PASS: all %d sums correct\n", N);
    return 0;
}
