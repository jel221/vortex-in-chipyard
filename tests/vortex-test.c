// vortex-test.c
//
// Minimal host-side test for the VortexRoCC accelerator on VortexRocketConfig.
//
// The RoCC command (CUSTOM0, funct=0):
//   rs1 = startup_addr  -- kernel entry point written to DCR 0x001
//   rs2 = startup_arg   -- argument passed to kernel, written to DCR 0x003
//
// Rocket holds the pipeline (io.busy) until the GPU finishes, so no explicit
// polling is needed; the return from vortex_launch() is synchronised.
//
// Build (from chipyard/tests/build/):
//   cmake -S .. -B . -DCMAKE_BUILD_TYPE=Release
//   cmake --build . --target vortex-test
//
// Run (from chipyard/sims/verilator/ or sims/vcs/):
//   make run-binary BINARY=../../tests/build/vortex-test.riscv \
//       CONFIG=VortexRocketConfig

#include <stdint.h>
#include "rocc.h"
#include <stdio.h>

// ---------------------------------------------------------------------------
// Tiny kernel placed in its own section so we can take its address.
//
// In a real flow this would be a separate Vortex GPGPU ELF loaded via DMA.
// Here we embed a trivial kernel that:
//   1. Reads the argument passed through DCR 0x003 (reflected as rs2).
//   2. Writes a sentinel value to a shared memory location so the host can
//      verify the GPU ran.
//
// NOTE: This stub is RISC-V host code, not real Vortex GPU ISA.  It serves
//       only to exercise the RoCC interface (DCR writes, reset release, busy
//       protocol) during RTL simulation.  The GPU will start executing from
//       kernel_entry once gpuReset is de-asserted.
// ---------------------------------------------------------------------------

// Shared result buffer (in DRAM, accessible by both host and GPU)
static volatile uint32_t result_buf[4] __attribute__((aligned(64))) = {
    0xCCCCCCCC, 0xCCCCCCCC, 0xCCCCCCCC, 0xCCCCCCCC
};

// Sentinel the kernel is expected to write
#define KERNEL_SENTINEL  0xDEADBEEFu


static inline void vx_tmc(int thread_mask) {                                                                            
  __asm__ volatile (".insn r 0x0B, 0, 0, x0, %0, x0" :: "r"(thread_mask));                     
}

// ---------------------------------------------------------------------------
// Kernel stub — would normally be Vortex GPU code loaded from an ELF.
// For simulation smoke-testing the RoCC interface we just need the address
// to be non-zero and valid so the DCR write goes through.  Real kernels
// require the full Vortex toolchain and driver.
// ---------------------------------------------------------------------------
__attribute__((noinline, section(".text.kernel")))
static void kernel_entry(uint32_t *buf)
{
    result_buf[0] = KERNEL_SENTINEL;
    asm volatile ("fence" ::: "memory");
    vx_tmc(0);
}

// ---------------------------------------------------------------------------
// Host helpers
// ---------------------------------------------------------------------------

// Issue the single Vortex RoCC launch command.
// Rocket stalls here (io.busy) until the GPU signals completion.
static inline void vortex_launch(uintptr_t startup_addr, uintptr_t startup_arg)
{
    // Ensure all preceding stores are visible before the GPU starts.
    asm volatile ("fence" ::: "memory");
    printf("kernel pointer: %p \n", startup_addr);
    printf("argument pointer: %p \n", startup_arg);
    ROCC_INSTRUCTION_SS(0, startup_addr, startup_arg, 0);
    // Rocket re-checks busy before retiring the next instruction, so no
    // explicit fence is needed after the RoCC instruction itself.
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(void)
{
    // Pass the kernel entry point as startup_addr and the buffer pointer as
    // startup_arg.  The VortexRoCC state machine writes these to
    // DCR 0x001 (STARTUP_ADDR0) and DCR 0x003 (STARTUP_ARG0) respectively,
    // then releases the GPU from reset.
    vortex_launch((uintptr_t)kernel_entry, (uintptr_t)result_buf);
    

    // GPU has finished (Rocket was stalled until busy de-asserted).
    // Verify the sentinel.
    if (result_buf[0] != KERNEL_SENTINEL) {
        printf("not good \n");
        return 1;  // FAIL
    }

    return 0;  // PASS
}
