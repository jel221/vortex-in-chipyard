   cd /home/richard/chipyard/sims/verilator
 make run-binary-debug CONFIG=VortexRocketConfig \
    BINARY=../../tests/build/vortex-test.riscv \
    SIM_FLAGS="+dump-start=900000 +dump-end=1200000 +max-cycles=1200000" 
