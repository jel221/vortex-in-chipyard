   cd /home/richard/chipyard/sims/verilator
 make run-binary-debug CONFIG=VortexRocketConfig \
    BINARY=../../tests/build/vortex-vadd.riscv \
    SIM_FLAGS="+dump-start=2500000 +max-cycles=4500000" 
