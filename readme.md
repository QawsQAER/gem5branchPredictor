This project implement the YAGS and gshare branch predictor for gem5 simulator

please put the yags.cc, yags.hh, gshare.cc, gshare.hh under [your gem5 folder]/src/cpu/pred/

To enable set associativity in yags, change the _SET_ACCOCITY in yags.hh to either 2, 4 or 8.

Then recompile the source code.
