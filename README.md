# NTTU Phase 1 Verilog

Phase 1 NTT/SoF Verilog RTL, ROM vectors, and ModelSim testbenches.

## Single-on-fly regression

The primary regression testbench is `tb_single_on_fly16_general.v`.

- Input vectors: `wrap/sof_in_general.hex`
- Golden vectors: `wrap/sof_exp_wrap_jlane_rot0_lr0.hex`
- Coverage: 4 x 4 tiles x 16 rows = 256 frames
- PASS criterion: every `out_valid` 960-bit output (16 lanes x 60 bits) is
  exactly equal to the matching golden-vector line.

Run from a ModelSim-compatible ASCII-only path.  ModelSim 2020.1 can fail to
create its library database when the project parent path contains Korean text.

```powershell
vlib sim_work_single
vmap sim_work_single sim_work_single
vlog -work sim_work_single modules.v barrett_mult.v shoup_mult.v single_on_fly.v twiddle16_rom_multibank.v single_twist_rom.v q_const_rom_bank.v single_on_fly_wrap.v tb_single_on_fly16_general.v
vsim sim_work_single.tb_single_on_fly_general
```

Then in the ModelSim Transcript pane:

```tcl
add wave -r /*
run -all
```

Expected completion message:

```text
[PASS] single_on_fly_wrap general (4x4 tiles)
```

## 2026-09-17 changes

- Corrected the default optional stimulus path in `tb_phase1_256pe_gen.v` to
  `ntt16_data/ntt16_out/level0/ntt16_stim.hex`.
- Moved DC testbench output paths to `phase1_result/dc_*.hex`, avoiding the
  missing `phase1_vector` directory and preserving general-test results.
- Removed a testbench sampling race by changing reset/input driving in
  `tb_single_on_fly16_general.v` to the falling clock edge.
- Added `TILES_PER_ROW` to `single_on_fly_wrap`; the 4 x 4 TB sets it to 4 so
  tile progression is `(0,3) -> (1,0)`, matching its golden vectors.
- Fixed Barrett reduction in `barrett_mult.v` to use the full `T * mu` product
  and a non-truncating `2 * q_width` shift count.
- Fixed the Shoup quotient estimate in `shoup_mult.v` to take the high half of
  the full-width `a * z_pre` product.
- Verified the single-on-fly regression in ModelSim: compile completed with
  zero errors/warnings and all 256 frames passed at 2695 ns.

`rot0_lr0` is the active golden-vector convention. `rot0_lr1` and `rot1_lr0`
are alternate lane-order and rotation conventions, respectively; they are not
expected to pass with this TB configuration.
