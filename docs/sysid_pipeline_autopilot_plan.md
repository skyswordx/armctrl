# SysId Pipeline Autopilot Plan

Goal: implement the next four system-identification upgrades in a staged, testable way:
base-parameter reduction, real-regressor-aware Fourier optimization, friction/bias augmented solving, and zero-phase offline filtering.

## Architecture

- Keep Pinocchio as the deterministic local regressor builder because it is already integrated.
- Prefer FIGAROH when importable for QR/base-parameter utilities, but provide a NumPy/SVD fallback so Windows tests do not require Linux-only packages.
- Treat `gravity_only`, `full_base`, and `full_augmented` as separate solver runs with explicit parameter subset metadata.
- Treat filtering as postprocess metadata and deterministic CSV generation, not as hidden preprocessing.

## Tasks

1. Add base-parameter reduction:
   - Modify `src/armctrl/identification/solver.py`.
   - Add a helper that uses `figaroh.tools.qrdecomposition.QRDecomposer` when importable.
   - Fallback to SVD independent-column selection over standardized columns.
   - Replace the full 60-column LS solve with a base subset for full dynamics.
   - Keep original full regressor metrics for diagnostics.

2. Add augmented solver:
   - Extend solver to append per-joint bias, viscous, and Coulomb-like columns.
   - Report rigid-body column count and augmented column count separately.
   - Mark physical consistency as not applicable for augmented parameter vectors.

3. Add zero-phase filtering:
   - Modify `postprocess_dataset()` and CLI postprocess args.
   - Support `--filter-mode moving_average|zero_phase`.
   - Implement dependency-free symmetric FIR filtering as zero-phase fallback.
   - Record filter mode, window, and derivative source in quality metrics and processed CSV semantics.

4. Add real-regressor-aware Fourier optimization hook:
   - Extend `optimize_fourier_multisine()` to accept an optional scorer callable.
   - Use existing surrogate scorer by default.
   - Add a Pinocchio scorer entry point that can be used when URDF and Pinocchio are available.
   - Make metadata clearly say whether optimization used `pinocchio_regressor_condition` or `surrogate_feature_condition`.

5. QA and delivery:
   - Add unit tests before each behavior change.
   - Run `uv run pytest tests/unit/test_identification.py -q`.
   - Commit only code/test/plan files for this task, leaving pre-existing dirty docs untouched.
