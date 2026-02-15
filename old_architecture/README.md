# Old Monolithic Architecture (Archived)

This directory contains the original monolithic simulation (`simulation_main.py`)
and the backward-compatible shim files (`core_shims/`) that let it import from
`core.X` instead of the processor-specific `core.stm32*/` paths.

**The active codebase uses the 3-processor architecture.**
See the main README in the project root.

## Running the old version

The old simulation may still work if you restore the shims to `core/`:

```bash
cp core_shims/*.py ../core/
python3 simulation_main.py
```

## Files

- `simulation_main.py` — Original monolithic simulation entry point
- `core_shims/` — 10 re-export shim files (e.g., `drone.py` re-exports from `core.stm32h7.drone`)
