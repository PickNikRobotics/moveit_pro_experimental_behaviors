# Repository behavior index

**This offline page combines the port indexes of all independent Behavior packages in this branch.**

Open [index.html](index.html) in a browser. Each package also carries its own offline page, so copying one
package keeps its port documentation.

## Update

1. Follow `docs/behavior_index/README.md` in each changed package to regenerate its data from C++ registrations.
2. Run `python3 tools/update_behavior_index.py` from the repository root to combine the data.
3. Run `python3 tools/update_behavior_index.py --check` or the root unit tests to verify the combined index.

The combiner rejects duplicate Behavior IDs. Each package's C++ test checks its data against the actual
registered IDs, metadata and ports. No package depends on the combiner or on another package's index.
