# PlaCo source provenance

This directory is a regular, vendored source snapshot. It is intentionally not
a Git submodule and contains no nested `.git` metadata.

- Upstream: `https://github.com/Rhoban/placo.git`
- Upstream tag: `v0.9.23`
- Upstream commit: `e6c288604639d67b979a16cb2ad26913413c8e3a`
- Imported with: `git archive`

The upstream `.gitattributes` excludes `docs/`, `python/tests/`, and
`python/examples/` from source archives. The C++ library, public headers,
Python bindings, utilities, build files, and license are present.

## Motion Control Lab changes

The imported source contains these deliberate integration changes:

1. `PLACO_BUILD_PYTHON_BINDINGS` allows the lab to build only the C++ core.
2. The Eigen lookup does not constrain the major version.
3. Pinocchio `Frame::parentJoint` replaces the removed `Frame::parent` API.
4. Eigen 5's `Eigen::placeholders::all` replaces `Eigen::all`.
5. `RobotWrapper::IGNORE_GEOMETRY` allows model-only IK to skip unavailable
   mesh resources without changing the existing `IGNORE_COLLISIONS` behavior;
   the flag is also exposed by the Python enum binding.

These changes are committed as normal repository files, so further PlaCo
experiments can edit `third_party/placo/` directly and review the resulting
diff together with the experiment definition.
