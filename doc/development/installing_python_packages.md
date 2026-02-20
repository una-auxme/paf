# Install Python packages

**Summary:** Python dependencies are managed from repository manifests and synchronized inside the running development container.

## Choose the correct requirements file

- `code/requirements.txt`: shared runtime dependencies.
- `code/requirements.cpu.txt`, `code/requirements.cuda.txt`, `code/requirements.rocm.txt`: hardware-specific runtime dependencies.
- `code/requirements_infrastructure.txt`: developer tooling (`ruff`, `pytest`, etc.).

Every dependency must be pinned with `==`.

## Add a dependency safely

1. Edit the matching `requirements*.txt` file.
2. Open a shell in the `agent-dev` container.
3. Run:

```bash
dep.sync
devbuild
```

4. Validate with:

```bash
python3 -m pip check
ruff check /workspace/code/
pytest -m unit
```

## Important container note

Avoid ad-hoc `pip install <package>` as a long-term fix. It only mutates your current container and is lost for other developers/CI unless the dependency is committed to `requirements*.txt`.

For the full ROS + Python process (including `package.xml` and rosdep), see [Dependency Management](./dependency_management.md).
