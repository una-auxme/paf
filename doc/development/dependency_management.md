# Dependency Management (Container-first)

This project is developed inside the `agent-dev` container. Dependency changes must be reproducible from repository files and must not depend on ad-hoc `pip install` in a single shell.

## Source of truth

- ROS/apt dependencies: `code/**/package.xml` resolved via `rosdep`.
- Python runtime dependencies: `code/requirements.txt` and flavor-specific files (`requirements.cpu.txt`, `requirements.cuda.txt`, `requirements.rocm.txt`).
- Python dev/tooling dependencies: `code/requirements_infrastructure.txt`.
- Ruff tool version pin: `build/pins/ruff.env`.

## Rules

1. Do not install long-term dependencies manually with `pip install <pkg>` in the container shell.
2. Every added Python dependency must be pinned with `==` in the matching requirements file.
3. Every ROS package dependency must be declared in the relevant `package.xml`.
4. After dependency changes, run `dep.sync` and `devbuild` in the container.

## Standard workflow

### 1) Modify dependency manifests

- Python: update `requirements*.txt` with pinned versions.
- ROS: update `package.xml` (`depend`, `exec_depend`, `test_depend` as applicable).

### 2) Synchronize in container

In a shell inside the running `agent-dev` container:

```bash
dep.check        # optional pre-check
dep.sync         # installs rosdep + pip dependencies from repo files
devbuild         # rebuild workspace after dependency changes
```

`dep.sync` internally runs:

- `rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y -r`
- `install-python-requirements.sh` (all `requirements*.txt` + `requirements_infrastructure.txt`)
- `python3 -m pip check`

### 3) Validate

```bash
ruff check /workspace/code/
ruff format /workspace/code/ --check
pytest -m unit
```

## When container rebuild is required

Use `Dev Containers: Rebuild Container` when you change image-level inputs such as:

- `build/docker/agent-ros2/Dockerfile`
- `build/pins/*.env`
- scripts copied into the image under `build/docker/agent-ros2/scripts/`

For regular `package.xml` / `requirements*.txt` updates, an image rebuild is usually not needed; run `dep.sync` + `devbuild` in the existing container.

## Troubleshooting

- Build-time failures:
  - `/internal_workspace/rosdep_install.log`
  - `/internal_workspace/pip_install.log`
- Runtime dependency drift:
  - run `dep.check`
  - run `python3 -m pip check`
