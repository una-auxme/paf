# Publishing CARLA images to GHCR (GitHub Container Registry)

> [!NOTE] Pushing to GHCR is implemented in `build/docker/carla/build_carla_buildx.sh`

Purpose

- This guide shows where to publish prebuilt CARLA images for the `paf` repository and how to automate builds/pushes to GHCR so developers can pull images instead of building the large CARLA runtime locally.

Recommended location for images

- Use the repository/organisation namespace: `ghcr.io/una-auxme/<image-name>:<tag>`.
- Example image names we recommend for clarity:
  - `ghcr.io/una-auxme/carla-leaderboard-cuda:2.1`
  - `ghcr.io/una-auxme/carla-leaderboard-gpu:2.1`
  - `ghcr.io/una-auxme/carla-leaderboard-api:2.1`

Why GHCR?

- GHCR integrates nicely with GitHub Actions and allows fine-grained package permissions. Using GHCR keeps images close to the source and simplifies CI credentials.

## Authentication and permissions

- For pushing from GitHub Actions you can use the automatically provided `GITHUB_TOKEN`. Grant the workflow the `packages: write` permission (example below). The `GITHUB_TOKEN` is scoped to the repository; for org-wide pushes you may need a PAT with `write:packages`.
- For manual pushes from a developer machine, create a personal access token (PAT) with at least `write:packages` and `read:packages`, then login locally:

### Using `gh` as a token provider

#### `gh` login

```bash
gh auth login
```

#### docker login

```bash
gh auth token | docker login ghcr.io -u <github-username> --password-stdin
```

## Push

```bash
echo "${GHCR_PAT}" | docker login ghcr.io -u <github-username> --password-stdin
docker tag carla-leaderboard-cuda:2.1 ghcr.io/una-auxme/carla-leaderboard-cuda:2.1
docker push ghcr.io/una-auxme/carla-leaderboard-cuda:2.1
```

GitHub Actions example (push on tag / manual dispatch)

- Create `.github/workflows/publish-carla-ghcr.yml` with the following contents (adjust names and tags to your needs):

```yaml
name: Publish CARLA images to GHCR

on:
  workflow_dispatch: {}
  push:
    tags: ['v*']

permissions:
  contents: read
  packages: write

jobs:
  build-and-push:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Set up QEMU
        uses: docker/setup-qemu-action@v2
      - name: Set up Docker Buildx
        uses: docker/setup-buildx-action@v3
      - name: Login to GHCR
        uses: docker/login-action@v2
        with:
          registry: ghcr.io
          username: ${{ github.actor }}
          password: ${{ secrets.GITHUB_TOKEN }}
      - name: Build and push CARLA images
        uses: docker/build-push-action@v5
        with:
          context: ./build/docker/carla
          file: ./build/docker/carla/Dockerfile
          push: true
          tags: |
            ghcr.io/una-auxme/carla-leaderboard-gpu:2.1
            ghcr.io/una-auxme/carla-leaderboard-cuda:2.1
            ghcr.io/una-auxme/carla-leaderboard-api:2.1
          # build-args can be used to set BASE_FLAVOUR for the cuda image if needed
          build-args: |
            BASE_FLAVOUR=cuda
          cache-from: type=gha
          cache-to: type=gha,mode=max
```

Notes on the workflow

- The `docker/build-push-action` will run buildx for you. When pushing multiple tags from the same Dockerfile it's best to use buildx and cache export to speed up repeated runs.
- If you need to push images under an organisation account rather than the repository owner, either run the workflow from that organisation repo or use a PAT stored in `secrets.GHCR_PAT` and replace the `docker/login-action` step to use that secret.

Make the package public

- By default GHCR packages are private. After pushing the image, go to the package page in GitHub (Repository > Packages or https://github.com/orgs/una-auxme/packages) and set visibility to **Public** if you want everyone to be able to pull without authentication.

Using the images in `docker-compose`

- Edit your compose files to point to the registry images:

```yaml
services:
  carla-simulator:
    image: ghcr.io/una-auxme/carla-leaderboard-cuda:2.1
    # ...other settings...
```

- Alternatively provide a small override file `docker-compose.override.registry.yml` that only replaces local image names with registry names; this lets developers choose between local builds and pulling prebuilt images.

Tagging strategy & maintenance

- Use semantic tags (`2.1`, `2.1-rc1`, `latest`) and consider adding digest pins for reproducibility in CI.
- Clean up old packages or set a retention policy if your registry usage is heavy.

Where to put this doc

- I've created this guidance in `doc/development/push-to-ghcr.md` because publishing is part of the development/CI process. If you'd prefer a more user-facing note (installation/usage), move a short summary to `doc/general/README.md`.

If you want I can:

- Create the GitHub Actions workflow file under `.github/workflows/publish-carla-ghcr.yml` (you'll only need to adjust org/repo and secrets if required).
- Add a `docker-compose.override.registry.yml` so developers can pull the prebuilt images easily.
