# GitHub actions

(Kept from previous group [paf22])

**Summary:** This page explains the GitHub build action we use to create an executable image of our work.

- [GitHub actions](#github-actions)
  - [General](#general)
  - [The Dockerfile (`build/docker/build/Dockerfile`)](#the-dockerfile-builddockerbuilddockerfile)
  - [The `build-and-push-image` job](#the-build-and-push-image-job)
    - [1. Checkout repository (`actions/checkout@v3`)](#1-checkout-repository-actionscheckoutv3)
    - [2. Set up Docker Buildx (`docker/setup-buildx-action@v2`)](#2-set-up-docker-buildx-dockersetup-buildx-actionv2)
    - [3. Log in to the Container registry (`docker/login-action@v2`)](#3-log-in-to-the-container-registry-dockerlogin-actionv2)
    - [4. Bump version and push tag (`mathieudutour/github-tag-action`)](#4-bump-version-and-push-tag-mathieudutourgithub-tag-action)
    - [5. Get commit hash](#5-get-commit-hash)
    - [6. Build and push Docker image](#6-build-and-push-docker-image)

## General

The workflow defined in [`.github/workflows/build.yml`](../../.github/workflows/build.yml) creates an executable image
which can later be submitted to the [CARLA leaderboard](https://leaderboard.carla.org) and pushes it
to [GitHub Packages](ghcr.io).

The image can then be pulled with `docker pull ghcr.io/una-auxme/paf:latest` to get the latest version
or `docker pull ghcr.io/una-auxme/paf:<version>` to get a specific version.

If action is triggered by a pull request the created image is then used to execute a test run in the leaderboard, using
the devtest routes. The results of this simulation are then added as a comment to the pull request.

## The Dockerfile ([`build/docker/build/Dockerfile`](../../build/docker/build/Dockerfile))

The Dockerfile uses [Dockerfile+](https://github.com/edrevo/dockerfile-plus) to include
the [agent Dockerfile](../../build/docker/agent/Dockerfile) to avoid duplicate code.
The code folder is then copied into the container instead of mounting it as in our dev setup.

## The `build-and-push-image` job

### 1. Checkout repository ([`actions/checkout@v3`](https://github.com/actions/checkout))

Trivial, just checks out the repo.

### 2. Set up Docker Buildx ([`docker/setup-buildx-action@v2`](https://github.com/docker/setup-buildx-action))

Set's up Buildx. This is needed to set up the correct driver to allow caching in step 5.

Detailed description why this is needed can be
found [here](https://github.com/docker/build-push-action/issues/163#issuecomment-1053657228).

### 3. Log in to the Container registry ([`docker/login-action@v2`](https://github.com/docker/login-action))

Logs in with `GITHUB_TOKEN` into the registry (ghcr.io).

Example taken from [here](https://docs.github.com/en/actions/publishing-packages/publishing-docker-images)

### 4. Bump version and push tag ([`mathieudutour/github-tag-action`](https://github.com/mathieudutour/github-tag-action))

If the current commit is on the `main` branch, this action bumps the version and pushes a new tag to the repo.

Major releases can be done manually (e.g. `git tag v1.0.0`).

### 5. Get commit hash

If Step 4 was skipped, this step gets the commit hash of the current commit, to be used as a tag for the Docker image.

### 6. Build and push Docker image

Build and push the image to the registry. To avoid large downloads of the base image
the [GitHub Actions cache](https://docs.docker.com/build/building/cache/backends/gha/)
is used to cache the image after build.
If the action is run on a branch other than `main`, the image is tagged with the commit hash from Step 5.
Otherwise, the image is tagged with both the tag created in Step 4 and `latest`.
