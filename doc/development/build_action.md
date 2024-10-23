# GitHub actions

(Kept from previous group [paf22])

**Summary:** This page explains the GitHub build action we use to create an executable image of our work.

- [General](#general)
- [The `build-and-push-image` job](#the-build-and-push-image-job)
  - [1. Checkout repository (`actions/checkout@v3`)](#1-checkout-repository-actionscheckoutv3)
  - [2. Set up Docker Buildx (`docker/setup-buildx-action@v2`)](#2-set-up-docker-buildx-dockersetup-buildx-actionv2)
  - [3. Log in to the Container registry (`docker/login-action@v2`)](#3-log-in-to-the-container-registry-dockerlogin-actionv2)
  - [4. Test building the image (`docker/build-push-action@v3`)](#4-test-building-the-image-dockerbuild-push-actionv3)
  - [5. Build and push the image (`docker/build-push-action@v3`)](#5-build-and-push-the-image-dockerbuild-push-actionv3)

## General

The workflow defined in [`.github/workflows/build.yml`](../../.github/workflows/build.yml) creates an executable image
which can later be submitted to the [CARLA leaderboard](https://leaderboard.carla.org) and pushes it
to [GitHub Packages](ghcr.io).

The image can then be pulled with `docker pull ghcr.io/una-auxme/paf:latest` to get the latest version
or `docker pull ghcr.io/una-auxme/paf:<version>` to get a specific version.

If action is triggered by a pull request the created image is then used to execute a test run in the leaderboard, using
the devtest routes. The results of this simulation are then added as a comment to the pull request.

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

### 4. Test building the image ([`docker/build-push-action@v3`](https://github.com/docker/build-push-action/))

Tries to build the image without pushing it to the repository packages if the workflow was triggered with a pull request.

### 5. Build and push the image ([`docker/build-push-action@v3`](https://github.com/docker/build-push-action/))

This action builds the image and pushes it to repository under the `latest` tag if the workflow was triggered with a merge to the `main` branch.
To avoid large downloads of the base image
the [GitHub Actions cache](https://docs.docker.com/build/building/cache/backends/gha/)
is used to cache the image after build.
