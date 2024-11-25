# GitHub actions

(Kept from previous group [paf22])

**Summary:** This page explains the GitHub build action we use to create an executable image of our work.

- [General](#general)
- [The `build-and-push-image` job](#the-build-and-push-image-job)
  - [1. Checkout repository (`actions/checkout@v3`)](#1-checkout-repository-actionscheckoutv3)
  - [2. Set up Docker Buildx (`docker/setup-buildx-action@v2`)](#2-set-up-docker-buildx-dockersetup-buildx-actionv2)
  - [3. Cache Docker layers](#3-cache-docker-layers)
  - [4. Log in to the Container registry (`docker/login-action@v2`)](#4-log-in-to-the-container-registry-dockerlogin-actionv2)
  - [5. Test building the image (`docker/build-push-action@v3`)](#5-test-building-the-image-dockerbuild-push-actionv3)
  - [6. Build and push the image (`docker/build-push-action@v3`)](#6-build-and-push-the-image-dockerbuild-push-actionv3)
  - [7. Save pull request artifact](#7-save-pull-request-artifact)
  - [8. Save merge artifact](#8-save-merge-artifact)
  - [9. Upload artifact](#9-upload-artifact)
  - [10. Clean up PR Cache](#10-clean-up-pr-cache)
  - [11. Clean up merge Cache](#11-clean-up-merge-cache)
  - [12. Prune all images older than one day](#12-prune-all-images-older-than-one-day)

## General

The workflow defined in [`.github/workflows/build.yml`](../../.github/workflows/build.yml) creates an executable image
which can later be submitted to the [CARLA leaderboard](https://leaderboard.carla.org) and pushes it
to [GitHub Packages](ghcr.io).

The image can then be pulled with `docker pull ghcr.io/una-auxme/paf:latest` to get the latest version
or `docker pull ghcr.io/una-auxme/paf:<version>` to get a specific version.

After the action is finished the `drive` action is triggered.

## The `build-and-push-image` job

### 1. Checkout repository ([`actions/checkout@v3`](https://github.com/actions/checkout))

Trivial, just checks out the repo.

### 2. Set up Docker Buildx ([`docker/setup-buildx-action@v2`](https://github.com/docker/setup-buildx-action))

Set's up Buildx. This is needed to set up the correct driver to allow caching in step 5.

Detailed description why this is needed can be
found [here](https://github.com/docker/build-push-action/issues/163#issuecomment-1053657228).

### 3. Cache Docker layers

Creates (if not done previously) and sets up the cache for the Docker layers.

### 4. Log in to the Container registry ([`docker/login-action@v2`](https://github.com/docker/login-action))

Logs in with `GITHUB_TOKEN` into the registry (ghcr.io).

Example taken from [here](https://docs.github.com/en/actions/publishing-packages/publishing-docker-images)

### 5. Test building the image ([`docker/build-push-action@v3`](https://github.com/docker/build-push-action/))

Tries to build the image without pushing it to the repository packages if the workflow was triggered with a pull request.

### 6. Build and push the image ([`docker/build-push-action@v3`](https://github.com/docker/build-push-action/))

This action builds the image and pushes it to repository under the `latest` tag if the workflow was triggered with a merge to the `main` branch.
To avoid large downloads of the base image
the [GitHub Actions cache](https://docs.docker.com/build/building/cache/backends/gha/)
is used to cache the image after build.

### 7. Save pull request artifact

If the action was triggered by a pull request an artifact is created with the corresponding PR ID (used for commenting on the PR in the `drive` action).

### 8. Save merge artifact

If the action was triggered by a merge an artifact is created with an invalid PR ID to signalise the `drive` action that it was not triggered by a PR.

### 9. Upload artifact

Uploads the artifact to the given path with a retention of the given number of days.

### 10. Clean up PR Cache

Removes the previous obsolete PR cache.

### 11. Clean up merge Cache

Removes the previous obsolete test cache.

### 12. Prune all images older than one day

Removes all images from the runner that are older than one day to free disk space.
