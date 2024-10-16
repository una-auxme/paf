# First steps

**Summary**: These first steps should help you get familiar with the project and how to work with it.

- [First steps](#first-steps)
  - [Prepare your environment](#prepare-your-environment)
  - [Start the development container](#start-the-development-container)
  - [Linting and formatting](#linting-and-formatting)
    - [Python and Markdown linters](#python-and-markdown-linters)
    - [Python formatter](#python-formatter)
    - [Trigger linting and formatting manually](#trigger-linting-and-formatting-manually)
  - [Further information](#further-information)

## Prepare your environment

After you cloned the repository open the corresponding folder with VS Code. The repository comes with a suite of recommended extensions that simplify the development process. Navigate to the `Extensions` tab in VS Code (Shortcut `Ctrl+Shift+X`) and install the extensions.

## Start the development container

In order to make changes to the code or add new functionality you can edit the files directly in VS Code now. However, for the default development environment head to the `/build` folder and execute the `docker-compose.dev.yaml` file via right-click and selecting `Compose Up` in the menu:

![devcontainer.png](/doc/assets/development/devcontainer.png)

> This is the default way to execute every docker-compose file. If you try to start a container without it things may break!

Then navigate to the `Docker` tab in VS Code and attach a VS Code window to the `build-agent-dev` container:

![attach.png](/doc/assets/development/attach.png)

A new VS Code window will open, and you now have all the project files with the correct python paths and code completion.

## Linting and formatting

The linters and formatter are automatically running in the background as VS Code extensions.

### Python and Markdown linters

For Python and Markdown files the suggested changes are shown as warnings and are displayed in the file directly. These have to be resolved in order for the GitHub actions to complete on your pull request.

### Python formatter

The Python formatter automatically executes each time you save your file.

### Trigger linting and formatting manually

If you want to manually check the linting and formatting you can execute the `docker-compose.linter.yaml` file the same way you execute the dev file mentioned in [Start the development container](#start-the-development-container).

If you want to trigger the code formatting manually you can type the following command in a shell of the container:

```sh
black <file-or-folder>
```

To get a shell of the container you can either use the Terminal on the bottom of VS Code in the attached VS Code window (see [Start the development container](#start-the-development-container), Shortcut `` Ctrl+Shift+` ``) or attach a shell in the same right-click menu as attaching a VS Code window.

## Further information

These are additional resources that are useful for handling the project:

- [Executing different scenarios](/doc/general/execution.md)
- [Documentation requirements](/doc/development/documentation_requirements.md) (especially docstrings)
