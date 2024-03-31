# Git commit message conventions

(Kept from previous group [paf22])

[Conventional Commits](https://www.conventionalcommits.org/) are enforced by [comlipy](https://gitlab.com/slashplus-build/comlipy/) during commit. The commit message should be structured as follows:

```text
<type>(optional scope): <description>

[optional body]

[optional footer(s)]
```

(Example inspired by [https://www.conventionalcommits.org/](https://www.conventionalcommits.org/))

## Possible types

| type  | description                        |
|-------|------------------------------------|
| `docs`  | Changes in documentation           |
| `feat`  | A new feature                      |
| `fix`   | A bug fix                          |
| `other` | Anything else (should be avoided!) |

## Possible scopes

As scope we take the number of the issue the commit belongs to, prefixed by a #.

## Example

Some resulting commit message would be:

```text
feat(#18): Added front left camera to vehicle
docs(#18): Added documentation about front left camera
```

## 🚨 Common Problems

`Got permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock:...`

- Make sure your docker client is running
