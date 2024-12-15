# Project management

**Summary:** We use [issues](https://github.com/una-auxme/paf/issues) and [milestones](https://github.com/una-auxme/paf/milestones) to manage the project. This document explains how to create issues and pull requests.

- [1. Create bug or feature requests](#1-create-bug-or-feature-requests)
- [2. Create a Pull Request](#2-create-a-pull-request)

## 1. Create bug or feature requests

Issues can be added via the [issues overview](https://github.com/una-auxme/paf/issues).

![create issue](../assets/create_issue.png)

By clicking "New issue" in the overview you have different templates to create the issue.

After creating the issue it has to be added to the project board. The project board is used to keep track of the
progress of the issue. The project board can be found [here](https://github.com/orgs/una-auxme/projects/3).

For tracking time and complexity of the issue we will use the predefined `Priority` and `Size` states of the PAF Project
Backlog Board.

Priority has following states:

- `Urgent`: Critical bug causing system crash, needs immediate fix.
- `High`: Major feature request or significant bug affecting many users.
- `Medium`: Minor feature request or bug with a workaround.
- `Low`: Cosmetic changes or minor improvements.

Size has following states:

- `Tiny`: Small typo fix or minor code refactor. Estimated time: 30 minutes.
- `Small`: Simple bug fix or small feature addition. Estimated time: 1-2 hours.
- `Medium`: Moderate feature addition or multiple bug fixes. Estimated time: 1-2 days.
- `Large`: Major feature implementation or significant refactor. Estimated time: 1-2 weeks.
- `X-Large`: Large-scale feature or complete module overhaul. Estimated time: 2-4 weeks. Especially `EPIC` issues should
  be tagged with this size.

## 2. Create a Pull Request

To create a pull request, go to the [branches overview](https://github.com/una-auxme/paf/branches) and select ``New Pull Request`` for the branch you want to create a PR for.
![img.png](../assets/branch_overview.png)
<!-- TODO image is outdated -->

Merge the pull request after:

1. The review process is complete
2. All reviewer feedback has been addressed
3. All required checks have passed
4. The branch is up-to-date with the main branch

After merging, remember to delete the source branch to keep the repository clean.

>[!TIP] For more information about the review process, see [Review process](./review_guideline.md).
