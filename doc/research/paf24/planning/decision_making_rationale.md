# Decision Makaing Approaches

Responsible for making decisions based on information provided by other system components. It should account for all possible traffic scenarios to ensure comprehensive coverage and safe driving behavior

## Decision Tree - current implementations

- a tool to organizes decisions and their possible outcomes in a tree-like structure
- a branch represents a choice/condition which may lead to further options.
  - starting at the root: checking surrounding conditions (traffic lights, obstacles, speed limits etc) branching out based on yes/no questions at each node
- in our case each node is deterministic (as no ML is used)

### Pros and Cons

- `+` are easy to understand and interpret
- `+` decision paths are traceable → suitable for debugging
- `+` deterministic outputs beneficial when predictability is essential
- `+` compared to NN, low computational cost
- `-` Adding too many branches for every possible situation can make the tree unwieldy and hard to manage
  - can easily overfit with too many conditions
- `-` no reasoning and no probabilistic capabilities; trees tend to make suboptimal decisions in new, unclear situations
  - hard to account for every situation

### Changes in Previous Semesters

- removing redundant behaviours

  - reducing to `Intersection`, `Lane Switch` and `Cruise`

![decision_tree](/doc/assets/behaviour_tree.png)

## Finite State Machine - previous implementations

- model with an discrete number of possible states with rule-based control
  - can only inherit one state at a time
- each state represents a specific driving mode, such as "Lane Switching" "Stopping," or "Overtaking"
  - triggered by inputs like traffic signals, obstacles, or speed limits

### Pros and Cons

- `+` each state and transition is explicitly defined → suitable for debugging and testing
- `+` easier implementations than a decision tree
- `-` scalability issues with increasing number of states
- `-` less flexible than decision trees

## Combining Planning and Reinforcement Learning

[C.-J. Hoel, K. Driggs-Campbell, K. Wolff, L. Laine, and M. J. Kochenderfer, "Combining Planning and Deep Reinforcement Learning in Tactical Decision Making for Autonomous Driving," arXiv preprint arXiv:1905.02680, 2019.](https://arxiv.org/abs/1905.02680)

- Monte Carlo Tree Search (MCTS): Used for planning by creating a search tree that simulates possible actions, selecting those that maximize expected rewards (maintaining desired speed, reaching end points, avoinding collision etc.)
  - considers all possible actions → Each action creates a new branch that represents what the new situation might look like if the agent took that action
- deep neural network guides the MCTS by predicting action probabilities (which actions are most likely to lead to good outcomes: changing lanes, speeding up, breaking etc.) and state values (scores that tell the agent whether its current situation is favorable or not based on expected future rewards)

### Pros and Cons

- `+` generalizes better to complex scenarios
  - takes advantage of continuous and high-dimensional state spaces
- `-` computational complexity and real-time feasibility
- `-` extensive training data and tuning required to handle a broad range of driving situations
