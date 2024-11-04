# Decision Makaing Approaches

Responsible for making decisions based on information provided by other system components. It should account for all possible traffic scenarios to ensure comprehensive coverage and safe driving behavior

## Decision Tree - current implementations

- a tool to organizes decisions and their possible outcomes in a tree-like structure
- a branch represents a choice/condition which may lead to further options.
  - starting at the root: checking surrounding conditions (traffic lights, obstacles, speed limits etc) branching out based on yes/no questions at each node
- in our case each node is deterministic (as no ML is used)

### Pros and Cons

<ul style="list-style-type: none;">
    <li><strong>+</strong> are easy to understand and interpret</li>
    <li><strong>+</strong> decision paths are tracable → suitable for debugging</li>
    <li><strong>+</strong> deterministic outputs beneficial when predictability is essential</li>
    <li><strong>+</strong> compared to NN low computational cost</li>
    <li><strong>-</strong> Adding too many branches for every possible situation can make the tree unwieldy and hard to manage</li>
    <ul>
        <li> can easily overfit with too many conditions </li>
    </ul>
    <li><strong>-</strong> no reasoning and no probabilistic capabilities, trees tend to make not optimal decisions in new unclear situations</li>
    <ul>
        <li> hard to account for every situation </li>
    </ul>
</ul>

### Changes in Previous Semesters

- removing redundant behaviours

  - reducing to `Intersection`, `Lane Switch` and `Cruise`

![decision_tree](doc/assets/behaviour_tree.png)

## Finite State Machine - previous implementations

- model with an discrete number of possible states with rule-based control
  - can only inherit one state at a time
- each state represents a specific driving mode, such as "Lane Switching" "Stopping," or "Overtaking"
  - triggered by inputs like traffic signals, obstacles, or speed limits

### Pros and Cons

<ul style="list-style-type: none;">
    <li><strong>+</strong> each state and transition is explicitly defined → debugging and testing</li>
    <li><strong>+</strong> easier implementations than a decision tree</li>
    <li><strong>-</strong> scalability issues with increasing amount of states</li>
    <li><strong>-</strong> less flexible than decision trees</li>
</ul>

## Combining Planning and Reinforcement Learning

[C.-J. Hoel, K. Driggs-Campbell, K. Wolff, L. Laine, and M. J. Kochenderfer, "Combining Planning and Deep Reinforcement Learning in Tactical Decision Making for Autonomous Driving," arXiv preprint arXiv:1905.02680, 2019.](https://arxiv.org/abs/1905.02680)

- Monte Carlo Tree Search (MCTS): Used for planning by creating a search tree that simulates possible actions, selecting those that maximize expected rewards (maintaining desired speed, reaching end points, avoinding collision etc.)
  - considers all possible actions → Each action creates a new branch that represents what the new situation might look like if the agent took that action
- deep neural network guides the MCTS by predicting action probabilities (which actions are most likely to lead to good outcomes: changing lanes, speeding up, breaking etc.) and state values (scores that tell the agent whether its current situation is favorable or not based on expected future rewards)

### Pros and Cons

<ul style="list-style-type: none;">
    <li><strong>+</strong> generalizes better to complex scenarios</li>
    <ul>
        <li> takes advantage of continous and high-dimensional state spaces </li>
    </ul>
    <li><strong>-</strong> computational complexity and real-time feasibility</li>
    <li><strong>-</strong> extensive training data and tuning to handle a broad range of driving situations</li>
</ul>
