# Sprint 1

- [ ]  Namen auf Präsentationen packen
- [ ]  cleaning up github repo {Robert, Leon}
    - [ ]  Research issues PR / merge
- [ ]  5- 25 h work for submission fix (Work for 2) {Samuel, Maxi}
    - [ ]  constructing & updating docker image (archive into docker image)
    - [ ]  update of branches
- [ ]  Analyzation of projects AIM State and IS State (every group)
- [ ]  Start working on code
- [ ]  detailed understanding of components
    
    Perception: {Leon, Maxi}
    
    - [ ]  removing panopticsegmentation (if non breaking)
        - [ ]  what does happen when its removed?
    - [ ]  (→ replacing it with object detection by Pylot [issue for backlog])
    
    Planning: {Julius, Samuel, Alex}
    
    - [ ]  creating visual architecture
    - [ ]  understanding of components
    - [ ]  understanding of current Decision Tree
        - [ ]  does the global routing by the Decision Tree work?
        - [ ]  lane decisions correct? (holding lane, switching lanes)
        - [ ]  how well does it perform
        - [ ]  limitations?
    - [ ]  (Decision Tree, (vs. ) State Machine [issue for backlog])
        - [ ]  comparing to performance of State Machine by Driving Score (if Decision Tree hits its limitations)
        - [ ]  Decision Tree might be better without needing to test State Machine
    - [ ]  Composition of local planner
    
    Acting: {Robert, Alex}
    
    - [ ]  Detailed understanding of components
    - [ ]  Comparing trajectory with actual actions
    - [ ]  which new interfaces are needed? (if needed)