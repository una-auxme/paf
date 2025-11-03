import paf_agent_base as agent


def get_entry_point():
    return "PAFAgentDeploy"


class PAFAgentDeploy(agent.PAFAgent):

    def get_ros_entrypoint(self):
        return {
            "package": "leaderboard_launcher",
            "launch_file": "leaderboard.deploy.xml",
        }
