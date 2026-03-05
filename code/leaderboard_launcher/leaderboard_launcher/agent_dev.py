import paf_agent_base as agent


def get_entry_point():
    return "PAFAgentDev"


class PAFAgentDev(agent.PAFAgent):
    def get_ros_entrypoint(self):
        return {
            "package": "leaderboard_launcher",
            "launch_file": "leaderboard.dev.xml",
        }
