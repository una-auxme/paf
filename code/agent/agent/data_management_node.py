from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.service import Service
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import Bool, String
from carla_msgs.msg import CarlaRoute
from planning_interfaces.srv import GetOpenDriveString, GetCarlaRoute
from paf_common.sync import startup_topic


class DataManagement(Node):
    def __init__(self):
        super().__init__(type(self).__name__)
        self.get_logger().info(f"{type(self).__name__} node initializing...")

        # Persistent data
        self.open_drive_string: Optional[str] = None
        self.global_plan: Optional[CarlaRoute] = None

        # Parameters
        self.role_name = self.declare_parameter("role_name", "hero").value

        # Services
        self.open_drive_service: Optional[Service] = None
        self.global_plan_service: Optional[Service] = None

        self.open_drive_service = self.create_service(
            GetOpenDriveString,
            f"/paf/{self.role_name}/data/planning/get_open_drive",
            self.get_open_drive_service,
        )
        self.global_plan_service = self.create_service(
            GetCarlaRoute,
            f"/paf/{self.role_name}/data/planning/get_global_plan",
            self.get_global_plan_service,
        )

        # Subscriptions
        self.create_subscription(
            msg_type=String,
            topic=f"/carla/{self.role_name}/OpenDRIVE",
            callback=self.open_drive_callback,
            qos_profile=QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(
            msg_type=CarlaRoute,
            topic="/carla/" + self.role_name + "/global_plan",
            callback=self.global_plan_callback,
            qos_profile=QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        # Publisher
        self.open_drive_updated_pub = self.create_publisher(
            Bool, f"/paf/{self.role_name}/data/planning/open_drive_updated", 10
        )
        self.global_plan_updated_pub = self.create_publisher(
            Bool, f"/paf/{self.role_name}/data/planning/global_plan_updated", 10
        )

        self.startup_ready_pub = self.create_publisher(
            Bool,
            startup_topic(self.role_name, "data_management"),
            qos_profile=QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )

        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def open_drive_callback(self, data: String):
        self.get_logger().info("Received open drive data.")
        self.open_drive_string = data.data
        # with open("/workspace/OpenDriveString.xml", "w") as text_file:
        #     text_file.write(self.open_drive_string)
        self.open_drive_updated_pub.publish(Bool(data=True))
        self._publish_startup_ready_if_available()

    def get_open_drive_service(
        self, req: GetOpenDriveString.Request, res: GetOpenDriveString.Response
    ) -> GetOpenDriveString.Response:
        if self.open_drive_string is not None:
            res.success = True
            res.msg = "success"
            res.data = self.open_drive_string
        else:
            res.success = False
            res.msg = "data not available"
        return res

    def global_plan_callback(self, data: CarlaRoute):
        self.get_logger().info("Received global plan data.")
        self.global_plan = data
        self.global_plan_updated_pub.publish(Bool(data=True))

    def _publish_startup_ready_if_available(self) -> None:
        if self.open_drive_string is not None and self.open_drive_service is not None:
            self.startup_ready_pub.publish(Bool(data=True))

    def get_global_plan_service(
        self, req: GetCarlaRoute.Request, res: GetCarlaRoute.Response
    ) -> GetCarlaRoute.Response:
        if self.global_plan is not None:
            res.success = True
            res.msg = "success"
            res.data = self.global_plan
        else:
            res.success = False
            res.msg = "data not available"
        return res


def main(args=None):
    rclpy.init(args=args)

    try:
        node = DataManagement()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
