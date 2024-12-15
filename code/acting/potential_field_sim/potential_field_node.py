import rospy
import numpy as np
from your_custom_msgs.msg import Entity
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def attractive_force(goal, position, gain=1.0):
    """Berechne attraktive Kraft Richtung Ziel."""
    diff = np.array(goal) - np.array(position)
    return gain * diff


def repulsive_force(obstacle, position, gain=1.0, radius=1.0):
    """Berechne repulsive Kraft von Hindernissen."""
    diff = np.array(position) - np.array(obstacle)
    distance = np.linalg.norm(diff)
    if distance < radius:
        return gain * (1 / distance - 1 / radius) * (diff / distance**2)
    else:
        return np.array([0.0, 0.0])


def compute_trajectory(start, goal, obstacles, steps=100):
    """Berechne eine Trajektorie vom Start zum Ziel."""
    trajectory = [start]
    position = np.array(start)

    for _ in range(steps):
        # Attraktive Kraft
        f_attr = attractive_force(goal, position)

        # Repulsive Kräfte
        f_rep = np.sum([repulsive_force(ob, position) for ob in obstacles], axis=0)

        # Gesamtkraft
        f_total = f_attr + f_rep

        # Aktualisiere Position
        position += f_total
        trajectory.append(position.tolist())

        # Ziel erreicht?
        if np.linalg.norm(np.array(goal) - position) < 0.1:
            break

    return trajectory


def publish_trajectory(trajectory):
    """Veröffentliche die Trajektorie als nav_msgs/Path."""
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = rospy.Time.now()

    for point in trajectory:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        path.poses.append(pose)

    trajectory_pub.publish(path)


def entity_callback(msg):
    """Callback, um Hindernisdaten zu verarbeiten."""
    global obstacles

    # Extrahiere Hindernisposition und Form
    transform = msg.transform  # Transform2D
    x = transform.translation.x
    y = transform.translation.y

    # Speichere Hindernisposition
    obstacles.append([x, y])


def main():
    global trajectory_pub, obstacles

    rospy.init_node("potential_field_node")
    obstacles = []  # Liste der Hindernisse

    # Subscriber für Entity-Nachrichten
    rospy.Subscriber("/entity_topic", Entity, entity_callback)

    # Publisher für die Trajektorie
    trajectory_pub = rospy.Publisher("/trajectory", Path, queue_size=10)

    rate = rospy.Rate(10)

    # Start- und Zielposition (Beispiel)
    start = [0.0, 0.0]
    goal = [5.0, 5.0]

    while not rospy.is_shutdown():
        if obstacles:
            # Berechne Trajektorie
            trajectory = compute_trajectory(start, goal, obstacles)

            # Veröffentliche Trajektorie
            publish_trajectory(trajectory)

        rate.sleep()


if __name__ == "__main__":
    main()
