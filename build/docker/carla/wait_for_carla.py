import carla
import os


def main():
    success = False
    while not success:
        try:
            client = carla.Client(
                os.getenv("CARLA_SIM_HOST", "localhost"),
                int(os.getenv("CARLA_SIM_PORT", "2000")),
            )
            client.set_timeout(0.5)
            print(f"Done waiting for carla. Version: {client.get_server_version()}")
            success = True
        except Exception as error:
            print(f"Waiting for carla: {error}")


if __name__ == "__main__":
    main()
