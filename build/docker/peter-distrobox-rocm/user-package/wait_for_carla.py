import carla
import os


def main():
    success = False
    while not success:
        try:
            client = carla.Client("localhost", int(os.environ["CARLA_PORT"]))
            client.set_timeout(0.5)
            print(f"Done waiting for carla. Version: {client.get_server_version()}")
            success = True
        except BaseException as error:
            print(f"Waiting for carla: {error}")


if __name__ == "__main__":
    main()
