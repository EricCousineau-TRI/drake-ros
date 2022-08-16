import os
import subprocess
from threading import Thread

from bazel_tools.tools.python.runfiles import runfiles
import rclpy
import rclpy.node
from rmw_isolation import isolate_rmw_by_path

from ros2_example_apps_msgs.msg import Status


class StatusPublisher(rclpy.node.Node):
    def __init__(self):
        super().__init__("test")
        self._pub = self.create_publisher(Status, "/status", 10)
        self._timer = self.create_timer(0.1, self._callback)

    def _callback(self):
        self._pub.publish(Status())


def do_main():
    manifest = runfiles.Create()
    ros2 = manifest.Rlocation("ros2/ros2")
    subprocess.run(
        [ros2, "topic", "echo", "--once", "/status"],
        check=True,
    )


def main():
    if "TEST_TMPDIR" in os.environ:
        isolate_rmw_by_path(os.environ["TEST_TMPDIR"])
        os.environ["HOME"] = os.path.join(os.environ["TEST_TMPDIR"])

    rclpy.init()
    publisher = StatusPublisher()
    spinner = Thread(target=rclpy.spin, args=(publisher,))
    spinner.start()
    try:
        do_main()
    finally:
        publisher.destroy_node()
        rclpy.shutdown()
        spinner.join()

    print("[ Done ]")


if __name__ == "__main__":
    main()
