#!/usr/bin/env python3
import threading
import time
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
import rclpy
from rclpy.node import Node


class LifecycleManager(Node):
    def __init__(self):
        super().__init__("lifecycle_manager")
        self.declare_parameter(
            "managed_nodes", ["sensor_station_1", "sensor_station_2"]
        )
        raw_nodes = (
            self.get_parameter("managed_nodes").get_parameter_value().string_array_value
        )
        self.managed_nodes = [
            n.strip(" \t\n\r'\"[]") for n in raw_nodes if n.strip(" \t\n\r'\"[]")
        ]

        self.get_logger().info(
            f"LifecycleManager initialized. Managing {len(self.managed_nodes)} nodes:"
        )
        for name in self.managed_nodes:
            self.get_logger().info(f"  - /{name}")

        self.worker_thread = threading.Thread(target=self.run_orchestration)
        self.worker_thread.daemon = True
        self.worker_thread.start()

    def run_orchestration(self):
        self.get_logger().info(
            "==> Step 0: Waiting for lifecycle services for all nodes..."
        )
        for node_name in self.managed_nodes:
            change_cli = self.create_client(ChangeState, f"/{node_name}/change_state")
            get_cli = self.create_client(GetState, f"/{node_name}/get_state")

            while not change_cli.wait_for_service(timeout_sec=2.0):
                if not rclpy.ok():
                    return
                self.get_logger().info(f"Waiting for /{node_name}/change_state...")
            while not get_cli.wait_for_service(timeout_sec=2.0):
                if not rclpy.ok():
                    return
                self.get_logger().info(f"Waiting for /{node_name}/get_state...")

        self.print_current_states("Initial State")
        time.sleep(2.0)

        # Step 1: Configure all nodes
        self.get_logger().info(
            "==> Step 1: CONFIGURE all nodes (allocating publishers, timers, parameters)..."
        )
        self.change_state_all(Transition.TRANSITION_CONFIGURE, "configure")
        self.print_current_states("After Configure")

        self.get_logger().info(
            "[NOTE] Nodes are now INACTIVE. Timers are ticking, but LifecyclePublishers are inactive."
        )
        self.get_logger().info(
            "[NOTE] Monitor receives NO messages during this 4-second period."
        )
        time.sleep(4.0)

        # Step 2: Synchronous Activation
        self.get_logger().info(
            "==> Step 2: SIMULTANEOUS ACTIVATION of all managed nodes!"
        )
        self.change_state_all(Transition.TRANSITION_ACTIVATE, "activate")
        self.print_current_states("After Simultaneous Activation")

        self.get_logger().info(
            "[NOTE] All nodes are now ACTIVE. Both stations are actively publishing messages simultaneously!"
        )
        time.sleep(8.0)

        # Step 3: Synchronous Deactivation
        self.get_logger().info(
            "==> Step 3: SIMULTANEOUS DEACTIVATION of all managed nodes!"
        )
        self.change_state_all(Transition.TRANSITION_DEACTIVATE, "deactivate")
        self.print_current_states("After Simultaneous Deactivation")

        self.get_logger().info(
            "[NOTE] Nodes returned to INACTIVE. Message streams stop immediately while nodes remain alive."
        )
        time.sleep(4.0)

        # Step 4: Cleanup
        self.get_logger().info(
            "==> Step 4: CLEANUP all nodes (freeing publishers and timers)..."
        )
        self.change_state_all(Transition.TRANSITION_CLEANUP, "cleanup")
        self.print_current_states("After Cleanup")
        time.sleep(2.0)

        # Step 5: Shutdown
        self.get_logger().info(
            "==> Step 5: SHUTDOWN all nodes to finalized state..."
        )
        self.change_state_all(
            Transition.TRANSITION_UNCONFIGURED_SHUTDOWN, "shutdown"
        )
        self.print_current_states("After Shutdown")

        self.get_logger().info(
            "==> Coordinated multi-node lifecycle orchestration completed successfully!"
        )

    def change_state(self, node_name: str, transition_id: int, transition_label: str) -> bool:
        client = self.create_client(ChangeState, f"/{node_name}/change_state")
        req = ChangeState.Request()
        req.transition.id = transition_id
        req.transition.label = transition_label

        future = client.call_async(req)
        # Wait for future with spin or loop
        start = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start > 5.0:
                self.get_logger().error(f"Timed out calling /{node_name}/change_state")
                return False
            time.sleep(0.05)

        if future.result() and future.result().success:
            self.get_logger().info(
                f"  [SUCCESS] Node /{node_name} transitioned via '{transition_label}'"
            )
            return True
        else:
            self.get_logger().error(
                f"  [FAILED] Node /{node_name} failed transition via '{transition_label}'"
            )
            return False

    def change_state_all(self, transition_id: int, transition_label: str):
        for node_name in self.managed_nodes:
            self.change_state(node_name, transition_id, transition_label)

    def get_state(self, node_name: str) -> str:
        client = self.create_client(GetState, f"/{node_name}/get_state")
        req = GetState.Request()
        future = client.call_async(req)
        start = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start > 3.0:
                return "TIMEOUT"
            time.sleep(0.05)

        if future.result():
            return future.result().current_state.label
        return "UNKNOWN"

    def print_current_states(self, header: str):
        self.get_logger().info(f"--- State Check [{header}] ---")
        for node_name in self.managed_nodes:
            state = self.get_state(node_name)
            self.get_logger().info(f"    /{node_name} is in state: [{state}]")


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
