import time
import can
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from winch_msgs.action import SetTarget

class WinchServer(Node, can.listener.Listener):

    ERROR_CODES = {
        'NONE'                  : 0,
        'UNKNOWN'               : 200,
        'INVALID_DURATION'      : 201,
        'CAN_SEND_FAILED'       : 202,
        'TIMEOUT'               : 203,
        'NO_RESULT_RECEIVED'    : 204,
        'CAN_INIT_FAILED'       : 205,
        'INVALID_COMMAND'       : 206,
        'SDO_OBJECT_NOT_FOUND'  : 207
    }

    def __init__(self):
        super().__init__('winch')

        # Initialize CAN parameters
        self.node_id = 0x15
        self.rsdo_can_id = 0x600 + self.node_id  # 0x615
        self.tsdo_can_id = 0x580 + self.node_id  # 0x595
        self.debug_can_id = 0x666
        self.error_code = 0

        # SDO command codes
        self.SDO_WRITE = 0x23
        self.SDO_RESPONSE = 0x60
        self.SDO_ERROR = 0x80
        self.OD_CONTROL_COMMAND = 0x2000
        self.OD_FEEDBACK = 0x2001
        self.OD_RESULT = 0x2002
        self.CMD_EXTEND = 0x02
        self.CMD_RETRACT = 0x01
        self.CMD_STOP = 0x03

        # Initialize CAN bus with 3 retries
        self.bus = None
        retries = 3
        for attempt in range(retries):
            try:
                self.bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)
                self.get_logger().info("CAN bus initialized successfully")
                break
            except can.CanError as e:
                self.get_logger().error(f"Failed to initialize CAN bus (attempt {attempt+1}/{retries}): {str(e)}")
                if attempt < retries - 1:
                    time.sleep(0.1)
        if self.bus is None:
            self.get_logger().error("Failed to initialize CAN bus")
            raise RuntimeError(f"CAN bus initialization failed (error code: {self.ERROR_CODES['CAN_INIT_FAILED']})")

        # Action server state
        self.latest_feedback = None
        self.latest_result = None
        self.latest_ack = None

        # Create action server
        self._action_server = ActionServer(
            self,
            SetTarget,
            'winch_control',
            self.execute_callback
        )

        # Setup CAN listener
        can.Notifier(self.bus, [self])

        self.get_logger().info("Winch control action server is ready.")

    def on_message_received(self, msg: can.Message):
        if msg.arbitration_id == self.tsdo_can_id:
            if msg.data[0] == self.SDO_RESPONSE:
                index = int.from_bytes(msg.data[1:3], 'little')
                sub_index = msg.data[3]
                self.latest_ack = (True, 0)
                self.get_logger().info(f"Received SDO acknowledgment: index=0x{index:04X}, sub_index=0x{sub_index:02X}")
            elif msg.data[0] == self.SDO_ERROR:
                index = int.from_bytes(msg.data[1:3], 'little')
                sub_index = msg.data[3]
                error_code = int.from_bytes(msg.data[4:6], 'little')
                self.latest_ack = (False, error_code)
                self.get_logger().error(f"Received SDO error: index=0x{index:04X}, sub_index=0x{sub_index:02X}, error_code={error_code}")
            elif msg.data[0] == self.SDO_WRITE:
                index = int.from_bytes(msg.data[1:3], 'little')
                sub_index = msg.data[3]
                if index == self.OD_FEEDBACK and sub_index == 0x00:
                    status = float(msg.data[4]) / 255
                    self.latest_feedback = status
                    self.get_logger().info(f"Received SDO feedback: status={status:.2f}")
                elif index == self.OD_RESULT and sub_index == 0x00:
                    success = msg.data[4] == 1
                    error_code = int.from_bytes(msg.data[5:7], 'little')
                    self.latest_result = (success, error_code)
                    self.get_logger().info(f"Received SDO result: success={success}, error_code={error_code}")

        elif msg.arbitration_id == self.debug_can_id:
            self.decode_debug_message(msg.data)

    def decode_debug_message(self, data):
        msg_type = data[0]
        payload = data[1:8]
        
        msg_type_str = {
            0x01: "INIT",
            0x02: "CAN_DATA",
            0x03: "RETRACT",
            0x04: "EXTEND",
            0x05: "STOP",
            0x06: "UNKNOWN",
            0x07: "ERROR",
            0x08: "SDO_FEEDBACK",
            0x09: "SDO_RESULT",
            0x0A: "SDO_ACK"
        }.get(msg_type, "UNKNOWN")
        
        if msg_type == 0x02 or msg_type == 0x08 or msg_type == 0x09 or msg_type == 0x0A:
            data_str = " ".join([f"{b:02X}" for b in payload if b != 0])
            self.get_logger().info(f"Debug [{msg_type_str}]: {data_str}")
        else:
            try:
                msg_str = ''.join([chr(b) for b in payload if b != 0])
                self.get_logger().info(f"Debug [{msg_type_str}]: {msg_str}")
            except UnicodeDecodeError:
                self.get_logger().info(f"Debug [{msg_type_str}]: Non-ASCII data")

    def send_can_command(self, data, retries=3):
        if len(data) != 8:
            self.get_logger().error(f"Invalid CAN message length: {len(data)}")
            return False, self.ERROR_CODES['INVALID_COMMAND']

        self.get_logger().debug(f"Preparing to send CAN message: ID=0x{self.rsdo_can_id:X}, Data={[hex(x) for x in data]}")
        msg = can.Message(arbitration_id=self.rsdo_can_id, data=data, is_extended_id=False)
        
        for attempt in range(retries):
            try:
                self.bus.send(msg)
                self.get_logger().info(f"Sent CAN message: ID=0x{self.rsdo_can_id:X}, Data={[hex(x) for x in data]}")
                return True, self.ERROR_CODES['NONE']
            except can.CanError as e:
                self.get_logger().error(f"Failed to send CAN message (attempt {attempt + 1}/{retries}): {str(e)}")
                time.sleep(0.1)
        
        return False, self.ERROR_CODES['CAN_SEND_FAILED']

    def send_stop_command(self):
        stop_data = [self.SDO_WRITE, self.OD_CONTROL_COMMAND & 0xFF, (self.OD_CONTROL_COMMAND >> 8) & 0xFF, 0x00, self.CMD_STOP, 0x00, 0x00, 0x00]
        success, error_code = self.send_can_command(stop_data)
        if success:
            self.get_logger().info("Stop command sent to motor")
        else:
            self.get_logger().error(f"Failed to send STOP command (error code: {error_code})")
        return success


    def execute_callback(self, goal_handle:ServerGoalHandle):
        
        goal = goal_handle.request
        pull = goal.pull
        use_timer = goal.use_timer
        duration = goal.duration

        self.get_logger().info(f"Executing: pull={pull}, use_timer={use_timer}, duration={duration}")

        feedback_msg = SetTarget.Feedback()
        result = SetTarget.Result()

        self.latest_feedback = None
        self.latest_result = None
        self.latest_ack = None

        try:
            duration_ms = int(duration * 1000)
            if duration < 0 or duration_ms > 0xFFFF:    ## 0 to 60 seconds
                raise ValueError("Duration out of bounds")
            self.get_logger().info(f"Duration (ms): {duration_ms}")
            command = [
                self.SDO_WRITE,
                self.OD_CONTROL_COMMAND & 0xFF,
                (self.OD_CONTROL_COMMAND >> 8) & 0xFF,
                0x00,
                self.CMD_RETRACT if pull else self.CMD_EXTEND,
                1 if use_timer else 0,
                duration_ms & 0xFF, # Low byte of duration or 0
                (duration_ms >> 8) & 0xFF,  # Middle byte
            ]
            self.get_logger().debug(f"Constructed command: {[hex(x) for x in command]}")
        except ValueError as e:
            self.get_logger().error(f"Invalid duration: {str(e)}")
            result.success = False
            result.error_code = self.ERROR_CODES['INVALID_DURATION']
            goal_handle.abort()
            return result

        # Send CAN command and wait for acknowledgment
        success, self.error_code = self.send_can_command(command)
        if not success:
            self.send_stop_command()
            result.success = False
            result.error_code = self.error_code
            goal_handle.abort()
            return result

        # Wait for SDO acknowledgment
        start_time = time.time()
        while self.latest_ack is None and (time.time() - start_time) < 2.0:
            time.sleep(0.1)
        if self.latest_ack is None:
            self.get_logger().error(f"No SDO acknowledgment received (error code: {self.ERROR_CODES['NO_RESULT_RECEIVED']})")
            self.send_stop_command()
            result.success = False
            result.error_code = self.ERROR_CODES['NO_RESULT_RECEIVED']
            goal_handle.abort()
            return result
        elif not self.latest_ack[0]:
            self.get_logger().error(f"SDO command rejected (error code: {self.latest_ack[1]})")
            self.send_stop_command()
            result.success = False
            result.error_code = self.latest_ack[1]
            goal_handle.abort()
            return result
        
        # Process feedback and result
        start_time = time.time()
        timeout = (duration + 4) if use_timer else 20

        while self.latest_result is None:
            if self.latest_feedback is not None:
                feedback_msg.status = float(self.latest_feedback)
                goal_handle.publish_feedback(feedback_msg)

            elapsed = time.time() - start_time
            if elapsed > timeout:
                self.get_logger().error(f"Timeout occurred while waiting for result (error code: {self.ERROR_CODES['TIMEOUT']})")
                self.send_stop_command()
                result.success = False
                result.error_code = self.ERROR_CODES['TIMEOUT']
                goal_handle.abort()
                return result

            time.sleep(0.1)

        # Check results
        result.success, result.error_code = self.latest_result
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self.send_stop_command()
        return result

def main(args=None):
    rclpy.init(args=args)

    winch_server = WinchServer()
    executor = MultiThreadedExecutor()
    executor.add_node(winch_server)

    try:
        executor.spin()

    except RuntimeError as e:
        winch_server.get_logger().info(f"Failed to start winch server: {str(e)}")

    except KeyboardInterrupt:
        winch_server.get_logger().info("Shutting down...")

    finally:
        executor.shutdown()
        winch_server._action_server.destroy()
        if winch_server.bus:
            winch_server.bus.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()