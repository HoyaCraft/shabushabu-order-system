import rclpy
from rclpy.node import Node
from service_interface.srv import MenuOrder
import threading
import sys
import select


class TableServer(Node):
    def __init__(self):
        super().__init__('table_server')
        self.srv = self.create_service(MenuOrder, 'order_service', self.order_callback)  # 서비스 생성
        self.current_request = None
        self.current_response = None
        self.lock = threading.Lock()  # 동기화를 위한 Lock
        self.get_logger().info("Service server is ready and waiting for requests.")

        # 키보드 입력 감지를 위한 스레드 실행
        self.keyboard_thread = threading.Thread(target=self.listen_keyboard_input, daemon=True)
        self.keyboard_thread.start()

    def order_callback(self, request, response):
        """클라이언트 요청을 처리"""
        with self.lock:  # 동기화 블록
            if self.current_request is not None:
                response.message = "Server is busy. Try again later."
                return response

            # 현재 요청 및 응답 저장
            self.current_request = request.menu
            self.current_response = response

        self.get_logger().info(f"Received request: {request.menu}. Waiting for input.")
        self.get_logger().info("Press '1' for 'ok' or '2' for 'sorry'.")

        while True:  # 응답 준비될 때까지 대기
            if self.current_response.message:
                break

        self.get_logger().info(f"Responded to client with: {self.current_response.message}")
        with self.lock:  # 동기화 블록
            self.current_request = None
            return self.current_response

    def listen_keyboard_input(self):
        """키보드 입력 감지 및 응답 처리"""
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:  # 비차단 입력 감지
                key = sys.stdin.read(1).strip()
                with self.lock:  # 동기화 블록
                    if self.current_request and self.current_response:
                        if key == "1":
                            self.get_logger().info("Key '1' pressed: Sending 'ok' to client.")
                            self.current_response.message = "ok"
                        elif key == "2":
                            self.get_logger().info("Key '2' pressed: Sending 'sorry' to client.")
                            self.current_response.message = "sorry"
                        else:
                            self.get_logger().info(f"Invalid key '{key}' pressed. Try again.")
                    else:
                        self.get_logger().info("No active request to respond to.")


def main():
    rclpy.init()
    server_node = TableServer()
    try:
        rclpy.spin(server_node)
    except KeyboardInterrupt:
        print("Shutting down server...")
    finally:
        server_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
