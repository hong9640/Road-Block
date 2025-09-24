#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, json, math
from morai_msgs.msg import EgoVehicleStatus, CtrlCmd
import websocket

def to_yaw(heading):
    # 값이 2π보다 크면 degree로 보고 rad로 변환
    return math.radians(heading) if abs(heading) > 6.283185307179586 else heading

class WSClient:
    def __init__(self, url, timeout=5):
        self.url = url
        self.timeout = timeout
        self.ws = None
        self.connect()

    def connect(self):
        try:
            self.ws = websocket.create_connection(self.url, timeout=self.timeout)
            rospy.loginfo("[ws] connected: %s", self.url)
        except Exception as e:
            rospy.logwarn("[ws] connect failed: %s", e)
            self.ws = None

    def send_json(self, obj):
        data = json.dumps(obj, ensure_ascii=False)
        if self.ws is None:
            self.connect()
        if self.ws:
            try:
                self.ws.send(data)
                return True
            except Exception as e:
                rospy.logwarn("[ws] send failed, retry once: %s", e)
                try:
                    self.ws.close()
                except Exception:
                    pass
                self.ws = None
                self.connect()
                if self.ws:
                    try:
                        self.ws.send(data)
                        return True
                    except Exception as e2:
                        rospy.logerr("[ws] send failed after reconnect: %s", e2)
        return False

    def close(self):
        try:
            if self.ws:
                self.ws.close()
        except Exception:
            pass
        self.ws = None

class EgoCtrlWSNode:
    def __init__(self):
        # -------------------
        # 하드코딩된 기본값(아무 옵션 없이 실행해도 동작)
        # -------------------
        self.ego_topic   = "/Ego_topic"                       # MORAI ROS1 기본 토픽
        self.ctrl_topic  = "/ctrl_cmd"
        self.ws_url      = "ws://70.12.246.49:8000/ws/vehicles"
        self.client_id   = "morai-noetic"
        self.frame_id    = "ego_frame"
        self.pub_rate_hz = 1.0

        # 제어 기본값: Velocity control로 12 m/s
        self.longlCmdType = 2        # 1:Throttle, 2:Velocity, 3:Acceleration
        self.velocity     = 12.0     # m/s
        self.accel        = 0.5      # longlCmdType=2에선 의미 거의 없음(유지)
        self.brake        = 0.0
        self.steering     = 0.0      # rad

        # -------------------
        # (선택) ROS 파라미터가 있으면 덮어쓰기
        # -------------------
        self.ego_topic   = rospy.get_param("~ego_status_topic", self.ego_topic)
        self.ctrl_topic  = rospy.get_param("~ctrl_topic",       self.ctrl_topic)
        self.ws_url      = rospy.get_param("~ws_url",           self.ws_url)
        self.client_id   = rospy.get_param("~client_id",        self.client_id)
        self.frame_id    = rospy.get_param("~frame_id",         self.frame_id)
        self.pub_rate_hz = float(rospy.get_param("~publish_rate", self.pub_rate_hz))

        self.longlCmdType = int(rospy.get_param("~longlCmdType", self.longlCmdType))
        self.velocity     = float(rospy.get_param("~velocity",     self.velocity))
        self.accel        = float(rospy.get_param("~accel",        self.accel))
        self.brake        = float(rospy.get_param("~brake",        self.brake))
        self.steering     = float(rospy.get_param("~steering",     self.steering))

        # ---- pubs/subs/ws ----
        self.publisher  = rospy.Publisher(self.ctrl_topic, CtrlCmd, queue_size=10)
        self.subscriber = rospy.Subscriber(self.ego_topic, EgoVehicleStatus, self.ego_callback, queue_size=10)
        self.ws_client  = WSClient(self.ws_url)
        rospy.on_shutdown(self.ws_client.close)

        # ---- prebuilt ctrl msg ----
        self.ctrl_msg = CtrlCmd()
        self.ctrl_msg.longlCmdType = self.longlCmdType
        self.ctrl_msg.velocity     = self.velocity
        self.ctrl_msg.accel        = self.accel
        self.ctrl_msg.brake        = self.brake
        self.ctrl_msg.steering     = self.steering

        rospy.loginfo("[ctrl] mode=%d vel=%.2f steer=%.3f pub=%.1fHz topic=%s",
                      self.longlCmdType, self.velocity, self.steering, self.pub_rate_hz, self.ctrl_topic)
        rospy.loginfo("[ego]  sub topic=%s  [ws]=%s", self.ego_topic, self.ws_url)

    def ego_callback(self, msg: EgoVehicleStatus):
        # 콘솔 로그
        try:
            rospy.loginfo('------------------Ego Vehicle Status------------------')
            rospy.loginfo('position     : x = %.3f , y = %.3f , z = %.3f',
                          msg.position.x, msg.position.y, msg.position.z)
            rospy.loginfo('velocity     : x = %.3f , y = %.3f , z = %.3f m/s',
                          msg.velocity.x, msg.velocity.y, msg.velocity.z)
            rospy.loginfo('acceleration : x = %.3f , y = %.3f , z = %.3f m/s^2',
                          msg.acceleration.x, msg.acceleration.y, msg.acceleration.z)
            rospy.loginfo('heading      : %.3f (deg or rad)', msg.heading)
        except Exception:
            pass

        # WebSocket payload
        yaw = to_yaw(getattr(msg, "heading", 0.0))
        payload = {
            "type": "pose",
            "source": "ego",
            "clientId": self.client_id,
            "ts": int(rospy.Time.now().to_nsec() // 1_000_000),  # ms
            "frame": self.frame_id,
            "position": {
                "x": float(msg.position.x),
                "y": float(msg.position.y),
                "z": float(getattr(msg.position, "z", 0.0))
            },
            "orientation": {"yaw": float(yaw)}
        }
        self.ws_client.send_json(payload)

    def spin(self):
        rate = rospy.Rate(self.pub_rate_hz)
        # 바로 퍼블리시 시작(파라미터 미지정이어도 기본값으로 주행)
        while not rospy.is_shutdown():
            self.publisher.publish(self.ctrl_msg)
            rate.sleep()

def main():
    rospy.init_node("ego_status_and_ctrl_ws", anonymous=True)  # 프로세스당 1회만
    node = EgoCtrlWSNode()
    node.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

