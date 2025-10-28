#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import torch
from std_msgs.msg import Float64MultiArray
import geort  # 사용자가 제공한 모델 모듈

class FingertipToQposNode:
    def __init__(self):
        # 파라미터 (필요시 rosparam으로 변경 가능)
        self.input_topic  = rospy.get_param("-input_topic",  "/senseglove/rh/fingertip_positions")
        self.output_topic = rospy.get_param("-output_topic", "/shadowhand/qpos")
        self.checkpoint_tag = rospy.get_param("-checkpoint_tag", "best_model")
        self.epoch = int(rospy.get_param("-epoch", -1))

        # 모델 로드
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = geort.load_model(self.checkpoint_tag, epoch=self.epoch)
        # self.model.to(self.device)
        # self.model.eval()

        # 퍼블리셔/서브스크라이버
        self.pub_qpos = rospy.Publisher(self.output_topic, Float64MultiArray, queue_size=10)
        self.sub_ft   = rospy.Subscriber(self.input_topic, Float64MultiArray, self.cb, queue_size=10)

        # 고정 변환 행렬
        self.R = np.array([
            [0, 1, 0],
            [0, 0, 1],
            [1, 0, 0]
        ], dtype=np.float32)

        rospy.loginfo("FingertipToQposNode ready. Sub: %s  Pub: %s  Device: %s",
                      self.input_topic, self.output_topic, self.device)

    def cb(self, msg: Float64MultiArray):
        # 기대 크기: 15 (= 5 fingertips × 3 coords)
        arr = np.asarray(msg.data, dtype=np.float32)
        if arr.size != 15:
            rospy.logwarn_throttle(5.0, "Unexpected data size: got %d, expected 15.", arr.size)
            return

        # (1) reshape + 단위 변환(mm -> m)
        pts = arr.reshape(5, 3) / 1000.0

        # (2) z 오프셋 보정
        pts[:, 2] -= 0.045

        # (3) 좌표계 회전
        pts = pts.dot(self.R.T)

        # (4) 앞 4개 포인트만 사용
        pts4 = pts[:4, :]  # shape (4,3)

        # (5) 모델 추론
        qpos = self.model.forward(pts4)

        # (6) 퍼블리시
        out = Float64MultiArray(data=qpos.tolist())
        self.pub_qpos.publish(out)

def main():
    rospy.init_node("fingertip_to_qpos", anonymous=False)
    node = FingertipToQposNode()
    rospy.spin()

if __name__ == "__main__":
    main()
