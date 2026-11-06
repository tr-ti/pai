#!/usr/bin/env bash
# Gazebo 起動後に RViz2 を止めて CPU 負荷を下げる（実時間比が低いとき用）。
# Gazebo ごと終了してしまう場合は、RViz2 を止めずウィンドウの最小化だけにする。
# 使い方: Gazebo が落ち着いてから別ターミナルで bash reduce_gazebo_load.sh
pkill rviz2 && echo "rviz2 を停止しました" || echo "rviz2 は動いていません"
