# kuams3 🤖

KUAMS3 (Kansai University Autonomous Measurement System 3) ROS 2パッケージ

## 目次
<!-- TOC -->

- [概要](#概要)
- [ハードウェア & 開発環境](#ハードウェア--開発環境)
- [パッケージ構成](#パッケージ構成)
- [インストール方法](#インストール方法)
- [使用方法](#使用方法)

<!-- /TOC -->

## 概要
ROS 2とNavigation2を用いてナビゲーションを行うためのパッケージを提供します。
実機には関西大学 計測システム研究室が制作するKUAMS3を使用します。

## ハードウェア & 開発環境
- ハードウェア
    - メカナムローバーVer.3.0:
        - 製品ページ: [https://www.vstone.co.jp/products/wheelrobot/ver.3.0_mecanum.html](https://www.vstone.co.jp/products/wheelrobot/ver.3.0_mecanum.html)
        - 販売ページ: [https://www.vstone.co.jp/robotshop/index.php?main_page=product_info&products_id=5345](https://www.vstone.co.jp/robotshop/index.php?main_page=product_info&products_id=5345)
    - Livox Mid-360:
        - 製品ページ: [https://www.livoxtech.com/jp/mid-360](https://www.livoxtech.com/jp/mid-360)

- 開発環境
    - Ubuntu Linux - Jammy Jellyfish (22.04)
    - ROS 2 Humble Hawksbill

## パッケージ構成
- `kuams3` : KUAMS3のメタパッケージ
- `kuams3_bringup` : KUAMS3、各種センサの起動を行うためのlaunchファイルを提供するパッケージです。
- `kuams3_description` : KUAMS3の物理モデルやURDFモデル、Meshファイルを含むパッケージです。
- `kuams3_navigation` : Nav2の起動を行うためのlaunchファイルを提供するパッケージです。
- `kuams3_teleop` : Joyコントローラを使用してKUAMS3を操作するためのコード、launchファイルを提供するパッケージです。

## インストール方法  
以下の手順に沿って各パッケージのインストールを行ってください  
1. **ROS 2 Humble**のセットアップ:  
   [こちら](https://docs.ros.org/en/humble/Installation.html)の手順に従って、ROS 2 Humbleをインストールしてください。

2. [**kuams3**](https://github.com/kzm784/kuams3) のセットアップ:
   ```bash
   mkdir -p ~/kuams3_ws/src
   cd ~/kuams3_ws/src
   git clone https://github.com/kzm784/kuams3.git
   cd ~/kuams3_ws
   rosdep update && rosdep install --from-paths src --ignore-src -y
   colcon build
    ```

3. [**micro-ROS**](https://micro.ros.org/) Agent のセットアップ:
   ```bash
   cd ~/kuams3_ws/src
   git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git
   cd ~/kuams3_ws
   rosdep update && rosdep install --from-paths src --ignore-src -y
   colcon build
   source install/local_setup.bash

   ros2 run micro_ros_setup create_agent_ws.sh
   ros2 run micro_ros_setup build_agent.sh
   source install/local_setup.bash
    ```
5. [**Livox SDK2**](https://github.com/Livox-SDK/Livox-SDK2.git) のセットアップ:
   ```bash
   git clone https://github.com/Livox-SDK/Livox-SDK2.git
   cd ~/Livox-SDK2/
   mkdir build
   cd build
   cmake .. && make -j4
   sudo make install
   ```

4. [**livox_ros_driver2**](https://github.com/Livox-SDK/livox_ros_driver2) のセットアップ:
   ```bash
   cd ~/kuams3_ws/src
   git clone https://github.com/Livox-SDK/livox_ros_driver2.git
   mv livox_ros_driver2/package_ROS2.xml livox_ros_driver2/package.xml
   cd ~/kuams3_ws
   colcon build --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION="ROS2" -DHUMBLE_ROS="humble" --symlink-install
    ```
## 使用方法
- **kuams3の起動**:    
    ⚠️ **注意**: 初期セットアップ時にudevルールを編集し、USBデバイスのシンボリックリンクを作成してください。
    ```bash
    sudo mv ~/kuams3_ws/src/kuams3/docs/99-mecanumrover3-serial.rules /etc/udev/rules.d/
    sudo udevadm control --reload
    ```
    メカナムローバーVer.3.0とPCをシリアル接続し、以下のコマンドを実行して各種センサを含むkuams3の実機を起動します。
    ```bash
    cd kuams3_ws
    source install/setup.bash
    ros2 launch kuams3_bringup kuams3.launch.py
    ```

- **Joyコントローラーを用いたkuams3の操縦**:
    JoyコントローラーとPCを接続した後、以下のコマンドを実行しkuams3を操縦することができます。
    ```bash
    cd kuams3_ws
    source install/setup.bash
    ros2 launch kuams3_teleop kuams3_teleop.launch.py
    ```
    ボタンの割当、速度調節のパラメータは`kuams3_teleop/config/config_kuams3_teleop.yaml`を編集してください。
    ```yaml
    kuams3_teleop:
        ros__parameters:
            axis_linear_x: 1     # 前後方向の速度を操作するジョイスティック軸
            axis_linear_y: 0     # 左右方向の速度を操作するジョイスティック軸
            axis_angular: 3      # 回転速度を操作するジョイスティック軸
            axis_deadman: 5      # デッドマンスイッチのボタン番号
            scale_linear: 0.3    # 直線速度のスケール（最大速度）
            scale_angular: 0.9   # 回転速度のスケール（最大回転速度）
    ```


