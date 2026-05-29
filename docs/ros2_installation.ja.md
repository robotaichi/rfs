# ROS2 インストール方法

このガイドでは、Ubuntu 24.04 に ROS2 Jazzy Jalisco をインストールし、セットアップする方法について説明します。

## インストールの前準備
必要となるパッケージのインストールとリポジトリの追加を行う．
```bash
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
```
続いて，ROS2のリポジトリの暗号鍵の取得とリポジトリの追加を行う．
```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## ROS2のインストール
以下のようにしてROS2をインストールする．
```bash
sudo apt update
sudo apt install ros-jazzy-desktop
```

## ROS2の開発ツールのインストール
ROS2では，ビルドツールとしてcolconが用いられる．
このツールなどを以下のようにインストールする．
```bash
sudo apt update && sudo apt install ros-dev-tools
```

## 環境変数の設定
以下のようにして環境変数を追加する．
```bash
cd
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## ROS2の初期設定
以下のようにして初期設定を行う．
```bash
sudo rosdep init
rosdep update
```

## RFSのワークスペースのセットアップ
本プロジェクトでは、クローンした `rfs` リポジトリ自体がROS2ワークスペースとなります。
以下のようにして、リポジトリをクローンしてビルドします。
```bash
# ホームディレクトリにリポジトリをクローン
cd ~
git clone https://github.com/robotaichi/rfs.git
cd rfs

# ワークスペースをビルド
colcon build
```
RFSのパッケージ群は `src` ディレクトリ以下に配置されていますので、そのまま開発を進めていくことができます。

## 細かい設定
このままでも開発はできるのだが，新しいターミナルを開く度にsetup.bashが呼ばれていたほうがいいので，以下のコマンドを実行する（ユーザ名が変わっても対応できるよう、`$HOME/rfs` または `~/rfs` を使用します）。
```bash
echo "source \$HOME/rfs/install/setup.bash" >> ~/.bashrc
```
これで，ターミナルを開いたタイミングではsrc以下でビルド済みのパッケージの参照ができるようになる．
