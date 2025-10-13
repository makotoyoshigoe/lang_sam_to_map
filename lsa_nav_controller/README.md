# ros2_pkg_template
## About
- ROS 2パッケージのテンプレート
- パッケージ名の変更が容易

## Dependencies
- ROS 2
- xmllint

## Usage
### Install
```
cd ~/ros2_ws/src
git clone https://github.com/makotoyoshigoe/ros2_pkg_template.git
cd ros2_pkg_template
```

### How to change package name
```
source .setup_own.bash new_package_name
```
- 実行後はCMakeLists.txt、package.xmlのパッケージ名、include/にあるディレクトリの名前が新パッケージ名に変わります

### Example（hogehogeに名前を変える場合）
```
source .setup_own.bash hogehoge
```

