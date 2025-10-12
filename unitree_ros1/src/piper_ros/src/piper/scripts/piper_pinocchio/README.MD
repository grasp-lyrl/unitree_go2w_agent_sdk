# piper 匹诺曹逆解

## conda环境搭建

```shell
conda create -n tv python=3.9
conda activate tv
conda install pinocchio=3.6.0 -c conda-forge
pip install meshcat
pip install casadi
```

## 运行

运行piper单臂控制节点(也可以单独运行逆解，这一步是用来控制真实机械臂的)

```bash
roslaunch piper start_single_piper.launch
```

直接使用conda的python运行文件

```bash
source /{你的路径}/piper_ws/devel/setup.sh
conda activate tv
python /{你的路径}/piper_ws/src/piper/scripts/piper_pinocchio/piper_pinocchio.py
```
