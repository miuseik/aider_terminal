#### 查看所有电机状态
```
python examples/debug/motors.py get_motors_states \
  --port /dev/ttyACM0
```

#### 仅控制移动底座
```
python examples/debug/wheels.py \
   --port /dev/ttyACM0
```

#### 仅控制升降轴
```
python examples/debug/axis.py \
   --port /dev/ttyACM0
```

#### 禁用所有机械臂电机的扭矩
```
python examples/debug/motors.py reset_motors_torque  \
  --port /dev/ttyACM0
```

#### 通过 ID 旋转特定电机
```
python examples/debug/motors.py move_motor_to_position \
  --id 1 \
  --position 2 \
  --port /dev/ttyACM1
```


#### 设置新的电机 ID
```
python examples/debug/motors.py configure_motor_id \
  --id 10 \
  --set_id 8 \
  --port /dev/ttyACM0
```


#### 将当前位置重置为电机中点
```
python examples/debug/motors.py reset_motors_to_midpoint \
  --port /dev/ttyACM1
```


#### 在机械臂上执行动作脚本
```
python examples/debug/motors.py move_motors_by_script \
   --script_path action_scripts/test_dance.txt  \
   --port /dev/ttyACM0
```


