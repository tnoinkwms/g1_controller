# g1-controller

## Install (editable)
```bash
pip install -e .
```

## AXES INFO
```
# ==== WAIST ====
#12 WAIST_YAW   : +左回り / -右回り                range[-90, +90]    normal=0
#13 WAIST_ROLL  : +左曲げ / -右曲げ(ロボット視点)    range[-20, +20]    normal=0
#14 WAIST_PITCH : +お辞儀 / -反り返り(危ない)        range[0, +30]      normal=0

# ==== LEFT ARM ====
#15 L_SHOULDER_PITCH : +前 / -後ろ                  range[-120, +120]  normal=0
#16 L_SHOULDER_ROLL  : +脇上げ / -脇閉め            range[-30, +110]   normal=0
#17 L_SHOULDER_YAW   : +外回り / -内回り            range[-90, +90]    normal=0
#18 L_ELBOW          : +前方曲げ / -背面曲げ        range[-10, +120]   normal=0
#19 L_WRIST_ROLL     : +外側 / -内側                range[-90, +90]    normal=0
#20 L_WRIST_SIDE(BEND): +前方曲げ / -背面曲げ       range[-90, +90]    normal=0
#21 L_WRIST_YAW      : +外側曲げ / -内側曲げ        range[-90, +90]    normal=0

# ==== RIGHT ARM ====
#22 R_SHOULDER_PITCH : +前 / -後ろ                  range[-120, +120]  normal=0
#23 R_SHOULDER_ROLL  : +脇上げ / -脇閉め            range[-30, +110]   normal=0
#24 R_SHOULDER_YAW   : +外回り / -内回り            range[-90, +90]    normal=0
#25 R_ELBOW          : +前方曲げ / -背面曲げ        range[-10, +120]   normal=0
#26 R_WRIST_ROLL     : +外側 / -内側                range[-90, +90]    normal=0
#27 R_WRIST_SIDE(BEND): +前方曲げ / -背面曲げ       range[-90, +90]    normal=0
#28 R_WRIST_YAW      : +外側曲げ / -内側曲げ        range[-90, +90]    normal=0

```

## how to use
```python
from g1_controller import G1ArmController

arm.start()
arm = G1ArmController()
arm.set_axes({22:90, 15:90, 25:60, 18:60, 28:-30}, 3.0)
arm.set_all_axes_to_zero(2.0)
arm.stop()
