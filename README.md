# ICA_Lab_UMI

## ICA_Lab_UMI_Config.yaml 轉移矩陣解釋

```flow
G : 法蘭
C : 手眼相機
E : 夾爪末端點
W : 手臂基座
a : 旋轉過後的apriltag frame
A : dt_apriltag 原始偵測到的apriltag frame

T_A_a: 用於旋轉dt_apriltag偵測的到的tag方向 改為z軸射出紙面 可以直接把小寫a當作 apriltag frame

T_C_G: 手眼標定後得到的相機到法蘭的轉移矩陣 此矩陣會在執行完 hand_eye_calib_offline.py 後被修改 但需要先執行hand_eye_calib.py的main() 來得到pkl檔

T_G_E: 法蘭到夾爪末端的轉移矩陣 z軸比原本官方給的少了0.01m

T_W_a: apriltag到世界的轉移矩陣 此矩陣會在做完 hand_eye_calib.py 的 getTagPos() 後被修改

```