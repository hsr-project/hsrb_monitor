Overview
++++++++

提供機能
--------

- HSR-Bの各関節や各種センサのダイアグを購読して、状態表示用のLEDを点灯させるROSノード。
- 起動時には、LEDをアプリケーション起動中の色で点灯させる。
- 1つでもエラーとなっている関節やセンサがあった場合は、LEDをエラー発生時の色で点灯させる。

ROS Interface
++++++++++++++

Nodes
-----

- **status_led** 状態表示LED点灯ノード

Subscribed Topics
^^^^^^^^^^^^^^^^^

- **/diagnostics_agg** (:ros:msg:`diagnostic_msgs/DiagnosticArray`) ROS Diagnostics仕様に従った情報

Published Topics
^^^^^^^^^^^^^^^^

- **command_status_led_rgb** (:ros:msg:`std_msgs/ColorRGBA`) 状態表示LEDの色

    デフォルトの状態表示LEDの色

    正常時:青緑(ColorRGBA(g=1.0, b=1.0))

    アプリケーション起動中：黄(ColorRGBA(r=1.0, g=1.0))

    エラー発生時:赤(ColorRGBA(r=1.0))

Parameter
^^^^^^^^^

- **~publish_rate** (float64: 1.0) 状態表示LEDの出版周期[Hz]

- **~update_timeout**  (float64: 5.0) トピックが発行されていなと判断する時間[s]

- **~status_boot_timeout** (float64: 60.0) アプリケーション起動中表示のタイムアウト時間[s]

- **~ok_color**  (associative array: {'g':1.0, 'b':1.0}) 正常時の状態表示LEDの色

'r'または'g'または'b'のキーを持つ辞書.指定可能な値の範囲はそれぞれ[0.0, 1.0]

- **~boot_color**  (associative array: {'r':1.0, 'g':1.0}) アプリケーション起動中の状態表示LEDの色

'r'または'g'または'b'のキーを持つ辞書.指定可能な値の範囲はそれぞれ[0.0, 1.0]

- **~error_color**  (associative array: {'r':1.0}) エラー発生時の状態表示LEDの色

'r'または'g'または'b'のキーを持つ辞書.指定可能な値の範囲はそれぞれ[0.0, 1.0]

- **~error_blinking_period**  (float64: 0.0) エラー発生時の点滅周期[s]， 0または負なら点滅させない

- **~ignored_diagnostics**  (list of string: []) 無視するDiagnosticsの名前


**補足**

LEDの色と対応するColorRGBAデータ

======== =====================
LEDの色  ColorRGBA
======== =====================
消灯     (r=0.0, g=0.0, b=0.0)
赤       (r=1.0, g=0.0, b=0.0)
緑       (r=0.0, g=1.0, b=0.0)
黄       (r=1.0, g=1.0, b=0.0)
青       (r=0.0, g=0.0, b=1.0)
紫       (r=1.0, g=0.0, b=1.0)
青緑     (r=0.0, g=1.0, b=1.0)
白       (r=1.0, g=1.0, b=1.0)
======== =====================
