# 実機テストでのトラブルシューティング<br>
- pigpioを初回時にセットアップしているか<br>
- 電源周りに不具合は起きてないか<br>

# 機能の追加方法<br>
## flipper制御
### コード概要
- flipper_semi_autonomousがmainとなっている<br>
- Rotation.cppでDXL個別制御モード全体制御モードを実装している<br>
- robot_motion/DXL.hで力制御,位置制御などのフィードバック系統を実装している<br>
- robot_motion/semi_autonomous.hで半自動制御モードの実装をしている<br>
機能を追加する場合は追加したい機能が呼び出されるモードのファイルでそれぞれ追加する<br>
### 個別制御モード,全体制御モードの機能追加<br>
フィードバック系の機能を増やしたいのであれば,<br>
DXL::MODEにフィードバック名を追加しRotation.cppでそれぞれの向きに対して実装を追加するまた,
フィードバック系内の機能を追加したいのであればRotation.cppでそれぞれの向きに対して実装を追加する<br>
### 半自動制御モードの機能追加
struct feedBackTypesのメンバ変数を増やして,operator()内で実装を追加する<br>
### 新しいモードの追加
crawler_system/include/robot_motionに.hファイルを作成し,実装をする<br>
flipper_semi_autonomous.cppでそのファイルをincludeする<br>
keyFlagにモード名, while(ros::ok())内のswitch caseに分岐を追加し,呼び出せるようにする


