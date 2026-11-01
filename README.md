# 画材の使い分けを支援する卓上ロボットの試作

Physical AI 最終課題 / OmniCampus ID: 2572

ROS 2 Humble / Gazebo Fortress / SO-101 followerを用い、置き場所と向きが変わった画材を新しい画像から取り出す動作を検証した試作です。

## 実行方法

[起動・実行手順](project/ros2_gazebo/README.md)を参照してください。

## 構成と検証範囲

- 日本語の画材選択：学習済みの文字特徴分類器。
- 位置・向き：画像の色と形から計算。
- 搬送：逆運動学とあらかじめ決めた手順で制御。
- Gazebo上の位置情報も動作中の監視と成否判定に使用。

箱形の3点を、重ならず十分に離した配置で確認しています。多数の画材の連続した使い替え、実物・実機での動作、利用者への効果は未検証です。ロボットの返却先は固定し、利用者による置き直しは停止中の評価用操作で再現しています。

[最終レポート本文](report/SUBMISSION_FINAL.md) / [修正後の結果](report/REVIEW_FIX_RESULTS.md) / [初期設定の試験結果](report/ROBUSTNESS_RESULTS.md)

preparation/とSUBMISSION_DRAFT.mdは開発時の記録です。再現確認済みZIPを基に、公開用READMEと最終レポート本文を追加・更新しています。ロボット制御コードは変更していません。

SO-101モデルの出典・固定コミット・Apache-2.0ライセンスは[vendor/](project/ros2_gazebo/so101/vendor/)に収録しています。これらの原著者の権利は元のライセンスに従います。
