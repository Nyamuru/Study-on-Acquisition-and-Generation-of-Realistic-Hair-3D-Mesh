## 目的
独立した髪の毛の形状を再現を目指しています。

## 概要


## それぞれのコード説明

### 1.make_hair_only.py
髪の毛のみを抽出するコードです。
髪の毛ありの頭部のモデルと頭部の毛量を抑えた2つのモデルを位置合わせを行い、差分から髪の毛のみを抽出します。

### 2.make_basic_hair.py
1で得た髪の毛のみの点群をもとに、ベースメッシュを作成します。

### 3.increase_mesh.py
毛量によるスキャンで得たベースメッシュと頭皮の間の空洞を、埋めてより実物に近い作りにするためのものです。
具体的には、ベースメッシュを複製し間の空洞に配置します。

