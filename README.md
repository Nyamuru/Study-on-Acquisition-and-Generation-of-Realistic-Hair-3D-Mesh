## 目的
髪の毛は数が多く細かい形状であり、実際の髪の毛を仮想空間上で再現することは難しいとされています。
そこで、３Dスキャナで頭部の立体形状を取得し、頭頂部から毛先まで連続した独立した髪の毛の形状を再現します。

## 概要
![image](https://github.com/user-attachments/assets/5dd13718-fbc7-48b6-99e8-e3faca36b301)

### 1.３Dスキャナで頭部のスキャン
スキャンしたモデルは点群データとして取得されます。頭部を普通にスキャンしたものと、頭部の毛量を抑えた状態のもの２つのスキャナを行います。
### 2.2つのモデルの差分から、髪の毛のみの点群モデルを抽出
モデルAから、モデルB（頭部の毛量を抑えたモデル）と重なる部分を削除します。これにより、髪の毛のみに処理を加えることができます。
### 3.髪の毛のメッシュを作るための点群を抽出
頭頂部と結び目の点は自身で設定する必要があります。
頭頂部と結び目と任意の点の3点を用意することで平面を作り、それをもとに髪の毛の線を抽出します。（現在はストレートの髪の毛のみの対応となります）
### 4.点群の数と位置を調整しメッシュを作成
抽出したままメッシュを作成するときれいにならないので調整を行います。詳しくはコードに書いてあります。
### 5.ベースメッシュの完成
1-4の処理を繰り返すことで、ベースメッシュ（F）が完成します。

## 増毛を行う
毛量による髪の毛の厚みで、スキャンしたモデルの髪の毛と、頭皮には大きな空洞ができます。
ベースモデルと頭皮の間に空洞を埋め、より実物に近いリアルな状態を目指します。

### 6.頭皮から生えるように髪の毛の始点を用意する
毛量を抑えたモデルから髪の毛が生え始める位置を得ます。ダウンサンプリングを行うことで、増毛するための適切な量に設定します。
### 7.始点から近いメッシュを複製
６で用意した始点から最も近くにあるメッシュを複製する。その時、始点から近いポリゴンから毛先までの部分を複製する。
### 8.回転、並行移動、縮小
空洞を埋め、始点（頭皮）から生えているように移動と縮小の処理をする。


## それぞれのコード説明

### 1.make_hair_only.py
髪の毛のみを抽出するコードです。
髪の毛ありの頭部のモデルと頭部の毛量を抑えた2つのモデルを位置合わせを行い、差分から髪の毛のみの点群モデルを抽出します。

### 2.make_basic_hair.py
1で得た髪の毛のみの点群モデルをもとに、ベースメッシュを作成します。

### 3.increase_mesh.py
毛量によるスキャンで得たベースメッシュと頭皮の間の空洞を、埋めてより実物に近い作りにするためのものです。
具体的には、ベースメッシュを複製し間の空洞に配置します。

