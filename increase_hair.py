"""
前回の４から、goal_point付近であるかどうかを判断する分岐点を作りたい
"""

import bpy
import numpy as np
import open3d as o3d
import csv
import time
from scipy.spatial.transform import Rotation as R
from mathutils import Vector, Matrix

def create_mesh(content, closest_point_inde):
    # メッシュを作成
    mesh = bpy.data.meshes.new(name="Mesh")
    obj_mesh = bpy.data.objects.new("Mesh_Object", mesh)
    bpy.context.collection.objects.link(obj_mesh)
    verts = []
    faces = []

    for i, row in enumerate(content):
        if i >= closest_point_index:
    

            #メッシュの平均値を出すためにここに格納する
            hair_start_point_data = []
            hair_start_point = np.array([float(row[0]), float(row[1]), float(row[2])])
            hair_end_point = np.array([float(row[3]), float(row[4]), float(row[5])])
                    
            size = 1
            #start_pointの位置を格納
            hair_start_point_data.append(hair_start_point)

            point_1 = [hair_start_point[0] + size, hair_start_point[1] + size, hair_start_point[2] + size]
            point_2 = [hair_end_point[0] + size, hair_end_point[1] + size, hair_end_point[2] + size]
            point_3 = [hair_start_point[0] - size, hair_start_point[1] - size, hair_start_point[2] + size]
            point_4 = [hair_end_point[0] - size, hair_end_point[1] - size, hair_end_point[2] + size]
                    
            verts.extend([hair_start_point, point_1, point_2, hair_end_point])
            face_start_index = len(verts) - 4
            faces.append((face_start_index, face_start_index + 1, face_start_index + 2, face_start_index + 3))
                    
            verts.extend([hair_start_point, point_3, point_4, hair_end_point])
            face_start_index = len(verts) - 4
            faces.append((face_start_index, face_start_index + 1, face_start_index + 2, face_start_index + 3))

    # メッシュに頂点と面を設定
    mesh.from_pydata(verts, [], faces)
    mesh.update()

    return obj_mesh



def rotate(obj, point_y, point_x, goal_point):
    if obj is None:
        print("オブジェクトが指定されていません。")
        return
    
    A = Vector(goal_point)
    B = Vector(point_y)
    C = Vector(point_x)

    # 回転軸をつくる
    # 外積を計算（法線ベクトル）
    vector_AB = B - A
    vector_AC = C - A

    # 法線ベクトルを正規化
    axis = vector_AB.cross(vector_AC).normalized()


    # 回転角度
    # ベクトルABとACの内積を計算
    dot_product = vector_AB.dot(vector_AC)

    # ベクトルABとACの大きさを計算
    magnitude_AB = np.linalg.norm(vector_AB)
    magnitude_AC = np.linalg.norm(vector_AC)

    # 角度θを計算（ラジアンで）
    cos_theta = dot_product / (magnitude_AB * magnitude_AC)
    theta_rad = np.arccos(cos_theta)

    #rotation_matrix = Matrix.Rotation(角度, 4, 回転軸)
    rotation_matrix = Matrix.Rotation(theta_rad, 4, axis)
    

    #もとの位置を保存
    orignal = obj.location

    # オブジェクトの位置を移動してAを原点に
    obj.location = -A 

    # オブジェクトを90度回転
    obj.matrix_world = rotation_matrix @ obj.matrix_world

    # オブジェクトの位置を移動してAを原点に
    obj.location = orignal



    print("回転完了")
    

def jage_goal(contents):

    for i, row_2 in enumerate(contents):
        x, y, z = float(row_2[0]), float(row_2[1]), float(row_2[2])
        point = np.array([x, y, z])
        
        # 距離を計算
        distance = np.linalg.norm(point - goal_point)
        
        # 直径10以内にいるかを判断
        if distance <= 30:
            print(f"Point {i} is within diameter 10: {point}, Distance: {distance}")
            #直径１０以内に入る点があれば、回転処理を行う
            return True

def move(obj, point_y, point_x):
    obj.location = point_x  - point_y 

def filtered(point_x, start_point, goal_point):

    distance = np.linalg.norm(start_point - goal_point)/2

    # start_pointからdistanceまでの距離にある点を抽出
    distances = np.linalg.norm(point_x - start_point, axis=1)
    filtered_points = point_x[distances <= distance]
       # PLYファイルとして保存
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(filtered_points)    
    # PLYファイルに保存
    o3d.io.write_point_cloud("filtered_points.ply", point_cloud)

        
    return filtered_points



# ダウンサンプリングされた点群データを読み込む
model_path = "C:\\j220139\\master_1\\head_point_kansei\\model\\bald_model_center.ply"
back_head = o3d.io.read_point_cloud(model_path)
size = np.abs((back_head.get_max_bound() - back_head.get_min_bound())).max() / 35
down_A = back_head.voxel_down_sample(size * 0.5)



# 点群データの座標を取得
points_x_array = np.asarray(down_A.points)


# 2点間のユークリッド距離を計算
start_point = np.array([-10, 130, 30])
goal_point = np.array([1, -10, -30])
distance_a = np.linalg.norm(goal_point - start_point)

#頭の一定の場所で点群を残す
points_x_array = filtered(points_x_array, start_point, goal_point)

#csvファイルをそれぞれ格納する

#各ファイルの内容を格納するリスト
contents = []
contents_xyz = []

# 1から10までのファイルを読み込む
for i in range(0, 399):
    filename = f"C:\j220139\master_1\head_point2\hair_relation\hair_relation_{i}.csv"
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        # ファイルの内容を2次元リストとして追加
        #点のつながりを含めて情報が入っている
        file_content = [row for row in reader]
        contents.append(file_content)

        # 点の位置のみの情報⇒これで近い点を探す⇒点のつながり情報の同じところからメッシュを作る
        # 各ファイルごとの[x, y, z]を格納するリスト
        file_xyz = []  
        for row in file_content:
            if len(row) >= 3:  # 最低3つの値がある場合
                x, y, z = float(row[0]), float(row[1]), float(row[2])
                file_xyz.append([x, y, z])
        contents_xyz.append(file_xyz)


# ここにすべてのメッシュの点と点のつながりを格納
print("5つ目のファイルの2行目:", contents[5][2])
print("5つ目のファイルの2行目:", contents_xyz[5][2])

rotate_number = 0

if len(points_x_array) > 0:
    for i, point_x in enumerate(points_x_array) :
        #print(len(points_x_array))
        #print(i)

        # 縮小率を求める
        distance_b = np.linalg.norm(goal_point - point_x)
        ratio = distance_b / distance_a
        #print("ratio:", ratio)

        #point_xに最も近いcontents_xyz[file_number][x,y,z]を見つける
        min_distance = float('inf')  # 最小距離を無限大で初期化
        closest_point_index = None   # 最も近い点の位置を格納する変数
        closest_file_index = None    # 最も近い点が含まれるファイル番号を格納する変数

    # contents_xyz の各ファイルをチェック
        for file_index, file_xyz in enumerate(contents_xyz):
            #行　列
            for point_index, (x, y, z) in enumerate(file_xyz):
                #列　値（x,y,z)
                point = np.array([x, y, z])
                distance = np.linalg.norm(point_x - point)  # 距離を計算

                if distance < min_distance:
                    min_distance = distance
                    #列
                    closest_point_index = point_index
                    #行
                    closest_file_index = file_index


        #point_y = point_xに近い点が何番目のファイルの何行目か見つけることができた        
        print(f"Point {i} is closest to point {closest_point_index} in file {closest_file_index} with distance {min_distance}")
    
        #メッシュを作成 
        obj = create_mesh(contents[closest_file_index], closest_point_index)


        point_y = contents_xyz[closest_file_index][closest_point_index]


        
        #goal_point付近を通るか通らないか判断
        if jage_goal(contents[closest_file_index]) == True:
            rotate(obj, point_y, point_x, goal_point)
            rotate_number += 1
        
            #そうでなければ平行移動を行う
        else:
            print("むずび目を通りません")
            move(obj, point_y, point_x)     

        # ratioに基づいてオブジェクトのスケールを調整
        #obj.scale = (ratio, ratio, ratio)

    
print(rotate_number)
    