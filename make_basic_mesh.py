"""
髪の毛を作るための点群をCSVファイルに格納する
基本メッシュを作る
"""

#頭全体をつくるコード
#点群を平均化し、ボクセル数のサイズや探索範囲を変更することで、meshの数を減らす
#メッシュもでるから、線をどりだしつないで一本の線にする。

import open3d as o3d
import numpy as np
from scipy.spatial import cKDTree
import csv
from scipy.spatial import distance
import bpy
import time

"""
パラメータ設定：点を幅と髪の毛の本数
つなぐときの点同士の探索範囲とつなぐもの
"""
# 実行時間の計測開始
start_time = time.perf_counter()

#ボクセルを可視化する際につかうもの
def make_voxel(unique_rounded_points, voxel_size):

    cube_side_length = voxel_size
    cubes = []
    for point in unique_rounded_points:
        cube = o3d.geometry.TriangleMesh.create_box(width=cube_side_length, height=cube_side_length, depth=cube_side_length)
        cube.translate(point)
        cubes.append(cube)

    # 一つのメッシュにまとめる
    combined_mesh = o3d.geometry.TriangleMesh()
    for cube in cubes:
        combined_mesh += cube

    # 点群を中心に立方体を作成した結果を表示
    #o3d.visualization.draw_geometries([point_cloud, combined_mesh])
    # 一つのメッシュとして保存
    #o3d.io.write_triangle_mesh(f"./result/voxel_mesh_{voxel_size}.ply", combined_mesh)

#任意の点Cを円状に作成している
def points_on_perpendicular_circle(num_points, center):
    # line_point: 直線上の1点 (x0, y0, z0)
    # line_direction: 直線の方向ベクトル (a, b, c)
    # radius: 円の半径
    # num_points: 円周上の点の数
    #start_pointの値に設定する

    radius = 100
    theta = np.linspace(0, 2 * np.pi, num_points)
    x = center[0] + radius * np.cos(theta)
    y = center[1] * np.ones_like(theta)  # zx平面上に配置するためy座標は一定
    z = center[2] + radius * np.sin(theta)

    # Combine x, y, and z coordinates
    points = np.column_stack((x, y, z))

    # Create an Open3D PointCloud
    circle_point_cloud = o3d.geometry.PointCloud()
    circle_point_cloud.points = o3d.utility.Vector3dVector(points)
    #o3d.io.write_point_cloud("./result/circle.ply",  circle_point_cloud)
    return circle_point_cloud



#三点から平面の式をもとめる
def plane_from_points(point1, point2, point3):
    
    # 三点から法線ベクトルを求める
    normal_vector = np.cross(point2 - point1, point3 - point1)

    # 法線ベクトルを浮動小数点数型に変換
    normal_vector = normal_vector.astype(float)

    # 法線ベクトルを正規化
    normal_vector /= np.linalg.norm(normal_vector)

    # 平面の方程式の係数を取得
    A, B, C = normal_vector
    D = -np.dot(normal_vector, point1)

    plane1 = np.array([A, B, C, D])
    # 平面の方程式を作成
    equation = f"{A:.2f}x + {B:.2f}y + {C:.2f}z - ({D:.2f}) = 0"
    print("平面の方程式:",equation)

    return plane1


#平面沿いにある点群を抽出する
def is_point_between_planes(model, plane1):

    new_cloud = []  

    # 平面の法線ベクトル
    normal_vector_1 = np.array([plane1[0], plane1[1], plane1[2]])
    # 平面上の任意の点
    point_on_plane_1 = np.array([0, 0, -plane1[3] / plane1[2]])
    
    for i in range(len(model.points)):
        point = model.points[i]
        # 点と平面上の点とのベクトル
        vector_to_plane = point - point_on_plane_1
        distance1 = np.abs(np.dot(vector_to_plane, normal_vector_1) / np.linalg.norm(normal_vector_1))
        #print("distance1",distance1, "distance2", distance2)
        
        if distance1 < haba:
            #print("点は2つの平面の間にあります。")
            #model.colors[i] = color_value
            new_cloud.append(point)

    return new_cloud

#メッシュを作るための点群を位置と数の調整　
#一面にまとめ、数を減らす
def project_point_onto_plane(points, plane_coefficients, csv_num):

    #位置調整
    p_projs = []
    A, B, C, D = plane_coefficients

    for i in range(len(points)):
        p = np.array(points[i])
        n = np.array([A, B, C])
        p_proj = p - ((np.dot(p, n) + D) / np.linalg.norm(n)**2) * n
        p_projs.append(p_proj)
    #p_projsは一面にぺったんにしたもの
    point_cloud_array = np.array(p_projs)

    #数を減らす
    #ボクセルにする　#各座標軸ごとに四捨五入
    rounded_points = np.round(point_cloud_array /voxel_size).astype(int)*voxel_size
    
    # 複数あるものをひとつにする
    unique_cells = np.unique(rounded_points, axis=0)

    # 各セルごとに入っている点の平均を出しそれを保存している
    averaged_points = []
    for cell in unique_cells:
        cell_points = point_cloud_array[np.all(rounded_points == cell, axis=1)]
        averaged_point = np.mean(cell_points, axis=0)
        averaged_points.append(averaged_point)

    relation_hair_point(averaged_points, csv_num)
       

#　ポリゴンを作るための繋がりを取得
# 頂点から髪の毛の近い者同士の関係を保存する
def relation_hair_point(points, bone_number):

    #boneのpoint座標をimport
    #頭頂部（スタート地点）
    start_point = np.array([-10, 130, 30], dtype=float)
    min_dist = float('inf')
    closest_point = None

    #ポイントの数
    point_number = len(points)
    point_number /= 2
    point_number = round(point_number)

    # 最初の最も近い点を見つける
    for point in points:
        dist = distance.euclidean(start_point, point)
        if dist < min_dist:
            min_dist = dist
            closest_point = point
            #一番上だけ違う名前をつける
            top_point = point

    # 使用済みのポイントを保存するリストを初期化
    head_point = closest_point
    used_points = [head_point]

    # boneの接続点を格納するリスト
    hair_csv = []


    # 近くの点を見つけて連結する
    for _ in range(point_number):
        min_dist = float('inf')
        closest_point = None
        for other_point in points:
            if not any(np.array_equal(other_point, used_point) for used_point in used_points):
                dist = distance.euclidean(head_point, other_point)
                if dist < min_dist:
                    min_dist = dist
                    closest_point = other_point

        if closest_point is not None and min_dist < 50:
            hair_csv.append((head_point, closest_point))
            #head pointの色を作成
            #color_csv.append(head_point.colors)
            head_point = closest_point
            used_points.append(closest_point)

    print(f"Number of used points: {len(used_points)}")

    #反対側の髪の毛を作成
    hair_csv_2 = []

    # 2つ目のセットの開始点を見つける
    min_dist = float('inf')
    closest_point = None
    for point in points:
        if not any(np.array_equal(point, used_point) for used_point in used_points):
            dist = distance.euclidean(start_point, point)
            if dist < min_dist:
                min_dist = dist
                closest_point = point

    head_point2 = top_point
    used_points.append(head_point2)

    # 2つ目のセットの連結
    for _ in range(point_number):
        min_dist = float('inf')
        closest_point = None
        for other_point in points:
            if not any(np.array_equal(other_point, used_point) for used_point in used_points):
                dist = distance.euclidean(head_point2, other_point)
                if dist < min_dist:
                    min_dist = dist
                    closest_point = other_point

        
        if closest_point is not None and min_dist < 50:
            hair_csv_2.append((head_point2, closest_point))
            #color_csv_2.append(head_point2.colors)
            head_point2 = closest_point
            used_points.append(closest_point)
    bone_number_1 = bone_number * 2
    create_mesh(hair_csv)
    plus_hair(hair_csv)
    create_mesh(hair_csv_2)
    plus_hair(hair_csv_2)


def plus_hair(csv):

    file_xyz = []
    for row in csv:
            if len(row) >= 3:  # 最低3つの値がある場合
                x, y, z = float(row[0]), float(row[1]), float(row[2])
                file_xyz.append([x, y, z])


#relationをもとに、basic meshを作る
def create_mesh(hair_relations):
    # メッシュを作成
    mesh = bpy.data.meshes.new(name="Mesh")
    obj_mesh = bpy.data.objects.new("Mesh_Object", mesh)
    bpy.context.collection.objects.link(obj_mesh)
    
    verts = []
    faces = []
    #メッシュの平均値を出すためにここに格納する
    hair_start_point_data = []

    #size = 0.1

    for row in hair_relations:
        hair_start_point = row[0]
        hair_end_point = row[1]
                 
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


    # アーマチュアの解除
    bpy.context.view_layer.objects.active = None
    for obj in bpy.context.selected_objects:
        obj.select_set(False)


    return obj_mesh

#入力(自身で設定)　1.メッシュの本数　2.頭頂部の設定　3.結び目　4.　
hair_number = 500 #髪の毛の本数
start_point = np.array([-10, 130, 30]) #頭頂部
goal_point =np.array([1, -50, -30]) #結び目
size = 1

#きれいなメッシュができない場合のパラメータ調整
voxel_size = 17
haba = 1 

#頭部の点群データモデルを読み込む
model_path = "C:\\j220139\\master_1\\head_point_kansei\\model\\hair_only.ply"
point_cloud = o3d.io.read_point_cloud(model_path)

#全方向に髪の毛を作るための点の位置
circle_points = points_on_perpendicular_circle(hair_number, start_point)

combined_meshes = []
csv_num = 0
for i in range(len(circle_points.points)): 
    
    kari_point =  circle_points.points[i]

    #3点から平面を求める。
    plane1 = plane_from_points(point1=start_point, point2=goal_point, point3=kari_point) 
    
    #作成した平面付近の点を抽出
    new_cloud = is_point_between_planes(point_cloud, plane1)

    #メッシュをつくるための点群の位置と量を調整
    project_point_onto_plane(new_cloud, plane1, csv_num)
    csv_num += 1 

    print(i,"番目のメッシュ作成")


#終了時刻
end_time = time.perf_counter()
# 実行時間の表示
execution_time = end_time - start_time
print(f"Bone hierarchy created successfully in {execution_time:.2f} seconds")
