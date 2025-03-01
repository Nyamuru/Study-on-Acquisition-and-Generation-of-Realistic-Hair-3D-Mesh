"""
髪の毛ありと髪の毛なしの位置合わせを行い
髪の毛のみを抽出する
それを中心に持っていく

"""

import open3d as o3d
import numpy as np
#同じ部分を抽出
#def show で表示



def show(object1, object2, model_to_scene_trans=np.identity(4)):

        object1.transform(model_to_scene_trans)
        #o3d.io.write_point_cloud("./model/sameposition_bold.ply", object1)

        delete_same_position(object1, object2)


def matching(object_bald, object_hair):

        # いろいろなサイズの元： model点群の1/10を基本サイズsizeにする
        # 方法としては、点群が含まれる最小の矩形領域の10分の１の距離をもとめる。
        size = np.abs((object_hair.get_max_bound() - object_hair.get_min_bound())).max() / 35

        #特徴量計算時の探索範囲を設定するためのパラメータ
        kdt_n = o3d.geometry.KDTreeSearchParamHybrid(radius=size, max_nn=50)
        kdt_f = o3d.geometry.KDTreeSearchParamHybrid(radius= size * 50 , max_nn = 150)

        #法線ベクトルを推定


        # ダウンサンプリング
        print("downsampling")
        down_A = object_bald.voxel_down_sample(size * 0.2)
        down_B = object_hair.voxel_down_sample(size * 0.2)

        #法線ベクトルを推定
        down_A.estimate_normals(kdt_n)
        down_B.estimate_normals(kdt_n)


        # 特徴量計算
        print("feature")
        feature_A = o3d.pipelines.registration.compute_fpfh_feature(down_A, kdt_f)
        feature_B = o3d.pipelines.registration.compute_fpfh_feature(down_B, kdt_f)


        # RANSACマッチングを行う上での条件と設定
        #対応した点の同士の距離とエッジ:無駄をそぎ落とす
        print("ready")
        checker = [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.95),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(size * 2)]
        #点同士の変換行列を推定
        est_ptp = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        #点から平面の変換行列の推定
        est_ptpln = o3d.pipelines.registration.TransformationEstimationPointToPlane()

        #適合しているものを見つける時の条件
        max_iteration=40000
        max_validation=500
        criteria = o3d.pipelines.registration.RANSACConvergenceCriteria(max_iteration, max_validation)

        print("start matching")
        # RANSACマッチング
        result1 = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(down_A, down_B,
                        feature_A, feature_B, 
                        mutual_filter = True,
                        max_correspondence_distance=size * 2,
                        estimation_method=est_ptp,
                        ransac_n=3,
                        checkers=checker,
                        criteria=criteria
                        )

        print("finish")



        # ICPで微修正 上で得た対応する位置にあわせる。
        result2 = o3d.pipelines.registration.registration_icp(down_A, down_B, size * 0.3, result1.transformation, est_ptpln)

        #object_Aをobject_Bと重なるように移動させる。
        #object_A.transform(result2.transformation)


        
        #self.show(extracted_object_A, extracted_object_B, result2.transformation)
        show(object_bald, object_hair, result2.transformation)
        #return extracted_object_B


def delete_same_position(object_bald, object_hair):
            #same_position.pyで近くにあるものを生成し、delete_position.pyで同じ距離のものを削除する
            #できがいまいちで目元がよく消えない

            print("start")
 
            # KDツリーを作成して最近傍点検索を行う
            kdtree_A = o3d.geometry.KDTreeFlann(object_bald)
            threshold_distance = 4

            print("for")
            # 近くにあるポイントを削除する
            far_points = []
            far_points_colors = []
            for i, point in enumerate(object_hair.points):
                [k, idx, _] = kdtree_A.search_radius_vector_3d(point, threshold_distance)
                if len(idx) == 0:
                    far_points.append(point)
                    far_points_colors.append(object_hair.colors[i])


            # 削除されたポイントを持つ新しいポイントクラウドを作成
            far_point_cloud = o3d.geometry.PointCloud()
            far_point_cloud.points = o3d.utility.Vector3dVector(far_points)
            far_point_cloud_colors = o3d.utility.Vector3dVector(far_points_colors)
            far_point_cloud.colors = far_point_cloud_colors


    # ポイントクラウドの色情報を設定
            
            # ポイントクラウドの表示
            #o3d.visualization.draw_geometries([object_hair])

        
            print("finish")
            # 新しいポイントクラウドを保存
            #o3d.io.write_point_cloud("./model/hair_only.ply", far_point_cloud)

            move_center(object_bald, far_point_cloud)


    # 頭のみの点群の中心を取得し、それに合わせて髪の毛のみの点群も移動させる
def move_center(bald, hair_only):

    bald_center = bald.get_center()
    print(f"Original Point Cloud Center: {bald_center}")

    # 平行移動するためのベクトルを計算
    translation_vector = - bald_center

    # 点群を移動
    bald.translate(translation_vector)
    hair_only.translate(translation_vector)

    # 新しい位置に移動した点群モデルを保存
    o3d.io.write_point_cloud("./model/bald_model_center.ply", bald)
    o3d.io.write_point_cloud("./model/hair_only.ply", hair_only)  

object_bald = o3d.io.read_point_cloud("./model/bald_model.ply")
object_hair = o3d.io.read_point_cloud("./model/orignal.ply")
start = matching( object_bald, object_hair)