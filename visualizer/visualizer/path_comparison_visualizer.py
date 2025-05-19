import rclpy  
from rclpy.node import Node  
import numpy as np  
from sensor_msgs.msg import Image  
from nav_msgs.msg import Path, Odometry  
from geometry_msgs.msg import PoseStamped  
from visualization_msgs.msg import Marker, MarkerArray  
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  
import math  
import io  
import cv2  
from cv_bridge import CvBridge  
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas  
from matplotlib.figure import Figure  
import yaml  

# f110_msgsがROS2に移植されている場合  
try:  
    from f110_msgs.msg import WpntArray, Wpnt  
    F110_MSGS_AVAILABLE = True  
except ImportError:  
    F110_MSGS_AVAILABLE = False  
    print("f110_msgs not available, using nav_msgs.Path instead")  


class PathComparisonVisualizer(Node):  
    def __init__(self):  
        super().__init__('path_comparison_visualizer')  

        # QoSプロファイルの設定  
        qos = QoSProfile(  
            reliability=ReliabilityPolicy.RELIABLE,  
            history=HistoryPolicy.KEEP_LAST,  
            depth=10  
        )  

        # 基本パラメータ  
        self.declare_parameter('max_path_length', 1000)  
        self.declare_parameter('comparison_frequency', 10.0)  
        self.declare_parameter('waypoints_dist', 0.1)  # ウェイポイント間の距離[m]  

        # セクター設定ファイルのパラメータ  
        self.declare_parameter('sector_config_file', '/home/mana/ws/src/race_stack/stack_master/maps/GLC_smile_small/speed_scaling.yaml')  # セクター設定ファイルのパス  

        # パラメータの取得  
        self.max_path_length = self.get_parameter('max_path_length').value  
        self.comparison_frequency = self.get_parameter('comparison_frequency').value  
        self.waypoints_dist = self.get_parameter('waypoints_dist').value  

        # セクター情報の読み込み  
        self.sectors = {}  
        self.num_sectors = 0  
        self.load_sector_config()  

        # セクター関連のデータ  
        self.current_sector = 0  
        self.lap_count = 0  
        self.cur_s = 0.0  # 現在のs座標（フレネット座標系）  

        # セクターごとの誤差データ  
        self.sector_errors = {i: [] for i in range(self.num_sectors)}  
        self.lap_errors = []  # 各周回の誤差を格納  
        self.current_lap_errors = [[] for _ in range(self.num_sectors)]  

        # 実際の走行経路を記録するための配列  
        self.actual_path = []  

        # 最新の計画経路  
        self.planned_path = None  

        # サブスクライバー  
        if F110_MSGS_AVAILABLE:  
            self.planned_path_sub = self.create_subscription(  
                WpntArray,  
                '/local_waypoints',  
                self.planned_path_callback,  
                qos  
            )  
        else:  
            self.planned_path_sub = self.create_subscription(  
                Path,  
                '/local_waypoints',  
                self.planned_path_callback,  
                qos  
            )  

        self.actual_pose_sub = self.create_subscription(  
            PoseStamped,  
            '/car_state/pose',  
            self.actual_pose_callback,  
            qos  
        )  

        self.frenet_pose_sub = self.create_subscription(  
            Odometry,  
            '/car_state/frenet/odom',  
            self.frenet_pose_callback,  
            qos  
        )  

        # パブリッシャー  
        self.path_marker_pub = self.create_publisher(  
            MarkerArray,  
            '/path_comparison/markers',
            qos  
        )  

        self.error_marker_pub = self.create_publisher(  
            MarkerArray,  
            '/path_comparison/error',  
            qos  
        )  

        self.graph_image_pub = self.create_publisher(  
            Image,  
            '/path_comparison/error_graph',  
            qos  
        )   

        # タイマー  
        self.timer = self.create_timer(  
            1.0/self.comparison_frequency,  
            self.compare_and_visualize  
        )  

        self.graph_timer = self.create_timer(  
            2.0,  # 2秒ごとにグラフを更新  
            self.publish_error_graph  
        )  

        self.get_logger().info("Path comparison visualizer initialized (sector boundary markers disabled).")  

    def load_sector_config(self):  
        """セクター設定ファイルを読み込む"""  
        config_file = self.get_parameter('sector_config_file').value  
        
        if not config_file:  
            self.get_logger().info("Using hardcoded sector configuration")  
            self.sectors = {  
                0: {"start": 0, "end": 31, "scaling": 0.5, "only_FTG": False, "no_FTG": False},  
                1: {"start": 32, "end": 59, "scaling": 0.5, "only_FTG": False, "no_FTG": False},  
                2: {"start": 60, "end": 103, "scaling": 0.5, "only_FTG": False, "no_FTG": False},  
                3: {"start": 104, "end": 131, "scaling": 0.5, "only_FTG": False, "no_FTG": False},  
                4: {"start": 132, "end": 151, "scaling": 0.5, "only_FTG": False, "no_FTG": False},  
                5: {"start": 152, "end": 173, "scaling": 0.5, "only_FTG": False, "no_FTG": False},  
                6: {"start": 174, "end": 202, "scaling": 0.5, "only_FTG": False, "no_FTG": False},  
                7: {"start": 203, "end": 240, "scaling": 0.5, "only_FTG": False, "no_FTG": False},  
                8: {"start": 241, "end": 263, "scaling": 0.5, "only_FTG": False, "no_FTG": False},  
            }  
            self.num_sectors = 9  
        else:  
            try:  
                with open(config_file, 'r') as file:  
                    config = yaml.safe_load(file)  
                    sector_config = config.get('sector_tuner', {}).get('ros__parameters', {})  
                    
                    self.num_sectors = sector_config.get('n_sectors', 0)  
                    self.get_logger().info(f"Loaded {self.num_sectors} sectors from config file")  
                    
                    for i in range(self.num_sectors):  
                        sector_key = f"Sector{i}"  
                        if sector_key in sector_config:  
                            self.sectors[i] = sector_config[sector_key]  
    
            except Exception as e:  
                self.get_logger().error(f"Failed to load sector config: {str(e)}")  
                raise

        self.sector_errors = {i: [] for i in range(self.num_sectors)}
        self.current_lap_errors = [[] for _ in range(self.num_sectors)]
        if self.num_sectors == 0:
             self.get_logger().warn("Number of sectors is 0 after load_sector_config. Path colorization and error graphing by sector might not work correctly.")


    def planned_path_callback(self, msg):  
        """計画経路の更新"""  
        self.planned_path = msg  

    def actual_pose_callback(self, msg):  
        """実際の車両位置を記録"""  
        pose = msg.pose  
        current_x = pose.position.x  
        current_y = pose.position.y  

        # 実際の軌跡に追加 (現在のセクター情報も一緒に記録)
        if self.num_sectors > 0 :
            self.actual_path.append((current_x, current_y, self.current_sector))  
        else: 
            self.actual_path.append((current_x, current_y, 0)) 


        if len(self.actual_path) > self.max_path_length:  
            self.actual_path.pop(0)  

        if self.planned_path:  
            min_dist = self.calculate_min_distance_to_planned_path(current_x, current_y)  
            if min_dist is not None and self.num_sectors > 0 and self.current_sector < len(self.current_lap_errors):  # 配列の範囲チェックを追加
                self.current_lap_errors[self.current_sector].append(min_dist)  

    def frenet_pose_callback(self, msg):  
        """フレネット座標系での位置情報を更新"""  
        self.cur_s = msg.pose.pose.position.x  
        
        if self.num_sectors > 0: # セクター情報が有効な場合のみ実行
            self.detect_sector_from_frenet()  

    def detect_sector_from_frenet(self):  
        """フレネット座標系のs座標を使用してセクターを判定"""  
        if self.waypoints_dist == 0: # ゼロ除算を避ける
            self.get_logger().warn("waypoints_dist is 0, cannot detect sector from Frenet.")
            return

        current_waypoint_idx = self.cur_s / self.waypoints_dist  
        
        previous_sector = self.current_sector  
        
        # セクター判定 (self.sectors が空でないことを確認)
        if not self.sectors:
            # self.get_logger().debug("Sectors dictionary is empty, skipping sector detection.")
            return

        found_sector = False
        for sector_id, sector_info in self.sectors.items():  
            if sector_info.get("start", -1) <= current_waypoint_idx <= sector_info.get("end", -1):  # .getでキー存在確認
                self.current_sector = sector_id  
                found_sector = True
                break  

        if previous_sector != self.current_sector:  
            self.get_logger().info(f"Sector changed: {previous_sector} -> {self.current_sector}")  
            
            if previous_sector > 0 and self.current_sector == 0:  
                self.lap_count += 1  
                self.get_logger().info(f"Lap completed: {self.lap_count}")  
                
                # セクターごとの誤差を更新 (self.current_lap_errors の範囲チェック)
                for sector_idx, errors in enumerate(self.current_lap_errors):  
                    if errors and sector_idx < self.num_sectors : # sector_idx が有効な範囲内か確認
                        if sector_idx in self.sector_errors:
                             self.sector_errors[sector_idx].append(np.mean(errors))  
                        
                self.lap_errors.append(list(self.current_lap_errors))
                self.current_lap_errors = [[] for _ in range(self.num_sectors)]  

    def calculate_min_distance_to_planned_path(self, x, y):  
        min_dist = float('inf')  
        closest_point = None  

        if not self.planned_path: # planned_path が None の場合の早期リターン
            return None

        if F110_MSGS_AVAILABLE:  
            if not hasattr(self.planned_path, 'wpnts') or not self.planned_path.wpnts:  
                return None  
            for wpnt in self.planned_path.wpnts:  
                dist = math.sqrt((wpnt.x_m - x)**2 + (wpnt.y_m - y)**2)  
                if dist < min_dist:  
                    min_dist = dist  
                    closest_point = (wpnt.x_m, wpnt.y_m)  
        else:  
            if not hasattr(self.planned_path, 'poses') or not self.planned_path.poses:  
                return None  
            for pose in self.planned_path.poses:  
                px = pose.pose.position.x  
                py = pose.pose.position.y  
                dist = math.sqrt((px - x)**2 + (py - y)**2)  
                if dist < min_dist:  
                    min_dist = dist  
                    closest_point = (px, py)  
        return min_dist if closest_point else None  

    def compare_and_visualize(self):  
        if not self.planned_path or len(self.actual_path) < 1: 
            return  

        marker_array = MarkerArray()  

        planned_marker = Marker()  
        planned_marker.header.frame_id = "map"  
        planned_marker.header.stamp = self.get_clock().now().to_msg()  
        planned_marker.ns = "planned_path"  
        planned_marker.id = 0  
        planned_marker.type = Marker.LINE_STRIP  
        planned_marker.action = Marker.ADD  
        planned_marker.scale.x = 0.1  
        planned_marker.color.r = 0.0  
        planned_marker.color.g = 1.0  
        planned_marker.color.b = 0.0  
        planned_marker.color.a = 1.0  

        if F110_MSGS_AVAILABLE:  
            if hasattr(self.planned_path, 'wpnts'):
                for wpnt in self.planned_path.wpnts:  
                    p = PoseStamped()  
                    p.pose.position.x = wpnt.x_m  
                    p.pose.position.y = wpnt.y_m  
                    planned_marker.points.append(p.pose.position)  
        else:  
            if hasattr(self.planned_path, 'poses'):
                for pose in self.planned_path.poses:  
                    planned_marker.points.append(pose.pose.position)  
        marker_array.markers.append(planned_marker)  

        # 実際の走行経路のマーカー（セクターごとに色を変える）  
        if self.num_sectors > 0: # num_sectors が 0 の場合は色分けできないためスキップまたは単色で表示
            sector_path_markers = {} 
            for i in range(self.num_sectors):  
                marker = Marker()  
                marker.header.frame_id = "map"  
                marker.header.stamp = self.get_clock().now().to_msg()  
                marker.ns = f"actual_path_sector_{i}"  
                marker.id = 100 + i  # IDが計画経路や他のマーカーと衝突しないように注意
                marker.type = Marker.LINE_STRIP  
                marker.action = Marker.ADD  
                marker.scale.x = 0.1  
                
                hue = i / self.num_sectors if self.num_sectors > 0 else 0 # ゼロ除算回避
                r, g, b = self.hsv_to_rgb(hue, 1.0, 1.0)  
                marker.color.r = r  
                marker.color.g = g  
                marker.color.b = b  
                marker.color.a = 1.0  
                sector_path_markers[i] = marker  

            # 実際の走行経路をセクターごとに分ける (self.actual_path は (x,y,sector_id) のタプルを含む)
            for x, y, sector_id_for_point in self.actual_path:  
                if sector_id_for_point in sector_path_markers:  
                    p = PoseStamped()
                    p.pose.position.x = x  
                    p.pose.position.y = y  
                    sector_path_markers[sector_id_for_point].points.append(p.pose.position)  
            
            for marker in sector_path_markers.values():  
                if marker.points:  
                    marker_array.markers.append(marker)  
        else: # num_sectors が0の場合のフォールバック: 実際のパスを単色で表示
            actual_path_single_marker = Marker()
            actual_path_single_marker.header.frame_id = "map"
            actual_path_single_marker.header.stamp = self.get_clock().now().to_msg()
            actual_path_single_marker.ns = "actual_path_single_color"
            actual_path_single_marker.id = 100 
            actual_path_single_marker.type = Marker.LINE_STRIP
            actual_path_single_marker.action = Marker.ADD
            actual_path_single_marker.scale.x = 0.1
            actual_path_single_marker.color.r = 1.0 
            actual_path_single_marker.color.g = 0.0
            actual_path_single_marker.color.b = 0.0
            actual_path_single_marker.color.a = 1.0
            for x, y, _ in self.actual_path: # sector_id は無視
                p = PoseStamped()
                p.pose.position.x = x
                p.pose.position.y = y
                actual_path_single_marker.points.append(p.pose.position)
            if actual_path_single_marker.points:
                marker_array.markers.append(actual_path_single_marker)


        self.path_marker_pub.publish(marker_array)  

        self.visualize_path_error()  

    def visualize_path_error(self):  
        if not self.planned_path or len(self.actual_path) < 1:  
            return  

        current_x, current_y, current_sector_of_point = self.actual_path[-1]  
        min_dist = self.calculate_min_distance_to_planned_path(current_x, current_y)  

        if min_dist is not None:  
            error_array = MarkerArray()  
            text_marker = Marker()  
            text_marker.header.frame_id = "map"  
            text_marker.header.stamp = self.get_clock().now().to_msg()  
            text_marker.ns = "error_text"  
            text_marker.id = 3 # IDが他のマーカーと衝突しないように注意
            text_marker.type = Marker.TEXT_VIEW_FACING  
            text_marker.action = Marker.ADD  
            text_marker.pose.position.x = current_x  
            text_marker.pose.position.y = current_y  
            text_marker.pose.position.z = 0.5  
            text_marker.scale.z = 0.3  
            text_marker.color.r = 1.0  
            text_marker.color.g = 1.0  
            text_marker.color.b = 1.0  
            text_marker.color.a = 1.0  

            # セクターごとの平均誤差を計算 (current_sector_of_point が有効なキーか確認)
            if self.num_sectors > 0 and current_sector_of_point in self.sector_errors and self.sector_errors[current_sector_of_point]:  
                avg_error = np.mean(self.sector_errors[current_sector_of_point])  
                text_marker.text = f"Sector {current_sector_of_point}\nCurrent: {min_dist:.2f}m\nAvg: {avg_error:.2f}m"  
            else:  
                text_marker.text = f"Current Error: {min_dist:.2f}m" # セクター情報がないか、データがない場合
                if self.num_sectors > 0 :
                     text_marker.text = f"Sector {current_sector_of_point}\nCurrent: {min_dist:.2f}m"


            error_array.markers.append(text_marker)  
            self.error_marker_pub.publish(error_array)  

    def publish_error_graph(self):   

        fig = Figure(figsize=(10, 6))  
        canvas = FigureCanvas(fig)  
        ax = fig.add_subplot(111)  

        sectors_to_plot = sorted([s_id for s_id in self.sector_errors.keys() if s_id < self.num_sectors]) # 有効なセクターIDのみ

        avg_errors = [np.mean(self.sector_errors[s]) if self.sector_errors.get(s) else 0 for s in sectors_to_plot]  
        std_errors = [np.std(self.sector_errors[s]) if self.sector_errors.get(s) and len(self.sector_errors[s]) > 1 else 0 for s in sectors_to_plot]  

        bars = ax.bar(range(len(sectors_to_plot)), avg_errors, yerr=std_errors, capsize=5) # x軸をインデックスに変更

        for i, bar in enumerate(bars):  
            # 色付けはセクターIDに基づいて行う (sectors_to_plot[i] がセクターID)
            hue = sectors_to_plot[i] / self.num_sectors if self.num_sectors > 0 else 0
            r, g, b = self.hsv_to_rgb(hue, 0.8, 0.9)  
            bar.set_color((r, g, b))  

        ax.set_xlabel('Sector')  
        ax.set_ylabel('Average Error (m)')  
        ax.set_title(f'Tracking Error by Sector (Laps: {self.lap_count})')  
        ax.set_xticks(range(len(sectors_to_plot))) # x軸の目盛り位置
        ax.set_xticklabels([f'S{s_id}' for s_id in sectors_to_plot]) # x軸のラベル
        ax.grid(True, linestyle='--', alpha=0.7)  

        for i, v in enumerate(avg_errors):  
            ax.text(i, v + (max(avg_errors) * 0.02 if avg_errors else 0.01) , f'{v:.3f}', ha='center', va='bottom', fontsize=8)  # オフセットを動的に

        fig.tight_layout()  
        canvas.draw()  
        buf = io.BytesIO()  
        canvas.print_png(buf)  
        buf.seek(0)  
        img_array = np.frombuffer(buf.getvalue(), dtype=np.uint8)  
        image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)  
        bridge = CvBridge()  
        msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")  
        msg.header.stamp = self.get_clock().now().to_msg()  
        msg.header.frame_id = "map" # 通常は固定フレーム
        self.graph_image_pub.publish(msg)  

    def hsv_to_rgb(self, h, s, v):  
        if s == 0.0:  
            return v, v, v  
        i = int(h * 6.0)  
        f = (h * 6.0) - i  
        p = v * (1.0 - s)  
        q = v * (1.0 - s * f)  
        t = v * (1.0 - s * (1.0 - f))  
        i %= 6  
        if i == 0: return v, t, p  
        if i == 1: return q, v, p  
        if i == 2: return p, v, t  
        if i == 3: return p, q, v  
        if i == 4: return t, p, v  
        return v, p, q  

def main(args=None):  
    rclpy.init(args=args)  
    visualizer = PathComparisonVisualizer()  
    try:
        rclpy.spin(visualizer)  
    except KeyboardInterrupt:
        pass 
    finally:
        visualizer.destroy_node()  
        rclpy.shutdown()  

if __name__ == '__main__':  
    main()