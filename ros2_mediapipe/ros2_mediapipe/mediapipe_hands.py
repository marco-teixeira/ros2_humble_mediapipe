import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ros2_mediapipe_msgs.msg import MediapipePose
from ros2_mediapipe_msgs.msg import MediapipeHands

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from geometry_msgs.msg import Point 


import cv2
import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands


class MediapipeHandsROS2(Node):
   
    
    def __init__(self):
        super().__init__('ros2_mediapipe_hands')
        
        self.bridge = []
        self.bridge = CvBridge()
        
        #publisher
        self.publisher_2D_ = self.create_publisher(MediapipeHands, 'ros2_mediapipe/mediapipe_hands/normalized_image_2D', 10)
        self.publisher_3D_ = self.create_publisher(MediapipeHands, 'ros2_mediapipe/mediapipe_hands/points_3D_coordinates', 10)
        self.publisher_pixel_ = self.create_publisher(MediapipeHands, 'ros2_mediapipe/mediapipe_hands/pixel', 10)
        self.publisher_image_ = self.create_publisher(Image, 'ros2_mediapipe/mediapipe_hands/image', 10)
        
        self.publisher_marker_ = self.create_publisher(MarkerArray,'ros2_mediapipe/mediapipe_hands/normalized_image_2D_MarkerArray',10)
        self.publisher_marker_ = self.create_publisher(MarkerArray,'ros2_mediapipe/mediapipe_hands/normalized_image_2D_MarkerArray',10)
        
        self.publisher_marker_ = self.create_publisher(MarkerArray,'ros2_mediapipe/mediapipe_hands/normalized_image_2D_MarkerArray',10)
        self.publisher_marker_3D_ = self.create_publisher(MarkerArray,'ros2_mediapipe/mediapipe_hands/points_3D_coordinates_world_MarkerArray',10)
        
        
        #Subscription
        self.subscription = self.create_subscription(
            Image,
            'ros2_mediapipe/input/image_raw',
            self.listener_callback,
            0)
        self.subscription  # prevent unused variable warning
        

    def listener_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.apply_mediapipe(image)

        
    def apply_mediapipe(self, image):
        with mp_hands.Hands(
        model_complexity=0,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as hands:
             # pass by reference.
            image.flags.writeable = False
            results = hands.process(image)
            
            self.publish_results(results, image)

            
              
    def publish_results(self, results, image):    
            
            MediapipeHands_2D = MediapipeHands() 
            left_hand_points_2D = []
            right_hand_points_2D = []
            
            MediapipeHands_3D = MediapipeHands() 
            left_hand_points_3D = []
            right_hand_points_3D = []
            
            MediapipeHands_pixel = MediapipeHands() 
            left_hand_points_pixel = []
            right_hand_points_pixel = []
            
            height = image.shape[1]
            width = image.shape[0]
            
           
                      
            if results.multi_handedness != None: 
                    cont_for = 0
                    for i in range(0,len(results.multi_handedness)):
                        cont = 0
                        hand_points =[]
                        hand_points_3D =[]
                        hand_points_pixel = []
                        for j in range(0,len(results.multi_hand_landmarks[i].landmark)):
                            mp = MediapipePose()
                            mp.id = j
                            mp.x = results.multi_hand_landmarks[i].landmark[j].x
                            mp.y = results.multi_hand_landmarks[i].landmark[j].y
                            mp.z = results.multi_hand_landmarks[i].landmark[j].z
                            hand_points.append(mp)
                            
                            
                            mp3D = MediapipePose()
                            mp3D.id = j
                            mp3D.x = results.multi_hand_world_landmarks[i].landmark[cont].x
                            mp3D.y = results.multi_hand_world_landmarks[i].landmark[cont].y
                            mp3D.z = results.multi_hand_world_landmarks[i].landmark[cont].z
                            hand_points_3D.append(mp3D)
                            
                            mpPixel = []
                            mpPixel = MediapipePose()
                            mpPixel.id = j
                            mpPixel.x = results.multi_hand_landmarks[i].landmark[j].x*height
                            mpPixel.y = results.multi_hand_landmarks[i].landmark[j].y*width
                            mpPixel.z = 0.0
                            hand_points_pixel.append(mpPixel)
                            
                            cont += 1 
                            
                           
                        if results.multi_handedness[i].classification[0].label == "Left":
                            left_hand_points_2D = []
                            left_hand_points_2D = hand_points
                            left_hand_points_3D = []
                            left_hand_points_3D = hand_points_3D
                            left_hand_points_pixel = []
                            left_hand_points_pixel = hand_points_pixel
                        elif results.multi_handedness[i].classification[0].label == "Right":
                            right_hand_points_2D = []
                            right_hand_points_2D =  hand_points
                            right_hand_points_3D = []
                            right_hand_points_3D =  hand_points_3D
                            right_hand_points_pixel = []
                            right_hand_points_pixel = hand_points_pixel
                
           
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if results.multi_hand_landmarks:
              for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    image,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style())  
    

            for i in (right_hand_points_pixel):
                cv2.putText(image,str(i.id),(int(i.x),int(i.y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,100,200), 2)
        
            for i in (left_hand_points_pixel):
                cv2.putText(image,str(i.id),(int(i.x),int(i.y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,100,200), 2)   

          
            MediapipeHands_2D.left_hand_landmarks = left_hand_points_2D   
            MediapipeHands_2D.right_hand_landmarks =  right_hand_points_2D
            self.publisher_2D_.publish(MediapipeHands_2D)     
            
            
            #Plotting in 3D in rviz, using markers
            self.rviz_3d_hand(left_hand_points_2D, right_hand_points_2D, self.publisher_marker_, 0.05)
            self.rviz_3d_hand(left_hand_points_3D, right_hand_points_3D, self.publisher_marker_3D_, 0.01)
            
            
            MediapipeHands_3D.left_hand_landmarks = left_hand_points_3D   
            MediapipeHands_3D.right_hand_landmarks =  right_hand_points_3D
            self.publisher_3D_.publish(MediapipeHands_3D)  
            
            MediapipeHands_pixel.left_hand_landmarks = left_hand_points_pixel   
            MediapipeHands_pixel.right_hand_landmarks =  right_hand_points_pixel
            self.publisher_pixel_.publish(MediapipeHands_pixel)  
             
            self.publisher_image_.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
            
            
                  

    def rviz_3d_hand(self, left_hand_points, right_hand_points, pub, factor):
        markerArray = MarkerArray()
        
        
        if (left_hand_points!= []):
            for i in range(0,len(left_hand_points)):
                marker = Marker()
                marker.lifetime = 	Duration(seconds=1).to_msg()
                marker.header.frame_id = "/ros2_mediapipe"
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = factor
                marker.scale.y = factor
                marker.scale.z = factor
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = left_hand_points[i].x
                marker.pose.position.y = left_hand_points[i].y
                marker.pose.position.z = left_hand_points[i].z
                
                markerArray.markers.append(marker)
            
            
            marker_links = self.add_links_markers(left_hand_points,(factor/2))
            markerArray.markers.append(marker_links)
        
        if (right_hand_points!= []):    
            for i in range(0,len(right_hand_points)):
                marker = Marker()
                marker.lifetime = 	Duration(seconds=1).to_msg()
                marker.header.frame_id = "/ros2_mediapipe"
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = factor
                marker.scale.y = factor
                marker.scale.z = factor
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = right_hand_points[i].x
                marker.pose.position.y = right_hand_points[i].y
                marker.pose.position.z = right_hand_points[i].z
                
                markerArray.markers.append(marker)   
                
            marker_links = self.add_links_markers(right_hand_points,(factor/2))
            markerArray.markers.append(marker_links)
       
            
        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1
     
                
        pub.publish(markerArray)
        
    def add_links_markers(self,points, factor):
            
            marker = Marker()
            marker.lifetime = 	Duration(seconds=1).to_msg()
            marker.header.frame_id = "/ros2_mediapipe"
            marker.type = 5
            marker.action = marker.ADD
            marker.scale.x = factor
            marker.scale.y = factor
            marker.scale.z = factor
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            
            for i in range(0,4):
                p = Point()
                p.x = points[i].x;
                p.y = points[i].y;
                p.z = points[i].z;
                marker.points.append(p) 
                
                p = Point()
                p.x = points[i+1].x;
                p.y = points[i+1].y;
                p.z = points[i+1].z;
                marker.points.append(p) 
               
            for i in range(5,8):
                p = Point()
                p.x = points[i].x;
                p.y = points[i].y;
                p.z = points[i].z;
                marker.points.append(p)
                
                p = Point()
                p.x = points[i+1].x;
                p.y = points[i+1].y;
                p.z = points[i+1].z;
                marker.points.append(p) 
                
                
            for i in range(9,12):
                p = Point()
                p.x = points[i].x;
                p.y = points[i].y;
                p.z = points[i].z;
                marker.points.append(p)
                
                p = Point()
                p.x = points[i+1].x;
                p.y = points[i+1].y;
                p.z = points[i+1].z;
                marker.points.append(p)     
                
            for i in range(13,16):
                p = Point()
                p.x = points[i].x;
                p.y = points[i].y;
                p.z = points[i].z;
                marker.points.append(p)
                
                p = Point()
                p.x = points[i+1].x;
                p.y = points[i+1].y;
                p.z = points[i+1].z;
                marker.points.append(p)   
                
            for i in range(17,20):
                p = Point()
                p.x = points[i].x;
                p.y = points[i].y;
                p.z = points[i].z;
                marker.points.append(p)
                
                p = Point()
                p.x = points[i+1].x;
                p.y = points[i+1].y;
                p.z = points[i+1].z;
                marker.points.append(p)           
            
            position = [0,5,9,13,17,0]         
            for i in range(0,len(position)-1):
                p = Point()
                p.x = points[position[i]].x;
                p.y = points[position[i]].y;
                p.z = points[position[i]].z;
                marker.points.append(p)
                
                p = Point()
                p.x = points[position[i+1]].x;
                p.y = points[position[i+1]].y;
                p.z = points[position[i+1]].z;
                marker.points.append(p)    
                
            return marker          
                     
    
    
    
def main(args=None):
    rclpy.init(args=args)
    mediapipe_hands = MediapipeHandsROS2()
    rclpy.spin(mediapipe_hands)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
