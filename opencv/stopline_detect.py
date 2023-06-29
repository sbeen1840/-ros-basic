import cv2
import numpy as np
import rospy
from xycar_msgs.msg import Motor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


#%%
'''
Stop Line Detector의 Callback 구성

[1] cvtImg to cv_img
[2] cvtColor to Gray
[3] Gaussian Blur
[4] Canny Edge
[5] ROI
[6] HoughLineP
[7] Filtering
[8] Detection
[9] Draw on original
[10] Show img

'''

#%%

class StopLineDetector:
    
    def __init__(self):
        rospy.init_node('stop_line_detector')
        self.sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
        self.pub = rospy.Publisher('xycar_motor', Motor, queue_size=1)
        self.bridge = CvBridge()
        self.lanedetected = False
        
        
    def callback(self, data):
        # This function is the callback function for the ROS subscriber. 
        # It takes in a single argument data, which is the image data received 
        # from the ROS topic /usb_cam/image_raw. Inside the function, 
        # the image data is converted to an OpenCV image format, and various 
        # image processing techniques are applied to detect the stop line.    

#%%
        'cvt to cv_img'
        # Convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

#%%    
        'cvtColor-Gray'
        # Convert image to grayscale and blur it to reduce noise
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

#%%        
        'GaussianBlur'
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        # (5, 5)
        # This function applies a Gaussian blur with a kernel size of (5, 5) to the grayscale image gray. 
        # The kernel size determines the amount of blurring applied to the image.
        # A larger kernel size will result in more smoothing, 
        # while a smaller kernel size will result in less smoothing.
        
        # 0
        # 수평 방향으로 이미지에 적용되는 스무딩 또는 블러링의 양에 영향을 미칩니다.
        # 표준 편차 값이 클수록 종 모양의 곡선이 더 넓고 평평해져 이미지가 더욱 매끄럽거나 흐려집니다. 
        # 반대로 표준 편차 값이 작을수록 더 좁고 더 큰 종 모양의 곡선이 되어 덜 매끄럽거나 흐려집니다.
        # 1에서 10 사이의 표준 편차 값이 일반적으로 사용
        # 이미지에 보존해야 할 미세한 세부 사항이 많은 경우 더 작은 표준 편차 값이 더 적절
 
#%%
        'Canny Edges'
        edges = cv2.Canny(blurred, 50, 150)
        # Apply Canny edge detection to find edges
        # The two arguments, 50 and 150, are the low and high thresholds for the edges.
        # 50 미만의 기울기 크기를 가진 가장자리 픽셀은 노이즈로 간주되어 폐기
        # 150 이상의 기울기 크기를 가진 가장자리 픽셀은 강한 가장자리로 간주되며 최종 가장자리 맵의 일부로 유지
        # 일반적인 방법은 하한 임계값을 상한 임계값의 1/3로 설정하는 것

#%%
        'ROI영역지정'
        height, width = edges.shape
        roi = np.array([[(0, height), (width/2, height/2), (width, height)]], dtype=np.int32)
        # edges NumPy 배열이어야 
        # roi 관심 영역을 정의하는 정점, 튜플의 목록, 이미지의 꼭지점 좌표(x, y)가 포함
        mask = np.zeros_like(edges) # 입력 이미지와 같은 모양의 검정색 이미지
        cv2.fillPoly(mask, roi, 255) 
        # 정점은 마스크 이미지에서 흰색 픽셀(값 255)로 채워집니다. 
        # 이렇게 하면 관심 영역 내부의 픽셀에 대해 값이 255이고 
        # 영역 외부의 픽셀에 대해 0인 이진 마스크가 생성됩니다.
        masked_edges = cv2.bitwise_and(edges, mask)
        # and연산
        # 관심 영역 외부의 모든 픽셀을 0으로 설정하고 관심 영역 내부의 픽셀을 유지
         
#%%       
        'HoughLinesP'
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, 50, minLineLength=100, maxLineGap=10)
        # Find lines using Hough transform
        
        # 1
        # 감지할 수 있는 선 사이의 거리 단위(픽셀)
        # 거리변환에 사용되는 해상도. 값이 작을수록 짧은 선을 더 정확하게 감지
        
        # np.pi/180 
        # 감지할 수 있는 선 사이의 각도 단위(라디안)
        # [np.pi/180] 값은 각도 단위가 1도임을 의미. 값이 작을수록 각도가 작은 선을 더 정확하게 감지
    
        # 50
        # 선을 감지하는 데 필요한 최소 교차점 수
        # 값이 높을수록 더 적은 수의 라인이 감지되지만 보다 안정적인 라인이 감지되고, 
        # 값이 낮을수록 더 많은 라인이 감지되지만 더 많은 오탐지가 발생
        
        # 100
        # 감지된 라인의 최소 길이(픽셀)
        # 값이 높을수록 더 길고 의미 있는 라인이 감지되고, 값이 낮을수록 더 많지만 더 짧은 라인이 감지
        
        # 10
        # 병합될 선 세그먼트 사이에 허용되는 최대 간격
        # 값이 높을수록 더 긴 줄이 감지되고 값이 낮을수록 더 많지만 더 짧은 줄이 감지
    
#%% 
        'Filtering'      
        filtered_lines = []
        slope_threshold=0.5 #slope_threshold-  사용자가 원하는 유지할 선범위
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 - x1 == 0:
                continue #-> 다음 for문 순서로
            slope = (y2 - y1) / (x2 - x1)
            if abs(slope) > slope_threshold:
                filtered_lines.append(line)
        return filtered_lines
        
#%%   
        'Detection'
        # Check if any lines were detected
        if filtered_lines is not None:
            self.lanedetected = True

#%%
        'Draw on original'
        # Draw lines on original image
        for line in filtered_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2) #(0,255,0)color / 2 thickness
        
#%%       
        'Show img'
        # Show the image with detected stop lines (for debugging)
        cv2.imshow('Stop Line Detector', cv_image) #이미지 보여주는 것
        cv2.waitKey(1) #waits for a key press for 1 millisecond


#%%
if __name__ == '__main__':
    try:
        StopLineDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass