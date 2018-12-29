from styx_msgs.msg import TrafficLight
import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        pass


    def find_circles(self, bw_image):
        img = cv2.GaussianBlur(bw_image, (5, 5), 0)
        min_radius = bw_image.shape[0] / 200
        max_radius = bw_image.shape[0] / 20
        min_distance = min_radius * 6
        circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT,
                                1,
                                minDist = int(min_distance),
                                param1=50,
                                param2=10,
                                minRadius=int(min_radius),
                                maxRadius=int(max_radius))
        if circles is not None:
            return circles
        else:
            return []

    def find_red(self, hsv):
        red_lower_mask = cv2.inRange(hsv, (0,160,160), (10,255,255))
        red_upper_mask = cv2.inRange(hsv, (170,160,160), (180,255,255))
        mask = red_lower_mask | red_upper_mask
        circles = self.find_circles(mask)
        return circles

    def find_yellow(self, hsv):
        mask = cv2.inRange(hsv, (20,160,160), (40, 255,255))
        circles = self.find_circles(mask)
        return circles

    def find_green(self, hsv):
        mask = cv2.inRange(hsv, (55,160,160), (75, 255,255))
        circles = self.find_circles(mask)
        return circles

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
            cv2 array: Image in BGR8 with marked traffic lights

        """
        #TODO implement light color prediction

        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        rc = self.find_red(hsv_img)
        yc = self.find_yellow(hsv_img)
        gc = self.find_green(hsv_img)

        rcount = len(rc)
        ycount = len(yc)
        gcount = len(gc)

        if rcount == 0 and ycount == 0 and gcount == 0:
            return TrafficLight.UNKNOWN, image
        
        if rcount > ycount and rcount > gcount:
            img = self.show_circles(rc, image, "RED")
            return TrafficLight.RED, img
        
        if ycount > rcount and ycount > gcount:
            img = self.show_circles(yc, image, "YELLOW")
            return TrafficLight.YELLOW, img

        if gcount > rcount and gcount > ycount:
            img = self.show_circles(gc, image, "GREEN")
            return TrafficLight.GREEN, img
        
        return TrafficLight.UNKNOWN, image

    def show_circles(self, circles, image, color_name):
        result = np.copy(image)
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                if i[2] > 1:
                    cv2.circle(result,(i[0],i[1]),i[2] + 8,(255,255,0), 8)
        
        height, width, _ = image.shape
        cv2.putText(result, color_name, (width/2, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,0), 5, cv2.LINE_AA)

        return result
        

