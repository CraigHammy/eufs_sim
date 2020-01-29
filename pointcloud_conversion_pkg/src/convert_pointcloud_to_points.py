import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from pointcloud_conversion_pkg.srv import GetPointCloudPoints, GetPointCloudPointsResponse
from geometry_msgs.msg import Point
from std_msgs.msg import Header

def convert(pointcloud_message):
    ''' 
    Function that takes in a sensor_msgs/PointCloud2 message and returns a 
    2D array made of a list of Point(x, y, z) coordinates of each point in the cloud
    '''

    #get points from PointCloud data 
    field_names = [field.name for field in pointcloud_message]
    pointcloud_data = list(pc2.read_points(pointcloud_message, skip_nans=True, field_names=field_names))
    
    #check if message is empty 
    if len(pointcloud_data) == 0:
        rospy.logwarn("The current PointCloud2 message is empty.")
        return None

    #return array of LocationXYZ objects 
    rospy.logwarn("Converting PointCloud message into Point[].")
    xyz = [Point(x=x, y=y, z=z) for x, y, z in pointcloud_data]
    header = Header()
    header.frame_id, header.seq = pointcloud_message.header.frame_id, pointcloud_message.header.seq
    header.stamp = rospy.Time.now()
    return header, xyz

def response_callback(request):
    ''' 
    Callback function used by the service server to process
    requests from clients. It returns a getXYZReponse
    '''
    rospy.logwarn("Running PointCloud to Point[] conversion request...")
    response = convert(request.pointcloud_message)
    return GetPointCloudPointsResponse(header=response[0], locations=response[1])

    
if __name__ == '__main__':
    rospy.init_node("pc2points_service_server_node")
    service = rospy.Service('/get_pointcloud_points', GetPointCloudPoints, response_callback)
    rospy.spin()

