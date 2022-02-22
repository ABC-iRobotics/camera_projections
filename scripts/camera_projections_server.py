#! /usr/bin/env python

import rospy
import actionlib
import tf
import tf2_ros
from geometry_msgs.msg import Point
from bark_msgs.msg import CameraProjectionsAction, CameraProjectionsResult

import numpy as np
import time

class CameraProjectionsServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('camera_projections', CameraProjectionsAction, self.execute, False)
    self.server.start()
    self.listener = tf.TransformListener()
    rospy.sleep(0.4)
    try:
      self.cam_mtx = rospy.get_param('camera_matrix')
      self.dist = rospy.get_param('distortion_vector')
    except:
      rospy.logerr("Could not load camera_matrix or distortion_vector from ROS param server, using default values ...")
      pass
      # self.cam_mtx = # TODO
      # self.dist = # TODO


  def project_3d_to_2d(self, points_3d, parent_frame):
    '''
    points3d: geometry_msgs/Point[] type 3D points in coordinate frame "parent_frame"
    parent_frame: string, name of the frame in which the 3D points are defined
    '''
    if self.listener.frameExists(parent_frame) and self.listener.frameExists('camera'):
      points_3d_numpy = []
      for p in points_3d:
        v = [p.x, p.y, p.z]
        points_3d_numpy.append(v)
      points_3d_numpy = np.array(points_3d_numpy).astype(np.float32)
      t = self.listener.getLatestCommonTime('camera', parent_frame)
      (trans, rot) = self.listener.lookupTransform('camera', parent_frame, t)
      (rx,ry,rz) = tf.transformations.euler_from_quaternion(rot)
      rvec = np.array([[rx], [ry], [rz]]).astype(np.float)
      tvec = np.array([[trans[0]], [trans[1]], [trans[2]]]).astype(np.float)
      points_2d, _ = cv2.projectPoints(points_3d_numpy, rvec, tvec, self.cam_mtx, self.dist)
      result = CameraProjectionsResult()
      result.space_type = result.IMAGE_2D
      for p in points_2d:
        point_2d = Point()
        point_2d.x = p[0]
        point_2d.y = p[1]
        # TODO: point_2d.z = 
        result.points.append(point_2d)
      return result
    else:
      return None


  def deproject_2d_to_3d(self, points_2d, target_frame):
    '''
    points2d: geometry_msgs/Point[] type 2D points in the image space (x and y are in pixel units in the image, z is the distance of the point from the camera)
    target_frame: string, name of the frame in which the deprojected 3D points will be returned
    '''
    if self.listener.frameExists(target_frame) and self.listener.frameExists('camera'):
      result = CameraProjectionsResult()
      result.space_type = result.PHYSICAL_3D
      result.header.frame_id = target_frame
      t = self.listener.getLatestCommonTime(target_frame, "camera")
      (trans, rot) = self.listener.lookupTransform(target_frame, 'camera', t)
      target_to_camera_trans = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(rot))
      for p in points_2d:
        uv1 = np.array([p.x,p.y,1])
        xyz = np.linalg.inv(self.cam_mtx).dot(uv1*p.z)
        x_tilde = xyz[0]/xyz[2]
        y_tilde = xyz[1]/xyz[2]
        r_sqr = x_tilde**2 + y_tilde**2
        k1,k2,p1,p2,k3 = self.dist # TODO: Check if distortion vector has enough elements
        x_2_tilde = x_tilde*(1+k1*r_sqr+k2*r_sqr**2+k3*r_sqr**3)+2*p1*x_tilde*y_tilde+p2*(r_sqr+2*x_tilde**2)
        y_2_tilde = y_tilde*(1+k1*r_sqr+k2*r_sqr**2+k3*r_sqr**3)+2*p2*x_tilde*y_tilde+p1*(r_sqr+2*y_tilde**2)
        xyz = np.array([x_2_tilde*xyz[2], y_2_tilde*xyz[2], xyz[2], 1])
        (tx, ty, tz, _) = np.dot(target_to_camera_trans, xyz)
        point_3d = Point()
        point_3d.x = tx
        point_3d.y = ty
        point_3d.z = tz
        result.points.append(point_3d)
      return result
    else:
      return None


  def execute(self, goal):
    if goal.space_type == goal.PHYSICAL_3D:
      result = self.project_3d_to_2d(goal.points, goal.header.frame_id)
    else:
      result = self.deproject_2d_to_3d(goal.points, goal.header.frame_id) 
    if result:   
      self.server.set_succeeded(result)
    else:
      self.server.set_aborted(text="Header's frame_id seems to be invalid or unknown or there is no 'camera' frame")

if __name__ == '__main__':
  rospy.init_node('camera_projections_server')

  server = CameraProjectionsServer()
  rospy.spin()