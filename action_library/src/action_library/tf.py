import rospy
# import tf2_ros
import tf

class TfInterface:

  parent_frame='base_link'
  num_attempts=5000
  timeout_duration=5.0

  def __init__(self):
    # self.tf_buffer = tf2_ros.Buffer()
    # tf2_ros.TransformListener(self.tf_buffer)
    self.tf_li = tf.TransformListener()

  # def get(self, frame_id):
  #   tf = self.tf_buffer.lookup_transform('base_link', frame_id, rospy.Time())
  #   t = tf.transform.translation
  #   r = tf.transform.rotation
  #   return [t.x, t.y, t.z], [r.x, r.y, r.z, r.w]


  # def get_unsafe(self, frame_id):
  #   trans, rot = self.tf_li.lookupTransform('base_link', frame_id, now)

  def get(self, frame_id):
    timeout = rospy.Duration(self.timeout_duration)
    trans = None
    for i in range(self.num_attempts):
      now = rospy.Time.now()
      self.tf_li.waitForTransform('base_link', frame_id, now, timeout)
      trans, rot = self.tf_li.lookupTransform('base_link', frame_id, now)
      if trans is not None:
        break
      rospy.sleep(0.001)
    else:
      raise ValueError('error when looking up transform')
    return trans, rot

tf_interface = TfInterface()